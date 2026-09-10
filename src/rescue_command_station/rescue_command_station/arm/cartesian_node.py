"""ROS adapter for the shared planner. Full preflight, no post-IK clipping."""
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger, SetBool
from rescue_interfaces.srv import CartesianGoto, CartesianTrajectory
from rescue_interfaces.msg import CartesianState, ArmStatus
from .configuration import configured_arm, settings, joint_drivers
from .motion import plan_waypoints, Plan
from .cartesian_controller import within_tolerance


class CartesianNode(Node):
    def __init__(self):
        super().__init__('cartesian')
        self.arm, self.signs = configured_arm(self)
        self.names = settings(self)['joint_order']; self.drivers = joint_drivers(self)
        self.q = np.zeros(6); self.seen = np.zeros(6)
        self.lock = threading.Lock(); self.cancel = threading.Event()
        self.command_lock = threading.RLock()
        self.worker = None; self.speed = 100.0
        self.bus_ready = {}
        self.maintenance = False
        self.state = CartesianState()
        self.publisher = self.create_publisher(CartesianState, '/cartesian/state', 10)
        self.commands = {d:self.create_publisher(JointState, topic, 10)
                         for d,topic in [('ax','/ax12a/joint_cmd'),('ex','/ex106/joint_cmd')]}
        self.io_group = ReentrantCallbackGroup()
        # One admission group rejects new work while another request is planning.
        self.request_group = ReentrantCallbackGroup()
        self.create_subscription(JointState, '/joint_states', self._joints, 10, callback_group=self.io_group)
        self.create_subscription(JointState, '/arm/manual_joint_cmd', self._manual, 10, callback_group=self.io_group)
        self.create_subscription(Float64, '/cartesian/speed_scale', self._speed, 10, callback_group=self.io_group)
        for driver,topic in [('ax','/ax12a/status'),('ex','/ex106/status')]:
            self.create_subscription(ArmStatus, topic, lambda m,d=driver:self._status(d,m), 10, callback_group=self.io_group)
        self.create_service(CartesianGoto, '/cartesian/goto', self._goto, callback_group=self.request_group)
        self.create_service(CartesianTrajectory, '/cartesian/trajectory', self._trajectory, callback_group=self.request_group)
        self.create_service(Trigger, '/cartesian/cancel', self._cancel, callback_group=self.io_group)
        self.create_service(SetBool, '/cartesian/maintenance', self._maintenance, callback_group=self.io_group)
        self.create_timer(0.1, self._feedback, callback_group=self.io_group)

    def _joints(self, msg):
        with self.lock:
            for name, value in zip(msg.name, msg.position):
                if name in self.names and np.isfinite(value):
                    i = self.names.index(name)
                    self.q[i] = value*self.signs[i]; self.seen[i] = time.monotonic()

    def _speed(self, msg):
        if np.isfinite(msg.data): self.speed = float(np.clip(msg.data, 0, 100))

    def _status(self, driver, msg):
        ready=not msg.emergencia and not msg.modo_calib and msg.conectado
        with self.command_lock:
            self.bus_ready[driver]=(ready,time.monotonic())
            if not ready: self.cancel.set()

    def _check_buses(self):
        if self.maintenance:
            raise ValueError('Control bloqueado por mantenimiento; confirmar calibracion para liberar')
        for driver in set(d for d,_ in self.drivers.values()):
            ready,stamp=self.bus_ready.get(driver,(False,0))
            if not ready or time.monotonic()-stamp>2:
                raise ValueError('Ambos buses deben estar conectados, sin emergencia y con status reciente')

    def _current(self):
        with self.lock:
            if np.any(time.monotonic()-self.seen > 1.0):
                raise ValueError('Se requieren estados recientes de las seis articulaciones')
            return self.q.copy()

    def _goto(self, req, res):
        try:
            q = self._current(); T = self.arm.fk(q)['T06']
            waypoints = [(T[:3,3],T[:3,:3],1),
                         (np.array([req.x,req.y,req.z]),np.array(req.r).reshape(3,3),max(1,int(req.n_steps)))]
            plan = self._start(q, waypoints, req.step_dt, req.vel_pct)
            res.success = True; res.steps_sent = 0
            res.mensaje = f'Plan validado: {len(plan.q)-1} muestras; ejecucion asincrona'
        except Exception as exc:
            res.success = False; res.steps_sent = 0; res.mensaje = str(exc)
        return res

    def _trajectory(self, req, res):
        try:
            q = self._current(); T = self.arm.fk(q)['T06']; wps = list(req.waypoints)
            if len(wps)<2: raise ValueError('Se requieren dos waypoints')
            parsed=[]
            for i,w in enumerate(wps):
                if i==0 and len(w.r)==0: parsed.append((T[:3,3],T[:3,:3],1))
                else: parsed.append((np.array([w.x,w.y,w.z]),np.array(w.r).reshape(3,3),max(1,int(w.n_steps))))
            # CartesianWaypoint.duration is TOTAL seconds, independently per segment.
            self._start(q, parsed, 0.05, req.vel_pct,
                        req.path_tol_pos,req.path_tol_rot,req.goal_tol_pos,req.goal_tol_rot,
                        durations=[w.duration for w in wps[1:]])
            res.success=True; res.waypoints_accepted=len(wps); res.mensaje='Plan completo validado'
        except Exception as exc:
            res.success=False; res.waypoints_accepted=0; res.mensaje=str(exc)
        return res

    def _manual(self, msg):
        """Serialize GUI direct commands and preflight/execution at one publisher."""
        try:
            with self.command_lock:
                if self.state.in_progress or (self.worker and self.worker.is_alive()):
                    raise ValueError('Comando directo rechazado durante planificacion/trayectoria')
                if len(msg.name) != 6 or set(msg.name) != set(self.names) or len(msg.position) != 6:
                    raise ValueError('El comando manual requiere las seis articulaciones, sin duplicados')
                indices = [msg.name.index(name) for name in self.names]
                q = np.asarray(msg.position, float)[indices]
                velocities = np.asarray(msg.velocity, float)
                if (velocities.shape != (6,) or not np.all(np.isfinite(velocities))
                        or np.any(velocities < 1) or np.any(velocities > 100)):
                    raise ValueError('Se requieren seis velocidades entre 1 y 100')
                model = q * self.signs
                if (not np.all(np.isfinite(model)) or np.any(model < self.arm.limits[:, 0])
                        or np.any(model > self.arm.limits[:, 1])):
                    raise ValueError('Comando manual fuera de limites o no finito')
                self._current(); self._check_buses()
                self.cancel.clear()
                self._publish(q, float(np.min(velocities)))
        except Exception as exc:
            self.get_logger().error(str(exc))

    def _start(self, q, waypoints, dt, velocity, ptol=0.0, rtol=0.0, gptol=0.01, grtol=0.1,
               durations=None):
        if not np.all(np.isfinite([dt,velocity,ptol,rtol,gptol,grtol])) or min(ptol,rtol,gptol,grtol)<0:
            raise ValueError('Tiempos, velocidad o tolerancias invalidas')
        dt = max(0.02,dt) if dt>0 else 0.05
        velocity = float(np.clip(velocity if velocity>0 else 30,1,100))
        with self.command_lock:
            self._check_buses()
            if self.state.in_progress or (self.worker and self.worker.is_alive()):
                raise ValueError('Planificacion/trayectoria activa; cancelar antes de solicitar otra')
            self.cancel.clear()
            self.state.in_progress=True
            self.state.progress=0.0
            self.state.total_waypoints=len(waypoints)
        try:
            if durations is None:
                plan = plan_waypoints(self.arm,q,waypoints,dt)
                boundaries=np.cumsum([max(0.2,n*dt) for _,_,n in waypoints[1:]])
            else:
                durations=np.asarray(durations,float)
                if (durations.shape != (len(waypoints)-1,) or not np.all(np.isfinite(durations))
                        or np.any(durations < 0.2) or np.any(durations > 120)):
                    raise ValueError('duration es el total por segmento y debe estar entre 0.2 y 120 s')
                # Adapt service units to the delivered planner; no new interpolation.
                times=[0.0]; joints=[q.copy()]
                for i,duration in enumerate(durations):
                    if self.cancel.is_set():
                        raise ValueError('Planificacion cancelada')
                    start_pose=self.arm.fk(joints[-1])['T06']
                    first=waypoints[0] if i==0 else (start_pose[:3,3],start_pose[:3,:3],1)
                    last=waypoints[i+1]
                    segment=plan_waypoints(self.arm,joints[-1],[first,last],float(duration)/last[2])
                    times.extend((segment.t[1:]+times[-1]).tolist()); joints.extend(segment.q[1:])
                plan=Plan(np.asarray(times),np.asarray(joints))
                boundaries=np.cumsum(durations)
            with self.command_lock:
                self._check_buses()
                if np.max(abs(self._current()-q))>np.radians(0.5) or self.cancel.is_set():
                    raise ValueError('El estado cambio durante la planificacion; vuelva a solicitar')
                self.worker=threading.Thread(target=self._execute,
                    args=(plan,velocity,ptol,rtol,gptol,grtol,boundaries),daemon=True)
                self.worker.start()
            return plan
        except Exception:
            with self.command_lock:
                self.state.in_progress=False
                self.state.within_path_tol=False
            raise

    def _execute(self, plan, velocity, ptol, rtol, gptol, grtol, boundaries):
        self.state.in_progress=True; self.state.progress=0.0; self.state.within_path_tol=True
        elapsed=0.0; previous=time.monotonic()
        try:
            while not self.cancel.is_set():
                now=time.monotonic(); scale=self.speed/100
                elapsed=min(plan.t[-1],elapsed+(now-previous)*scale); previous=now
                actual=self._current()
                self._check_buses()
                if scale<=0:
                    self.cancel.wait(0.02); continue
                q=plan.sample(elapsed); target=self.arm.fk(q)['T06']
                e=self.arm.pose_error(actual,target[:3,:3],target[:3,3])
                pe,re=float(np.linalg.norm(e[:3])),float(np.linalg.norm(e[3:]))
                ok=within_tolerance(pe,re,ptol,rtol)
                self.state.pos_error=pe; self.state.rot_error=re; self.state.within_path_tol=ok
                if not ok: raise ValueError('Tolerancia de seguimiento excedida')
                self._publish(q*self.signs,velocity*scale)
                self.state.progress=float(elapsed/plan.t[-1])
                self.state.waypoint_idx=int(min(np.searchsorted(boundaries,elapsed),len(boundaries)-1))
                if elapsed>=plan.t[-1]: break
                self.cancel.wait(0.02)
            if not self.cancel.is_set():
                target=self.arm.fk(plan.q[-1])['T06']; deadline=time.monotonic()+3
                while not self.cancel.is_set():
                    self._check_buses()
                    e=self.arm.pose_error(self._current(),target[:3,:3],target[:3,3])
                    pe,re=float(np.linalg.norm(e[:3])),float(np.linalg.norm(e[3:]))
                    self.state.pos_error=pe; self.state.rot_error=re
                    if within_tolerance(pe,re,gptol,grtol): break
                    if time.monotonic()>deadline: raise ValueError('No alcanzo tolerancia final')
                    self.cancel.wait(0.02)
        except Exception as exc:
            self.state.within_path_tol=False; self.get_logger().error(f'Trayectoria detenida: {exc}')
        finally:
            with self.command_lock:
                if self.cancel.is_set():
                    self.state.within_path_tol=False
                self.state.in_progress=False

    def _publish(self, q, velocity):
        with self.command_lock:
            self._publish_locked(q, velocity)

    def _publish_locked(self, q, velocity):
        if self.cancel.is_set(): return
        self._check_buses()
        for driver,publisher in self.commands.items():
            indices=[i for i,n in enumerate(self.names) if self.drivers[n][0]==driver]
            msg=JointState(); msg.header.stamp=self.get_clock().now().to_msg()
            msg.name=[self.names[i] for i in indices]; msg.position=[float(q[i]) for i in indices]
            msg.velocity=[float(np.clip(velocity,1,100))]*len(indices)
            publisher.publish(msg)

    def _cancel(self, req, res):
        with self.command_lock:
            self.cancel.set()
        res.success=True
        res.message='Sin nuevos puntos; cancelar no sustituye al paro de emergencia del driver'
        return res

    def _maintenance(self, req, res):
        # Latch under the same lock as admission/publication. No automatic expiry:
        # a lost GUI cannot resume a trajectory during calibration or rescue.
        with self.command_lock:
            if req.data:
                self.maintenance = True
                self.cancel.set()
            elif self.state.in_progress or (self.worker and self.worker.is_alive()):
                res.success = False
                res.message = 'Espere la finalizacion del ejecutor antes de liberar mantenimiento'
                return res
            else:
                self.maintenance = False
        res.success = True
        res.message = 'Mantenimiento activo' if req.data else 'Mantenimiento liberado'
        return res

    def _feedback(self):
        try:
            T=self.arm.fk(self._current())['T06']
            self.state.x,self.state.y,self.state.z=map(float,T[:3,3])
            self.state.feedback_valid=True
        except ValueError:
            self.state.x=self.state.y=self.state.z=float('nan')
            self.state.feedback_valid=False
        self.state.maintenance=self.maintenance
        self.state.header.frame_id='base_link'
        self.state.header.stamp=self.get_clock().now().to_msg(); self.publisher.publish(self.state)


def main(args=None):
    rclpy.init(args=args); node=CartesianNode()
    from rclpy.executors import MultiThreadedExecutor
    executor=MultiThreadedExecutor(num_threads=3); executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.cancel.set()
        if node.worker: node.worker.join(timeout=1)
        executor.shutdown(); node.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()
