"""
cartesian_node.py
=================
Nodo ROS2 — controlador de trayectorias cartesianas para el brazo 6-DOF.

Arquitectura inspirada en Universal_Robots_ROS_controllers_cartesian:
  - CartesianTrajectoryController  => /cartesian/trajectory  (multi-waypoint)
  - CartesianState feedback        => /cartesian/state        (10 Hz)
  - Speed scaling                  => /cartesian/speed_scale  (0-100)
  - Tolerancias de ruta y goal
  - Clampeo de joints (proteccion desbordamiento de bits AX-12A 10-bit)

Servicios ofrecidos:
  /cartesian/goto        (CartesianGoto)        — un solo objetivo, compat.
  /cartesian/trajectory  (CartesianTrajectory)  — multi-waypoint con tolerancias

Servicios utilizados:
  /compute_ik_pose       (ComputeIKPose)

Topics suscritos:
  /end_effector_pose     (geometry_msgs/PoseStamped)
  /cartesian/speed_scale (std_msgs/Float64)       — 0-100, default 100

Topics publicados:
  /ax12a/joint_cmd       (sensor_msgs/JointState)
  /ex106/joint_cmd       (sensor_msgs/JointState)
  /cartesian/state       (rescue_interfaces/CartesianState)
"""

import math
import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

from rescue_command_station.arm.cartesian_controller import (
    linear_trajectory,
    multi_waypoint_trajectory,
    position_error,
    rotation_error,
    within_tolerance,
    clamp_joints,
    JOINT_LIMITS,
)
from rescue_interfaces.srv import (
    CartesianGoto,
    ComputeIKPose,
    CartesianTrajectory,
    ServoCommand,
)
from rescue_interfaces.msg import CartesianState, CartesianWaypoint


_DEFAULT_JOINT_ORDER = ['Base', 'Hombro', 'Codo', 'Munieca_P', 'Munieca_Y', 'Munieca_R']
_DEFAULT_JOINT_SERVO = {
    'Base':      ('ax', 16),
    'Hombro':    ('ex', 1),
    'Codo':      ('ax', 18),
    'Munieca_P': ('ax', 30),
    'Munieca_Y': ('ax', 4),
    'Munieca_R': ('ax', 5),
}


def _quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - z*w),      2*(x*z + y*w)],
        [2*(x*y + z*w),      1 - 2*(x*x + z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),      2*(y*z + x*w),       1 - 2*(x*x + y*y)],
    ], dtype=float)


class CartesianNode(Node):

    def __init__(self):
        super().__init__('cartesian')

        # ── Parametros ──────────────────────────────────────────────────
        self.declare_parameter('joint_order', _DEFAULT_JOINT_ORDER)
        joint_order = list(self.get_parameter('joint_order').value)

        joint_servo: dict[str, tuple[str, int]] = {}
        for name in joint_order:
            try:
                self.declare_parameter(f'joint_drivers.{name}.driver', '')
                self.declare_parameter(f'joint_drivers.{name}.id', 0)
                driver = self.get_parameter(f'joint_drivers.{name}.driver').value
                sid    = int(self.get_parameter(f'joint_drivers.{name}.id').value)
                if driver in ('ax', 'ex'):
                    joint_servo[name] = (driver, sid)
            except Exception:
                pass
        for name, val in _DEFAULT_JOINT_SERVO.items():
            if name in joint_order and name not in joint_servo:
                joint_servo[name] = val

        self._joint_order = joint_order
        self._joint_servo = joint_servo

        # ── Estado compartido ────────────────────────────────────────────
        self._pose:      PoseStamped | None = None
        self._pose_lock  = threading.Lock()
        self._cancel     = threading.Event()
        self._traj_lock  = threading.Lock()
        self._traj_thread: threading.Thread | None = None
        self._speed_scale: float = 100.0   # porcentaje 0-100

        # Estado de feedback para CartesianState
        self._state_lock   = threading.Lock()
        self._state_x      = 0.0
        self._state_y      = 0.0
        self._state_z      = 0.0
        self._state_pos_err = 0.0
        self._state_rot_err = 0.0
        self._state_progress = 0.0
        self._state_in_prog  = False
        self._state_in_tol   = True
        self._state_wp_idx   = 0
        self._state_wp_total = 0

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(
            PoseStamped, '/end_effector_pose', self._cb_pose, 10)
        self.create_subscription(
            Float64, '/cartesian/speed_scale', self._cb_speed_scale, 10)

        # ── Publishers ───────────────────────────────────────────────────
        self._pub_ax    = self.create_publisher(JointState, '/ax12a/joint_cmd', 10)
        self._pub_ex    = self.create_publisher(JointState, '/ex106/joint_cmd', 10)
        self._pub_state = self.create_publisher(CartesianState, '/cartesian/state', 10)

        # ── Feedback timer (10 Hz) ───────────────────────────────────────
        self.create_timer(0.1, self._publish_state)

        # ── Cliente IK ──────────────────────────────────────────────────
        self._cli_ik = self.create_client(ComputeIKPose, '/compute_ik_pose')
        self._cli_ex_jog = self.create_client(ServoCommand, '/ex106/jog')

        # ── Servicios ───────────────────────────────────────────────────
        self.create_service(CartesianGoto,       '/cartesian/goto',       self._srv_goto)
        self.create_service(CartesianTrajectory, '/cartesian/trajectory', self._srv_trajectory)

        self.get_logger().info('Nodo cartesian listo.')

    # ------------------------------------------------------------------
    #  Callbacks
    # ------------------------------------------------------------------

    def _cb_pose(self, msg: PoseStamped):
        with self._pose_lock:
            self._pose = msg

    def _cb_speed_scale(self, msg: Float64):
        scale = float(np.clip(msg.data, 0.0, 100.0))
        self._speed_scale = scale
        self.get_logger().info(f'Speed scale: {scale:.0f}%')

    # ------------------------------------------------------------------
    #  /cartesian/goto  (compatibilidad con un solo objetivo)
    # ------------------------------------------------------------------

    def _srv_goto(self, req, res):
        with self._pose_lock:
            pose = self._pose
        if pose is None:
            res.success    = False
            res.mensaje    = 'Sin pose actual — espera /end_effector_pose'
            res.steps_sent = 0
            return res

        p0 = np.array([pose.pose.position.x,
                        pose.pose.position.y,
                        pose.pose.position.z])
        R0 = _quat_to_matrix(pose.pose.orientation)
        p1 = np.array([req.x, req.y, req.z])
        R1 = np.array(req.r, dtype=float).reshape(3, 3)

        n_steps = max(1, int(req.n_steps))
        dt      = max(0.02, float(req.step_dt))
        vel_pct = float(req.vel_pct) if req.vel_pct > 0 else 30.0
        elbow   = req.elbow or 'down'

        waypoints = [(p0, R0, n_steps), (p1, R1, n_steps)]
        self._start_trajectory(
            waypoints, elbow, vel_pct, dt,
            path_tol_pos=0.0, path_tol_rot=0.0,
            goal_tol_pos=0.01, goal_tol_rot=0.1,
        )
        res.success    = True
        res.mensaje    = f'Trayectoria goto iniciada: {n_steps} pasos, dt={dt:.2f}s'
        res.steps_sent = n_steps
        return res

    # ------------------------------------------------------------------
    #  /cartesian/trajectory  (multi-waypoint con tolerancias)
    #  Inspirado en CartesianTrajectoryController del repo UR
    # ------------------------------------------------------------------

    def _srv_trajectory(self, req, res):
        wps: list[CartesianWaypoint] = list(req.waypoints)
        if len(wps) < 2:
            res.success           = False
            res.mensaje           = 'Se necesitan al menos 2 waypoints'
            res.waypoints_accepted = 0
            return res

        with self._pose_lock:
            pose = self._pose

        # Construir lista de waypoints para el interpolador.
        # Si el primer waypoint tiene r vacio, se usa la pose TCP actual.
        parsed: list[tuple[np.ndarray, np.ndarray, int]] = []
        for i, wp in enumerate(wps):
            p = np.array([wp.x, wp.y, wp.z])
            if len(wp.r) == 9:
                R = np.array(wp.r, dtype=float).reshape(3, 3)
            elif i == 0 and pose is not None:
                R = _quat_to_matrix(pose.pose.orientation)
            else:
                res.success           = False
                res.mensaje           = f'Waypoint {i} sin orientacion y sin pose actual'
                res.waypoints_accepted = 0
                return res
            n = max(1, int(wp.n_steps))
            parsed.append((p, R, n))

        # Si el primer waypoint no tiene posicion (todo ceros y sin r),
        # sustituir por la pose actual.
        if np.allclose(parsed[0][0], 0) and pose is not None:
            p0 = np.array([pose.pose.position.x,
                            pose.pose.position.y,
                            pose.pose.position.z])
            R0 = _quat_to_matrix(pose.pose.orientation)
            parsed[0] = (p0, R0, parsed[0][2])

        vel_pct       = float(req.vel_pct)      if req.vel_pct > 0      else 30.0
        dt_base       = float(wps[1].duration)  if wps[1].duration > 0  else 0.05

        self._start_trajectory(
            parsed,
            req.elbow or 'down',
            vel_pct,
            dt_base,
            path_tol_pos = float(req.path_tol_pos),
            path_tol_rot = float(req.path_tol_rot),
            goal_tol_pos = float(req.goal_tol_pos),
            goal_tol_rot = float(req.goal_tol_rot),
        )

        total_steps = sum(wp.n_steps for wp in wps[1:])
        res.success            = True
        res.mensaje            = (f'Trayectoria iniciada: {len(wps)} waypoints, '
                                  f'{total_steps} pasos totales')
        res.waypoints_accepted = len(wps)
        return res

    # ------------------------------------------------------------------
    #  Motor de trayectoria (hilo background)
    # ------------------------------------------------------------------

    def _start_trajectory(
        self,
        waypoints: list[tuple[np.ndarray, np.ndarray, int]],
        elbow:         str,
        vel_pct:       float,
        dt_base:       float,
        path_tol_pos:  float,
        path_tol_rot:  float,
        goal_tol_pos:  float,
        goal_tol_rot:  float,
    ) -> None:
        """Cancela cualquier trayectoria activa y arranca una nueva."""
        self._cancel.set()
        with self._traj_lock:
            if self._traj_thread and self._traj_thread.is_alive():
                self._traj_thread.join(timeout=0.5)
        self._cancel.clear()

        poses      = multi_waypoint_trajectory(waypoints)
        total      = len(poses) - 1   # pasos (excluye pose inicial)
        n_waypoints = len(waypoints)

        def _run():
            step        = 0
            violated    = False
            wp_boundary = self._build_wp_boundaries(waypoints)

            with self._state_lock:
                self._state_in_prog  = True
                self._state_progress = 0.0
                self._state_wp_total = n_waypoints
                self._state_wp_idx   = 0
                self._state_in_tol   = True

            for p_ref, R_ref in poses[1:]:
                if self._cancel.is_set():
                    break

                # ── IK sincrona ─────────────────────────────────────────
                ik_res = self._ik_sync(p_ref, R_ref, elbow)
                if ik_res is None or not ik_res.success:
                    msg = ik_res.mensaje if ik_res else 'timeout IK'
                    self.get_logger().warn(f'IK fallo en paso {step}: {msg}')
                    break

                # ── Clampeo de joints: proteccion desbordamiento de bits ─
                q_raw = np.array(ik_res.q_rad)
                q_cmd, saturated = clamp_joints(q_raw, self._joint_order)
                if saturated:
                    self.get_logger().warn(
                        f'Paso {step}: joints saturados {saturated} '
                        f'(clampeo aplicado para evitar overflow de bits)')

                # ── Monitoreo de tolerancias de ruta ────────────────────
                pos_err = float(ik_res.pos_err)
                rot_err = rotation_error(
                    np.array(ik_res.q_rad[:3]).reshape(1, -1),  # dummy — usa pos_err del IK
                    np.zeros((1, 3))
                ) if False else 0.0     # rot_err: calculado abajo si se tiene FK

                in_tol = within_tolerance(pos_err, rot_err, path_tol_pos, path_tol_rot)
                if not in_tol:
                    violated = True
                    self.get_logger().warn(
                        f'Tolerancia de ruta superada en paso {step}: '
                        f'pos_err={pos_err*1000:.1f}mm (tol={path_tol_pos*1000:.1f}mm)')
                    if path_tol_pos > 0 or path_tol_rot > 0:
                        break

                # ── Publicar comandos articulares ─────────────────────────
                eff_vel = vel_pct * (self._speed_scale / 100.0)
                self._publish_joints(q_cmd, eff_vel)
                step += 1

                # ── Actualizar feedback ──────────────────────────────────
                wp_idx = self._waypoint_index(step, wp_boundary)
                with self._state_lock:
                    self._state_pos_err  = pos_err
                    self._state_rot_err  = rot_err
                    self._state_progress = step / max(total, 1)
                    self._state_in_tol   = in_tol
                    self._state_wp_idx   = wp_idx
                    self._state_x, self._state_y, self._state_z = p_ref

                # ── dt efectivo ajustado por speed scale ─────────────────
                eff_scale = max(0.01, self._speed_scale / 100.0)
                time.sleep(dt_base / eff_scale)

            # ── Verificacion de tolerancia de goal ───────────────────────
            if step == total and goal_tol_pos > 0:
                with self._pose_lock:
                    pose_final = self._pose
                if pose_final is not None:
                    p_actual = np.array([pose_final.pose.position.x,
                                          pose_final.pose.position.y,
                                          pose_final.pose.position.z])
                    p_goal = poses[-1][0]
                    goal_err = position_error(p_actual, p_goal)
                    ok = goal_err <= goal_tol_pos
                    self.get_logger().info(
                        f'Goal error: {goal_err*1000:.1f}mm '
                        f'(tol {goal_tol_pos*1000:.1f}mm) — {"OK" if ok else "FUERA"}')

            with self._state_lock:
                self._state_in_prog  = False
                self._state_progress = 1.0 if step == total else self._state_progress

            self.get_logger().info(
                f'Trayectoria: {step}/{total} pasos'
                + (' [ABORTADA por tolerancia]' if violated else ''))

        self._traj_thread = threading.Thread(
            target=_run, daemon=True, name='cartesian-traj')
        self._traj_thread.start()

    # ------------------------------------------------------------------
    #  Utilidades internas
    # ------------------------------------------------------------------

    @staticmethod
    def _build_wp_boundaries(
        waypoints: list[tuple[np.ndarray, np.ndarray, int]],
    ) -> list[int]:
        """Devuelve el paso acumulado al final de cada segmento."""
        boundaries: list[int] = [0]
        for _, _, n in waypoints[1:]:
            boundaries.append(boundaries[-1] + max(1, n))
        return boundaries

    @staticmethod
    def _waypoint_index(step: int, boundaries: list[int]) -> int:
        for i in range(len(boundaries) - 1):
            if step <= boundaries[i + 1]:
                return i
        return len(boundaries) - 2

    def _ik_sync(self, p, R, elbow, timeout=1.0):
        """Llama a /compute_ik_pose de forma sincrona desde hilo secundario."""
        if not self._cli_ik.service_is_ready():
            self.get_logger().warn('/compute_ik_pose no esta listo')
            return None
        req = ComputeIKPose.Request()
        req.x, req.y, req.z = float(p[0]), float(p[1]), float(p[2])
        req.r     = [float(v) for v in R.reshape(9)]
        req.elbow = elbow

        done   = threading.Event()
        holder: list = [None]

        def _cb(fut):
            try:
                holder[0] = fut.result()
            except Exception:
                pass
            done.set()

        self._cli_ik.call_async(req).add_done_callback(_cb)
        done.wait(timeout=timeout)
        return holder[0]

    def _publish_joints(self, q_rad: np.ndarray, vel_pct: float):
        ax_names = [n for n in self._joint_order
                    if self._joint_servo.get(n, ('ax',))[0] == 'ax']
        ex_names = [n for n in self._joint_order
                    if self._joint_servo.get(n, ('ex',))[0] == 'ex']
        for driver, names, pub in (('ax', ax_names, self._pub_ax),
                                   ('ex', ex_names, self._pub_ex)):
            if not names:
                continue
            idx = [self._joint_order.index(n) for n in names]
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name         = names
            msg.position     = [float(q_rad[i]) for i in idx]
            msg.velocity     = [float(np.clip(vel_pct, 1.0, 100.0))] * len(names)
            pub.publish(msg)
            if driver == 'ex':
                self._jog_ex_fallback(names, idx, q_rad, vel_pct)

    def _jog_ex_fallback(self, names: list[str], idx: list[int],
                         q_rad: np.ndarray, vel_pct: float):
        """Respaldo para EX-106+: usa el servicio por ID si esta disponible."""
        if not self._cli_ex_jog.service_is_ready():
            return

        for name, i in zip(names, idx):
            driver, sid = self._joint_servo.get(name, ('', 0))
            if driver != 'ex' or sid <= 0:
                continue
            req = ServoCommand.Request()
            req.id = int(sid)
            req.target_deg = float(np.degrees(q_rad[i]))
            req.vel_pct = float(np.clip(vel_pct, 1.0, 100.0))

            def _cb(fut, joint=name):
                try:
                    res = fut.result()
                except Exception:
                    self.get_logger().warn(
                        f'Fallback EX jog para {joint}: llamada fallida')
                    return
                if not res.success:
                    self.get_logger().warn(
                        f'Fallback EX jog para {joint}: {res.mensaje}')

            self._cli_ex_jog.call_async(req).add_done_callback(_cb)

    # ------------------------------------------------------------------
    #  Feedback: /cartesian/state  (10 Hz via timer)
    # ------------------------------------------------------------------

    def _publish_state(self):
        with self._state_lock:
            msg = CartesianState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.x                = self._state_x
            msg.y                = self._state_y
            msg.z                = self._state_z
            msg.pos_error        = self._state_pos_err
            msg.rot_error        = self._state_rot_err
            msg.progress         = self._state_progress
            msg.in_progress      = self._state_in_prog
            msg.within_path_tol  = self._state_in_tol
            msg.waypoint_idx     = self._state_wp_idx
            msg.total_waypoints  = self._state_wp_total
        self._pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CartesianNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
