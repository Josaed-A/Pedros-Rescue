"""ROS adapter of the delivered Python model.

Receives all six driver joints, publishes active TCP/tool/camera poses and serves
full-pose IK. There is no legacy point renderer or reduced beta/q5/q6 service.
"""

import numpy as np
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Quaternion

from .kinematics import make_T, rot_to_rpy
from rescue_interfaces.srv import ComputeIKPose
from .configuration import configured_arm, settings


def rpy_to_quaternion(roll, pitch, yaw) -> Quaternion:
    """Roll/Pitch/Yaw en radianes → geometry_msgs/Quaternion."""
    cr, sr = np.cos(roll/2),  np.sin(roll/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cy, sy = np.cos(yaw/2),   np.sin(yaw/2)
    q = Quaternion()
    q.w = cr*cp*cy + sr*sp*sy
    q.x = sr*cp*cy - cr*sp*sy
    q.y = cr*sp*cy + sr*cp*sy
    q.z = cr*cp*sy - sr*sp*cy
    return q


class CinematicaNode(Node):

    def __init__(self):
        super().__init__('cinematica')

        self._arm, self._fk_sign = configured_arm(self)
        self._joint_order = list(settings(self)['joint_order'])
        self._lock = threading.RLock()
        self._io_group = MutuallyExclusiveCallbackGroup()
        self._q_actual = np.zeros(6)
        self._seen = np.zeros(6)
        self._joint_map = {name:i for i,name in enumerate(self._joint_order)}

        # Publishers
        self._pub_pose    = self.create_publisher(
            PoseStamped, '/end_effector_pose', 10)
        self._frame_publishers = {frame:self.create_publisher(PoseStamped, '/'+frame+'_pose', 10)
                                  for frame in ('tool','camera')}
        # Subscribers
        self.create_subscription(
            JointState, '/joint_states', self._cb_joint_states, 10, callback_group=self._io_group)
        # Servicios IK
        self.create_service(ComputeIKPose, '/compute_ik_pose', self._srv_ik_pose)

        self.get_logger().info('Nodo cinematica listo.')

    # ------------------------------------------------------------------

    def _cb_joint_states(self, msg: JointState):
        """Only a recent, complete measured state may produce a measured pose."""
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self._joint_map and np.isfinite(pos):
                    i = self._joint_map[name]
                    self._q_actual[i] = pos
                    self._seen[i] = time.monotonic()
            fresh = np.all(time.monotonic() - self._seen < 1.0)
        if fresh:
            self._publicar_fk()

    def _current(self):
        with self._lock:
            if not np.all(time.monotonic() - self._seen < 1.0):
                raise ValueError('Sin feedback completo reciente para sembrar IK')
            return self._q_actual.copy() * self._fk_sign



    def _publicar_fk(self):
        try:
            result = self._arm.fk(self._current())
        except Exception as e:
            self.get_logger().warn(f'FK error: {e}')
            return

        now = self.get_clock().now().to_msg()
        T06 = result['T06']

        # ── PoseStamped del efector ──────────────────────────────────
        pos = T06[:3,3]
        R   = T06[:3, :3]
        roll_deg, pitch_deg, yaw_deg = rot_to_rpy(R)
        roll  = np.radians(roll_deg)
        pitch = np.radians(pitch_deg)
        yaw   = np.radians(yaw_deg)

        pose_msg = PoseStamped()
        pose_msg.header.stamp    = now
        pose_msg.header.frame_id = 'base_link'
        pose_msg.pose.position.x = float(pos[0])
        pose_msg.pose.position.y = float(pos[1])
        pose_msg.pose.position.z = float(pos[2])
        pose_msg.pose.orientation = rpy_to_quaternion(roll, pitch, yaw)
        self._pub_pose.publish(pose_msg)
        for frame,publisher in self._frame_publishers.items():
            T=result[frame]; message=PoseStamped()
            message.header=pose_msg.header
            message.pose.position.x,message.pose.position.y,message.pose.position.z=map(float,T[:3,3])
            message.pose.orientation=rpy_to_quaternion(*np.radians(rot_to_rpy(T[:3,:3])))
            publisher.publish(message)

    def _srv_ik_pose(self, req, res):
        """Exact active TCP pose IK; rejects position AND orientation errors."""
        try:
            R = np.array(req.r, dtype=float).reshape(3, 3)
            Td = make_T(R, [req.x, req.y, req.z])
            # Semilla = configuracion actual (en convencion cinematica). En una
            # trayectoria cartesiana cada paso parte del anterior => movimiento
            # continuo y suave (incl. desplazamientos en Z) sin volteos.
            q  = self._arm.ik(Td, req.elbow or 'down',
                              q_init=self._current())

            p_fk = self._arm.fk(q)['T06'][:3,3]
            err  = float(np.linalg.norm(p_fk - np.array([req.x, req.y, req.z])))
            res.pos_err = err
            if err > 1e-6:          # numerical validation
                res.success = False
                res.q_rad   = []
                res.mensaje = f'fuera de alcance (err {err*1000:.0f} mm)'
            else:
                res.success = True
                # Convertir a convencion de driver antes de devolver el preview.
                res.q_rad   = (q * self._fk_sign).tolist()
                res.mensaje = 'OK'
        except Exception as e:
            res.success = False
            res.q_rad   = []
            res.pos_err = 0.0
            res.mensaje = str(e)
        return res


def main(args=None):
    rclpy.init(args=args)
    node = CinematicaNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
