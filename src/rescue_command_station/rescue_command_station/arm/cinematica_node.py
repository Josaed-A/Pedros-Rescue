"""
cinematica_node.py
==================
Nodo ROS2 — cinematica directa e inversa del brazo 6-DOF.

Suscribe /joint_states, calcula FK continuamente y publica resultados.
Expone servicio /compute_ik para que la GUI calcule poses objetivo.

Topics suscritos:
  /joint_states           (sensor_msgs/JointState)

Topics publicados:
  /end_effector_pose      (geometry_msgs/PoseStamped)  — pose del efector
  /fk_points              (sensor_msgs/JointState)     — puntos 3D del brazo
                           (nombre[i] = "p{i}", position = x,y,z intercalados)

Servicios:
  /compute_ik             (rescue_interfaces/ComputeIK)
"""

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Quaternion

from rescue_command_station.arm.kinematics import Arm6DOF, ArmParams, make_T, rotx, rotz, roty, rot_to_rpy
from rescue_interfaces.srv import ComputeIK, ComputeIKPose


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

        # Parametros geometricos desde YAML
        self.declare_parameter('base_height',  0.14)
        self.declare_parameter('L1',           0.22)
        self.declare_parameter('L2',           0.20)
        self.declare_parameter('L3',           0.12)
        self.declare_parameter('tool_length',  0.10)
        self.declare_parameter('joint_order',
            ['Base', 'Hombro', 'Codo', 'Munieca_P', 'Munieca_Y', 'Munieca_R'])
        # Joints cuyo sentido de giro en la visualizacion va invertido respecto
        # al servo real (el modelo cinematico gira al reves que el hardware).
        self.declare_parameter('sim_invert_joints', ['Munieca_P'])

        p = ArmParams(
            base_height = self.get_parameter('base_height').value,
            L1          = self.get_parameter('L1').value,
            L2          = self.get_parameter('L2').value,
            L3          = self.get_parameter('L3').value,
            tool_length = self.get_parameter('tool_length').value,
        )
        self._arm         = Arm6DOF(p)
        self._joint_order = self.get_parameter('joint_order').value
        self._q_actual    = np.zeros(len(self._joint_order))
        self._joint_map   = {name: i for i, name in enumerate(self._joint_order)}

        # Signo de visualizacion por joint: -1 invierte el sentido en la FK
        # (solo afecta el dibujo/pose, no los comandos al servo).
        invertidos = list(self.get_parameter('sim_invert_joints').value)
        self._fk_sign = np.array(
            [-1.0 if name in invertidos else 1.0 for name in self._joint_order])

        # Publishers
        self._pub_pose    = self.create_publisher(
            PoseStamped, '/end_effector_pose', 10)
        self._pub_points  = self.create_publisher(
            JointState,  '/fk_points', 10)
        self._pub_preview = self.create_publisher(
            JointState,  '/sim/fk_points_preview', 10)

        # Subscribers
        self.create_subscription(
            JointState, '/joint_states',         self._cb_joint_states,   10)
        self.create_subscription(
            JointState, '/joint_states_preview',  self._cb_joint_preview,  10)

        # Servicios IK
        self.create_service(ComputeIK,     '/compute_ik',      self._srv_ik)
        self.create_service(ComputeIKPose, '/compute_ik_pose', self._srv_ik_pose)

        self.get_logger().info('Nodo cinematica listo.')

    # ------------------------------------------------------------------

    def _cb_joint_states(self, msg: JointState):
        """Recibe posiciones de ambos drivers y recalcula FK."""
        for name, pos in zip(msg.name, msg.position):
            if name in self._joint_map:
                self._q_actual[self._joint_map[name]] = pos

        self._publicar_fk()

    def _cb_joint_preview(self, msg: JointState):
        """Recibe q_rad de preview IK y publica /sim/fk_points_preview."""
        q_preview = self._q_actual.copy()
        for name, pos in zip(msg.name, msg.position):
            if name in self._joint_map:
                q_preview[self._joint_map[name]] = pos
        try:
            result = self._arm.fk(q_preview * self._fk_sign)
        except Exception:
            return
        pts = result['points']
        pts_msg = JointState()
        pts_msg.header.stamp = self.get_clock().now().to_msg()
        for i, pt in enumerate(pts):
            pts_msg.name.append(f'p{i}')
            pts_msg.position.extend([float(pt[0]), float(pt[1]), float(pt[2])])
        self._pub_preview.publish(pts_msg)

    def _publicar_fk(self):
        try:
            result = self._arm.fk(self._q_actual * self._fk_sign)
        except Exception as e:
            self.get_logger().warn(f'FK error: {e}')
            return

        now = self.get_clock().now().to_msg()
        pts = result['points']  # (6,3)
        T06 = result['T06']

        # ── PoseStamped del efector ──────────────────────────────────
        pos = pts[-1]
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

        # ── Puntos del brazo (para visualizacion) ────────────────────
        # Formato: name=["p0".."p5"], position=[x0,y0,z0, x1,y1,z1, ...]
        pts_msg = JointState()
        pts_msg.header.stamp = now
        for i, pt in enumerate(pts):
            pts_msg.name.append(f'p{i}')
            pts_msg.position.extend([float(pt[0]), float(pt[1]), float(pt[2])])
        self._pub_points.publish(pts_msg)

    # ------------------------------------------------------------------
    #  Servicio IK (implementar cuando este el srv custom)
    # ------------------------------------------------------------------

    def _srv_ik(self, req, res):
        try:
            q1t = np.arctan2(req.y, req.x)
            Rd  = (rotz(q1t)
                   @ roty(np.radians(req.beta_deg))
                   @ rotx(np.radians(req.q5_deg))   # Muñeca_Y (eje X)
                   @ rotz(np.radians(req.q6_deg)))[:3, :3]  # Muñeca_R (eje Z)
            Td  = make_T(Rd, [req.x, req.y, req.z])
            # Semilla = configuracion actual (en convencion cinematica) para
            # continuidad/suavidad y evitar volteos de munieca.
            q   = self._arm.ik(Td, req.elbow,
                               q_init=self._q_actual * self._fk_sign)
            res.success = True
            # Convertir de convencion cinematica a convencion de servo
            # (fk_sign invierte los joints marcados en sim_invert_joints).
            res.q_rad   = (q * self._fk_sign).tolist()
            res.mensaje = 'OK'
        except Exception as e:
            res.success = False
            res.q_rad   = []
            res.mensaje = str(e)
        return res

    def _srv_ik_pose(self, req, res):
        """IK desde una pose completa Td = [R|p] (para teleoperacion cartesiana).

        Usa el mismo solver geometrico self._arm.ik sin cambios. Solo arma Td
        con la R recibida y verifica que la POSICION sea alcanzable (la
        orientacion es best-effort: la munieca de 2 ejes puede no cubrir toda
        SO(3) en cualquier punto).
        """
        try:
            R = np.array(req.r, dtype=float).reshape(3, 3)
            Td = make_T(R, [req.x, req.y, req.z])
            # Semilla = configuracion actual (en convencion cinematica). En una
            # trayectoria cartesiana cada paso parte del anterior => movimiento
            # continuo y suave (incl. desplazamientos en Z) sin volteos.
            q  = self._arm.ik(Td, req.elbow or 'down',
                              q_init=self._q_actual * self._fk_sign)

            p_fk = self._arm.fk(q)['points'][-1]
            err  = float(np.linalg.norm(p_fk - np.array([req.x, req.y, req.z])))
            res.pos_err = err
            if err > 0.02:          # > 2 cm: objetivo fuera de alcance
                res.success = False
                res.q_rad   = []
                res.mensaje = f'fuera de alcance (err {err*1000:.0f} mm)'
            else:
                res.success = True
                # Convertir a convencion de servo antes de devolver (ver _srv_ik).
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
