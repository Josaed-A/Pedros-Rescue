"""
simulacion_node.py
==================
Nodo ROS2 — visualizacion 3D del brazo en tiempo real.

Suscribe /fk_points (publicado por cinematica_node) y renderiza
la estructura del brazo en matplotlib (vista 3D, frontal y superior).

Para preview IK antes de ejecutar, suscribe /sim/fk_points_preview
(publicado por gui_node cuando el usuario usa el simulador).

Topics suscritos:
  /fk_points              (sensor_msgs/JointState)  — posicion real
  /sim/fk_points_preview  (sensor_msgs/JointState)  — preview GUI

Parametros ROS:
  reach (float, default 0.74) — alcance maximo del brazo en metros,
                                  usado para escalar los ejes de las graficas.
                                  Leer desde servos.yaml si/simulacion/reach.

Solucion al conflicto matplotlib + rclpy:
  - rclpy.spin corre en un thread daemon separado
  - matplotlib corre en el thread principal (requerido por TkAgg/tkinter)
  - Los datos se pasan entre threads con un Lock
  - plt.pause(0.12) procesa los eventos de la ventana Tk cada 120 ms
"""

import threading
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt


def _parse_points(msg: JointState) -> np.ndarray:
    """Decodifica el formato [x0,y0,z0, x1,y1,z1,...] a array (N,3)."""
    flat = list(msg.position)
    n    = len(flat) // 3
    return np.array(flat).reshape(n, 3)


class SimulacionNode(Node):

    def __init__(self):
        super().__init__('simulacion')

        self.declare_parameter('reach', 0.74)
        self._reach = self.get_parameter('reach').value

        self._pts_real    : np.ndarray | None = None
        self._pts_preview : np.ndarray | None = None
        self._lock = threading.Lock()

        self.create_subscription(
            JointState, '/fk_points',             self._cb_real,    10)
        self.create_subscription(
            JointState, '/sim/fk_points_preview', self._cb_preview, 10)

        self.get_logger().info(
            f'Nodo simulacion listo. reach={self._reach:.2f} m')

    # ------------------------------------------------------------------

    def _cb_real(self, msg: JointState):
        with self._lock:
            self._pts_real = _parse_points(msg)

    def _cb_preview(self, msg: JointState):
        with self._lock:
            self._pts_preview = _parse_points(msg)

    # ------------------------------------------------------------------
    #  UI matplotlib — corre en el thread principal
    # ------------------------------------------------------------------

    def run_ui(self):
        """
        Construye la figura y ejecuta el loop de refresco.
        Debe llamarse desde el thread principal (requerido por Tk).
        """
        self._build_plot()
        try:
            while rclpy.ok():
                if not plt.fignum_exists(self._fig.number):
                    break
                self._update_plot()
                plt.pause(0.12)   # procesa eventos Tk y libera GIL
        except KeyboardInterrupt:
            pass
        finally:
            plt.close('all')

    def _build_plot(self):
        reach = self._reach

        self._fig = plt.figure(figsize=(6, 7), facecolor='#1a1a2e')
        self._fig.canvas.manager.set_window_title('Simulacion Brazo 6-DOF')
        gs = self._fig.add_gridspec(2, 2, hspace=0.35, wspace=0.3)

        self._ax3d  = self._fig.add_subplot(gs[0, :], projection='3d')
        self._ax_fr = self._fig.add_subplot(gs[1, 0])
        self._ax_tp = self._fig.add_subplot(gs[1, 1])

        # Vista 3D
        self._ax3d.set_facecolor('#0d1117')
        self._ax3d.set_xlim(-reach, reach)
        self._ax3d.set_ylim(-reach, reach)
        self._ax3d.set_zlim(0, reach * 1.2)
        self._ax3d.set_title('3D (real)', color='white', fontsize=9)
        self._ax3d.tick_params(colors='gray', labelsize=7)
        self._ax3d.set_xlabel('X', color='gray', fontsize=7)
        self._ax3d.set_ylabel('Y', color='gray', fontsize=7)
        self._ax3d.set_zlabel('Z', color='gray', fontsize=7)

        # Vistas 2D
        for ax, title, xlim, ylim in [
            (self._ax_fr, 'Frontal XZ', (-reach, reach), (0, reach * 1.2)),
            (self._ax_tp, 'Superior XY', (-reach, reach), (-reach, reach)),
        ]:
            ax.set_facecolor('#0d1117')
            ax.set_title(title, color='white', fontsize=9)
            ax.tick_params(colors='gray', labelsize=7)
            ax.grid(True, color='#333', lw=0.5)
            ax.set_xlim(*xlim)
            ax.set_ylim(*ylim)

        # Lineas del brazo real
        kw  = dict(color='#3498db', lw=3, ms=6, markerfacecolor='#e74c3c',
                   marker='o')
        kw2 = dict(color='#3498db', lw=2, ms=4, markerfacecolor='#e74c3c',
                   marker='o')
        self._ln3d, = self._ax3d.plot([], [], [], **kw)
        self._ln_fr, = self._ax_fr.plot([], [], **kw2)
        self._ln_tp, = self._ax_tp.plot([], [], **kw2)

        # Pinza (ultimo segmento muñeca→punta) en verde
        kwg = dict(color='#2ecc71', lw=5, ms=7, markerfacecolor='#2ecc71',
                   marker='o', solid_capstyle='round')
        self._ln3d_grip, = self._ax3d.plot([], [], [], **kwg)
        self._ln_fr_grip, = self._ax_fr.plot([], [], **dict(kwg, lw=4, ms=5))
        self._ln_tp_grip, = self._ax_tp.plot([], [], **dict(kwg, lw=4, ms=5))

        # Lineas de preview IK en naranja
        kwp  = dict(color='#e67e22', lw=2, ms=5, markerfacecolor='#f39c12',
                    marker='o', linestyle='--', alpha=0.7)
        self._ln3d_prev, = self._ax3d.plot([], [], [], **kwp)
        self._ln_fr_prev, = self._ax_fr.plot([], [], **kwp)
        self._ln_tp_prev, = self._ax_tp.plot([], [], **kwp)

    def _update_plot(self):
        with self._lock:
            pts     = self._pts_real    if self._pts_real    is not None else None
            pts_pre = self._pts_preview if self._pts_preview is not None else None

        if pts is not None:
            self._ln3d.set_data_3d(pts[:, 0], pts[:, 1], pts[:, 2])
            self._ax3d.view_init(elev=22, azim=-58)
            self._ln_fr.set_data(pts[:, 0], pts[:, 2])
            self._ln_tp.set_data(pts[:, 0], pts[:, 1])
            g = pts[-2:]   # pinza = ultimo segmento (muñeca → punta)
            self._ln3d_grip.set_data_3d(g[:, 0], g[:, 1], g[:, 2])
            self._ln_fr_grip.set_data(g[:, 0], g[:, 2])
            self._ln_tp_grip.set_data(g[:, 0], g[:, 1])

        if pts_pre is not None:
            self._ln3d_prev.set_data_3d(pts_pre[:, 0], pts_pre[:, 1], pts_pre[:, 2])
            self._ln_fr_prev.set_data(pts_pre[:, 0], pts_pre[:, 2])
            self._ln_tp_prev.set_data(pts_pre[:, 0], pts_pre[:, 1])


def main(args=None):
    rclpy.init(args=args)
    node = SimulacionNode()

    # ROS spin en thread daemon — no bloquea el thread principal
    ros_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True, name='sim-ros-spin')
    ros_thread.start()

    # Matplotlib necesita correr en el thread principal (Tk requirement)
    node.run_ui()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
