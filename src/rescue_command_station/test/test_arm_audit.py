"""Regressions for the arm control audit; transport is fake, hardware untouched."""
import ast
from concurrent.futures import Future
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np
import yaml
from rescue_command_station.arm import configuration as config
from rescue_command_station.arm.kinematics import rotx, roty, rotz
import test_executor_logic as executor_tests

sys.path.insert(0, str(Path(__file__).parents[1] / 'tools'))
from verify_arm_gui import load_gui


class AuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gui = load_gui()

    def test_reversed_limits_rejected_instead_of_silently_sorted(self):
        cfg = config.settings(); cfg['joint_min'] = [1.] * 6; cfg['joint_max'] = [-1.] * 6
        with patch.object(config, 'settings', return_value=cfg):
            with self.assertRaisesRegex(ValueError, 'min < max'):
                config.configured_arm()

    def test_driver_map_uses_enabled_names_and_rejects_collision(self):
        data = yaml.safe_load(config.config_path('rescue_robot_core', 'servos.yaml').read_text(encoding='utf-8'))
        params = data['ax12a_driver']['ros__parameters']
        params['servos']['Disabled'] = {'id': 16}
        cfg = config.settings()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'servos.yaml'
            path.write_text(yaml.safe_dump(data), encoding='utf-8')
            with patch.object(config, 'config_path', return_value=path), patch.object(config, 'settings', return_value=cfg):
                self.assertEqual(set(config.joint_drivers()), set(cfg['joint_order']))
                params['servo_names'].append('Disabled')
                path.write_text(yaml.safe_dump(data), encoding='utf-8')
                with self.assertRaisesRegex(ValueError, 'duplicado'):
                    config.joint_drivers()

    def test_copy_full_pose_preserves_orientation(self):
        arm, _ = config.configured_arm()
        # This pose differed by ~180 degrees through the retired beta/q5/q6 path.
        q = np.radians([20, 50, -40, -10, 25, 15])
        R = arm.fk(q)['T06'][:3, :3]
        r, p, y = np.radians(self.gui['rot_to_rpy'](R))
        # Same convention and precision as the actual six GUI fields.
        r, p, y = np.radians([float(f'{v:.7f}') for v in np.degrees([r, p, y])])
        np.testing.assert_allclose((rotz(y) @ roty(p) @ rotx(r))[:3, :3], R, atol=2e-9)

    def test_preview_never_sends_a_motor_command(self):
        app = object.__new__(self.gui['App'])
        preview = []
        app._node = NS(mostrar_preview=lambda q: preview.append(q))
        app._on_ik_result(True, [.1] * 6, 'OK')
        self.assertEqual(preview, [[.1] * 6])
        self.assertNotIn('_pending_ik_q', vars(app))

    def test_maintenance_latches_and_rejects_all_station_publication(self):
        node = executor_tests.ExecutorTests.node(self)
        res = node._maintenance(NS(data=True), NS())
        self.assertTrue(res.success); self.assertTrue(node.cancel.is_set())
        node.cancel.clear()  # The latch must also protect paths that clear cancellation.
        with self.assertRaisesRegex(ValueError, 'mantenimiento'):
            node._check_buses()
        node._manual(NS(name=node.names, position=list(node.q * node.signs), velocity=[20.] * 6))
        self.assertEqual(node.sent, [])
        node.state.in_progress = True
        self.assertFalse(node._maintenance(NS(data=False), NS()).success)
        node.state.in_progress = False
        self.assertTrue(node._maintenance(NS(data=False), NS()).success)
        node._check_buses()

    def test_gui_calibration_acquires_before_drivers_and_releases_after_both(self):
        node = self.gui['GUINode']()
        node.calib_start()
        self.assertEqual([t for t, _ in node.sent], [
            '/cartesian/maintenance', '/ax12a/calibrate_start', '/ex106/calibrate_start'])
        self.assertTrue(node._maintenance_held)
        self.assertFalse(node.motion_ready)
        status = NS(conectado=True, emergencia=False, modo_calib=True, mensaje='')
        node._cb_ax_status(status); node._cb_ex_status(status)
        node._cartesian_seen = time.monotonic()
        self.assertTrue(node.maintenance_ready)
        node.calib_confirm()
        self.assertEqual([t for t, _ in node.sent[-3:]], [
            '/ax12a/calibrate_confirm', '/ex106/calibrate_confirm', '/cartesian/maintenance'])
        self.assertFalse(node._maintenance_held)
        self.assertFalse(node.sent[-1][1].data)

    def test_partial_calibration_does_not_release_latch(self):
        node = self.gui['GUINode'](); node.calib_start()
        status = NS(conectado=True, emergencia=False, modo_calib=True, mensaje='')
        node._cb_ax_status(status); node._cb_ex_status(status)
        node._cartesian_seen = time.monotonic()
        future = Future(); future.set_result(NS(success=False))
        node._cli['ex_cal_ok'].call_async = lambda req: future
        node.calib_confirm()
        self.assertTrue(node._maintenance_held)
        self.assertFalse(node._maintenance_pending)
        self.assertEqual(sum(t == '/cartesian/maintenance' for t, _ in node.sent), 1)

    def test_stale_joystick_does_not_start_another_segment(self):
        app = object.__new__(self.gui['App'])
        app._node = NS(_joy_stamp=0.)
        app._set_joy_indicator = lambda active: None
        app._joystick_teleop_step()  # Any access to planning would fail on this stub.

    def test_failed_rotation_does_not_change_stabilization(self):
        app = object.__new__(self.gui['App'])
        app.var_stab = NS(get=lambda: True)
        app._stab_R = np.eye(3); app._tp_R = np.eye(3)
        app.lbl_tp_msg = NS(configure=lambda **kw: None); app._vel = lambda: 20
        app._node = NS(pedir_cartesian_goto=lambda *args: args[-1](False, 'rejected'))
        app._teleop_send(np.zeros(3), rotz(.5)[:3, :3])
        np.testing.assert_allclose(app._stab_R, np.eye(3))
        self.assertNotIn('_pending_teleop_commit', vars(app))

    def test_simulator_feedback_matches_signed_real_encoder(self):
        path = Path(__file__).parents[2] / 'rescue_robot_core/rescue_robot_core/nodes/dynamixel_sim_node.py'
        cls = next(n for n in ast.parse(path.read_text(encoding='utf-8')).body if isinstance(n, ast.ClassDef))
        scope = dict(Node=object, np=np, time=time,
                     JointState=lambda: NS(header=NS(), name=[], position=[]))
        exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), scope)
        node = object.__new__(scope['SimDriverNode'])
        node._shutdown_event = threading.Event(); node._lock = threading.RLock()
        node._running = node._conectado = True; node._emergencia = False
        node._pos = {'joint': 350.}; node._target = {'joint': None}; node._rate_s = .001
        node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: NS()))
        received = []
        def publish(msg):
            received.append(msg); node._shutdown_event.set()
        node._pub_js = NS(publish=publish)
        node._bucle_control()
        np.testing.assert_allclose(received[0].position, np.radians([-10.]))


if __name__ == '__main__':
    unittest.main()
