"""Integration regressions with fake transport, without importing ROS or motors."""
import ast
from concurrent.futures import Future
from dataclasses import replace
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
import unittest
import numpy as np

import test_executor_logic as executor_tests
from test_executor_logic import Controller, namespace
from rescue_command_station.arm.configuration import configured_arm
from rescue_command_station.arm.kinematics import Arm6DOF
from rescue_command_station.arm.motion import plan_segment
from rescue_command_station.arm.web_view import scene_payload

ARM = Path(__file__).parents[1] / 'rescue_command_station/arm'


def load_class(filename, name, **symbols):
    source = (ARM / filename).read_text(encoding='utf-8')
    cls = next(n for n in ast.parse(source).body if isinstance(n, ast.ClassDef) and n.name == name)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), filename, 'exec'), symbols)
    return symbols[name]


class StationIntegrationTests(unittest.TestCase):
    node = executor_tests.ExecutorTests.node

    def test_service_duration_is_total_per_segment(self):
        node = self.node()
        T = node.arm.fk(node.q)['T06']
        def waypoint(duration, n):
            return NS(x=T[0,3], y=T[1,3], z=T[2,3], r=T[:3,:3].flatten().tolist(), duration=duration, n_steps=n)
        req = NS(waypoints=[waypoint(0, 1), waypoint(.4, 12), waypoint(.6, 17)],
                 vel_pct=20, path_tol_pos=0, path_tol_rot=0, goal_tol_pos=.01, goal_tol_rot=.1)
        received = []
        node._execute = lambda plan, *args: received.append((plan, args[-1]))
        response = node._trajectory(req, NS())
        self.assertTrue(response.success, response.mensaje)
        node.worker.join(2)
        self.assertEqual(len(received), 1)
        self.assertAlmostEqual(received[0][0].t[-1], 1.0)
        np.testing.assert_allclose(received[0][1], [.4, 1.0])
        self.assertEqual(node.sent, [])

    def test_manual_and_second_request_rejected_during_preflight_then_cancel(self):
        node = self.node()
        T = node.arm.fk(node.q)['T06']
        waypoints = [(T[:3,3], T[:3,:3], 1), (T[:3,3], T[:3,:3], 10)]
        entered, release = threading.Event(), threading.Event()
        original = namespace['plan_waypoints']
        errors = []
        def slow(*args):
            entered.set()
            release.wait(2)
            return original(*args)
        def start():
            try:
                node._start(node.q, waypoints, .02, 20)
            except ValueError as exc:
                errors.append(str(exc))
        namespace['plan_waypoints'] = slow
        thread = threading.Thread(target=start)
        try:
            thread.start()
            self.assertTrue(entered.wait(1))
            self.assertTrue(node.state.in_progress)
            node._manual(NS())  # Busy rejection happens before reading any command.
            with self.assertRaises(ValueError):
                node._start(node.q, waypoints, .02, 20)
            node._cancel(None, NS())
            release.set(); thread.join(3)
            self.assertFalse(thread.is_alive())
            self.assertTrue(errors)
            self.assertFalse(node.state.in_progress)
            self.assertIsNone(node.worker)
            self.assertEqual(node.sent, [])
            self.assertIn('durante planificacion', node.errors[0])
        finally:
            release.set(); thread.join(3)
            namespace['plan_waypoints'] = original

    def test_manual_reorders_driver_angles_and_rejects_bad_commands(self):
        node = self.node()
        expected = node.q * node.signs
        msg = NS(name=list(reversed(node.names)), position=list(reversed(expected)), velocity=[20.] * 6)
        node._manual(msg)
        np.testing.assert_allclose(node.sent[0], expected)
        msg.position[2] = float('nan')
        node._manual(msg)
        self.assertEqual(len(node.sent), 1)
        self.assertTrue(node.errors)
        node.seen[0] = 0
        msg.position = list(reversed(expected))
        node._manual(msg)
        self.assertEqual(len(node.sent), 1)

    def test_loss_of_feedback_or_bus_stops_even_when_paused(self):
        for stale in ('joints', 'bus'):
            node = self.node(); node.speed = 0
            plan = plan_segment(node.arm, node.q, node.arm.fk(node.q)['T06'], .2)
            if stale == 'joints':
                node.seen[4] = 0
            else:
                node.bus_ready['ex'] = (True, 0)
            node._execute(plan, 20, 0, 0, .01, .1, np.array([.2]))
            self.assertFalse(node.state.in_progress)
            self.assertEqual(node.sent, [])
            self.assertTrue(node.errors)

    def test_cancel_prevents_publish_and_disconnect_latches_cancel(self):
        node = self.node(); node.cancel.set()
        # Uses the actual publisher path, not the fake tracking publisher.
        Controller._publish(node, node.q, 20)
        self.assertEqual(node.sent, [])
        node.cancel.clear()
        node._status('ax', NS(conectado=False, emergencia=False, modo_calib=False))
        self.assertTrue(node.cancel.is_set())

    def test_web_payload_retains_active_tcp_and_geometric_flange(self):
        arm, signs = configured_arm()
        arm = Arm6DOF(replace(arm.p, control_frame='camera', e56=.04, tool_length=.1,
                             camera_rpy=(.2, -.4, .3)), arm.limits)
        driver_q = np.radians([20, -50, 40, 10, 25, 15])
        data = scene_payload(arm, driver_q * signs, driver_q * signs + .01)
        f = arm.fk(driver_q * signs)
        np.testing.assert_allclose(data['fk']['T'], f['camera'])
        np.testing.assert_allclose(data['fk']['flange']['T'], f['flange'])
        np.testing.assert_allclose(data['fk']['points'][-1], f['tool'][:3,3])
        np.testing.assert_allclose(data['fk']['axes'], f['axes'])
        np.testing.assert_allclose(data['preview']['T'], arm.fk(driver_q * signs + .01)['T06'])

    def test_fk_waits_for_all_six_recent_joints(self):
        cls = load_class('cinematica_node.py', 'CinematicaNode', Node=object, JointState=NS, np=np, time=time)
        node = object.__new__(cls)
        node._q_actual = np.zeros(6); node._seen = np.zeros(6)
        node._lock = threading.RLock()
        node._joint_map = {str(i): i for i in range(6)}
        calls = []; node._publicar_fk = lambda: calls.append(True)
        node._cb_joint_states(NS(name=['0','1','2','3','4'], position=[0.] * 5))
        self.assertEqual(calls, [])
        node._cb_joint_states(NS(name=['5'], position=[0.]))
        self.assertEqual(calls, [True])
        node._seen[0] = 0
        node._cb_joint_states(NS(name=['5'], position=[float('nan')]))
        self.assertEqual(calls, [True])

    def test_stabilization_can_seed_before_any_teleop_request(self):
        cls = load_class('gui_node.py', 'App', ctk=NS(CTk=object), GUINode=object,
                         np=np, quaternion_to_matrix=lambda q: np.eye(3), COL={'ok':'ok','warn':'warn'})
        app = object.__new__(cls)
        app._node = NS(pose_actual=NS(pose=NS(position=NS(x=.1,y=.2,z=.3),orientation=NS())))
        app.var_stab = NS(get=lambda: True)
        app._stab_R = np.eye(3); app._tp_R = None
        app._update_tp_label = lambda: None
        app.lbl_tp_msg = NS(configure=lambda **kw: None)
        self.assertTrue(app._teleop_sync())
        np.testing.assert_allclose(app._tp_R, np.eye(3))

    def test_teleop_releases_pending_on_service_failure(self):
        cls = load_class('gui_node.py', 'App', ctk=NS(CTk=object), GUINode=object,
                         np=np, COL={'warn':'warn','ok':'ok','err':'err'})
        app = object.__new__(cls)
        app.var_stab = NS(get=lambda: False)
        app.lbl_tp_msg = NS(configure=lambda **kw: None)
        app._vel = lambda: 20
        app._node = NS(pedir_cartesian_goto=lambda *args: args[-1](False, 'unavailable'))
        app._teleop_send(np.zeros(3), np.eye(3))
        self.assertIn('unavailable', app._pending_teleop_msg[0])
        self.assertFalse(hasattr(app, '_pending_teleop_commit'))

    def test_service_timeout_cancels_once_and_releases_client(self):
        clock = [0.0]
        cls = load_class('gui_node.py', 'GUINode', Node=object, JointState=NS,
                         ArmStatus=NS, PoseStamped=NS, CartesianState=NS,
                         np=np, threading=threading, time=NS(monotonic=lambda: clock[0]))
        node = object.__new__(cls)
        node._lock = threading.Lock(); node._motion_pending = False
        ticks, cancelled, replies = [], [], []
        node.create_timer = lambda interval, fn: ticks.append(fn) or 'timer'
        node.destroy_timer = lambda timer: None
        node._call_trigger = cancelled.append
        future = Future()
        node._motion_request(NS(call_async=lambda req: future), NS(),
                             lambda f: replies.append('done'), lambda *args: replies.append(args))
        clock[0] = 61
        ticks[0](); ticks[0]()
        self.assertTrue(future.cancelled())
        self.assertEqual(cancelled, ['cartesian_cancel'])
        self.assertEqual(len(replies), 1)
        self.assertFalse(replies[0][0])
        self.assertFalse(node._motion_pending)


if __name__ == '__main__':
    unittest.main()
