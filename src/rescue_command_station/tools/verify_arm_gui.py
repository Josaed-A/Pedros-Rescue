"""Build the actual Tk GUI with fake ROS transport (no drivers, no ROS imports).

Run with the station on PYTHONPATH. This checks widget/callback wiring only;
generated interfaces, DDS and hardware still require the ROS validation list.
"""
import ast
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS
import time
import numpy as np
from rescue_command_station.arm import configuration


class FakeNode:
    def __init__(self, name):
        self.parameters = {}
        self.sent = []

    def declare_parameter(self, name, default, **kwargs):
        self.parameters[name] = default

    def get_parameter(self, name):
        return NS(value=self.parameters[name])

    def create_publisher(self, cls, topic, *args, **kwargs):
        return NS(publish=lambda msg: self.sent.append((topic, msg)))

    def create_subscription(self, *args, **kwargs):
        return NS()

    def create_client(self, cls, topic):
        def call(req):
            f = Future(); f.set_result(NS(success=True, message='fake', mensaje='fake'))
            self.sent.append((topic, req))
            return f
        return NS(service_is_ready=lambda: True, call_async=call)

    def create_timer(self, *args, **kwargs):
        return NS()

    def destroy_timer(self, timer):
        pass

    def get_logger(self):
        return NS(info=lambda *a: None, warn=lambda *a: None, error=lambda *a: None)

    def get_clock(self):
        return NS(now=lambda: NS(to_msg=lambda: NS(sec=0, nanosec=0)))


def load_gui():
    path = Path(__file__).parents[1] / 'rescue_command_station/arm/gui_node.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    excluded = ('rclpy', 'sensor_msgs', 'geometry_msgs', 'std_srvs', 'std_msgs',
                'rescue_interfaces', 'rescue_command_station.control', 'rescue_command_station.vision')
    body = []
    for n in tree.body:
        if isinstance(n, ast.Import) and any(a.name.startswith(excluded) for a in n.names):
            continue
        if isinstance(n, ast.ImportFrom) and ((n.module or '').startswith(excluded) or n.module == 'configuration'):
            continue
        body.append(n)
    scope = dict(__name__='rescue_command_station.arm.gui_audit', __package__='rescue_command_station.arm',
                 Node=FakeNode, QrDetector=lambda: NS(), QoSProfile=lambda **kw: NS(),
                 QoSHistoryPolicy=NS(KEEP_LAST=1), QoSReliabilityPolicy=NS(BEST_EFFORT=1),
                 configured_arm=lambda node=None: configuration.configured_arm(),
                 settings=lambda node=None: configuration.settings(),
                 joint_drivers=lambda node=None: configuration.joint_drivers(),
                 cfg=NS(BUTTON_R1=5, BUTTON_L1=4, ARM_DEADZONE=.15))
    for name in ('JointState', 'Joy', 'CompressedImage', 'PoseStamped', 'Bool', 'Float64',
                 'String', 'ArmStatus', 'CartesianState', 'CartesianWaypoint'):
        scope[name] = NS
    for name in ('Trigger', 'SetBool', 'ComputeIKPose', 'ServoCommand', 'ServoStatus',
                 'CartesianGoto', 'CartesianTrajectory'):
        scope[name] = NS(Request=NS)
    exec(compile(ast.Module(body=body, type_ignores=[]), str(path), 'exec'), scope)
    return scope


def main():
    gui = load_gui()
    node = gui['GUINode']()
    assert node._req_active, 'Standalone GUI must be visible without dashboard'
    app = gui['App'](node)
    # Manual UI ticks below must not create multiple perpetual timer chains.
    for timer in app.tk.call('after', 'info'):
        if '_loop_ui' in str(app.tk.call('after', 'info', timer)):
            app.after_cancel(timer)
    app.after = lambda *args, **kwargs: None
    errors = []
    app.report_callback_exception = lambda *e: errors.append(e)
    app.deiconify = lambda: None  # Keep this verification window hidden.
    try:
        for tab in app._tab_order:
            app.tabs.set(tab)
            app._loop_ui()
        q = np.radians([20, -50, 40, 10, 25, 15])
        node._cb_joint_states(NS(name=node.joint_order, position=q.tolist()))
        pose = NS(pose=NS(position=NS(x=.1, y=.2, z=.3), orientation=NS(x=0., y=0., z=0., w=1.)))
        node._cb_pose(pose)
        app._copy_current_pose_to_ik()
        p, R = app._read_pose_target()
        np.testing.assert_allclose(p, [.1, .2, .3]); np.testing.assert_allclose(R, np.eye(3))
        app._on_status_result([])
        app._loop_ui()
        # Incomplete/stale buses must still block moves, even with fresh pose.
        assert not node.motion_ready
        assert not node.maintenance_ready
        deadline = time.monotonic() + .4
        while time.monotonic() < deadline:
            app.update(); time.sleep(.01)
        assert not errors, errors
        assert not any(t.endswith('/joint_cmd') for t, _ in node.sent)
        print('GUI widgets, all tabs, callbacks and offline startup: OK (fake ROS)')
    finally:
        app._arm_canvas.close()
        app._arm_canvas._worker.join(timeout=8)
        app.destroy()


if __name__ == '__main__':
    main()
