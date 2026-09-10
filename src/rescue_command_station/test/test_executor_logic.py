"""Adapter logic with fake ROS transport; not a ROS graph/hardware test."""
import ast
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
import numpy as np
from rescue_command_station.arm.configuration import configured_arm, settings, joint_drivers
from rescue_command_station.arm.motion import plan_segment, plan_waypoints, Plan
from rescue_command_station.arm.cartesian_controller import within_tolerance


source=Path(__file__).parents[1]/'rescue_command_station/arm/cartesian_node.py'
tree=ast.parse(source.read_text())
cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='CartesianNode')
namespace=dict(Node=object,np=np,time=time,threading=threading,
               plan_waypoints=plan_waypoints,Plan=Plan,within_tolerance=within_tolerance)
exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),namespace)
Controller=namespace['CartesianNode']


class ExecutorTests(unittest.TestCase):
    def node(self):
        n=object.__new__(Controller); n.arm,n.signs=configured_arm()
        n.names=settings()['joint_order']; n.drivers=joint_drivers()
        n.q=np.radians([20,-50,40,10,25,15]); n.seen=np.full(6,time.monotonic())
        n.lock=threading.Lock(); n.cancel=threading.Event(); n.worker=None; n.speed=100
        n.command_lock=threading.RLock()
        n.maintenance=False
        n.bus_ready={'ax':(True,time.monotonic()),'ex':(True,time.monotonic())}
        n.state=SimpleNamespace(in_progress=False); n.errors=[]; n.sent=[]
        n.get_logger=lambda:SimpleNamespace(error=n.errors.append)
        def publish(q,velocity):
            n.sent.append(q.copy()); n.q=q*n.signs; n.seen[:]=time.monotonic()
        n._publish=publish
        return n

    def test_reject_without_any_publish(self):
        n=self.node(); T=n.arm.fk(n.q)['T06']
        with self.assertRaises(ValueError):
            n._start(n.q,[(T[:3,3],T[:3,:3],1),([4,0,0],np.eye(3),20)],.05,20)
        self.assertEqual(n.sent,[]); self.assertIsNone(n.worker)

    def test_stale_feedback(self):
        n=self.node(); n.seen[3]=0
        with self.assertRaises(ValueError): n._current()

    def test_disconnected_or_emergency_blocks(self):
        n=self.node(); n._status('ax',SimpleNamespace(emergencia=True,modo_calib=False,conectado=True))
        self.assertTrue(n.cancel.is_set())
        with self.assertRaises(ValueError): n._check_buses()

    def test_execution_uses_servo_sign_mapping(self):
        n=self.node(); target=n.arm.relative_target(n.q,[0,0,-.002],'base')
        plan=plan_segment(n.arm,n.q,target,.2)
        # This test supplies perfect fake feedback; planning can exceed one
        # second on a loaded CI host, so deliver a fresh sample before playback.
        n.seen[:]=time.monotonic()
        n.bus_ready={d:(True,time.monotonic()) for d in ('ax','ex')}
        n._execute(plan,20,0,0,.001,.001,np.array([.2]))
        self.assertFalse(n.errors); self.assertGreater(len(n.sent),1)
        np.testing.assert_allclose(n.sent[-1],plan.q[-1]*n.signs)
        self.assertFalse(n.state.in_progress)

    def test_zero_speed_and_cancel_publish_nothing(self):
        n=self.node(); n.speed=0
        plan=plan_segment(n.arm,n.q,n.arm.fk(n.q)['T06'],.2)
        thread=threading.Thread(target=n._execute,args=(plan,20,0,0,.01,.1,np.array([.2])))
        thread.start(); threading.Event().wait(.05); n.cancel.set(); thread.join(1)
        self.assertFalse(thread.is_alive()); self.assertEqual(n.sent,[])


if __name__=='__main__': unittest.main()
