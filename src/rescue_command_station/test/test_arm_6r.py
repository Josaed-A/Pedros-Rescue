"""Run without ROS: PYTHONPATH=src/rescue_command_station python -m unittest discover -s src/rescue_command_station/test -v"""
import unittest
from dataclasses import replace
import numpy as np
from rescue_command_station.arm.configuration import configured_arm, joint_drivers, settings
from rescue_command_station.arm.kinematics import Arm6DOF, make_T, rotx, roty, rotz
from rescue_command_station.arm.motion import plan_segment, plan_waypoints
from rescue_command_station.arm.cartesian_controller import slerp


def independent_fk(q, p):
    # Independent homogeneous products (not model FK or its helpers).
    def rotate(axis, angle):
        a=np.eye(3)[axis]; x,y,z=a
        K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        R=np.eye(4); R[:3,:3]=np.eye(3)+np.sin(angle)*K+(1-np.cos(angle))*(K@K)
        return R
    def move(x,y,z):
        T=np.eye(4); T[:3,3]=[x,y,z]; return T
    T=rotate(2,q[0])@move(0,0,p.base_height)
    for angle,L in zip(q[1:4],(p.L1,p.L2,p.L3)): T=T@rotate(1,-angle)@move(0,0,L)
    T=T@rotate(2,q[4])@move(p.e56,0,0)@rotate(0,q[5])
    tool=T@move(p.tool_length,0,0)
    r,pitch,y=p.camera_rpy
    camera=T@move(*p.camera_xyz)@rotate(2,y)@rotate(1,pitch)@rotate(0,r)
    return tool,camera


class ArmTests(unittest.TestCase):
    def setUp(self):
        self.arm,_=configured_arm()
        self.q=np.radians([20,-50,40,10,25,15])

    def test_independent_fk_and_inverse_1000(self):
        rng=np.random.default_rng(543)
        for i in range(1000):
            p=replace(self.arm.p, control_frame='camera' if i%2 else 'tool',
                      e56=rng.uniform(0,.08), camera_xyz=rng.uniform(-.08,.08,3),
                      camera_rpy=rng.uniform(-2,2,3))
            a=Arm6DOF(p,self.arm.limits)
            q=rng.uniform(a.limits[:,0]+.02,a.limits[:,1]-.02)
            f=a.fk(q); tool,camera=independent_fk(q,p)
            np.testing.assert_allclose(f['tool'],tool,atol=1e-12)
            np.testing.assert_allclose(f['camera'],camera,atol=1e-12)
            target=f['T06']; result=a.ik(target,q_init=q+rng.normal(0,.05,6))
            np.testing.assert_allclose(a.fk(result)['T06'],target,atol=1e-6)
            self.assertTrue(np.all(result>=a.limits[:,0]-1e-10))
            self.assertTrue(np.all(result<=a.limits[:,1]+1e-10))

    def test_zero_axes_and_camera_orbit(self):
        f=self.arm.fk(np.zeros(6)); q=np.zeros(6); q[5]=.7; g=self.arm.fk(q)
        np.testing.assert_allclose(f['axes'][4],[0,0,1])
        np.testing.assert_allclose(f['axes'][5],[1,0,0])
        np.testing.assert_allclose(f['tool'][:3,3],g['tool'][:3,3])
        self.assertGreater(np.linalg.norm(f['camera'][:3,3]-g['camera'][:3,3]),.01)

    def test_jacobian_and_frames(self):
        for active in ['tool','camera']:
            a=Arm6DOF(replace(self.arm.p,control_frame=active,camera_rpy=(.4,.2,-.3)),self.arm.limits)
            f=a.fk(self.q); J=a.jacobian(self.q)
            for i in range(6):
                q=self.q.copy(); q[i]+=1e-7
                dp=(a.fk(q)['T06'][:3,3]-f['T06'][:3,3])/1e-7
                np.testing.assert_allclose(J[:3,i],dp,atol=1e-7)
            for frame in ['tool','camera','base']:
                delta=np.array([.01,-.02,.03]); target=a.relative_target(self.q,delta,frame)
                basis=np.eye(3) if frame=='base' else f[frame][:3,:3]
                np.testing.assert_allclose(target[:3,3],f['T06'][:3,3]+basis@delta)
                np.testing.assert_allclose(a.relative_basis(f['T06'][:3,:3],frame),basis,atol=1e-12)

    def test_singular_inverse(self):
        for q in [np.zeros(6),np.array([.4,0,0,0,.3,.2]),
                  np.array([.2,-.6,.8,-.2,np.pi/2,.4]),
                  np.array([.2,-.6,.8,-.2,-np.pi/2,.4])]:
            T=self.arm.fk(q)['T06']; seed=q.copy(); seed[5]+=.01
            solved=self.arm.ik(T,q_init=seed)
            np.testing.assert_allclose(self.arm.fk(solved)['T06'],T,atol=1e-6)

    def test_invalid_inputs(self):
        for target in [make_T(np.eye(3),[4,0,0]),np.full((4,4),np.nan),make_T(np.zeros((3,3)),[0,0,0])]:
            with self.assertRaises(ValueError): self.arm.ik(target)
        with self.assertRaises(ValueError): configured_arm(overrides={'L1':0})
        with self.assertRaises(ValueError): configured_arm(overrides={'camera_xyz':[1,2]})

    def test_slerp_pi(self):
        R=rotx(np.pi)[:3,:3]
        np.testing.assert_allclose(slerp(np.eye(3),R,.5),rotx(np.pi/2)[:3,:3],atol=1e-8)

    def test_cartesian_paths_and_dense_playback(self):
        for active in ['tool','camera']:
            a=Arm6DOF(replace(self.arm.p,control_frame=active,camera_rpy=(.2,-.3,.4)),self.arm.limits)
            for frame in ['base','tool','camera']:
                T0=a.fk(self.q)['T06']; T1=a.relative_target(self.q,[.002,-.002,-.003],frame)
                plan=plan_segment(a,self.q,T1,duration=1)
                for t in np.linspace(0,1,301):
                    u=10*t**3-15*t**4+6*t**5
                    T=a.fk(plan.sample(t))['T06']
                    np.testing.assert_allclose(T[:3,3],T0[:3,3]+u*(T1[:3,3]-T0[:3,3]),atol=5e-5)
                    np.testing.assert_allclose(T[:3,:3],T0[:3,:3],atol=5e-4)
                np.testing.assert_allclose(a.fk(plan.q[-1])['T06'],T1,atol=1e-6)

    def test_joint_and_waypoints(self):
        T0=self.arm.fk(self.q)['T06']; T1=self.arm.relative_target(self.q,[0,0,-.01],'base')
        joint=plan_segment(self.arm,self.q,T1,kind='joint')
        np.testing.assert_allclose(self.arm.fk(joint.q[-1])['T06'],T1,atol=1e-6)
        plan=plan_waypoints(self.arm,self.q,[(T0[:3,3],T0[:3,:3],1),(T1[:3,3],T1[:3,:3],20),(T0[:3,3],T0[:3,:3],20)])
        self.assertTrue(np.all(np.diff(plan.t)>0))
        np.testing.assert_allclose(self.arm.fk(plan.q[-1])['T06'],T0,atol=1e-6)

    def test_vertical_unreachable_translation_rejected(self):
        q=np.zeros(6); T=self.arm.relative_target(q,[.01,0,0],'base')
        with self.assertRaises(ValueError): plan_segment(self.arm,q,T)
        for duration in [np.nan,-1,121]:
            with self.assertRaises(ValueError): plan_segment(self.arm,self.q,self.arm.fk(self.q)['T06'],duration)

    def test_driver_source(self):
        self.assertEqual(joint_drivers()['Munieca_R'],('ax',6))
        a,signs=configured_arm(); q=self.q*signs
        np.testing.assert_allclose(q*signs,self.q)
        self.assertEqual(settings()['joint_order'][0],'Base')


if __name__=='__main__': unittest.main()
