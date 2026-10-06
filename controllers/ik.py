"""Stationary top-down pick-and-lift baseline, not moving-target IK.

Only finger contact lifts the object: no weld, teleport, or attachment trick.
"""
import numpy as np
import mujoco


class PoseIK:
    def __init__(self, model):
        self.model = model
        self.work = mujoco.MjData(model)
        self.site = model.site('tcp').id
        self.joints = np.array([model.joint(f'joint{i}').id for i in range(1,8)])
        self.qpos = model.jnt_qposadr[self.joints]
        self.dofs = model.jnt_dofadr[self.joints]
        self.actuators = np.array([model.actuator(f'actuator{i}').id for i in range(1,8)])
        self.limits = model.jnt_range[self.joints]
        self.jp = np.zeros((3,model.nv))
        self.jr = np.zeros((3,model.nv))

    def solve(self, data, position, rotation, max_iterations=200):
        work = self.work
        work.qpos[:] = data.qpos
        position = np.asarray(position)
        for _ in range(max_iterations):
            mujoco.mj_forward(self.model,work)
            ep = position-work.site_xpos[self.site]
            current = work.site_xmat[self.site].reshape(3,3)
            quaternion = np.zeros(4)
            mujoco.mju_mat2Quat(quaternion,(rotation@current.T).ravel())
            if quaternion[0] < 0:
                quaternion *= -1
            er = np.zeros(3)
            mujoco.mju_quat2Vel(er,quaternion,1.)
            if np.linalg.norm(ep) < .0005 and np.linalg.norm(er) < .005:
                return work.qpos[self.qpos].copy()
            mujoco.mj_jacSite(self.model,work,self.jp,self.jr,self.site)
            J = np.vstack([self.jp[:,self.dofs],.25*self.jr[:,self.dofs]])
            error = np.r_[ep,.25*er]
            dq = J.T@np.linalg.solve(J@J.T+.015**2*np.eye(6),error)
            dq *= min(.8,.10/max(np.max(np.abs(dq)),1e-9))
            work.qpos[self.qpos] = np.clip(work.qpos[self.qpos]+dq,
                                           self.limits[:,0]+.001,self.limits[:,1]-.001)
        raise RuntimeError(f'IK did not converge: position error {np.linalg.norm(ep):.4f} m, '
                           f'orientation error {np.linalg.norm(er):.4f} rad')


class GraspController:
    DURATIONS = {'approach':2., 'descend':1.5, 'close':1., 'lift':2., 'hold':1.}

    def __init__(self, model, config, conveyor):
        self.model, self.config, self.conveyor = model, config['grasp'], conveyor
        self.ik = PoseIK(model)
        self.gripper = model.actuator('actuator8').id
        self.left = model.body('left_finger').id
        self.right = model.body('right_finger').id
        self.reset()

    def reset(self):
        self.phase = 'waiting' if self.config['enabled'] else 'disabled'
        self.phase_start = 0.
        self.target = None
        self.q_start = self.q_goal = None
        self.failure = None
        self.success_hold = 0.
        self.rotation = None

    def stops_conveyor(self, time):
        return self.config['enabled'] and time >= self.config['stop_time']

    def fingers_touching(self, data):
        bodies = set()
        for contact in data.contact:
            if contact.dist > 0:
                continue
            if contact.geom1 == self.conveyor.box_geom:
                bodies.add(self.model.geom_bodyid[contact.geom2])
            if contact.geom2 == self.conveyor.box_geom:
                bodies.add(self.model.geom_bodyid[contact.geom1])
        return self.left in bodies and self.right in bodies

    def _enter(self, phase, data):
        self.phase, self.phase_start = phase, data.time
        if phase in {'approach','descend','lift'}:
            point = self.target.copy()
            if phase == 'approach':
                point[2] += self.config['approach_height']
            elif phase == 'lift':
                point[2] += self.config['lift_height']
            self.q_start = data.qpos[self.ik.qpos].copy()
            self.q_goal = self.ik.solve(data,point,self.rotation)

    def _fail(self, message):
        self.phase, self.failure = 'failed', message

    def update(self, data):
        # Feed-forward gravity/bias compensation on the arm only.
        data.qfrc_applied[self.ik.dofs] = data.qfrc_bias[self.ik.dofs]
        if self.phase in {'disabled','failed','success'}:
            return
        if self.phase == 'waiting':
            data.ctrl[self.gripper] = 255
            ready_time = self.config['stop_time']+self.config['settle_time']
            if data.time < ready_time:
                return
            velocity = data.qvel[self.conveyor.dof:self.conveyor.dof+3]
            if np.linalg.norm(velocity) > .01 or not self.conveyor.is_touching(data):
                if data.time > ready_time+3.:
                    self._fail('Box did not settle on the belt before grasping')
                return
            self.target = self.conveyor.box_position(data)
            # Align closing direction with the box's local Y axis.
            box_rotation = data.xmat[self.conveyor.box_id].reshape(3,3)
            yaw = np.arctan2(box_rotation[1,0],box_rotation[0,0])
            c,s = np.cos(yaw),np.sin(yaw)
            self.rotation = np.array([[c,s,0],[s,-c,0],[0,0,-1.]])
            try:
                self._enter('approach',data)
            except RuntimeError as exc:
                self._fail(str(exc))
            return
        elapsed = data.time-self.phase_start
        duration = self.DURATIONS[self.phase]
        if self.phase in {'approach','descend','lift'}:
            fraction = np.clip(elapsed/duration,0,1)
            smooth = fraction*fraction*(3-2*fraction)
            data.ctrl[self.ik.actuators] = self.q_start+smooth*(self.q_goal-self.q_start)
        data.ctrl[self.gripper] = 0 if self.phase in {'close','lift','hold'} else 255
        if self.phase == 'hold':
            lifted = self.conveyor.box_position(data)[2] > self.target[2]+.10
            if lifted and self.fingers_touching(data):
                self.success_hold += self.model.opt.timestep
            else:
                self.success_hold = 0
            if self.success_hold >= .5:
                self.phase = 'success'
                return
        if elapsed < duration:
            return
        next_phase = {'approach':'descend','descend':'close','close':'lift','lift':'hold'}.get(self.phase)
        if self.phase == 'close' and not self.fingers_touching(data):
            self._fail('Closing finished without contact on both fingers')
            return
        if next_phase is None:
            self._fail('Box was not held 10 cm above its start for 0.5 seconds')
            return
        try:
            self._enter(next_phase,data)
        except RuntimeError as exc:
            self._fail(str(exc))
