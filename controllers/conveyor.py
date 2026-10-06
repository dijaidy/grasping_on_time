"""Contact-gated transport approximation, not a mechanically moving belt.

Positive speed follows +Y on straight belts and counterclockwise on loops.
Transport follows the local tangent wherever the box contacts the belt;
there is no attraction to the centerline. The path projection is geometry
used to determine direction, not a position target for the box.
The fixed surface keeps its native friction. Force feedback has finite tracking
error and is intended as a baseline for grasp/control experiments.
"""
import numpy as np
import mujoco


def target_speed(config, elapsed):
    speeds = config['speed']
    motion = config['motion']
    if motion == 'constant':
        return float(speeds[0])
    knots = config['milestone']
    if motion == 'discrete':
        index = np.clip(np.searchsorted(knots, elapsed, side='right') - 1, 0, len(speeds)-1)
        return float(speeds[index])
    return float(np.interp(elapsed, knots, speeds))


def straight_path(point, parameters):
    return np.array([0., point[1]]), np.array([0., 1.])


def circular_path(point, parameters):
    radius = parameters[0]
    norm = np.linalg.norm(point)
    normal = point/norm if norm > 1e-9 else np.array([1., 0.])
    return radius*normal, np.array([-normal[1], normal[0]])


def rounded_square_path(point, parameters):
    """Closest point and CCW tangent on four lines plus four quarter circles."""
    a, r = parameters[:2]
    c = a-r
    candidates = []
    for start,end in [([a,-c],[a,c]), ([c,a],[-c,a]),
                      ([-a,c],[-a,-c]), ([-c,-a],[c,-a])]:
        start,end = np.array(start),np.array(end)
        direction = end-start
        u = np.clip(np.dot(point-start,direction)/np.dot(direction,direction),0,1)
        candidates.append((start+u*direction,direction/np.linalg.norm(direction)))
    for center,begin in [(np.array([c,c]),0), (np.array([-c,c]),np.pi/2),
                         (np.array([-c,-c]),np.pi), (np.array([c,-c]),3*np.pi/2)]:
        delta = point-center
        angle = np.arctan2(delta[1],delta[0]) % (2*np.pi)
        # Pick the nearest angle in this interval, including wrap-around.
        angles = [np.clip(angle+k*2*np.pi,begin,begin+np.pi/2) for k in [-1,0,1]]
        for theta in angles:
            n = np.array([np.cos(theta),np.sin(theta)])
            candidates.append((center+r*n,np.array([-n[1],n[0]])))
    return min(candidates,key=lambda pair: np.linalg.norm(point-pair[0]))


class ConveyorController:
    def __init__(self, model, config):
        self.model, self.config = model, config
        self.box_id = model.body('target_box').id
        self.box_geom = model.geom('target_box_geom').id
        joint = model.joint('target_box_free').id
        self.qpos = model.jnt_qposadr[joint]
        self.dof = model.jnt_dofadr[joint]
        self.origin = np.array(config['conveyor_position'][:2])
        self.belt_geoms = {
            i for i in range(model.ngeom)
            if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i) or '').startswith('conveyor_belt')
        }
        self.path = {'straight':straight_path,'circular':circular_path,
                     'rounded_square':rounded_square_path}[config['conveyor']]
        self.parameters = (None if config['conveyor']=='straight'
                           else model.numeric('conveyor_path_parameters').data.copy())
        self.reset_state()

    def reset_state(self):
        self.touching = False
        self.commanded_speed = 0.
        self.force = np.zeros(2)
        self.integral_force = np.zeros(2)
        self.contact_gap = 0.
        self.previous_tangent = None

    def reset_box(self, data):
        data.qpos[self.qpos:self.qpos+3] = self.config['box_position']
        data.qpos[self.qpos+3:self.qpos+7] = [1,0,0,0]
        data.qvel[self.dof:self.dof+6] = 0
        data.xfrc_applied[self.box_id] = 0
        self.reset_state()
        mujoco.mj_forward(self.model,data)

    def box_position(self, data):
        return data.xpos[self.box_id].copy()

    def is_touching(self, data):
        return any(c.dist <= 0 and (
            (c.geom1 == self.box_geom and c.geom2 in self.belt_geoms) or
            (c.geom2 == self.box_geom and c.geom1 in self.belt_geoms))
            for c in data.contact)

    def update(self, data, *, stopped=False):
        # This controller owns the box's world-X/Y external-force slots.
        data.xfrc_applied[self.box_id,:2] = 0
        self.force[:] = 0
        self.commanded_speed = 0. if stopped else target_speed(self.config,data.time)
        self.touching = self.is_touching(data)
        if not self.touching:
            # Force is always zero off the belt. Preserve controller memory
            # through tiny contact-solver bounces, but clear it after separation.
            self.contact_gap += self.model.opt.timestep
            if (self.contact_gap > .05 or
                self.box_position(data)[2] > self.config['conveyor_position'][2]+.30):
                self.integral_force[:] = 0
            return
        self.contact_gap = 0.
        point = self.box_position(data)[:2]-self.origin
        _,tangent = self.path(point,self.parameters)
        # Keep accumulated velocity feedback in the local belt frame as the
        # tangent turns. A stale world-frame integral pushes outward on bends.
        # This rotates controller memory only; it adds no position attraction.
        if self.previous_tangent is not None:
            old = self.previous_tangent
            cosine = np.dot(old,tangent)
            sine = old[0]*tangent[1]-old[1]*tangent[0]
            self.integral_force = np.array([[cosine,-sine],[sine,cosine]]) @ self.integral_force
        self.previous_tangent = tangent.copy()
        # Match the local belt velocity, with no centerline position feedback.
        desired_velocity = self.commanded_speed*tangent
        velocity = data.qvel[self.dof:self.dof+2]
        error = desired_velocity-velocity
        # Limited integral feedback offsets drag from the fixed surface.
        if abs(self.commanded_speed) > 1e-9:
            self.integral_force += 60.*error*self.model.opt.timestep
            norm = np.linalg.norm(self.integral_force)
            self.integral_force *= min(1.,1.5/max(norm,1e-12))
        else:
            self.integral_force[:] = 0
        force = 35.*error+self.integral_force
        self.force = force*min(1.,3./max(np.linalg.norm(force),1e-12))
        data.xfrc_applied[self.box_id,:2] = self.force
