"""Run a JSON-selected conveyor experiment. Press Space to reset everything."""
import argparse
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from controllers.config import ROOT, load_config, validate_config
from controllers.conveyor import ConveyorController
from controllers.ik import GraspController


def build_model(config):
    # scene.xml stays directly loadable and acts as a shared template.
    root = ET.parse(ROOT/'scene.xml').getroot()
    for asset in root.findall('./asset/model'):
        path = (f'models/conveyors/{config["conveyor"]}.xml'
                if asset.get('name') == 'straight_conveyor' else asset.get('file'))
        asset.set('file',str(ROOT/path))
    root.find('./worldbody/frame[@name="conveyor_placement"]').set(
        'pos',' '.join(map(str,config['conveyor_position'])))
    root.find('./worldbody/frame[@name="box_placement"]').set(
        'pos',' '.join(map(str,config['box_position'])))
    return mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))


class Experiment:
    def __init__(self, config):
        self.config = validate_config(config)
        self.model = build_model(self.config)
        self.data = mujoco.MjData(self.model)
        self.conveyor = ConveyorController(self.model,self.config)
        self.grasp = GraspController(self.model,self.config,self.conveyor)
        self.reset_requested = threading.Event()
        self.reset()

    def request_reset(self, keycode):
        # The viewer callback signals; only the main thread changes physics.
        if keycode == 32:
            self.reset_requested.set()

    def reset(self):
        home = self.model.key('home').id
        mujoco.mj_resetDataKeyframe(self.model,self.data,home)
        self.data.xfrc_applied[:] = 0
        self.data.qfrc_applied[:] = 0
        self.conveyor.reset_box(self.data)
        self.grasp.reset()
        self.reset_requested.clear()

    def step(self):
        if self.reset_requested.is_set():
            self.reset()
        mujoco.mj_forward(self.model,self.data)
        self.conveyor.update(self.data,stopped=self.grasp.stops_conveyor(self.data.time))
        self.grasp.update(self.data)
        mujoco.mj_step(self.model,self.data)

    def result(self):
        mujoco.mj_forward(self.model,self.data)
        return {'time':round(self.data.time,3), 'phase':self.grasp.phase,
                'box_position':self.conveyor.box_position(self.data).tolist(),
                'failure':self.grasp.failure,
                'physics_warnings':int(self.data.warning.number.sum())}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'experiments/straight_constant.json')
    parser.add_argument('--headless',action='store_true',help='Run without a viewer')
    parser.add_argument('--duration',type=float,default=12.,help='Headless simulated seconds')
    parser.add_argument('--no-grasp',action='store_true',help='Run conveyor continuously')
    args = parser.parse_args()
    if args.duration <= 0 or not np.isfinite(args.duration):
        parser.error('--duration must be finite and positive')
    config = load_config(args.config)
    if args.no_grasp:
        config['grasp']['enabled'] = False
    experiment = Experiment(config)
    if args.headless:
        for _ in range(int(np.ceil(args.duration/experiment.model.opt.timestep))):
            experiment.step()
        result = experiment.result()
        print(result)
        if result['physics_warnings'] or (config['grasp']['enabled'] and result['phase'] != 'success'):
            raise SystemExit(1)
        return
    import mujoco.viewer
    print(f'{config["conveyor"]}: {config["motion"]}, speed={config["speed"]} m/s. Space: reset.')
    if config['grasp']['enabled']:
        print(f'Stationary grasp baseline: belt stops at {config["grasp"]["stop_time"]} s.')
    with mujoco.viewer.launch_passive(experiment.model,experiment.data,
                                     key_callback=experiment.request_reset) as viewer:
        last_phase = None
        while viewer.is_running():
            start = time.perf_counter()
            experiment.step()
            if experiment.grasp.phase != last_phase:
                print(f'{experiment.data.time:.2f}s: {experiment.grasp.phase}',
                      experiment.grasp.failure or '')
                last_phase = experiment.grasp.phase
            viewer.sync()
            remaining = experiment.model.opt.timestep-(time.perf_counter()-start)
            if remaining > 0:
                time.sleep(remaining)


if __name__ == '__main__':
    main()
