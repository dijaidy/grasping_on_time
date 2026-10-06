"""Read and validate one reproducible experiment configuration."""
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
KINDS = {'straight', 'circular', 'rounded_square'}


def validate_config(config):
    cfg = dict(config)
    if cfg.get('conveyor') not in KINDS:
        raise ValueError(f'conveyor must be one of {sorted(KINDS)}')
    if cfg.get('motion') == 'discete':
        cfg['motion'] = 'discrete'  # tolerate the original spelling
    if cfg.get('motion') not in {'constant', 'discrete', 'continuous'}:
        raise ValueError('motion must be constant, discrete, or continuous')
    speeds = cfg.get('speed')
    knots = cfg.get('milestone')
    if not isinstance(speeds, list) or not speeds or not np.isfinite(speeds).all():
        raise ValueError('speed must be a nonempty array of finite m/s values')
    if not isinstance(knots, list) or not np.isfinite(knots).all():
        raise ValueError('milestone must be an array of times in seconds')
    if cfg['motion'] == 'constant':
        if len(speeds) != 1 or knots:
            raise ValueError('constant requires one speed and milestone: []')
    elif (len(knots) != len(speeds) or len(knots) < 2 or knots[0] != 0
          or np.any(np.diff(knots) <= 0)):
        raise ValueError('milestone must start at 0, increase strictly, and match speed length (>=2)')
    for name in ['box_position', 'conveyor_position']:
        value = cfg.get(name)
        if not isinstance(value, list) or len(value) != 3 or not np.isfinite(value).all():
            raise ValueError(f'{name} must be [x, y, z] in world metres')
    grasp = {'enabled': True, 'stop_time': 0.8, 'settle_time': 1.0,
             'approach_height': 0.14, 'lift_height': 0.15}
    grasp.update(cfg.get('grasp', {}))
    if not isinstance(grasp['enabled'], bool):
        raise ValueError('grasp.enabled must be a boolean')
    for key in ['stop_time','settle_time','approach_height','lift_height']:
        if not np.isfinite(grasp[key]) or grasp[key] < 0:
            raise ValueError(f'grasp.{key} must be finite and nonnegative')
    if grasp['settle_time'] == 0 or grasp['lift_height'] < .1 or grasp['approach_height'] < .06:
        raise ValueError('Use settle_time > 0, lift_height >= 0.1, approach_height >= 0.06')
    cfg['grasp'] = grasp
    return cfg


def load_config(path):
    with Path(path).open() as f:
        return validate_config(json.load(f))
