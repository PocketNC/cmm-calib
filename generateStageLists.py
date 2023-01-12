import os
import json
from calibstate import Stages


POCKETNC_VAR_DIR = os.environ.get('POCKETNC_VAR_DIRECTORY')
CALIB_DIR = os.path.join(POCKETNC_VAR_DIR, 'calib')
if not os.path.exists(CALIB_DIR):
  os.makedirs(CALIB_DIR)

basic_calib_order = [
  Stages.ERASE_COMPENSATION,
  Stages.SETUP_CNC,
  Stages.SETUP_CMM,
  Stages.PROBE_MACHINE_POS,
  Stages.PROBE_SPINDLE_POS,
  Stages.HOMING_X,
  Stages.CHARACTERIZE_X,
  Stages.HOMING_Z,
  Stages.CHARACTERIZE_Z,
  Stages.PROBE_FIXTURE_BALL_POS,
  Stages.HOMING_Y,
  Stages.CHARACTERIZE_Y,
  Stages.PROBE_TOP_PLANE,
  Stages.PROBE_HOME_OFFSETS,
  Stages.HOMING_A,
  Stages.HOMING_B,
  Stages.CHARACTERIZE_A_SPHERE,
  Stages.CHARACTERIZE_B_SPHERE,
  Stages.TOOL_PROBE_OFFSET,
]

with open(os.path.join(CALIB_DIR, 'basic_order'), 'w') as f:
  f.write(json.dumps(basic_calib_order))

advanced_calib_order = [
  Stages.ERASE_COMPENSATION,
  Stages.SETUP_CNC,
  Stages.SETUP_CMM,
  Stages.PROBE_MACHINE_POS,
  Stages.PROBE_SPINDLE_POS,
  Stages.HOMING_X,
  Stages.CHARACTERIZE_X,
  Stages.HOMING_Z,
  Stages.CHARACTERIZE_Z,
  Stages.PROBE_FIXTURE_BALL_POS,
  Stages.HOMING_Y,
  Stages.CHARACTERIZE_Y,
  Stages.PROBE_TOP_PLANE,
  Stages.PROBE_HOME_OFFSETS,
  Stages.HOMING_A,
  Stages.HOMING_B,
  Stages.CHARACTERIZE_A_SPHERE,
  Stages.CHARACTERIZE_A_SPHERE_REVERSE,
  Stages.CHARACTERIZE_B_SPHERE,
  Stages.CHARACTERIZE_B_SPHERE_REVERSE,
  Stages.CHARACTERIZE_A_LINE,
  Stages.CHARACTERIZE_A_LINE_REVERSE,
  Stages.CHARACTERIZE_B_LINE,
  Stages.CHARACTERIZE_B_LINE_REVERSE,
  Stages.TOOL_PROBE_OFFSET,
]

with open(os.path.join(CALIB_DIR, 'advanced_order'), 'w') as f:
  f.write(json.dumps(advanced_calib_order))
