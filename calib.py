import sys
import os
import math
import copy
import importlib
from dataclasses import dataclass
from enum import Enum, auto
import numpy as np
import zmq
import asyncio
from zmq.asyncio import Context
import tornado
from tornado.ioloop import IOLoop
import logging
import json
import compensation

from ipp import Client, TransactionCallbacks, float3, CmmException, readPointData
import ipp_routines as routines

import metrology
import ini

def reload():
  importlib.reload(routines)
  importlib.reload(metrology)


logging.basicConfig(filename="/var/opt/pocketnc/calib/calib.log", 
  filemode='a', 
  level=logging.DEBUG,
  format='%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s',
  datefmt='%H:%M:%S',
)
logger = logging.getLogger(__name__)
fh = logging.FileHandler("/var/opt/pocketnc/calib/calib.log")
fh.setLevel(logging.DEBUG)
logger.addHandler(fh)
logger.debug('hello world')


ADDRESS_CMM = "10.0.0.1"
PORT_IPP = 1294

POCKETNC_VAR_DIR = os.environ.get('POCKETNC_VAR_DIRECTORY')
RESULTS_DIR = os.path.join(POCKETNC_VAR_DIR, 'calib')
CALIB_RESULTS_DIR = os.path.join(RESULTS_DIR, 'results')
if not os.path.exists(RESULTS_DIR):
    os.makedirs(RESULTS_DIR)
NEW_OVERLAY_FILENAME = os.path.join(RESULTS_DIR, "CalibrationOverlay.inc")
VERIFY_OVERLAY_FILENAME = os.path.join(RESULTS_DIR, "verify_overlay")
VERIFY_A_FILENAME = os.path.join(RESULTS_DIR, "verify_a")
VERIFY_B_FILENAME = os.path.join(RESULTS_DIR, "verify_b")
VERIFY_REPORT_FILENAME = os.path.join(RESULTS_DIR, "verify_report")
PART_CSY_SAVE_FILENAME = os.path.join(RESULTS_DIR, 'part_csy_savefile')
CNC_CSY_SAVE_FILENAME = os.path.join(RESULTS_DIR, 'cnc_csy_savefile')

calibrationOverlayFileName = os.path.join(POCKETNC_VAR_DIR, "CalibrationOverlay.inc")
iniFileName = os.path.join(POCKETNC_VAR_DIR, "PocketNC.ini")
aAngleFileName = os.path.join(POCKETNC_VAR_DIR, "calib/a-angle.txt")
bAngleFileName = os.path.join(POCKETNC_VAR_DIR, "calib/b-angle.txt")
xyzFileName = os.path.join(POCKETNC_VAR_DIR, "calib/xyz.txt")

aCompFileName = os.path.join(POCKETNC_VAR_DIR, "calib/a.comp")
bCompFileName = os.path.join(POCKETNC_VAR_DIR, "calib/b.comp")


def err_msg(msg):
  return "CMM_CALIB_ERROR: %s" % msg

def find_line_intersect_2d(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y

def find_line_intersect(p1,v1,p2,v2):
  v1 = np.array(v1[0:2])
  v2 = np.array(v2[0:2])
  p1 = np.array(p1[0:2])
  p2 = np.array(p2[0:2])

  v12 = p1 - p2
  v1_perp = np.array([-v1[1], v1[0]])
  denom = np.dot(v1_perp, v2)
  num = np.dot(v1_perp, v12)
  return (num / denom.astype(float))*v2 + p2

'''
Following two methods (unit_vector and angle_between) adapted from:
https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python
'''
def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in degrees between vectors 'v1' and 'v2'::
            >>> angle_between((1, 0, 0), (0, 1, 0))
            180/pi * 1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            180/pi * 0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            180/pi * 3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return (180/math.pi)*np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def angle_between_ccw_2d(v1,v2):
  print(v1)
  print(v2)
  dot = v1[0] * v2[0] + v1[1] * v2[1]
  det = v1[0] * v2[1] - v1[1] * v2[0]
  return math.atan2(det,dot) * 180/math.pi

class CalibException(Exception):
  pass

'''
Startup (go from many unknowns to => confident to proceed)
Characterize Z
Characterize planes of rotation for B, A axes
Characterize B
  Zero axis 
    Adjust home position until perpendicular to Z-norm
  Increment, measure B over full range
Characterize A
  Zero axis 
    Adjust home position until perpendicular to Z-norm
  Increment, measure A over full range
Find X, Y home offsets?

End position check (ensure we didn't move during process)
Calculate results
'''


'''
A step is an action or set of actions. 
Some steps are repeated (e.g. probing, clearance moves)
Some only once (e.g. setting up coord systems)
'''
class Steps(Enum):
  CONNECT_TO_CMM = auto()
  GO_TO_CLEARANCE_Z = auto()
  GO_TO_CLEARANCE_Y = auto()
  SETUP_CMM = auto()
  SETUP_PART_CSY = auto()
  PROBE_SPINDLE_POS = auto()
  SETUP_CNC_CSY = auto()
  PROBE_MACHINE_POS = auto()
  PROBE_FIXTURE_BALL_POS = auto()
  PROBE_TOP_PLANE = auto()
  PROBE_A_HOME = auto()
  PROBE_B_HOME = auto()
  PROBE_X_HOME = auto()
  PROBE_Y_HOME = auto()
  PROBE_Z_HOME = auto()
  VERIFY_A_HOME = auto()
  VERIFY_B_HOME = auto()
  VERIFY_X_HOME = auto()
  VERIFY_Y_HOME = auto()
  VERIFY_Z_HOME = auto()
  #need better names, but right now the HOMING steps are ran in Verification, after home offsets have been found and set
  VERIFY_A_HOMING = auto()
  VERIFY_B_HOMING = auto()
  PROBE_X = auto() 
  PROBE_Y = auto()
  PROBE_Z = auto()
  PROBE_A = auto()
  PROBE_B = auto()
  '''
  the find_pos_ steps use the same methods as probe_ steps, 
  but they use a different pattern for naming the metrology feature, 
  and instead of returning nothing they return the measured value 
  '''
  FIND_POS_A = auto() 
  FIND_POS_B = auto()
  CALC_CALIB = auto()
  WRITE_CALIB = auto()
  '''verification steps'''
  SETUP_VERIFY = auto()
  CALC_VERIFY = auto()
  WRITE_VERIFY = auto()
  DISCONNECT_FROM_CMM = auto()

  @classmethod
  def has_name(cls, name):
      return name in cls._member_names_ 

  def toJSON(self):
    return str(self)

'''
A stage is a progress point in the Calibration Process
The CalibManager
Steps may be repeated with (e.g. probing) or not (setting up coord systems)
'''
class Stages(Enum):
  ERASE_COMPENSATION = auto()
  SETUP_CNC = auto()
  SETUP_CMM = auto()
  PROBE_MACHINE_POS = auto()
  SETUP_PART_CSY = auto()
  PROBE_SPINDLE_POS = auto()
  HOMING_X = auto()
  HOMING_Y = auto()
  HOMING_Z = auto()
  HOMING_A = auto()
  HOMING_B = auto()
  CHARACTERIZE_X = auto()
  CHARACTERIZE_Y = auto()
  CHARACTERIZE_Z = auto()
  PROBE_TOP_PLANE = auto()
  PROBE_FIXTURE_BALL_POS = auto()
  SETUP_CNC_CSY = auto()
  CHARACTERIZE_A = auto()
  CHARACTERIZE_B = auto()
  CALC_CALIB = auto()
  WRITE_CALIB = auto()
  '''verification stages'''
  RESTART_CNC: auto()
  SETUP_VERIFY = auto()
  VERIFY_A_HOMING = auto()
  VERIFY_B_HOMING = auto()
  VERIFY_A = auto()
  VERIFY_B = auto()
  CALC_VERIFY = auto()
  WRITE_VERIFY = auto()

  @classmethod
  def has_name(cls, name):
      return name in cls._member_names_ 

  def toJSON(self):
    return str(self)


class CalibEncoder(json.JSONEncoder):
  def default(self, o):
    if isinstance(o, (Steps, Stages)):
      return str(o)
    return super().default(o)



'''
Some steps have prequisite stages that must be completed before they can be run
'''
STEP_PREREQS = {
  Steps.GO_TO_CLEARANCE_Z: [Stages.SETUP_CMM],
  Steps.GO_TO_CLEARANCE_Y: [Stages.SETUP_CMM],
  Steps.PROBE_MACHINE_POS: [Stages.SETUP_CMM],
  Steps.SETUP_PART_CSY: [Stages.PROBE_MACHINE_POS],
  Steps.PROBE_SPINDLE_POS: [Stages.SETUP_PART_CSY],

  # Steps.PROBE_TOP_PLANE: [Stages.SETUP_PART_CSY],
  
  Steps.PROBE_X_HOME: [Stages.PROBE_SPINDLE_POS],
  Steps.VERIFY_X_HOME: [Stages.PROBE_SPINDLE_POS],
  Steps.PROBE_X: [Stages.PROBE_SPINDLE_POS],

  Steps.PROBE_Y_HOME: [Stages.SETUP_PART_CSY],
  Steps.VERIFY_Y_HOME: [Stages.SETUP_PART_CSY],
  Steps.PROBE_Y: [Stages.SETUP_PART_CSY],
  
  Steps.PROBE_Z_HOME: [Stages.PROBE_SPINDLE_POS],
  Steps.VERIFY_Z_HOME: [Stages.PROBE_SPINDLE_POS],
  Steps.PROBE_Z: [Stages.PROBE_SPINDLE_POS],

  Steps.SETUP_CNC_CSY: [Stages.CHARACTERIZE_X, Stages.CHARACTERIZE_Y, Stages.CHARACTERIZE_Z],

  Steps.PROBE_A_HOME: [Stages.SETUP_CNC_CSY],
  Steps.VERIFY_A_HOME: [Stages.SETUP_CNC_CSY],
  Steps.FIND_POS_A: [Stages.SETUP_CNC_CSY],
  Steps.PROBE_A: [Stages.SETUP_CNC_CSY],

  Steps.PROBE_B_HOME: [Stages.SETUP_CNC_CSY],
  Steps.VERIFY_B_HOME: [Stages.SETUP_CNC_CSY],
  Steps.FIND_POS_B: [Stages.SETUP_CNC_CSY],
  Steps.PROBE_B: [Stages.SETUP_CNC_CSY],

  Steps.WRITE_CALIB: [Stages.CHARACTERIZE_A, Stages.CHARACTERIZE_B],
  Steps.WRITE_VERIFY: [Stages.VERIFY_A, Stages.VERIFY_B],
}

STEP_STAGES = {
  Steps.VERIFY_X_HOME: Stages.HOMING_X,
  Steps.VERIFY_Y_HOME: Stages.HOMING_Y,
  Steps.VERIFY_Z_HOME: Stages.HOMING_Z,
  Steps.VERIFY_A_HOME: Stages.HOMING_A,
  Steps.VERIFY_B_HOME: Stages.HOMING_B,
}

STAGE_PREREQS = {
  Stages.PROBE_MACHINE_POS: [Stages.SETUP_CMM],
  Stages.SETUP_PART_CSY: [Stages.PROBE_MACHINE_POS],
  Stages.PROBE_SPINDLE_POS: [Stages.SETUP_PART_CSY],
  Stages.CHARACTERIZE_X: [Stages.SETUP_PART_CSY],
  Stages.CHARACTERIZE_Y: [Stages.SETUP_PART_CSY],
  Stages.CHARACTERIZE_Z: [Stages.SETUP_PART_CSY],
  # Stages.PROBE_TOP_PLANE: [Stages.SETUP_PART_CSY],
  Stages.SETUP_CNC_CSY: [Stages.CHARACTERIZE_X, Stages.CHARACTERIZE_Y, Stages.CHARACTERIZE_Z],
  Stages.CHARACTERIZE_A: [Stages.SETUP_CNC_CSY],
  Stages.CHARACTERIZE_B: [Stages.SETUP_CNC_CSY],
  # Stages.WRITE_CALIB: [Stages.CHARACTERIZE_A, Stages.CHARACTERIZE_A],

  Stages.VERIFY_A: [Stages.SETUP_VERIFY],
  Stages.VERIFY_B: [Stages.SETUP_VERIFY],
  Stages.WRITE_VERIFY: [Stages.VERIFY_A, Stages.VERIFY_B],
}

STAGE_FEATURES = {
  Stages.PROBE_MACHINE_POS: lambda n: n in ['L_bracket_top_face', 'L_bracket_back_face', 'L_bracket_right_face'],
  Stages.CHARACTERIZE_X: lambda n: len(n) == 7 and n.find('x_') == 0,
  Stages.CHARACTERIZE_Y: lambda n: len(n) == 7 and n.find('y_') == 0,
  Stages.CHARACTERIZE_Z: lambda n: len(n) == 7 and n.find('z_') == 0,
  Stages.PROBE_TOP_PLANE: lambda n: n in ['fixture_top_face'],
  Stages.SETUP_CNC_CSY: lambda n: n in ['z_line', 'z_line_proj'],
  Stages.CHARACTERIZE_A: lambda n: n.startswith(('find_a_', 'probe_a_')),
  Stages.CHARACTERIZE_B: lambda n: n.startswith(('find_b_', 'probe_b_')),
  Stages.CALC_CALIB: lambda n: n.startswith(('z_line', 'z_line_proj', 'b_circle', 'proj_b', 'proj_probe_b', 'a_circle', 'proj_a', 'proj_probe_a')), 
  Stages.VERIFY_A: lambda n: len(n) == 7 and n.find('x_') == 0, 
}

FEATURE_NAMES_BY_STAGE = {
  Stages.PROBE_MACHINE_POS: lambda n: n in ['L_bracket_top_face', 'L_bracket_back_face', 'L_bracket_right_face'],
  Stages.CHARACTERIZE_X: lambda n: len(n) == 7 and n.find('x_') == 0,
  Stages.CHARACTERIZE_Y: lambda n: len(n) == 7 and n.find('y_') == 0,
  Stages.CHARACTERIZE_Z: lambda n: len(n) == 7 and n.find('z_') == 0,
  Stages.PROBE_TOP_PLANE: lambda n: n in ['fixture_top_face'],
  Stages.SETUP_CNC_CSY: lambda n: n in ['z_line', 'z_line_proj'],
  Stages.CHARACTERIZE_A: lambda n: n.startswith(('find_a_', 'probe_a_')),
  Stages.CHARACTERIZE_B: lambda n: n.startswith(('find_b_', 'probe_b_')),
  Stages.CALC_CALIB: lambda n: n.startswith(('z_line', 'z_line_proj', 'b_circle', 'proj_b', 'proj_probe_b', 'a_circle', 'proj_a', 'proj_probe_a')), 
  Stages.VERIFY_A: lambda n: len(n) == 7 and n.find('x_') == 0,
}

def does_feature_belong_to_stage(name, stage):
  return name in stage_features

def stage_features_include_name(name, stage_features):
  return name in stage_features

# def stage_feature_name_selectors(stage):

V2_VARIANT_10 = "V2-10"
V2_VARIANT_50 = "V2-50"

STATE_RUN = 'RUN'
STATE_IDLE = 'IDLE'
STATE_ERROR = 'ERROR'
STATE_FAIL = 'FAIL'
STATE_PAUSE = 'PAUSE'
STATE_STOP = 'STOP'

Z_CLEARANCE_PART_CSY = 250

FIXTURE_HEIGHT = 25.2476
FIXTURE_SIDE = 76.2
FIXTURE_DIAG = 107.76
TOP_BACK_RIGHT_TO_ORIG = float3()
X_ORIGIN = 528
Y_ORIGIN = 238
Z_ORIGIN = 400
CMM_ORIGIN = float3(X_ORIGIN,Y_ORIGIN,Z_ORIGIN)

FIXTURE_BALL_DIA = 6.35
SPINDLE_BALL_DIA = 6.35
Z_BALL_DIA = 6.35
PROBE_DIA = 4

TOOL_3_LENGTH = 117.8
B_LINE_LENGTH = 35

Z_MIN_LIMIT_V2_10_INCHES = -3.451
Z_MIN_LIMIT_V2_50_INCHES = -3.541

Y_MAX = 64.5
Y_MIN = -63.5
Y_STEP = -25
Z_MIN = -26
Z_STEP = -25
B_STEP = 5
B_MIN = 0
B_MAX = 360
A_STEP = 5
A_MIN = -25
A_MAX = 135
X_PROBE_END_TRIGGER = -50
Y_PROBE_END_TRIGGER = -63.5
Z_PROBE_END_TRIGGER = -24
A_PROBE_END_TRIGGER = 129
B_PROBE_END_TRIGGER = 356

LINEAR_HOMING_REPEATABILITY = 0.001 * 25.4 # 0.001 inches, 0.0254 mm
B_HOMING_REPEATABILITY = 0.04 # degrees
A_HOMING_REPEATABILITY = 0.08 # degrees
SPEC_ANGULAR_ACCURACY = 0.05 # degrees
SPEC_LINEAR_ACCURACY = 0.001 #Placeholder, we don't actually check this (yet)

V2_10_PROPS = {
  'A_MIN': -25,
  'A_MAX': 135,
  'Z_MAX': 0.1,
  'Z_MIN': -3.451,
}

V2_50_PROPS = {
  'A_MIN': -25,
  'A_MAX': 135,
  'Z_MAX': 0.0,
  'Z_MIN': -3.541,
}

FIXTURE_OFFSET_B_ANGLE = 225
OFFSET_B_POS_REL_Z = -135
OFFSET_B_NEG_REL_Z = 225
OFFSET_A_REL_Z = 90

PROBING_POS_Y = -63

FEAT_FIXTURE_SPHERE = 'fixture_sphere'
FEAT_SPINDLE_POS_SPHERE = 'spindle_pos_sphere'

D_MAT = np.array([[0,1,0,0],[0,0,1,0],[0,0,0,1],[1,1,1,1]])

MSG_WHY_STEP_COMPLETE = "STEP_COMPLETE"
MSG_WHY_UPDATE = "UPDATE"
MSG_WHY_ERROR = "ERROR"
MSG_WHY_FAIL = "FAIL"

#z pos is screwed up because we are mixing 2 origin positions
waypoints_from_pocket_origin = {
  'origin': float3(0.0, 0.0, 0.0),
  'top_l_bracket_front_right': float3(0.0, 0.0, 0.0),
  'top_l_bracket_back_right': float3(18.4, 22.5, 0.0),
  'probe_fixture_tip': float3(-33.90, -133.0, -83.53),
  'probe_fixture_tip_from_origin': float3(-406.7, -598.5, -199.33),
  # move maybe -2 in X from b_cor   
  # have now done that (from -108.87 to -110.87)
  # well... that was Y... soooo.... lets go from (-60.33, -110.87) to (-62.33, -108.87)
  # now looks like maybe -5 in Y from b_cor, start with -3... from (-62.33, -108.87, -0.33) to (-62.33, -111.87, -0.33)
  # another -1 in Y. From (-62.33, -111.87, -0.33) to (-62.33, -113.0, -0.33)
  # lets also go down a bit in Z. From -0.33 to -2

  'b_rot_cent_approx': float3(-62.33, -111.87, -2.0),
  'a_rot_cent_approx': float3(-63.10, -46.0, -6.93),
  'z_home_50': float3(42.5, -49.25, -68.83),
  'z_home_10': float3(72.0, -50.50, -68.83),
  'fixture_ball': float3(-94.0, -114.7, -123.4)
  # 'fixture_ball': float3(-94.4, -107.0, -123.4)
}

waypoints_table_center = {
  'top_l_bracket_back_right': float3(371.9, 466.8, 126.33),
  'top_l_bracket_front_right': float3(358.2, 448.4, 126.33),
  'probe_fixture_tip': float3(324.3, 318.9, 42.8),
  'probe_fixture_tip_from_origin': float3(-48.5, -150.1, -73.0),
  'b_rot_cent_approx': float3(297.87, 339.53, 126),
  'a_rot_cent_approx': float3(295.1, 412.7, 119.4),
  'z_home': float3(400.9,400.5,57.5),
}

waypoints_table = {
  'front_right_slot_origin': float3(653.0, 134.0, 126.5 ),
}

table_slot_euler_angles = {
  'front_right': [0, -90, 0]
}

waypoints = waypoints_from_pocket_origin

def transform_waypoints(translate_vec, rotate_ang):
  dir_path = os.path.dirname(os.path.realpath(__file__))
  file_path = os.path.join(dir_path, "waypoints_X%s_Y%s_Z%s_ROT%s" % (translate_vec.x, translate_vec.y, translate_vec.z, rotate_ang))

  with open(file_path, 'w'):
    for wp_key in waypoints.keys():
      rel_vec = float3(0,0,0)
      if "_from_origin" in wp_key:
        rel_vec = waypoints[wp_key]
      else:
        rel_vec = waypoints[wp_key]

def gen_spec_item():
  return {'val': None, 'pass': None}

SPECS = [
  'x_homing_repeatability', 'y_homing_repeatability', 'z_homing_repeatability', 'a_homing_repeatability', 'b_homing_repeatability',
  'xy_squareness', 'xz_squareness', 'yz_squareness',
  'by_parallelness', 'ax_parallelness',
  'b_runout',
  'b_comp_max_err', 'a_comp_max_err',
  'x_home_err', 'y_home_err', 'z_home_err', 'a_home_err', 'b_home_err',
  'b_max_err', 'a_max_err'
]

SPEC_DICT = { k: gen_spec_item() for k in SPECS }


ORIGIN = waypoints['top_l_bracket_back_right']
ORG_TO_B = float3(-3.15, -5.1, 0)

DEBUG = False
def debugWait():
  if DEBUG:
    input()
  return

class CalibException(Exception):
  pass

'''
Ensure startup conditions: homed, tool set, errors clear
Verify machine position
  Probe against top of A-backing plate for maximum clearance/safety
Characterize A motion
Characterize B motion
'''

calibManagerInstance = None

# ctx = Context.instance()

class Csy:
  def __init__(self, orig, x_dir, y_dir, z_dir, euler):
    self.orig = np.array(orig)
    self.x_dir = np.array(x_dir)
    self.y_dir = np.array(y_dir)
    self.z_dir = np.array(z_dir)
    self.euler = np.array(euler)

  def to_json_dict(self):
    d = {}
    d['orig'] = self.orig.tolist() if self.orig is not None else None
    d['x_dir'] = self.x_dir.tolist() if self.x_dir is not None else None
    d['y_dir'] = self.y_dir.tolist() if self.y_dir is not None else None
    d['z_dir'] = self.z_dir.tolist() if self.z_dir is not None else None
    d['euler'] = self.euler.tolist() if self.euler is not None else None
    return d

class CalibManager:
  def __init__(self):
    self.client = None

    self.config = {}
    self.config['skip_cmm'] = False
    self.config['table_slot'] = None

    self.status = {}
    self.status['is_running'] = False
    self.status['run_state'] = None
    self.status['cmm_error'] = False
    self.status['error'] = False
    self.status['error_msg'] = None
    self.status['spec_failure'] = False
    
    self.metrologyManager = metrology.FeatureManager.getInstance()
    self.next_feature_id = 1
    self.feature_ids = {}
    # stage_features keys are names of stages, values are lists of feature names
    self.stage_features = {}
    # stage_fitted_features keys are names of stages, values are lists of fitted feature names
    self.stage_fitted_features = {}
    self.stage_state = {}

    # keys for fitted_features dict are the feature names, the values are tuples of characterizing data
    # the meaning of the tuple values varies by feature type (circle, line, plane, etc.)
    # you'll need to look at the metrology module to see the specific meaning of the tuple values (length, point, norm, etc.)
    self.fitted_features = {}

    self.is_running = False
    self.stages_completed = {}
    for stage in Stages:
      self.stages_completed[str(stage)[len("Stages."):]] = False

    self.is_started_a = False
    self.is_started_b = False


    self.part_csy_pos = None
    self.part_csy_euler = None
    self.part_csy = None
    self.cnc_csy = None


    self.a_calib_probes = []
    self.b_calib_probes = []
    self.x_calib_probes = []
    self.y_calib_probes = []
    self.z_calib_probes = []
    self.a_verify_probes = []
    self.b_verify_probes = []
    self.a_home_values = []
    self.b_home_values = []
    self.a_home_probes = []
    self.b_home_probes = []
    self.x_home_probes = []
    self.y_home_probes = []
    self.z_home_probes = []

    self.ini_data = ini.read_ini_data(iniFileName)
    self.overlay_data = ini.read_ini_data(calibrationOverlayFileName)

    self.offsets = {}
    self.active_offsets = {}
    self.active_offsets['x'] =  float(ini.get_parameter(self.ini_data, "JOINT_0", "HOME_OFFSET")["values"]["value"])
    self.active_offsets['y'] =  float(ini.get_parameter(self.ini_data, "JOINT_1", "HOME_OFFSET")["values"]["value"])
    self.active_offsets['z'] =  float(ini.get_parameter(self.ini_data, "JOINT_2", "HOME_OFFSET")["values"]["value"])
    self.active_offsets['a'] =  float(ini.get_parameter(self.ini_data, "JOINT_3", "HOME_OFFSET")["values"]["value"])
    self.active_offsets['b'] =  float(ini.get_parameter(self.ini_data, "JOINT_4", "HOME_OFFSET")["values"]["value"])

    self.a_err = []
    self.b_err = []
    self.a_comp = []
    self.b_comp = []
    self.a_ver_err = []
    self.b_ver_err = []
    self.spec = SPEC_DICT


  def getInstance():
    global calibManagerInstance
    if calibManagerInstance == None:
      calibManagerInstance = CalibManager()
    return calibManagerInstance

  async def zmq_listen(self):
    # context = zmq.Context()
    # socket = context.socket(zmq.REQ)
    socket = ctx.socket(zmq.REQ)
    socket.connect('ipc:///tmp/cmm')
    while True:
      print('awaiting zmq message')
      msg = await socket.recv()
      print('got zmq message')
      print(msg)
    socket.close()

  def zmq_report(self, why_string='UPDATE', did_stage_complete=False, stage=None):
    logger.debug('Start of zmq_report')
    try:
      report = {}
      report['why'] = why_string
      report['status'] = self.status
      report['spec'] = self.spec
      report['did_stage_complete'] = did_stage_complete
      report['stage'] = stage.name if stage else ""
      
      context = zmq.Context()
      socket = context.socket(zmq.PUSH)
      socket.set(zmq.SNDTIMEO, 3000)
      socket.bind('ipc:///tmp/cmm')
      report_json = json.dumps(report, cls=CalibEncoder)
      socket.send_string(report_json)
    except Exception as e:
      logger.debug('exception in zmq_report')
      logger.debug(e)
      raise e


  def reload_features(self):
    for f in self.metrologyManager:
      print(f)

  def set_state(self, name, val):
    print('set_state')
    print(name)
    print(val)
    setattr(self, name, val)

  def add_state(self, name, val, stage=None):
    setattr(self, name, val)
    if stage is not None:
      self.stage_state.setdefault(stage, {})[name] = val

  def add_fitted_feature(self,feature_name,feature_data,stage=None):
    self.fitted_features[feature_name] = feature_data
    if stage is not None:
      self.stage_fitted_features.setdefault(stage, []).append(feature_name)

  def add_feature(self,feature_name,stage=None):
    if stage is not None:
      self.stage_features.setdefault(stage, []).append(feature_name)
    self.feature_ids[feature_name] = self.next_feature_id
    self.metrologyManager.getActiveFeatureSet().setActiveFeatureID(self.next_feature_id)
    self.next_feature_id = self.next_feature_id + 1
    return self.metrologyManager.getActiveFeatureSet().getActiveFeature()

  def get_feature(self,feature_name):
    try:
      fid = self.feature_ids[feature_name]
      return self.metrologyManager.getActiveFeatureSet().getFeature(fid)
    except Exception as e:
      print("Feature %s not present" % feature_name)
      return e

  def clear_features(self):
    self.metrologyManager = metrology.FeatureManager.getInstance()
    self.metrologyManager.sets.clear()
    # while len(self.metrologyManager.sets) > 0:
    #   self.metrologyManager.pop()

  def save_progress_for_stage(self, stage):
    try:
      save_data = {'features': {}, 'fitted_features': {}, 'state': {}, 'status': self.status, 'spec': self.spec}
      if stage in self.stage_features:
        for feat_name in self.stage_features[stage]:
          fid = self.feature_ids[feat_name]
          feat = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
          save_data['features'][feat_name] = feat.points().tolist()
      if stage in self.stage_fitted_features:
        for feat_name in self.stage_fitted_features[stage]:
          save_data['fitted_features'][feat_name] = {}
          feat_data = self.fitted_features[feat_name]
          for datum_name in feat_data.keys():
            try:
              save_data['fitted_features'][feat_name][datum_name] = feat_data[datum_name].tolist() #convert numpy ndarrays to list for JSON compatibility
            except AttributeError:
              save_data['fitted_features'][feat_name][datum_name] = feat_data[datum_name]
      if stage in self.stage_state:
        for val in self.stage_state[stage]:
          attr = getattr(self, val)
          if isinstance(attr, np.ndarray):
            save_data['state'][val] = attr.tolist()
          elif isinstance(attr, Csy):
            save_data['state'][val] = attr.to_json_dict()
          else:
            save_data['state'][val] = attr
      savefile_path = os.path.join(RESULTS_DIR, str(stage))
      with open(savefile_path, 'w') as f:
        f.write( json.dumps(save_data, cls=CalibEncoder) )
    except Exception as e:
      print(e)
      return e


  def load_stage_progress(self, stage, is_performing_stage=True):
    save_data = {}
    savefile_path = os.path.join(RESULTS_DIR, str(stage))
    with open(savefile_path, 'r') as f:
      save_data = json.loads(f.read())
    for feat_name in save_data['features'].keys():
      feat = self.add_feature(feat_name)
      for pt in save_data['features'][feat_name]:
        feat.addPoint(*pt)
    for feat_name in save_data['fitted_features'].keys():
      self.fitted_features[feat_name] = {}
      for datum_name in save_data['fitted_features'][feat_name].keys():
        self.fitted_features[feat_name][datum_name] = save_data['fitted_features'][feat_name][datum_name]
    for prop_name in save_data['state'].keys():
      print('loading ' + prop_name)
      prop = save_data['state'][prop_name]
      print(prop)
      if type(prop) is dict and all(csy_attr in prop for csy_attr in ['euler', 'orig', 'x_dir']):
      # in prop and hasattr(prop, 'orig') and hasattr(prop, 'x_dir'):
        #its a CSY object converted to builtins for JSON object. Create new Csy object
        csy = Csy(prop['orig'], np.array(prop['x_dir']), np.array(prop['y_dir']), np.array(prop['z_dir']), prop['euler'])
        print('loading csy')
        print(prop)
        print(csy)
        setattr(self, prop_name, csy)
      else:
        setattr(self, prop_name, save_data['state'][prop_name])

    # setattr(self, 'results', save_data['results'])
    # setattr(self, 'spec', save_data['spec'])

    if is_performing_stage:
      self.stages_completed[str(stage)[len("Stages."):]] = True
      self.zmq_report('STEP_COMPLETE', did_stage_complete=True, stage=stage)



  def save_features(self):
    try:
      save_data = {}
      for k in self.feature_ids.keys():
        fid = self.feature_ids[k]
        feat = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
        save_data[k] = feat.points().tolist()

      savefile_path = os.path.join(RESULTS_DIR, 'savefile')
      with open(savefile_path, 'w') as f:
        f.write( json.dumps(save_data, cls=CalibEncoder) )
    except Exception as e:
      print("Exception saving features")
      print(e)
      raise e

  def save_fitted_featues(self):
    try:
      save_data = {}
      for k in self.fitted_features.keys():
        save_data[k] = {}
        for datum_name, datum_vals in self.fitted_features[k]:
          try:
            save_data[k][datum] = datum_vals.tolist()
          except AttributeError:
            save_data[k][datum] = datum_vals

      savefile_path = os.path.join(RESULTS_DIR, 'fitted_features')
      with open(savefile_path, 'w') as f:
        f.write( json.dumps(save_data, cls=CalibEncoder) )
    except Exception as e:
      print("Exception saving features")
      print(e)
      raise e


  def load_stage_features(self, stage):
    try:
      selector = FEATURE_NAMES_BY_STAGE[stage]

      savefile_path = os.path.join(RESULTS_DIR, 'savefile')
      with open(savefile_path, 'r') as f:
        data = json.loads(f.read())
        for feat_name in data:
          if selector(feat_name):
            feat = self.add_feature(feat_name)
            feat_pts = data[feat_name] 
            for pt in feat_pts:
              feat.addPoint(*pt)
    except Exception as e:
      print("Exception in load_stage_features")
      print(e)


  def load_features(self):
    try:
      savefile_path = os.path.join(RESULTS_DIR, 'savefile')
      load_data = {}
      with open(savefile_path, 'r') as f:
        load_data = json.loads(f.read())
        for feat_name in load_data.keys():
          feat = self.add_feature(feat_name)
          for pt in load_data[feat_name]:
            feat.addPoint(*pt)

          if feat_name.find("find_") == 0 and feat_name.rfind("_proj") == 16:
            if "find_a_" in feat_name:
              self.feat_name_last_find_a_proj = feat_name
            if "find_b_" in feat_name:
              self.feat_name_last_find_b_proj = feat_name
          elif feat_name.find("probe_") == 0:
            # these are the A and B characterization features
            axis = feat_name[len("probe_")]
            if feat_name[len("probe_")] == 'b':
              self.b_calib_probes.append(feat_name)
            elif feat_name[len("probe_")] == 'a':
              self.a_calib_probes.append(feat_name)
      print("_______END___________")
    except Exception as e:
      print("Exception in load_features")
      print(e)

  def save_part_csy(self):
    try:
      # data = {}
      # data['orig'] = self.part_csy.orig.ToXYZString()
      # data['euler'] = self.part_csy.euler
      # with open(PART_CSY_SAVE_FILENAME, 'w') as f:
      #   f.write( json.dumps(data) )
      data = self.part_csy.to_json_dict()
      with open(PART_CSY_SAVE_FILENAME, 'w') as f:
        f.write( json.dumps(data, cls=CalibEncoder) )
    except Exception as e:
      print("Exception saving part_csy")
      print(e)
      raise e

  async def load_part_csy(self):
    try:
      with open(PART_CSY_SAVE_FILENAME, 'r') as f:
        data = json.loads(f.read())
        orig = np.array(data['orig']) if data['orig'] else None
        x_dir = np.array(data['x_dir']) if data['x_dir'] else None
        y_dir = np.array(data['y_dir']) if data['y_dir'] else None
        z_dir = np.array(data['z_dir']) if data['z_dir'] else None
        euler = np.array(data['euler']) if data['euler'] else None
        print('setting cnc_csy')
        print(orig, x_dir, y_dir, z_dir, euler)
        part_csy = Csy(orig, x_dir, y_dir, z_dir, euler)
        self.add_state('part_csy', part_csy, Stages.SETUP_PART_CSY)


        # data = json.loads(f.read())
        # orig = float3.FromXYZString(data['orig'])
        # euler = np.array(data['euler'])
        # await self.set_part_csy(orig, euler)
    except Exception as e:
      print("Exception saving features")
      print(e)
      raise e

  def save_cnc_csy(self):
    try:
      # data = {}
      # data['x_norm'] = self.x_norm.tolist()
      # data['y_norm'] = self.y_norm.tolist()
      # data['z_norm'] = self.z_norm.tolist()
      # data['cmm2cnc'] = self.cmm2cnc.tolist()
      data = self.cnc_csy.to_json_dict()
      with open(CNC_CSY_SAVE_FILENAME, 'w') as f:
        f.write( json.dumps(data, cls=CalibEncoder) )
    except Exception as e:
      print("Exception save_cnc_csy")
      print(e)
      raise e

  async def load_cnc_csy(self):
    try:
      with open(CNC_CSY_SAVE_FILENAME, 'r') as f:
        data = json.loads(f.read())
        orig = np.array(data['orig']) if data['orig'] else None
        x_dir = np.array(data['x_dir']) if data['x_dir'] else None
        y_dir = np.array(data['y_dir']) if data['y_dir'] else None
        z_dir = np.array(data['z_dir']) if data['z_dir'] else None
        euler = np.array(data['euler']) if data['euler'] else None
        print('setting cnc_csy')
        print(orig, x_dir, y_dir, z_dir, euler)
        cnc_csy = Csy(orig, x_dir, y_dir, z_dir, euler)
        self.add_state('cnc_csy', cnc_csy, Stages.SETUP_CNC_CSY)

        # self.x_norm = np.array(data['x_norm'])
        # self.y_norm = np.array(data['y_norm'])
        # self.z_norm = np.array(data['z_norm'])
        # self.cmm2cnc = np.array(data['cmm2cnc'])
    except Exception as e:
      print("Exception saving features")
      print(e)
      raise e

  def is_ready_for_step(self, step):
    print('checking is_ready_for_step')
    if step in STEP_PREREQS:
      print('step has prereqs')
      for prereqStage in STEP_PREREQS[step]:
        print('prereq stage %s ' % prereqStage)
        if not self.stages_completed[str(prereqStage)[len("Stages."):]]:
          return False
    return True

  def std_stage_complete_check(self, ret):
    if ret is True:
      return True
    else:
      return False

  def did_step_complete_stage(self, step, ret, *args):
    try:
      if step in [
        Steps.CONNECT_TO_CMM, Steps.GO_TO_CLEARANCE_Z, Steps.GO_TO_CLEARANCE_Y,
        Steps.PROBE_A_HOME, Steps.PROBE_B_HOME,
        Steps.PROBE_X_HOME, Steps.PROBE_Y_HOME, Steps.PROBE_Z_HOME,
        Steps.FIND_POS_A, Steps.FIND_POS_B, Steps.DISCONNECT_FROM_CMM,
      ]:
        return (False, None)
      elif step in [Steps.VERIFY_X_HOME, Steps.VERIFY_Y_HOME, Steps.VERIFY_Z_HOME, Steps.VERIFY_A_HOME, Steps.VERIFY_B_HOME ]:
        return (self.std_stage_complete_check(ret), STEP_STAGES[step])
      elif step is Steps.SETUP_CMM:
        return (self.std_stage_complete_check(ret), Stages.SETUP_CMM)
      elif step is Steps.SETUP_PART_CSY:
        return (self.std_stage_complete_check(ret), Stages.SETUP_PART_CSY)
      elif step is Steps.PROBE_SPINDLE_POS:
        return (self.std_stage_complete_check(ret), Stages.PROBE_SPINDLE_POS)
      elif step is Steps.SETUP_CNC_CSY:
        return (self.std_stage_complete_check(ret), Stages.SETUP_CNC_CSY)
      elif step is Steps.PROBE_FIXTURE_BALL_POS:
        return (self.std_stage_complete_check(ret), Stages.PROBE_FIXTURE_BALL_POS)
      elif step is Steps.PROBE_MACHINE_POS:
        return (self.std_stage_complete_check(ret), Stages.PROBE_MACHINE_POS)
      elif step is Steps.PROBE_TOP_PLANE:
        return (self.std_stage_complete_check(ret), Stages.PROBE_TOP_PLANE)
      elif step is Steps.PROBE_A:
        print('checking if PROBE_A completed')
        print(*args)
        stage = Stages.VERIFY_A if 'verify' in args[0] else Stages.CHARACTERIZE_A
        if ret is True:
          nominal_a_pos = args[2]
          if nominal_a_pos >= A_PROBE_END_TRIGGER:
            return(True, stage)
          else:
            return (False, stage)
        else:
          return (False, stage)
      elif step is Steps.PROBE_B:
        print('checking if PROBE_B completed')
        stage = Stages.VERIFY_B if 'verify' in args[0] else Stages.CHARACTERIZE_B
        if ret is True:
          nominal_b_pos = args[2]
          if nominal_b_pos >= B_PROBE_END_TRIGGER:
            print('true B completed')
            return(True, stage)
          else:
            return (False, stage)
        else:
          return (False, stage)
      elif step is Steps.PROBE_X:
        print('checking if PROBE_X completed')
        if ret is True:
          nominal_x_pos = args[0]
          if nominal_x_pos <= X_PROBE_END_TRIGGER:
            return(True, Stages.CHARACTERIZE_X)
          else:
            return (False, Stages.CHARACTERIZE_X)
        else:
          return (False, Stages.CHARACTERIZE_X)
      elif step is Steps.PROBE_Y:
        print('checking if PROBE_Y completed')
        if ret is True:
          nominal_y_pos = args[0]
          if nominal_y_pos <= Y_PROBE_END_TRIGGER:
            return(True, Stages.CHARACTERIZE_Y)
          else: 
            return (False, Stages.CHARACTERIZE_Y)
        else:
          return (False, Stages.CHARACTERIZE_Y)
      elif step is Steps.PROBE_Z:
        print('checking if PROBE_Z completed')
        if ret is True:
          nominal_z_pos = args[1]
          print(nominal_z_pos)
          if nominal_z_pos <= Z_PROBE_END_TRIGGER:
            print('yes z probe complete')
            return(True, Stages.CHARACTERIZE_Z)
          else:
            print('no z probe not complete')
            return (False, Stages.CHARACTERIZE_Z)
        else:
          return (False, Stages.CHARACTERIZE_Z)
      elif step is Steps.CALC_CALIB:
        return (self.std_stage_complete_check(ret), Stages.CALC_CALIB)
      elif step is Steps.WRITE_CALIB:
        return (self.std_stage_complete_check(ret), Stages.WRITE_CALIB)
      elif step is Steps.SETUP_VERIFY:
        return (self.std_stage_complete_check(ret), Stages.SETUP_VERIFY)
      elif step is Steps.VERIFY_A_HOMING:
        return (self.std_stage_complete_check(ret), Stages.VERIFY_A_HOMING)
      elif step is Steps.VERIFY_B_HOMING:
        return (self.std_stage_complete_check(ret), Stages.VERIFY_B_HOMING)
      elif step is Steps.CALC_VERIFY:
        return (self.std_stage_complete_check(ret), Stages.CALC_VERIFY)
      elif step is Steps.WRITE_VERIFY:
        return (self.std_stage_complete_check(ret), Stages.WRITE_VERIFY)
        
      else:
        logger.debug("Ran a STEP without entry in did_step_complete_stage")
        return (False, None)
    except Exception as e:
      msg = "Exception in did_step_complete_stage %s" % str(e)
      logger.debug(msg)
      return err_msg(msg)

  '''
  the find_pos_ steps use the same methods as probe_ steps, 
  but use a different name for the metrology feature, 
  and return the measured value instead of returning nothing
  '''
  def run_step(self, step, *args):
    logger.info("Running step: %s, args:" % (step))
    
    # this is intended to throw an exception if step arg is not defined in Step enum
    if type(step) is str:
      step = Step[step.upper()]
    step in Steps

    if not self.is_ready_for_step(step):
      logger.info('not ready for step')
      return err_msg("1 or more prerequisite STAGES not complete before running STEP %s")
    step_method = getattr(self, step.name.lower())
    try:
      self.status['run_state'] = STATE_RUN
      self.is_running = True
      self.zmq_report('UPDATE')
      self.status['step'] = step.toJSON()
      # self.status['stage'] = stage # stage isn't known yet here.... remove from status, or add "stage_for_step" dict?
      step_ret = asyncio.get_event_loop().run_until_complete(step_method(*args))
      logger.info('Returning from step %s' % (step))
      did_stage_complete, stage_for_step = self.did_step_complete_stage(step, step_ret, *args)
      if did_stage_complete:
        logger.info('Marking stage complete: %s' % (stage_for_step))
        if stage_for_step is Stages.CHARACTERIZE_A:
          self.stage_state.setdefault(Stages.CHARACTERIZE_A, {})
          self.stage_state[Stages.CHARACTERIZE_A]['a_calib_probes'] = self.a_calib_probes
        elif stage_for_step is Stages.CHARACTERIZE_B:
          self.stage_state.setdefault(Stages.CHARACTERIZE_B, {})
          self.stage_state[Stages.CHARACTERIZE_B]['b_calib_probes'] = self.b_calib_probes
        elif stage_for_step is Stages.CHARACTERIZE_X:
          self.stage_state.setdefault(Stages.CHARACTERIZE_X, {})
          self.stage_state[Stages.CHARACTERIZE_X]['x_calib_probes'] = self.x_calib_probes
        elif stage_for_step is Stages.CHARACTERIZE_Y:
          self.stage_state.setdefault(Stages.CHARACTERIZE_Y, {})
          self.stage_state[Stages.CHARACTERIZE_Y]['y_calib_probes'] = self.y_calib_probes
        elif stage_for_step is Stages.CHARACTERIZE_Z:
          self.stage_state.setdefault(Stages.CHARACTERIZE_Z, {})
          self.stage_state[Stages.CHARACTERIZE_Z]['z_calib_probes'] = self.z_calib_probes
        elif stage_for_step is Stages.VERIFY_A:
          self.stage_state.setdefault(Stages.VERIFY_A, {})
          self.stage_state[Stages.VERIFY_A]['a_verify_probes'] = self.a_verify_probes
        elif stage_for_step is Stages.VERIFY_B:
          self.stage_state.setdefault(Stages.VERIFY_B, {})
          self.stage_state[Stages.VERIFY_B]['b_verify_probes'] = self.b_verify_probes
        self.save_progress_for_stage(stage_for_step)
        #putting this in an if statement because maybe some steps will be run again 
        #AFTER their 'normal' stage is completed
        self.stages_completed[str(stage_for_step)[len("Stages."):]] = did_stage_complete
        #dirty hack, making special case for SETUP_VERIFY. Steps PROBE_A and PROBE_B have Stage.SETUP_CNC_CSY as prereq. 
        #In SETUP_VERIFY, the cnc csy is reloaded from file, and then applied to CMM
        if step is Steps.SETUP_VERIFY:
          self.stages_completed[str(Stages.SETUP_CNC_CSY)[len("Stages."):]] = True
        
        self.is_running = False
        self.status['run_state'] = STATE_IDLE

        logger.info('completing step %s' % step)
        self.zmq_report('STEP_COMPLETE', did_stage_complete=did_stage_complete, stage=stage_for_step)
      else:
        if self.status['spec_failure']:
          #failed a verification check, the process should stop
          self.zmq_report(MSG_WHY_FAIL)
        else:
          self.zmq_report('STEP_COMPLETE')
      
      logger.info('step %s return value %s' % (step, step_ret))
      return step_ret
      # self.stages_completed[stage] = isStageComplete
    except CmmException as e:
      msg = err_msg("CMM error occured while running step %s:  %s" % (step, str(e)))
      logger.error(msg)
      did_stage_complete, stage_for_step = self.did_step_complete_stage(step, e, *args)
      self.updateStatusException(e)
      self.zmq_report('ERROR', did_stage_complete=did_stage_complete, stage=stage_for_step)
      raise e
    except Exception as e:
      msg = err_msg("Error while running step %s:  %s" % (step, str(e)))
      logger.error(msg)
      did_stage_complete, stage_for_step = self.did_step_complete_stage(step, e, *args)
      self.updateStatusException(e)
      self.zmq_report('ERROR', did_stage_complete=did_stage_complete, stage=stage_for_step)
      raise e

  def updateStatusException(self, exception):
      self.status['cmm_error'] = True
      self.status['error'] = True
      self.status['error_msg'] = str(exception)
      self.status['is_running'] = False
      self.status['run_state'] = STATE_ERROR

  def updateStatusFailure(self):
      self.status['is_running'] = False
      self.status['run_state'] = STATE_FAIL

  async def connect_to_cmm(self):
    if self.config['skip_cmm']:
      return True
    try:
      if self.client and self.client.is_connected():
        return True
    except Exception as e:
      logger.error("Exception in connect_to_cmm during check for existing connection. Attempting to connect next. Exception is: %s" % e)

    try:
      self.client = Client(ADDRESS_CMM, PORT_IPP)
      await self.client.connect()
      await self.client.EndSession().complete()
      await self.client.StartSession().complete()
      return True
    except Exception as e:
      logger.error("Exception while connecting")
      logger.error(e)
      raise e

  async def setup_cmm(self):
    if not self.config['skip_cmm']:
      await self.client.ClearAllErrors().complete()
      await routines.ensure_homed(self.client)
      await routines.ensure_tool_loaded(self.client, "Component_3.1.50.4.A0.0-B0.0")
      await self.client.SetProp("Tool.GoToPar.Speed(500)").ack()
      await self.client.SetProp("Tool.GoToPar.Accel(450)").ack()
      await self.client.SetCsyTransformation("PartCsy, 0,0,0,0,0,0").complete()
      # await self.client.SetCsyTransformation("MachineCsy, 0,0,0,0,0,0").complete()
      await self.client.SetCoordSystem("MachineCsy").complete()
    await self.set_table_slot("front_right")
    if not self.config['skip_cmm']:
      await self.set_part_csy(self.part_csy_pos, self.part_csy_euler)
    return True

  async def disconnect_from_cmm(self):
    '''
    End
    '''
    if self.client is None or self.client.stream is None:
      logger.debug("already disconnected")
      return True

    try:
      await self.client.EndSession().send()
      await self.client.disconnect()
    except CmmException as ex:
      logger.error("CmmExceptions %s" % ex)


  async def end(self):
    '''
    End
    '''
    if self.client is None or self.client.stream is None:
      logger.debug("already disconnected")
      return True

    try:
      await self.client.EndSession().send()
      await self.client.disconnect()
    except CmmException as ex:
      logger.error("CmmExceptions %s" % ex)



  async def go_to_clearance_z(self):
    if self.config['skip_cmm']:
      return False
    await self.client.GoTo("Z(%s)" % Z_CLEARANCE_PART_CSY).complete()
    return False

  async def go_to_clearance_y(self):
    if self.config['skip_cmm']:
      return False
    await self.client.GoTo("Y(-250)").complete()
    return False

  async def set_table_slot(self, slot):
    self.config['table_slot'] = slot
    self.part_csy_pos = waypoints_table[slot + '_slot_origin']
    self.part_csy_euler = table_slot_euler_angles[slot]
    self.part_csy = Csy(np.array(self.part_csy_pos), None, None, None, self.part_csy_euler)
    # await self.set_part_csy(waypoints_table['front_right_slot_origin'], table_slot_euler_angles['front_right'])

  async def set_part_csy(self,csy_pos,csy_euler):
    try:
      # self.part_csy = Csy(csy_pos, None, None, None, csy_euler)
      part_csy_pos = self.part_csy.orig
      await self.client.SetCsyTransformation("PartCsy, %s, %s, %s, %s, %s, %s" % (self.part_csy.orig[0], self.part_csy.orig[1], self.part_csy.orig[2], self.part_csy.euler[0], self.part_csy.euler[1], self.part_csy.euler[2])).complete()
      await self.client.SetCoordSystem("PartCsy").complete()
      return True
    except Exception as ex:
      logger.error("set_part_csy exception %s" % str(ex))
      raise ex

  async def set_cmm_csy(self, csy):
    try:
      await self.client.SetCsyTransformation("PartCsy, %s, %s, %s, %s, %s, %s" % (csy.orig[0], csy.orig[1], csy.orig[2], csy.euler[0], csy.euler[1], csy.euler[2])).complete()
      await self.client.SetCoordSystem("PartCsy").complete()
      return True
    except Exception as ex:
      logger.error("set_cmm_csy exception %s" % str(ex))
      raise ex


  async def test_head(self, angle):
    '''
    Do a head-probe line against 1 face of the probing fixture
    '''
    try:
      print("Manually position probe to be +Y from the -X side of the XZ-aligned plane of the L-bracket")
      input()
      getBackPos = self.client.Get("X(),Y(),Z()")
      await getBackPos.complete()
      backPos = readPointData(getBackPos.data_list[0])
      await routines.headprobe_line(self.client,backPos,float3(1,0,0),100,10,5,20,1,15)
    except CmmException as ex:
      print("CmmExceptions %s" % ex)


  async def probe_machine_pos(self):
    '''
    Locate machine and verify it is in home position
    '''
    try:
      logger.info("Probing machine position")
      if self.config['table_slot'] is None:
        #table slot has not been set, raise exception,
        raise CalibException("Quitting VERIFY_MACHINE_POS, table slot has not been set")
      await self.client.GoTo("Z(250)").complete()
      await self.client.SetProp("Tool.PtMeasPar.Approach(10)").complete()
      await self.client.SetProp("Tool.PtMeasPar.Search(15)").complete()
      await self.client.SetProp("Tool.PtMeasPar.Retract(-1)").complete()
      await self.client.SetProp("Tool.PtMeasPar.HeadTouch(0)").complete()
      await self.client.GoTo("Tool.A(0),Tool.B(0)").complete()

      #ensure A-backing plate and B-trunnion are not raised above L-bracket by sweeping probe tip through area
      #sweeping potential location of B-motor when A approx 90 and Y raised near limit
      await self.client.GoTo(float3(-180, -155, 90).ToXYZString()).complete()
      await self.client.GoTo(float3(-40, -155, 90).ToXYZString()).complete()
      await self.client.GoTo(float3(-40, -155, 20).ToXYZString()).complete()
      await self.client.GoTo(float3(-40, 40, 20).ToXYZString()).complete()

      # L-bracket top face
      approachPos = float3(0,0,10)
      await self.client.GoTo(approachPos.ToXYZString()).complete()

      measPos = float3(0,2,0)
      L_bracket_top_face = self.add_feature('L_bracket_top_face', Stages.PROBE_MACHINE_POS)
      ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % (measPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      L_bracket_top_face.addPoint(pt.x, pt.y, pt.z)

      measPos = float3(-20,0,0)
      ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % (measPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      L_bracket_top_face.addPoint(pt.x, pt.y, pt.z)

      measPos = float3(-30,12,0)
      ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % (measPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      L_bracket_top_face.addPoint(pt.x, pt.y, pt.z)

      # L-bracket back face
      L_bracket_back_face = self.add_feature('L_bracket_back_face', Stages.PROBE_MACHINE_POS)
      approach_pos = float3(-35, 30, 10)
      await self.client.GoTo(approach_pos.ToXYZString()).complete()
      approach_pos = approach_pos + float3(0, 0, -20)
      await self.client.GoTo(approach_pos.ToXYZString()).complete()
      for i in range(3):
        meas_pos = approach_pos + float3(i * 15, -15, 0)
        pt_meas = await self.client.PtMeas("%s,IJK(0,1,0)" % (meas_pos.ToXYZString())).complete()
        pt = float3.FromXYZString(pt_meas.data_list[0])
        L_bracket_back_face.addPoint(pt.x, pt.y, pt.z)

      #back-to-side transition
      approach_pos = float3(25,30,-10)
      await self.client.GoTo(approach_pos.ToXYZString()).complete()

      # L-bracket right face
      approach_pos = float3(23,15,-10)
      await self.client.GoTo(approach_pos.ToXYZString()).complete()
      L_bracket_right_face = self.add_feature('L_bracket_right_face', Stages.PROBE_MACHINE_POS)
      for i in range(3):
        meas_pos = approach_pos + float3(-15, i * -8, 0)
        pt_meas = await self.client.PtMeas("%s,IJK(1,0,0)" % (meas_pos.ToXYZString())).complete()
        pt = float3.FromXYZString(pt_meas.data_list[0])
        L_bracket_right_face.addPoint(pt.x, pt.y, pt.z)
      return True
    except Exception as ex:
      logger.debug("probe_machine_pos exception %s" % str(ex))
      raise ex


  async def setup_part_csy(self):
    '''
    Setup PartCsy
      PartCsy is a coord system in the IPP spec, it is used for majority of this process
      Calculate the V2's orientation and position using data from verify_machine_pos
      Rotation will only be about the CMM Z axis,
      calibration will be halted if the machine is tilted signficantly relative to the CMM XY plane.

    Also, detect model of V2 by inspecting axis limits/offsets values from ini, 
    set model-dependent dimension/waypoint values
    '''
    fid = self.feature_ids['L_bracket_top_face']
    L_bracket_top_face = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
    top_plane_orig, top_plane_norm  = L_bracket_top_face.plane()
    top_face_avg_z = L_bracket_top_face.average()[2]
    tilt = angle_between(top_plane_norm, (0,0,1))
    print("Vertical Tilt %.2f deg" % tilt)
    if tilt > 2:
      print("Vertical Tilt greater than 2 degrees, halting")
      return False

    fid = self.feature_ids['L_bracket_back_face']
    L_bracket_back_face = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
    back_face_line_pt, back_face_line_dir  = L_bracket_back_face.line()
    print('back_face_line_dir')
    print(back_face_line_dir)

    #find rotation relative to ideal alignment (+x direction)
    ang_machine_z_to_partcsy_x = angle_between_ccw_2d([back_face_line_dir[0],back_face_line_dir[1]], [1,0])
    print('ang_machine_z_to_partcsy_x')
    print(ang_machine_z_to_partcsy_x)

    #find position of point on L-brackets top-back-right corner
    fid = self.feature_ids['L_bracket_right_face']
    L_bracket_right_face = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
    right_face_line_pt, right_face_line_dir  = L_bracket_right_face.line()

    xy_pos = find_line_intersect(
      [right_face_line_pt[0],right_face_line_pt[1], 0],
      [right_face_line_dir[0],right_face_line_dir[1], 0],
      [back_face_line_pt[0],back_face_line_pt[1], 0],
      [back_face_line_dir[0],back_face_line_dir[1], 0]
    )
    intersect_pos = float3(xy_pos[0],xy_pos[1],top_face_avg_z)
    pos_to_orig = -1*waypoints_from_pocket_origin['top_l_bracket_back_right']
 
    dist = np.linalg.norm( pos_to_orig )
    pos_to_orig_xy_ang = math.atan2( pos_to_orig.y, pos_to_orig.x ) * 180/math.pi
    ang_true_pos_to_true_orig = pos_to_orig_xy_ang - ang_machine_z_to_partcsy_x
    origin_offset = intersect_pos + float3(1,0,0) * dist * math.cos((ang_true_pos_to_true_orig)/180*math.pi) + float3(0,1,0) * dist * math.sin((ang_true_pos_to_true_orig)/180*math.pi)

    new_euler_angles = [self.part_csy.euler[0], self.part_csy.euler[1] - ang_machine_z_to_partcsy_x, self.part_csy_euler[2]]
    print(origin_offset)
    print(new_euler_angles)
    new_orig = self.part_csy_pos - origin_offset
    print('new_orig')
    print(new_orig)
    part_csy = Csy(np.array(new_orig), None, None, None, new_euler_angles)
    self.add_state('part_csy', part_csy, Stages.SETUP_CNC_CSY)
    
    if not self.config['skip_cmm']:
      await self.set_part_csy(new_orig, new_euler_angles)
    self.save_part_csy()

    z_min_limit_ini = float(ini.get_parameter(self.ini_data, "JOINT_2", "MIN_LIMIT")["values"]["value"])
    if abs(z_min_limit_ini - Z_MIN_LIMIT_V2_10_INCHES) < 0.0001:
      self.model = V2_VARIANT_10
      waypoints['z_home'] = waypoints['z_home_10']
      self.machine_props = V2_10_PROPS
    elif abs(z_min_limit_ini - Z_MIN_LIMIT_V2_50_INCHES) < 0.0001:
      self.model = V2_VARIANT_50
      waypoints['z_home'] = waypoints['z_home_50']
      self.machine_props = V2_50_PROPS
    else:
      return err_msg("CMM calibration halting, Z MIN_LIMIT abnormal, doesn't correspond to known model of V2. Z MIN_LIMIT: %s" % z_min_limit_ini)

    return True

  async def probe_fixture_ball_pos(self, y_pos_v2):
    '''
    Find fixture ball position
    '''
    try:
      feature = self.add_feature('fixutre_ball_pos', Stages.PROBE_FIXTURE_BALL_POS)
      orig = waypoints['fixture_ball'] + float3(0,0,-(y_pos_v2 - 63.5))
      contact_radius = (FIXTURE_BALL_DIA+PROBE_DIA)/2
      a_angle_probe_contact = math.atan2(contact_radius,TOOL_3_LENGTH)*180/math.pi

      await self.client.GoTo("Tool.Alignment(0,0,1,1,0,0)").complete()
      await self.client.GoTo((orig + float3(0,0,25)).ToXYZString()).complete()
      getCurrPosCmd = await self.client.Get("X(),Y(),Z()").complete()
      currPos = readPointData(getCurrPosCmd.data_list[0])

      #place tip pos +Y from target
      await self.client.GoTo("Tool.A(%s),Tool.B(90)" % (a_angle_probe_contact+5)).complete()
      await self.client.GoTo("Z(%s)" % (currPos.z - 25)).complete()
      await self.client.SetProp("Tool.PtMeasPar.HeadTouch(1)").complete()
      await self.client.SetProp("Tool.PtMeasPar.Search(6)").complete()

      # probe in -Y dir
      try:
        ptMeas = await self.client.PtMeas("%s,IJK(0,1,0)" % ((orig + float3(0,contact_radius,0)).ToXYZString())).complete()
        pt = float3.FromXYZString(ptMeas.data_list[0])
        feature.addPoint(*pt)
      except Exception as e:
        logger.error('probe_fixture_ball_pos -Y exception')
        logger.error(str(e))
        await self.client.ClearAllErrors().complete()


      # move to -X pos, probe in +X dir
      try:
        await self.client.GoTo("Tool.A(%s),Tool.B(180)" % (a_angle_probe_contact+5)).complete()
        ptMeas = await self.client.PtMeas("%s,IJK(-1,0,0)" % ((orig + float3(-contact_radius,0,0)).ToXYZString())).complete()
        pt = float3.FromXYZString(ptMeas.data_list[0])
        feature.addPoint(*pt)
      except Exception as e:
        logger.error('probe_fixture_ball_pos +X exception')
        logger.error(str(e))
        await self.client.ClearAllErrors().complete()

      # move to -Y pos, probe in +Y dir
      try:
        await self.client.GoTo("Tool.A(%s),Tool.B(270)" % (a_angle_probe_contact+5)).complete()
        ptMeas = await self.client.PtMeas("%s,IJK(0,-1,0)" % ((orig + float3(0,-contact_radius,0)).ToXYZString())).complete()
        pt = float3.FromXYZString(ptMeas.data_list[0])
        feature.addPoint(*pt) 
      except Exception as e:
        logger.error('probe_fixture_ball_pos +Y exception')
        logger.error(str(e))
        await self.client.ClearAllErrors().complete()

      #rise 2mm and probe another 3 points
      measPos2 = orig + float3(0,0,2)
      getCurrPosCmd = await self.client.Get("X(),Y(),Z()").complete()
      currPos = readPointData(getCurrPosCmd.data_list[0])
      # await self.client.GoTo((currPos + float3(0,0,+2)).ToXYZString()).complete()
      await self.client.GoTo("Tool.A(%s),Tool.B(270)" % (a_angle_probe_contact+5)).complete()
      await self.client.GoTo("Z(%s)" % (currPos.z + 2)).complete()

      # probe in +Y dir
      try:
        await self.client.GoTo("Tool.A(%s),Tool.B(270)" % (a_angle_probe_contact+5)).complete()
        ptMeas = await self.client.PtMeas("%s,IJK(0,-1,0)" % ((measPos2 + float3(0,-contact_radius,0)).ToXYZString())).complete()
        pt = float3.FromXYZString(ptMeas.data_list[0])
        feature.addPoint(*pt)
      except Exception as e:
        logger.error('probe_fixture_ball_pos +2+Y exception')
        logger.error(str(e))
        await self.client.ClearAllErrors().complete()

      # move to -X pos, probe in +X dir
      try:
        await self.client.GoTo("Tool.A(%s),Tool.B(180)" % (a_angle_probe_contact+5)).complete()
        ptMeas = await self.client.PtMeas("%s,IJK(-1,0,0)" % ((measPos2 + float3(-contact_radius,0,0)).ToXYZString())).complete()
        pt = float3.FromXYZString(ptMeas.data_list[0])
        feature.addPoint(*pt) 
      except Exception as e:
        logger.error('probe_fixture_ball_pos +2+X exception')
        logger.error(str(e))
        await self.client.ClearAllErrors().complete()

      # move to +Y pos, probe in -Y dir
      try:
        await self.client.GoTo("Tool.A(%s),Tool.B(90)" % (a_angle_probe_contact+5)).complete()
        ptMeas = await self.client.PtMeas("%s,IJK(0,1,0)" % ((measPos2 + float3(0,contact_radius,0)).ToXYZString())).complete()
        pt = float3.FromXYZString(ptMeas.data_list[0])
        feature.addPoint(*pt) 
      except Exception as e:
        logger.error('probe_fixture_ball_pos +2-Y exception')
        logger.error(str(e))
        await self.client.ClearAllErrors().complete()

      await self.client.GoTo((orig + float3(0,0,50)).ToXYZString()).complete()

      #fit sphere to feature, use for more accurate waypoint
      try:
        (radius, pos) = feature.sphere()
        if abs(radius*2 - (Z_BALL_DIA+PROBE_DIA)) > 0.1:
          raise CalibException("Unexpected spindle tip sphere diameter %s" % (radius*2))
        pos = pos + float3(0,0,(y_pos_v2 - 63.5))
        self.add_fitted_feature(FEAT_FIXTURE_SPHERE, {'pos': pos, 'radius': radius}, Stages.PROBE_FIXTURE_BALL_POS)
        return True
      except Exception as ex:
        logger.error("probe_fixture_ball_pos fit sphere exception %s" % str(ex))
        raise ex
    except Exception as ex:
      logger.error("probe_fixture_ball_pos exception %s" % str(ex))
      raise ex



  async def probe_spindle_pos(self, x_pos_v2, z_pos_v2):
      '''
      Find the spindle pos using long search distance probes to accomodate variation
      '''
      try:
        logger.info("Finding spindle position")
        spindle_pos = self.add_feature('spindle_pos', Stages.PROBE_SPINDLE_POS)

        if self.config['table_slot'] is None:
          #table slot has not been set, raise exception,
          raise CalibException("Quitting PROBE_SPINDLE_POS, table slot has not been set")
        await self.client.GoTo("Z(%s)" % Z_CLEARANCE_PART_CSY).complete()
        await self.client.SetProp("Tool.PtMeasPar.Approach(10)").complete()
        await self.client.SetProp("Tool.PtMeasPar.Search(100)").complete()
        await self.client.SetProp("Tool.PtMeasPar.Retract(-1)").complete()
        await self.client.SetProp("Tool.PtMeasPar.HeadTouch(0)").complete()
        await self.client.GoTo("Tool.A(0),Tool.B(0)").complete()

        orig = waypoints['z_home'] + float3(z_pos_v2,x_pos_v2 - 63.5,0)

        if self.model == V2_VARIANT_10:
          spindle_side_clearance = 22.5
        elif self.model == V2_VARIANT_50:
          spindle_side_clearance = 30
        

        #probe against spindle shaft, approx. 45 mm in the PART_CSY +X direction from typical spindle tip position
        await self.client.GoTo((orig + float3(45, spindle_side_clearance, 25)).ToXYZString()).complete()
        await self.client.GoTo((orig + float3(45, spindle_side_clearance, 0)).ToXYZString()).complete()
        pt_meas = await self.client.PtMeas("%s,IJK(0,1,0)" % ((orig + float3(45, 10, 0)).ToXYZString())).complete()
        spindle_shaft_pt_1 = float3.FromXYZString(pt_meas.data_list[0])

        #up and over shaft
        await self.client.GoTo((orig + float3(45, spindle_side_clearance, 25)).ToXYZString()).complete()
        await self.client.GoTo((orig + float3(45, -spindle_side_clearance, 25)).ToXYZString()).complete()

        #probe spindle shaft from other side (from PartCsy -Y side, probing in +Y direction)
        await self.client.GoTo((orig + float3(45, -spindle_side_clearance, 0)).ToXYZString()).complete()
        pt_meas = await self.client.PtMeas("%s,IJK(0,-1,0)" % ((orig + float3(45, -10, 0)).ToXYZString())).complete()
        spindle_shaft_pt_2 = float3.FromXYZString(pt_meas.data_list[0])

        # spindle tip Y position should be at mid point of previous two probes
        spindle_shaft_y = (spindle_shaft_pt_1.y + spindle_shaft_pt_2.y) / 2
        self.add_state('spindle_shaft_y', spindle_shaft_y, Stages.PROBE_SPINDLE_POS)
        
        #now probe along the spindle center line until we hit the tip
        await self.client.GoTo((orig + float3(-50, -30, 0)).ToXYZString()).complete()
        await self.client.GoTo(float3(orig.x - 50, self.spindle_shaft_y, orig.z).ToXYZString()).complete()
        meas_pos = float3(orig.x, self.spindle_shaft_y, orig.z)
        pt_meas = await self.client.PtMeas("%s,IJK(-1,0,0)" % (meas_pos.ToXYZString())).complete()
        tip_pt = float3.FromXYZString(pt_meas.data_list[0])
        spindle_pos.addPoint(*tip_pt)

        #take additional points on spindle tip sphere
        #from +Y (probe in -Y)
        await self.client.GoTo((tip_pt + float3(0, 10, 0)).ToXYZString()).ack()
        await self.client.GoTo((tip_pt + float3(5, 10, 0)).ToXYZString()).complete()
        meas_pos = tip_pt + float3(5, Z_BALL_DIA/2, 0)
        pt_meas = await self.client.PtMeas("%s,IJK(0,1,0)" % (meas_pos.ToXYZString())).complete()
        pt = float3.FromXYZString(pt_meas.data_list[0])
        spindle_pos.addPoint(*pt)
        #from +Z (probe in -Z)
        await self.client.GoTo((tip_pt + float3(5, 10, 5)).ToXYZString()).ack()
        await self.client.GoTo((tip_pt + float3(5, 0, 5)).ToXYZString()).ack()
        meas_pos = tip_pt + float3(5, 0, Z_BALL_DIA/2)
        pt_meas = await self.client.PtMeas("%s,IJK(0,0,1)" % (meas_pos.ToXYZString())).complete()
        pt = float3.FromXYZString(pt_meas.data_list[0])
        spindle_pos.addPoint(*pt)
        #from -Y (probe in +Y)
        await self.client.GoTo((tip_pt + float3(5, -10, 5)).ToXYZString()).ack()
        await self.client.GoTo((tip_pt + float3(5, -10, 0)).ToXYZString()).ack()
        meas_pos = tip_pt + float3(5, -Z_BALL_DIA/2, 0)
        pt_meas = await self.client.PtMeas("%s,IJK(0,-1,0)" % (meas_pos.ToXYZString())).complete()
        pt = float3.FromXYZString(pt_meas.data_list[0])
        spindle_pos.addPoint(*pt)

        (radius, pos) = spindle_pos.sphere()
        if abs(radius*2 - (Z_BALL_DIA+PROBE_DIA)) > 0.1:
          raise CalibException("Unexpected spindle tip sphere diameter %s" % (radius*2))

        self.add_fitted_feature(FEAT_SPINDLE_POS_SPHERE, {'pos': pos, 'radius': radius}, Stages.PROBE_SPINDLE_POS)
        return True
      except Exception as ex:
        logger.debug("probe_spindle_pos exception %s" % str(ex))
        raise ex


  async def find_b_cor(self):
    orig = waypoints['b_rot_cent_approx']
    approachPos = orig + float3(0,0,100)
    await self.client.GoTo(approachPos.ToXYZString()).complete()
    await self.client.SetProp("Tool.PtMeasPar.HeadTouch(0)").complete()

    self.next_feature_id = self.next_feature_id + 1
    self.feature_ids['probeFixtureTopFace'] = self.next_feature_id
    self.metrologyManager.getActiveFeatureSet().setActiveFeatureID(self.next_feature_id)
    probeFixtureTopFace = self.metrologyManager.getActiveFeatureSet().getActiveFeature()
    ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % (orig.ToXYZString())).complete()
    pt = float3.FromXYZString(ptMeas.data_list[0])
    probeFixtureTopFace.addPoint(*pt)
    ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % ((orig + float3(3,3,0)).ToXYZString())).complete()
    pt = float3.FromXYZString(ptMeas.data_list[0])
    probeFixtureTopFace.addPoint(*pt)
    ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % ((orig + float3(-3,3,0)).ToXYZString())).complete()
    pt = float3.FromXYZString(ptMeas.data_list[0])
    probeFixtureTopFace.addPoint(*pt)

    await self.client.SetProp("Tool.PtMeasPar.Search(15)").complete()

    #back-right
    self.next_feature_id = self.next_feature_id + 1
    self.feature_ids['probeFixtureTopBackRight'] = self.next_feature_id
    self.metrologyManager.getActiveFeatureSet().setActiveFeatureID(self.next_feature_id)
    probeFixtureTopBackRight = self.metrologyManager.getActiveFeatureSet().getActiveFeature()
    x_shift = 5
    y_diff = -x_shift
    approachPos = orig + float3(x_shift,FIXTURE_DIAG/2, 10)
    await self.client.GoTo(approachPos.ToXYZString()).complete()
    approachPos = approachPos + float3(0,0,-15)
    await self.client.GoTo(approachPos.ToXYZString()).complete()
    lineStartPos = approachPos + float3(0,y_diff,0)
    backRightPoints = await routines.probe_line(self.client, lineStartPos, float3(1,-1,0), float3(1, 1, 0), 40, 10, 2, -1)
    for pt in backRightPoints:
      probeFixtureTopBackRight.addPoint(*pt)

    #front-right
    self.next_feature_id = self.next_feature_id + 1
    self.feature_ids['probeFixtureTopFrontRight'] = self.next_feature_id
    self.metrologyManager.getActiveFeatureSet().setActiveFeatureID(self.next_feature_id)
    probeFixtureTopFrontRight = self.metrologyManager.getActiveFeatureSet().getActiveFeature()
    y_shift = -5
    x_diff = y_shift
    approachPos = orig + float3(FIXTURE_DIAG/2 + 10,0, 10)
    await self.client.GoTo(approachPos.ToXYZString()).complete()
    approachPos = orig + float3(FIXTURE_DIAG/2, y_shift, -5)
    await self.client.GoTo(approachPos.ToXYZString()).complete()
    lineStartPos = approachPos + float3(x_diff,0,0)
    frontRightPoints = await routines.probe_line(self.client, lineStartPos, float3(-1,-1,0), float3(1, -1, 0), 40, 10, 2,-1)
    for pt in frontRightPoints:
      probeFixtureTopFrontRight.addPoint(*pt)


    #front-left
    self.next_feature_id = self.next_feature_id + 1
    self.feature_ids['probeFixtureTopFrontLeft'] = self.next_feature_id
    self.metrologyManager.getActiveFeatureSet().setActiveFeatureID(self.next_feature_id)
    probeFixtureTopFrontLeft = self.metrologyManager.getActiveFeatureSet().getActiveFeature()
    x_shift = -5
    y_diff = -x_shift
    approachPos = orig + float3(0,-FIXTURE_DIAG/2-10, 10)
    await self.client.GoTo(approachPos.ToXYZString()).complete()
    approachPos = orig + float3(x_shift,-FIXTURE_DIAG/2, -5)
    await self.client.GoTo(approachPos.ToXYZString()).complete()
    lineStartPos = approachPos + float3(0,y_diff,0)
    frontLeftPoints = await routines.probe_line(self.client, lineStartPos, float3(-1,1,0), float3(-1, -1, 0), 40, 10, 2, -1)
    for pt in frontLeftPoints:
      probeFixtureTopFrontLeft.addPoint(*pt)

    #rear-left
    self.next_feature_id = self.next_feature_id + 1
    self.feature_ids['probeFixtureTopRearLeft'] = self.next_feature_id
    self.metrologyManager.getActiveFeatureSet().setActiveFeatureID(self.next_feature_id)
    probeFixtureTopRearLeft = self.metrologyManager.getActiveFeatureSet().getActiveFeature()
    y_shift = 5
    x_diff = y_shift
    approachPos = orig + float3(-(FIXTURE_DIAG/2 + 10),0, 10)
    await self.client.GoTo(approachPos.ToXYZString()).complete()
    approachPos = orig + float3(-(FIXTURE_DIAG/2 + 10), y_shift, -5)
    await self.client.GoTo(approachPos.ToXYZString()).complete()
    lineStartPos = approachPos + float3(x_diff,0,0)
    rearLeftPoints = await routines.probe_line(self.client, lineStartPos, float3(1,1,0), float3(-1, 1, 0), 40, 10, 2, -1)
    for pt in rearLeftPoints:
      probeFixtureTopRearLeft.addPoint(*pt)

    #project the side lines to the top plane


  async def probe_top_plane(self, y_pos_v2):
    try:
      orig = waypoints['b_rot_cent_approx'] + float3(0,0,(-y_pos_v2 - 63.5))
      await self.client.GoTo("Tool.Alignment(0,0,1)").complete()
      await self.client.SetProp("Tool.PtMeasPar.HeadTouch(0)").complete()
      await self.client.GoTo((orig + float3(0,0,100)).ToXYZString()).complete()
      await self.client.GoTo((orig + float3(0,0,10)).ToXYZString()).complete()

      fixture_top_face = self.add_feature('fixture_top_face', Stages.PROBE_TOP_PLANE)

      ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % ((orig).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      fixture_top_face.addPoint(*pt)

      ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % ((orig + float3(-25,-5,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      fixture_top_face.addPoint(*pt)

      ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % ((orig + float3(-25,5,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      fixture_top_face.addPoint(*pt)

      ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % ((orig + float3(25,5,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      fixture_top_face.addPoint(*pt)
      
      ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % ((orig + float3(25,-5,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      fixture_top_face.addPoint(*pt)
      return True
    except Exception as ex:
      logger.error("probe_top_plane exception %s" % str(ex))
      raise ex

  async def probe_fixture_ball(self, y_pos_v2, feature):
    try:
      # orig = waypoints['fixture_ball'] + float3(0,0,-(y_pos_v2 - 63.5))
      orig = float3(self.fitted_features[FEAT_FIXTURE_SPHERE]['pos']) + float3(0,0,-(y_pos_v2 - 63.5))
      contact_radius = (FIXTURE_BALL_DIA+PROBE_DIA)/2
      a_angle_probe_contact = math.atan2(contact_radius,TOOL_3_LENGTH)*180/math.pi

      await self.client.GoTo("Tool.Alignment(0,0,1,1,0,0)").complete()
      await self.client.GoTo((orig + float3(0,0,25)).ToXYZString()).complete()
      getCurrPosCmd = await self.client.Get("X(),Y(),Z()").complete()
      currPos = readPointData(getCurrPosCmd.data_list[0])

      #place tip pos +Y from target
      await self.client.GoTo("Tool.A(%s),Tool.B(90)" % (a_angle_probe_contact+5)).complete()
      await self.client.GoTo("Z(%s)" % (currPos.z - 25)).complete()
      await self.client.SetProp("Tool.PtMeasPar.HeadTouch(1)").complete()
      await self.client.SetProp("Tool.PtMeasPar.Search(6)").complete()

      # probe in -Y dir
      ptMeas = await self.client.PtMeas("%s,IJK(0,1,0)" % ((orig + float3(0,contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature.addPoint(*pt)

      # move to -X pos, probe in +X dir
      await self.client.GoTo("Tool.A(%s),Tool.B(180)" % (a_angle_probe_contact+5)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(-1,0,0)" % ((orig + float3(-contact_radius,0,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature.addPoint(*pt)

      # move to -Y pos, probe in +Y dir
      await self.client.GoTo("Tool.A(%s),Tool.B(270)" % (a_angle_probe_contact+5)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(0,-1,0)" % ((orig + float3(0,-contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature.addPoint(*pt) 

      #rise 2mm and probe another 3 points
      measPos2 = orig + float3(0,0,2)
      getCurrPosCmd = await self.client.Get("X(),Y(),Z()").complete()
      currPos = readPointData(getCurrPosCmd.data_list[0])
      # await self.client.GoTo((currPos + float3(0,0,+2)).ToXYZString()).complete()
      await self.client.GoTo("Tool.A(%s),Tool.B(270)" % (a_angle_probe_contact+5)).complete()
      await self.client.GoTo("Z(%s)" % (currPos.z + 2)).complete()

      # probe in +Y dir
      await self.client.GoTo("Tool.A(%s),Tool.B(270)" % (a_angle_probe_contact+5)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(0,-1,0)" % ((measPos2 + float3(0,-contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature.addPoint(*pt)

      # move to -X pos, probe in +X dir
      await self.client.GoTo("Tool.A(%s),Tool.B(180)" % (a_angle_probe_contact+5)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(-1,0,0)" % ((measPos2 + float3(-contact_radius,0,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature.addPoint(*pt) 

      # move to +Y pos, probe in -Y dir
      await self.client.GoTo("Tool.A(%s),Tool.B(90)" % (a_angle_probe_contact+5)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(0,1,0)" % ((measPos2 + float3(0,contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature.addPoint(*pt) 

      await self.client.GoTo((orig + float3(0,0,50)).ToXYZString()).complete()
      return True
    except Exception as ex:
      print("exception %s" % str(ex))
      raise ex


  async def probe_spindle_tip(self, x_pos_v2, z_pos_v2, feature):
    '''
    Probe against the ball mounted in the spindle
    Probe should be aligned vertical (or close) before running
    The first move is to the position 25mm above the target
    '''
    try:
      # orig = waypoints['z_home'] + float3(z_pos_v2,x_pos_v2 - 63.5,0)
      orig = float3(self.fitted_features[FEAT_SPINDLE_POS_SPHERE]['pos']) + float3(z_pos_v2,x_pos_v2,0)
      contact_radius = (Z_BALL_DIA+PROBE_DIA)/2
      a_angle_probe_contact = math.atan2(contact_radius,TOOL_3_LENGTH)*180/math.pi

      #orient tool down and place tip 25 mm above target
      await self.client.GoTo("Tool.Alignment(0,0,1,1,0,0)").complete()
      await self.client.GoTo((orig + float3(0,0,25)).ToXYZString()).complete()
      getCurrPosCmd = await self.client.Get("X(),Y(),Z()").complete()
      currPos = readPointData(getCurrPosCmd.data_list[0])
      #place tip pos +Y from target
      await self.client.GoTo("Tool.A(%s),Tool.B(90)" % (a_angle_probe_contact+5)).complete()
      await self.client.GoTo("Z(%s)" % (currPos.z - 25)).complete()
      
      await self.client.SetProp("Tool.PtMeasPar.HeadTouch(1)").complete()
      await self.client.SetProp("Tool.PtMeasPar.Search(6)").complete()

      # probe in -Y dir
      ptMeas = await self.client.PtMeas("%s,IJK(0,1,0)" % ((orig + float3(0,contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature.addPoint(*pt)

      # move to -X pos, probe in +X dir
      await self.client.GoTo("Tool.A(%s),Tool.B(180)" % (a_angle_probe_contact+5)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(-1,0,0)" % ((orig + float3(-contact_radius,0,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature.addPoint(*pt)

      # move to -Y pos, probe in +Y dir
      await self.client.GoTo("Tool.A(%s),Tool.B(270)" % (a_angle_probe_contact+5)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(0,-1,0)" % ((orig + float3(0,-contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature.addPoint(*pt) 

      #rise 2mm and probe another 3 points
      measPos2 = orig + float3(0,0,2)
      getCurrPosCmd = await self.client.Get("X(),Y(),Z()").complete()
      currPos = readPointData(getCurrPosCmd.data_list[0])
      # await self.client.GoTo((currPos + float3(0,0,+2)).ToXYZString()).complete()
      await self.client.GoTo("Tool.A(%s),Tool.B(270)" % (a_angle_probe_contact+5)).complete()
      await self.client.GoTo("Z(%s)" % (currPos.z + 2)).complete()

      # probe in +Y dir
      await self.client.GoTo("Tool.A(%s),Tool.B(270)" % (a_angle_probe_contact+5)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(0,-1,0)" % ((measPos2 + float3(0,-contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature.addPoint(*pt)

      # move to -X pos, probe in +X dir
      await self.client.GoTo("Tool.A(%s),Tool.B(180)" % (a_angle_probe_contact+5)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(-1,0,0)" % ((measPos2 + float3(-contact_radius,0,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature.addPoint(*pt) 

      # move to +Y pos, probe in -Y dir
      await self.client.GoTo("Tool.A(%s),Tool.B(90)" % (a_angle_probe_contact+5)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(0,1,0)" % ((measPos2 + float3(0,contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature.addPoint(*pt) 

      await self.client.GoTo((orig + float3(0,0,50)).ToXYZString()).complete()
    except Exception as ex:
      msg = "Exception in probe_spindle_tip %s" % str(ex)
      logger.debug(msg)
      return err_msg(msg)


  async def probe_x(self, x_pos_v2, z_pos_v2):
    feat_name = "x_%+.4f" % x_pos_v2
    self.x_calib_probes.append(feat_name)
    feature_x_pos = self.add_feature(feat_name, Stages.CHARACTERIZE_X)
    try:
      await self.probe_spindle_tip(x_pos_v2, z_pos_v2, feature_x_pos)
      return True
    except Exception as ex:
      logger.error("exception %s" % str(ex))
      raise ex

  async def probe_x_home(self, x_pos_v2, z_pos_v2):
    '''
    Find the X axis position after homing
    '''
    x_home_count = len(self.x_home_probes)
    feat_name = "home_x_%d" % (x_home_count + 1)
    self.x_home_probes.append(feat_name)
    feat_x_home = self.add_feature(feat_name, Stages.CHARACTERIZE_X)
    try:
      await self.probe_spindle_tip(x_pos_v2, z_pos_v2, feat_x_home)
      return True
    except Exception as ex:
      logger.error("exception %s" % str(ex))
      raise ex

  async def verify_x_home(self):
    result = {}
    home_x_positions = []
    for x_feat_name in self.x_home_probes:
      fid = self.feature_ids[x_feat_name]
      x_feat = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
      (rad, pos) = x_feat.sphere()
      if abs(rad*2 - (Z_BALL_DIA+PROBE_DIA)) > 0.1:
        raise CalibException("Deviation in best-fit spindle-tip sphere. Diameter %s" % (radius*2))
      home_x_positions.append(pos)
    avg_home_x_pos = np.mean(home_x_positions, axis=0)
    max_dist = 0
    for idx, home_x_pos in enumerate(home_x_positions):
      other_positions = home_x_positions[:idx] + home_x_positions[idx+1:]
      for pos in other_positions:
        dist = np.linalg.norm( home_x_pos - pos )
        if dist > max_dist:
          max_dist = dist
    result['max_diff'] = max_dist
    result['avg_pos'] = avg_home_x_pos.tolist()
    self.status['x_home'] = result
    self.spec['x_homing_repeatability']['val'] = max_dist
    if max_dist > LINEAR_HOMING_REPEATABILITY:
      self.spec['x_homing_repeatability']['pass'] = False
      self.status['spec_failure'] = True
      # self.zmq_report('FAILURE', stage=Stages.CHARACTERIZE_X, step=Steps.VERIFY_X_HOME)
      # raise CalibException("FAIL: x homing variation exceeded spec: %s" % max_dist)
      return False
    else:
      self.spec['x_homing_repeatability']['pass'] = True
      return True

  async def probe_y(self, y_pos_v2):
    feat_name = "y_%+.4f" % y_pos_v2
    self.y_calib_probes.append(feat_name)
    feature = self.add_feature(feat_name, Stages.CHARACTERIZE_Y)
    try:
      await self.probe_fixture_ball(y_pos_v2, feature)
      return True
    except Exception as ex:
      logger.error("exception %s" % str(ex))
      raise ex


  async def probe_y_home(self, y_pos_v2):
    '''
    Find the Y axis position after homing
    '''
    y_home_count = len(self.y_home_probes)
    feat_name = "home_y_%d" % (y_home_count + 1)
    self.y_home_probes.append(feat_name)
    feat_y_home = self.add_feature(feat_name, Stages.CHARACTERIZE_Y)
    try:
      await self.probe_fixture_ball(y_pos_v2, feat_y_home)
      return True
    except Exception as ex:
      print("exception %s" % str(ex))
      raise ex

  async def verify_y_home(self):
    result = {}
    home_positions = []
    for feat_name in self.y_home_probes:
      fid = self.feature_ids[feat_name]
      feat = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
      (rad, pos) = feat.sphere()
      if abs(rad*2 - (FIXTURE_BALL_DIA+PROBE_DIA)) > 0.1:
        raise CalibException("Deviation in best-fit fixture sphere. Diameter %s" % (radius*2))
      home_positions.append(pos)
    avg_home_pos = np.mean(home_positions, axis=0)
    max_dist = 0
    for idx, home_pos in enumerate(home_positions):
      other_positions = home_positions[:idx] + home_positions[idx+1:]
      for pos in other_positions:
        dist = np.linalg.norm( home_pos - pos )
        if dist > max_dist:
          max_dist = dist
    result['max_diff'] = max_dist
    result['avg_pos'] = avg_home_pos.tolist()
    self.status['y_home'] = result
    self.spec['y_homing_repeatability']['val'] = max_dist
    if max_dist > LINEAR_HOMING_REPEATABILITY:
      self.spec['y_homing_repeatability']['pass'] = False
      self.status['spec_failure'] = True
      return False
      # raise CalibException("FAIL: Y homing variation exceeded spec: %s" % max_dist)
    else:
      self.spec['y_homing_repeatability']['pass'] = True
      return True


  async def probe_z(self, x_pos_v2, z_pos_v2):
    '''
    Probe against the ball mounted in the spindle
    Probe should be aligned vertical (or close) before running
    The initial movement orients the tool downwards and then moves the position to 25mm above the target
    '''
    # orig = waypoints['z_home'] + float3(z_pos_v2,x_pos_v2 - 63.5,0)
    # orig = self.fitted_features[FEAT_SPINDLE_POS_SPHERE]['pos'] + float3(z_pos_v2,x_pos_v2 - 63.5,0)
    feat_name = "z_%+.4f" % z_pos_v2
    self.z_calib_probes.append(feat_name)
    feature_z_pos = self.add_feature(feat_name, Stages.CHARACTERIZE_Z)
    try:
      await self.probe_spindle_tip(x_pos_v2, z_pos_v2, feature_z_pos)
      return True
    except Exception as ex:
      print("exception %s" % str(ex))
      raise ex

  async def probe_z_home(self, x_pos_v2, z_pos_v2):
    '''
    Find the Z axis position after homing
    '''
    z_home_count = len(self.z_home_probes)
    feat_name = "home_z_%d" % (z_home_count + 1)
    self.z_home_probes.append(feat_name)
    feat_z_home = self.add_feature(feat_name, Stages.CHARACTERIZE_Z)
    try:
      await self.probe_spindle_tip(x_pos_v2, z_pos_v2, feat_z_home)
      return True
    except Exception as ex:
      print("exception %s" % str(ex))
      raise ex

  async def verify_z_home(self):
    result = {}
    home_positions = []
    for feat_name in self.z_home_probes:
      fid = self.feature_ids[feat_name]
      feat = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
      (rad, pos) = feat.sphere()
      if abs(rad*2 - (FIXTURE_BALL_DIA+PROBE_DIA)) > 0.1:
        raise CalibException("Deviation in best-fit fixture sphere. Diameter %s" % (radius*2))
      home_positions.append(pos)
    avg_home_pos = np.mean(home_positions, axis=0)
    max_dist = 0
    for idx, home_pos in enumerate(home_positions):
      other_positions = home_positions[:idx] + home_positions[idx+1:]
      for pos in other_positions:
        dist = np.linalg.norm( home_pos - pos )
        if dist > max_dist:
          max_dist = dist
    result['max_diff'] = max_dist
    result['avg_pos'] = avg_home_pos.tolist()
    self.status['z_home'] = result
    self.spec['z_homing_repeatability']['val'] = max_dist
    if max_dist > LINEAR_HOMING_REPEATABILITY:
      self.spec['z_homing_repeatability']['pass'] = False
      self.status['spec_failure'] = True
      return False
    else:
      self.spec['z_homing_repeatability']['pass'] = True
      return True


  async def probe_a(self, feat_name, y_pos_v2, a_pos_v2):
    '''
    Find start position and driving direction from angle
    '''
    a_cor = waypoints['a_rot_cent_approx'] + float3(0,0,0)
    a_cor = a_cor + float3(0,0,(-y_pos_v2 - 63.5))
    vec_a_cor_to_orig_fixture_tip = (waypoints['probe_fixture_tip']+float3(0,0,(63.5 - y_pos_v2))) - (waypoints['a_rot_cent_approx'] + float3(0,0,0))
    fixture_offset_angle = 0
    fixture_length = 50.8
    vec_cor_to_orig_start = vec_a_cor_to_orig_fixture_tip + float3(0,0,-2) # down a bit away from edge
    dist_cor_to_start = math.sqrt(math.pow(vec_cor_to_orig_start.x,2) + math.pow(vec_cor_to_orig_start.z,2)) + 2
    angle_offset_start = math.atan2(vec_cor_to_orig_start.z,vec_cor_to_orig_start.x)*180/math.pi
    if not self.is_started_a:
      await self.client.SetProp("Tool.PtMeasPar.Search(12)").ack()
      await self.client.GoTo((a_cor + float3(0, -100, 150)).ToXYZString()).complete()
      await self.client.GoTo("Tool.Alignment(0,-1,0)").complete()
      await self.client.GoTo((a_cor + float3(0, -200, 0)).ToXYZString()).complete()
      self.is_started_a = True

    '''
    Do a head-probe line against 1 face of the probing fixture
    '''
    try:
      logger.debug("Finding A position %s" % a_pos_v2)
      stage = None
      if "find_a" in feat_name:
        stage = Stages.CHARACTERIZE_A
      elif "probe_a" in feat_name:
        stage = Stages.CHARACTERIZE_A
        self.a_calib_probes.append(feat_name)
      elif "verify_a" in feat_name:
        stage = Stages.VERIFY_A
        self.a_verify_probes.append(feat_name)
      a_line = self.add_feature(feat_name, stage)

      total_angle =  fixture_offset_angle - a_pos_v2
      angle_start_pos = total_angle + angle_offset_start
      logger.debug('vec_cor_to_orig_start %s' % vec_cor_to_orig_start)
      logger.debug('angle_offset_start %s' % angle_offset_start)
      logger.debug('start origin Rad %s' % dist_cor_to_start)
      logger.debug('start pos angle %s' % angle_start_pos)
      
      start_pos = a_cor + float3(dist_cor_to_start * math.cos(angle_start_pos*math.pi/180), vec_a_cor_to_orig_fixture_tip.y,dist_cor_to_start * math.sin(angle_start_pos*math.pi/180))
      logger.debug('start pos %s' % start_pos)
      
      drive_angle = total_angle - 90
      drive_vec = float3(math.cos(drive_angle*math.pi/180), 0, math.sin(drive_angle*math.pi/180))
      face_norm = float3(-drive_vec.z, 0, drive_vec.x)
      logger.debug("drive_vec %s" % drive_vec)
      try:
        #there are issues, the yz routine assumes the CNC is aligned so that its X-axis is parallel with the CMM MachineCsy X-axis
        if self.config['table_slot'] == "front_right":
          points = await routines.headprobe_line_yz(self.client,start_pos,drive_vec,B_LINE_LENGTH,face_norm,3,1)
        else:
          points = await routines.headprobe_line_xz(self.client,start_pos,drive_vec,B_LINE_LENGTH,face_norm,3,1)
        for pt in points:
          a_line.addPoint(*pt)
      except CmmException as e:
        pass

      end_pos = start_pos + drive_vec * B_LINE_LENGTH
      retract_pos = end_pos + face_norm * 20

      # face_norm_xz_angle_rad = math.atan2(face_norm.x, face_norm.z)
      # retract_a_frac = math.cos(face_norm_xz_angle_rad)
      # retract_b_frac = math.sin(face_norm_xz_angle_rad)
      # retract_a_angle_deg = 90 - 0.1*(90*retract_a_frac)
      # retract_a_angle_deg = min(115, retract_a_angle_deg)
      # retract_b_angle_deg = 180 - 30 * (1 - retract_b_frac)

      # await self.client.GoTo((start_pos + 5*float3(-drive_vec.z, 0, drive_vec.x)).ToXYZString()).complete()
      # await self.client.GoTo("Tool.A(%s),Tool.B(%s)" % (retract_a_angle_deg, retract_b_angle_deg)).complete()
      await self.client.GoTo((retract_pos).ToXYZString()).complete()

      return True
    except Exception as ex:
      logger.error("probe_a exception %s" % str(ex))
      raise ex

  async def probe_a_home(self, y_pos_v2, a_pos_v2):
    '''
    Find the A axis position after homing
    '''
    try:
      await self.find_pos_a(y_pos_v2, a_pos_v2, is_homing_check=True)
      return True
    except Exception as ex:
      logger.error("probe_a_home exception %s" % str(ex))
      raise ex

  async def verify_a_home(self):
    result = {}
    result['min'] = min(self.a_home_values)
    result['max'] = max(self.a_home_values)
    result['avg'] = np.mean(self.a_home_values).tolist()
    self.status['a_home'] = result
    diff = result['max'] - result['min']
    self.spec['a_homing_repeatability']['val'] = diff
    if diff > A_HOMING_REPEATABILITY:
      self.spec['a_homing_repeatability']['pass'] = False
      self.status['spec_failure'] = True
      return False
    else:
      self.spec['a_homing_repeatability']['pass'] = True
      return True

  async def verify_a_homing(self):
    """
    Verify A homing repeatability and position.
    Repeatability means range from max to min, spec <0.08 (for A-axis, B-axis is <0.04)
    Position is defined relative to spindle plunge (Z-axis) direction, spec <0.05
    """
    try:
      self.status['a_homing'] = {}
      min_a = min(self.a_home_values)
      max_a = max(self.a_home_values)
      diff = max_a - min_a
      max_err = max(abs(max_a), abs(min_a))
      self.status['a_homing']['repeatability'] = diff
      self.status['a_homing']['max_err'] = max_err
      if diff > A_HOMING_REPEATABILITY or max_err > 0.5*A_HOMING_REPEATABILITY:
        self.status['a_homing']['pass'] = False
        return False
      else:
        self.status['a_homing']['pass'] = True
        return True
    except Exception as ex:
      logger.error("verify_a_homing exception %s" % str(ex))
      raise ex

  async def probe_b(self, feat_name, y_pos_v2, b_pos_v2):
    '''
    Do a head-probe line against 1 face of the probing fixture
    The fixture face being probed is on the upper rectangle, opposite from the peak of the vertical fin
    '''
    pos_bcor = waypoints['b_rot_cent_approx']
    pos_bcor = pos_bcor + float3(0,0,(-y_pos_v2 - 63.5))
    fixtureOffsetAngle = FIXTURE_OFFSET_B_ANGLE
    fixture_length = 76.2
    #calculate some offsets 
    #we'll use the nominal b-45 start-pos for reference
    #(at b-45, the fixture face is approx. parallel to PartCsy Y axis)
    vec_bcor_to_startpos45 = float3(-0.5*fixture_length,0.5*fixture_length-20,0)
    dist_bcor_to_startpos45 = math.sqrt(math.pow(vec_bcor_to_startpos45.x,2) + math.pow(vec_bcor_to_startpos45.y,2))
    ang_bcor_to_startpos45 = math.atan2(vec_bcor_to_startpos45.y,vec_bcor_to_startpos45.x)*180/math.pi
    # start_posOffsetAngle = math.atan2(vec_bcor_to_startpos45.y,vec_bcor_to_startpos45.x)*180/math.pi
    try:
      logger.debug("Finding B position")
      # feat_name = "b-%d" % b_pos_v2
      stage = None
      if "find_b" in feat_name:
        stage = Stages.CHARACTERIZE_B
      elif "probe_b" in feat_name:
        stage = Stages.CHARACTERIZE_B
        self.b_calib_probes.append(feat_name)
      elif 'verify_b' in feat_name:
        stage = Stages.VERIFY_B
        self.b_verify_probes.append(feat_name)
      b_line = self.add_feature(feat_name, stage)

      ang_bcor_to_startpos = (b_pos_v2 - 45) + ang_bcor_to_startpos45
      logger.debug('ang_bcor_to_startpos %s' % ang_bcor_to_startpos)
      ang_fixture_face = b_pos_v2 + FIXTURE_OFFSET_B_ANGLE
      logger.debug('ang_fixture_face %s' % ang_fixture_face)

      fixture_face_angle = b_pos_v2 + FIXTURE_OFFSET_B_ANGLE
      totAngle = b_pos_v2 + fixtureOffsetAngle
      # start_posAngle = totAngle + start_posOffsetAngle
      # print('start_posOriginVec %s' % start_posOriginVec)
      # print('start_posOffsetAngle %s' % start_posOffsetAngle)
      # print('start origin Rad %s' % start_posOriginRad)
      # print('start pos angle %s' % start_posAngle)
      start_pos = pos_bcor + float3(dist_bcor_to_startpos45 * math.cos(ang_bcor_to_startpos*math.pi/180), dist_bcor_to_startpos45 * math.sin(ang_bcor_to_startpos*math.pi/180),-6)
      # start_pos = orig + float3(start_posOriginRad * math.cos(start_posAngle*math.pi/180), start_posOriginRad * math.sin(start_posAngle*math.pi/180),-6)
      await self.client.SetProp("Tool.PtMeasPar.Search(15)").complete()
      if not self.is_started_b:
        await self.client.SetProp("Tool.PtMeasPar.HeadTouch(1)").complete()
        await self.client.GoTo("Tool.A(0)").complete()
        await self.client.GoTo((start_pos + float3(0, 0, 25)).ToXYZString()).complete()
        self.is_started_b = True
      logger.debug('start pos %s' % start_pos)
      # await self.client.GoTo("Tool.Alignment(0,0,1)").complete()
      # await self.client.GoTo((orig + float3(0, 0, 100)).ToXYZString()).complete()
      # await self.client.GoTo((start_pos + float3(0, 0, 15)).ToXYZString()).complete()

      lineAngle = totAngle + 90
      lineVec = float3(math.cos(ang_fixture_face*math.pi/180), math.sin(ang_fixture_face*math.pi/180),0)
      # lineVec = float3(math.cos(lineAngle*math.pi/180), math.sin(lineAngle*math.pi/180),0)
      logger.debug("lineVec %s" % lineVec)

      # points = await routines.probe_line(self.client,start_pos, lineVec, float3(lineVec.y, -lineVec.x,0),65,10,3,1)
      try:
        points = await routines.headprobe_line(self.client,start_pos, lineVec, 35, 15, 3, 10, -1, 5)
      except CmmException as e:
        logger.error("CmmException in probe_b, raising")
        raise e
      # await routines.headprobe_line(client,start_pos,float3(-1,0,0),75,10,5,10,-1,15)

      for pt in points:
        b_line.addPoint(*pt)

      # await self.client.GoTo("Z(%s)" % (orig.z + 15)).ack()

      return True
    except Exception as ex:
      logger.error("probe_b exception %s" % str(ex))
      raise ex

  async def probe_b_home(self, y_pos_v2, b_pos_v2):
    '''
    Find the B axis position after homing
    '''
    try:
      await self.find_pos_b(y_pos_v2, b_pos_v2, is_homing_check=True)
      return True
    except Exception as ex:
      logger.error("probe_b_home exception %s" % str(ex))
      raise ex

  async def verify_b_home(self):
    result = {}
    result['min'] = min(self.b_home_values)
    result['max'] = max(self.b_home_values)
    result['avg'] = np.mean(self.b_home_values).tolist()
    # self.add_state('b_home', result, Stages.HOMING_B)
    self.status['b_home'] = result

    diff = result['max'] - result['min']
    self.spec['b_homing_repeatability']['val'] = diff
    if diff > B_HOMING_REPEATABILITY:
      self.spec['b_homing_repeatability']['pass'] = False
      self.status['spec_failure'] = True
      return False
    else:
      self.spec['b_homing_repeatability']['pass'] = True
      return True

  async def verify_b_homing(self):
    """
    Verify B homing repeatability and position.
    Repeatability means range from max to min, spec <0.04
    Position is defined relative to spindle plunge (Z-axis) direction, spec <0.05
    """
    try:
      self.status['b_homing'] = {}
      min_b = min(self.b_home_values)
      max_b = max(self.b_home_values)
      diff = max_b - min_b
      max_err = max(abs(max_b), abs(min_b))
      self.status['b_homing']['repeatability'] = diff
      self.status['b_homing']['max_err'] = max_err
      if diff > B_HOMING_REPEATABILITY or max_err > 0.5*B_HOMING_REPEATABILITY:
        self.status['b_homing']['pass'] = False
        return False
      else:
        self.status['b_homing']['pass'] = True
        return True
    except Exception as ex:
      logger.error("verify_b_homing exception %s" % str(ex))
      raise ex


  async def find_pos_a(self, y_nominal, a_nominal, is_homing_check=False):
    '''
    Probe A
    Project points onto POCKET coord sys YZ plane (defined by X-axis dir)
    Translate projected points into POCKET space
    Compare angular position to Z-norm
    '''
    try:
      if is_homing_check:
        a_home_count = len(self.a_home_probes)
        probeline_name = "home_a_%d" % (a_home_count + 1)
        self.a_home_probes.append(probeline_name)
        self.add_state('a_home_probes', self.a_home_probes, Stages.CHARACTERIZE_A)
      else:
        probeline_name = "find_a_%+.6f" % a_nominal
      await self.probe_a(probeline_name, y_nominal, a_nominal)
    except Exception as ex:
      logger.error("find_pos_a exception (while probing): %s" % str(ex))
      raise ex

    #translate points into POCKET coord sys
    #find best-fit line through points
    #find angle in YZ plane compared to Z-norm
    try:
      #project points onto planed defined by CNC CSY X-axis
      probeline_id = self.feature_ids[probeline_name]
      a_line = self.metrologyManager.getActiveFeatureSet().getFeature(probeline_id)
      proj_probeline_name = 'find_a_%+.6f_proj' % a_nominal
      a_line_proj = self.add_feature(proj_probeline_name, Stages.CHARACTERIZE_A)
      for pt in a_line.points():
        #using the top plane point seems a bit strange because its not related to the YZ plane
        #but it should be fine, we just need to get the line on a plane to measure the angle
        plane_orig_to_pt = pt - self.cnc_csy.orig
        dist = np.dot(plane_orig_to_pt, self.cnc_csy.x_dir)
        proj_pt = pt - dist * self.cnc_csy.x_dir
        a_line_proj.addPoint(proj_pt[0], proj_pt[1], proj_pt[2])
      
      a_line_proj_line = a_line_proj.line()
      a_proj_line_dir = a_line_proj_line[1]
      # A-line dirs around A0 should have a positive Z-component in PartCsy
      if a_proj_line_dir[2] < 0:
        a_proj_line_dir = -1*a_proj_line_dir

      self.fitted_features[proj_probeline_name] = {'pt': a_line_proj_line[0], 'dir': a_proj_line_dir}
      a_line_translated_dir = np.matmul(self.cmm2cnc, np.append(a_proj_line_dir, 0))
      logger.debug('a_line_translated_dir')
      logger.debug(a_line_translated_dir)
      # z_norm_2d_xz = [self.z_norm[0], self.z_norm[2]]
      #don't use z_norm, that is in the PartCsy. We are in the ideal CNC Csy
      a_dir_2d_yz = [a_line_translated_dir[1], a_line_translated_dir[2]]
      # a_pos_rel_z = angle_between_ccw_2d(z_norm_2d_xz, a_dir_2d_yz)
      
      a_pos_rel_z = angle_between_ccw_2d([1,0], a_dir_2d_yz)
      # if a_line_translated_dir[1] >= 0:
      # else:
      #   a_pos_rel_z = angle_between_ccw_2d([-1,0], a_dir_2d_yz)
      
      a_pos = a_pos_rel_z

      logger.debug("found A pos %s" % a_pos)
      if is_homing_check:
        self.status['a_home_err'] = a_pos
        self.a_home_values.append(a_pos)
        self.add_state('a_home_values', self.a_home_values, Stages.CHARACTERIZE_A)
      else:
        self.add_state('feat_name_last_find_a_proj', proj_probeline_name, Stages.CHARACTERIZE_A)
      return a_pos
    except Exception as ex:
      logger.error("find_pos_a exception (while calculating A position): %s" % str(ex))
      raise ex


  async def find_pos_b(self, y_nominal, b_nominal, is_homing_check=False):
    '''
    Probe B
    Project points onto plane of rotation defined by top surface of fixture
    Translate projected points into POCKET space
    Compare angular position to Z-norm
    '''
    try:
      if is_homing_check:
        b_home_count = len(self.b_home_probes)
        probeline_name = "home_b_%d" % (b_home_count + 1)
        self.b_home_probes.append(probeline_name)
        self.add_state('b_home_probes', self.b_home_probes, Stages.CHARACTERIZE_B)
      else:
        probeline_name = "find_b_%+.6f" % b_nominal
      await self.probe_b(probeline_name, y_nominal, b_nominal)
    except Exception as ex:
      logger.error("find_pos_b exception (while probing): %s" % str(ex))
      raise ex

    #translate projected points into POCKET coord sys
    #find best-fit line through projected points
    #find angle in XZ plane compared to Z-norm
    try:
      fid = self.feature_ids[probeline_name]
      b_line = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
      proj_probeline_name = 'find_b_%+.6f_proj' % b_nominal
      b_line_proj = self.add_feature(proj_probeline_name, Stages.CHARACTERIZE_B)
      for pt in b_line.points():
        plane_orig_to_pt = pt - self.cnc_csy.orig
        dist = np.dot(plane_orig_to_pt, self.cnc_csy.y_dir)
        proj_pt = pt - dist * self.cnc_csy.y_dir
        # proj_pt_in_plane = np.matmul(rot_mat,np.append(proj_pt,1))
        b_line_proj.addPoint(proj_pt[0], proj_pt[1], proj_pt[2])
      
      find_b_proj_line = b_line_proj.line()
      vec_find_b_proj = find_b_proj_line[1]
      """make sure the best-fit line is in correct direction
      In the PartCsy, the X- and Y- components should be negative when probing near B0
      """
      if vec_find_b_proj[0] > 0:
        vec_find_b_proj = -1 * vec_find_b_proj
      self.fitted_features[proj_probeline_name] = {'pt': find_b_proj_line[0], 'dir': vec_find_b_proj}
      b_line_translated = np.matmul(self.cmm2cnc, np.append(vec_find_b_proj, 0))
      logger.debug('b_line_translated')
      logger.debug(b_line_translated)

      b_dir_2d = [b_line_translated[0],b_line_translated[2]]
      b_pos_rel_z = angle_between_ccw_2d(b_dir_2d, [0,1])
      logger.debug("b_pos_rel_z %s" % b_pos_rel_z)
      b_pos = b_pos_rel_z - OFFSET_B_POS_REL_Z
      
      logger.debug("found B pos %s" % b_pos)
      if is_homing_check:
        self.status['b_home_err'] = b_pos
        self.b_home_values.append(b_pos)
        self.add_state('b_home_values', self.b_home_values, Stages.CHARACTERIZE_B)
      else:
        self.add_state('feat_name_last_find_b_proj', proj_probeline_name, Stages.CHARACTERIZE_B)
      return b_pos
    except Exception as ex:
      logger.error("find_pos_b exception (while calculating B position): %s" % str(ex))
      raise ex


  async def set_tool_probe_z(self, tool_probe_z):
    self.tool_probe_z = tool_probe_z


  async def setup_cnc_csy(self):
    '''
    define CNC coord sys from TOP_PLANE and Z_VEC
    '''
    z_line_partcsy_dir = None
    z_vec_proj = None 
    z_vec_cross_top_plane_norm = None
    rot_mat = None
    pocket_yz_plane_norm = None
    # try:
    #   with open('/sysroot/home/pocketnc/results', 'w') as f:
    #     f.write('huh\n')
    # except Exception as e:
    #   print(e)

    fixture_top_face = self.metrologyManager.getActiveFeatureSet().getFeature( self.feature_ids['fixture_top_face'] )
    plane_fixture_top_face = fixture_top_face.plane()
    top_plane_pt = plane_fixture_top_face[0]
    top_plane_norm = plane_fixture_top_face[1]
    self.add_fitted_feature('fixture_top_face', {'pt': top_plane_pt, 'norm': top_plane_norm}, Stages.SETUP_CNC_CSY)

    '''
    Use Z positions to define a line
    '''
    try:
      z_line = self.add_feature('z_line', Stages.SETUP_CNC_CSY)
      z_line_proj = self.add_feature('z_line_proj', Stages.SETUP_CNC_CSY)
      print(self.z_calib_probes)
      for z_probe_feat_name in self.z_calib_probes:
        # fid = self.feature_ids['z_%+.4f' % i]
        fid = self.feature_ids[z_probe_feat_name]
        z_pos_feat = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
        print(z_pos_feat.points())
        z_pos_sphere = z_pos_feat.sphere()
        z_pos = z_pos_sphere[1]
        z_line.addPoint(*z_pos)
        #project point onto top-fixture-plane
        plane_orig_to_pt = z_pos - top_plane_pt
        dist = np.dot(plane_orig_to_pt, top_plane_norm)
        proj_pt = z_pos - dist * top_plane_norm
        z_line_proj.addPoint(*proj_pt)

      (z_point_real, dir_z_axis_partcsy) = z_line.line()
      #in PartCSY, the X component of Z-axis dir should be positive 
      #(PartCsy X-axis is near-parallel to CncCsy Z-axis)
      if dir_z_axis_partcsy[0] < 0:
        dir_z_axis_partcsy = -1*dir_z_axis_partcsy
      self.add_fitted_feature('z_line_real', {'pt': z_point_real, 'norm': dir_z_axis_partcsy}, Stages.SETUP_CNC_CSY)
    except Exception as ex:
      logger.error("setup_cnc_csy exception (while defining z line): %s" % str(ex))
      raise ex

    '''
    Use Y positions to define a line
    '''
    try:
      print('y')

      y_line = self.add_feature('y_line', Stages.SETUP_CNC_CSY)
      y_line_proj = self.add_feature('y_line_proj', Stages.SETUP_CNC_CSY)
      # for i in range(Y_MAX,Y_MIN,Y_STEP):
      for y_probe_feat_name in self.y_calib_probes:
        # fid = self.feature_ids['y_%+.4f' % i]
        fid = self.feature_ids[y_probe_feat_name]
        y_pos_feat = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
        y_pos_sphere = y_pos_feat.sphere()
        y_pos = y_pos_sphere[1]
        y_line.addPoint(*y_pos)

      (pos_y_line, dir_y_axis_partcsy) = y_line.line()
      #in PartCSY, the Z component of Y-axis dir should be positive (PartCsy Z-axis is near-parallel to CncCsy Y-axis)
      if dir_y_axis_partcsy[2] < 0:
        dir_y_axis_partcsy = -1*dir_y_axis_partcsy
      self.add_fitted_feature('y_line_real', {'pt': pos_y_line, 'norm': dir_y_axis_partcsy}, Stages.SETUP_CNC_CSY)
    except Exception as ex:
      logger.error("setup_cnc_csy exception (while defining y line): %s" % str(ex))
      raise ex

    '''
    Use X positions to define a line
    '''
    try:
      print('x')

      x_line = self.add_feature('x_line', Stages.SETUP_CNC_CSY)
      x_line_proj = self.add_feature('x_line_proj', Stages.SETUP_CNC_CSY)
      for x_probe_feat_name in self.x_calib_probes:
        fid = self.feature_ids[x_probe_feat_name]
        x_pos_feat = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
        x_pos_sphere = x_pos_feat.sphere()
        x_pos = x_pos_sphere[1]
        x_line.addPoint(*x_pos)

      (pos_x_line, dir_x_axis_partcsy) = x_line.line()
      #in PartCSY, the Y component of X-axis dir should be positive (PartCsy Y-axis is near-parallel to CncCsy X-axis)
      if dir_x_axis_partcsy[1] < 0:
        dir_x_axis_partcsy = -1*dir_x_axis_partcsy
      self.add_fitted_feature('x_line_real', {'pt': pos_x_line, 'norm': dir_x_axis_partcsy}, Stages.SETUP_CNC_CSY)
    except Exception as ex:
      logger.error("setup_cnc_csy exception (while defining x line): %s" % str(ex))
      raise ex

    #use results to create coord sys and tranformation matrices
    try:
      print('results')

      # y_norm = dir_y_axis_partcsy
      dir_x = np.cross(dir_y_axis_partcsy, dir_z_axis_partcsy)
      # pocket_yz_plane_norm = x_norm
      dir_y = np.cross(dir_z_axis_partcsy, dir_x)
      dir_z = dir_z_axis_partcsy
      p2m_construct = np.array([dir_x,dir_y,dir_z,top_plane_pt])
      p2m = np.vstack((p2m_construct.transpose(),[0,0,0,1]))
      cmm2cnc = np.linalg.inv(p2m)
      cnc_csy = Csy(top_plane_pt, dir_x, dir_y, dir_z, None)
      self.add_state('cnc_csy', cnc_csy, Stages.SETUP_CNC_CSY)
      self.add_state('dir_x', dir_x, Stages.SETUP_CNC_CSY)
      self.add_state('dir_y', dir_y, Stages.SETUP_CNC_CSY)
      self.add_state('dir_z', dir_z, Stages.SETUP_CNC_CSY)
      self.add_state('cmm2cnc', cmm2cnc, Stages.SETUP_CNC_CSY)
      self.save_cnc_csy()
      return True
    except Exception as ex:
      logger.error("setup_cnc_csy exception (while creating csy): %s" % str(ex))
      raise ex


  #TODO Remove?
  async def calc_a_err(self, a_feat_list, feat_name_suffix, a0_feat_id, nominal_a_0=0.0,  ):
    try:
      """Project the points from A-probing onto a plane defined by X-dir of CNC CSY
      """
      for a_feat_name in a_feat_list:
        fid = self.feature_ids[a_feat_name]
        a_line = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
        a_line_proj = self.add_feature('proj_' + a_probe_feat_name, Stages.WRITE_CALIB)
        for pt in a_line.points():
          orig_to_pt = pt - self.cnc_csy.orig
          dist = np.dot(orig_to_pt, self.cnc_csy.x_dir)
          proj_pt = pt - dist * self.cnc_csy.x_dir
          a_line_proj.addPoint(proj_pt[0], proj_pt[1], proj_pt[2])

      """Create nominal A0 vector variable for comparison in loop
      """
      feat_a0 = self.feature_ids[a0_feat_id]
      (pt_a0, dir_a0) = feat_a0.line()
      #A lines close to 0 should always have a positive Z-component in PartCsy
      if dir_a0[2] < 0:
        dir_a0 = -1*dir_a0
      dir_a0_cnc = np.matmul(self.cmm2cnc,np.append(dir_a0,0))
      dir_a0_cnc_2d_xz = np.array([dir_a0_cnc[0],dir_a0_cnc[2]])

      """Compare the A-feature positions to A0
      """
      a_err_list = []
      for idx, a_feat_name in enumerate(a_feat_list):
        fid_a_line_proj = self.feature_ids['proj_' + a_feat_name]
        feat_proj_a = self.metrologyManager.getActiveFeatureSet().getFeature(fid_a_line_proj)
        (pt_proj_a, dir_proj_a) = feat_proj_a.line()

        """make sure the best-fit line is in right direction
        """
        points = feat_proj_a.points()
        #the sign of (last - first) should be same as on dir
        if points[0][0] > points[-1][0]:
          #first X greater than last, dir-X should be positive
          if dir_proj_a[0] < 0:
            dir_proj_a = -1 * dir_proj_a
        else:
          #first X less than or equal to last, dir-X should be 0 or negative
          if dir_proj_a[0] > 0:
            dir_proj_a = -1 * dir_proj_a

        dir_proj_a_cnc = np.matmul(self.cmm2cnc ,np.append(dir_proj_a,0))
        dir_proj_a_cnc_2d_xz = np.array([dir_proj_a_cnc[0],dir_proj_a_cnc[2]])

        #get the nominal position from feature name
        #pattern for feature name depends on calib('probe_a_') or verify('verify_a_')
        nominal_pos = float(a_feat_name[len(feat_name_suffix):])
        nominal_pos_adjusted = nominal_pos - a_home_err
        line_angle_rel_0 = angle_between_ccw_2d(dir_proj_a_cnc_2d_xz, dir_proj_a_cnc_2d_xz)
        if line_angle_rel_0 < 0 and nominal_pos > 135:
          #shift [-180,0] portion to [180,360]
          line_angle_rel_0 = line_angle_rel_0 + 360
        elif nominal_pos > 360:
          line_angle_rel_0 = line_angle_rel_0 + 360

        err = line_angle_rel_0 - nominal_pos
        a_err_list.append((nominal_pos, err))

      return a_err_list

    except Exception as e:
      logger.error(str(e))
      raise e


  #TODO Remove?
  async def project_b_features(self, b_feat_list):
    try:
      """Project points from B-probing features onto plane defined by CNC CSY Y-axis
      """
      b_proj_feat_list = []
      for b_feat_name in b_feat_list:
        fid = self.feature_ids[b_feat_name]
        b_line = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
        b_proj_feat_name = 'proj_' + b_probe_feat_name
        b_proj_feat_list.append(b_proj_feat_name)
        b_line_proj = self.add_feature(b_proj_feat_name)
        for pt in b_line.points():
          orig_to_pt = pt - self.cnc_csy.orig
          dist = np.dot(orig_to_pt, self.cnc_csy.y_dir)
          proj_pt = pt - dist * self.cnc_csy.y_dir
          # proj_pt_in_plane = np.matmul(rot_mat,np.append(proj_pt,1))
          b_line_proj.addPoint(proj_pt[0], proj_pt[1], proj_pt[2])
      return b_proj_feat_list

    except Exception as e:
      logger.error(str(e))
      raise e


  def project_feats_to_plane(self, feat_list, orig, norm):
    try:
      proj_feat_list = []
      for feat_name in feat_list:
        fid = self.feature_ids[feat_name]
        feat = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
        proj_feat_name = feat_name + "_proj"
        proj_feat_list.append(proj_feat_name)
        proj_feat = self.add_feature(proj_feat_name)
        for pt in feat.points():
          orig_to_pt = pt - orig
          dist = np.dot(orig_to_pt, norm)
          proj_pt = pt - (dist * norm)
          proj_feat.addPoint(*proj_pt)
      return proj_feat_list
    except Exception as ex:
      logger.error("project_feats_to_plane exception: %s" % str(ex))
      raise ex


  def calc_b_results(self, b_feat_list, feat_name_suffix, home_err, b0_dir, feat_b_circle):
    """
    For a list of B-axis probing features:
    -Create projected features, projecting onto plane defined in PartCsy by CNC Y-axis
    -Create bestFit Lines for projected features
    -Transform bestFit lines into CNC Csy
    -Find angular position of bestFit lines compared to a zero/origin vector
    -Find position error (true minus nominal)
    -Find intersect position with subsequent B-axis feature
    """
    try:
      proj_b_names = self.project_feats_to_plane(b_feat_list, self.cnc_csy.orig, self.cnc_csy.y_dir)
      fit_lines_2d = []
      results = []

      for idx, feat_name in enumerate(proj_b_names):
        fid = self.feature_ids[feat_name]
        feat = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
        (b_pt, b_dir) = feat.line()
        points = feat.points()
        #ensure consistent direction
        if points[0][0] > points[-1][0]:
          #first X greater than last, dir-X should be positive
          if b_dir[0] < 0:
            b_dir = -1 * b_dir
        elif b_dir[0] > 0:
          #first X less than or equal to last, dir-X should be 0 or negative
          b_dir = -1 * b_dir
        b_dir_transformed = np.matmul(self.cmm2cnc,np.append(b_dir,0))
        b_dir_transformed_2d = np.array([b_dir_transformed[0],b_dir_transformed[2]])
        line_angle_rel_0 = angle_between_ccw_2d(b_dir_transformed_2d, b0_dir)
        nominal_pos = float(feat_name[len(feat_name_suffix):-len('_proj')])
        if line_angle_rel_0 < 0 and nominal_pos > 135:
          #return from angle_between_ccw_2d has range [-180,180]
          line_angle_rel_0 = line_angle_rel_0 + 360
        if idx == len(proj_b_names) - 1:
          #the last B-probe is at nominal 360
          line_angle_rel_0 = line_angle_rel_0 + 360
        true_nominal_pos = nominal_pos - home_err
        # err = true_nominal_pos - line_angle_rel_0
        err = line_angle_rel_0 - true_nominal_pos
        results.append((true_nominal_pos, err))
        fit_lines_2d.append((np.array([b_pt[0], b_pt[1]]), np.array([b_dir[0], b_dir[1]])))
      
      for idx, (b_pt, b_dir) in enumerate(fit_lines_2d[:-1]):
        (next_b_pt, next_b_dir) = fit_lines_2d[idx + 1]
        intersect_pos = find_line_intersect(b_pt, b_dir, next_b_pt, next_b_dir)
        feat_b_circle.addPoint(intersect_pos[0], intersect_pos[1], 0)
      
      return results
    except Exception as ex:
      logger.error("calc_b_results exception: %s" % str(ex))
      raise ex


  def calc_a_results(self, a_feat_list, feat_name_suffix, home_err, a0_dir, feat_a_circle):
    """
    For a list of A-axis probing features:
    -Create projected features, projecting onto plane defined in PartCsy by CNC X-axis
    -Create bestFit Lines for projected features
    -Transform bestFit lines into CNC Csy
    -Find angular position of bestFit lines compared to a zero/origin vector
    -Find position error (true minus nominal)
    -Find intersect position with subsequent B-axis feature
    """
    try:
      proj_feat_names = self.project_feats_to_plane(a_feat_list, self.cnc_csy.orig, self.cnc_csy.x_dir)
      fit_lines_2d = []
      results = []

      for feat_name in proj_feat_names:
        fid = self.feature_ids[feat_name]
        feat = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
        (a_pt, a_dir) = feat.line()
        points = feat.points()
        #ensure consistent direction
        if points[0][2] > points[-1][2]:
          #first point is higher than last, so line should be upwards to be away from center
          if a_dir[2] < 0:
            a_dir = -1 * a_dir
        else:
          #first point is lower than last, so line should be downwards to be away from center
          if a_dir[2] > 0:
            a_dir = -1 * a_dir
        a_dir_transformed = np.matmul(self.cmm2cnc,np.append(a_dir,0))
        a_dir_transformed_2d = np.array([a_dir_transformed[1],a_dir_transformed[2]])
        line_angle_rel_0 = -1*angle_between_ccw_2d(a_dir_transformed_2d, a0_dir)
        nominal_pos = float(feat_name[len(feat_name_suffix):-len('_proj')])
        true_nominal_pos = nominal_pos - home_err
        # err = true_nominal_pos - line_angle_rel_0
        err =  line_angle_rel_0 - true_nominal_pos
        results.append((true_nominal_pos, err))
        fit_lines_2d.append((np.array([a_pt[0], a_pt[2]]), np.array([a_dir[0], a_dir[2]])))
      
      for idx, (a_pt, a_dir) in enumerate(fit_lines_2d[:-1]):
        (next_pt, next_dir) = fit_lines_2d[idx + 1]
        intersect_pos = find_line_intersect(a_pt, a_dir, next_pt, next_dir)
        feat_a_circle.addPoint(intersect_pos[0], 0, intersect_pos[1])

      return results

    except Exception as ex:
      logger.error("calc_a_results exception: %s" % str(ex))
      raise ex


  async def calc_b_err(self, b_feat_list, feat_name_suffix, b0_feat_id, nominal_b_0=0.0,  ):
    try:
      """Project the points from B-probing onto a plane defined by Y-dir of CNC CSY
      """
      for b_feat_name in b_feat_list:
        fid = self.feature_ids[b_feat_name]
        b_line = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
        b_line_proj = self.add_feature('proj_' + b_probe_feat_name, Stages.WRITE_CALIB)
        for pt in b_line.points():
          orig_to_pt = pt - self.cnc_csy.orig
          dist = np.dot(orig_to_pt, self.cnc_csy.y_dir)
          proj_pt = pt - dist * self.cnc_csy.y_dir
          # proj_pt_in_plane = np.matmul(rot_mat,np.append(proj_pt,1))
          b_line_proj.addPoint(proj_pt[0], proj_pt[1], proj_pt[2])

      """Create nominal B0 vector variable for comparison in loop
      """
      feat_b0 = self.feature_ids[b0_feat_id]
      (pt_b0, dir_b0) = feat_b0.line()
      #B lines close to 0 should always have a positive Z-component in PartCsy
      if dir_b0[2] < 0:
        dir_b0 = -1*dir_b0
      dir_b0_cnc = np.matmul(self.cmm2cnc,np.append(dir_b0,0))
      dir_b0_cnc_2d_xz = np.array([dir_b0_cnc[0],dir_b0_cnc[2]])

      """Compare the B-feature positions to B0
      """
      b_err_list = []
      for idx, b_feat_name in enumerate(b_feat_list):
        fid_b_line_proj = self.feature_ids['proj_' + b_feat_name]
        feat_proj_b = self.metrologyManager.getActiveFeatureSet().getFeature(fid_b_line_proj)
        (pt_proj_b, dir_proj_b) = feat_proj_b.line()

        """make sure the best-fit line is in right direction
        """
        points = feat_proj_b.points()
        #the sign of (last - first) should be same as on dir
        if points[0][0] > points[-1][0]:
          #first X greater than last, dir-X should be positive
          if dir_proj_b[0] < 0:
            dir_proj_b = -1 * dir_proj_b
        else:
          #first X less than or equal to last, dir-X should be 0 or negative
          if dir_proj_b[0] > 0:
            dir_proj_b = -1 * dir_proj_b

        dir_proj_b_cnc = np.matmul(self.cmm2cnc ,np.append(dir_proj_b,0))
        dir_proj_b_cnc_2d_xz = np.array([dir_proj_b_cnc[0],dir_proj_b_cnc[2]])

        #get the nominal position from feature name
        #pattern for feature name depends on calib('probe_b_') or verify('verify_b_')
        nominal_pos = float(b_feat_name[len(feat_name_suffix):])
        nominal_pos_adjusted = nominal_pos - b_home_err
        line_angle_rel_0 = angle_between_ccw_2d(dir_proj_b_cnc_2d_xz, dir_proj_b_cnc_2d_xz)
        if line_angle_rel_0 < 0 and nominal_pos > 135:
          #shift [-180,0] portion to [180,360]
          line_angle_rel_0 = line_angle_rel_0 + 360
        elif nominal_pos > 360:
          line_angle_rel_0 = line_angle_rel_0 + 360

        err = line_angle_rel_0 - nominal_pos
        b_err_list.append((nominal_pos, err))

      return b_err_list

    except Exception as ex:
      logger.error("calc_b_err exception: %s" % str(ex))
      raise ex


  async def calc_calib(self):
    '''
    Get B results, including center of rotation
    Get A results, including center of rotation
    Find X and Y home offsets
    '''
    offsets = {}
    z_line_partcsy_dir = None
    z_vec_proj = None 
    z_vec_cross_top_plane_norm = None
    rot_mat = None
    pocket_yz_plane_norm = None

    fixture_top_face = self.metrologyManager.getActiveFeatureSet().getFeature( self.feature_ids['fixture_top_face'] )
    plane_fixture_top_face = fixture_top_face.plane()
    top_plane_pt = plane_fixture_top_face[0]
    top_plane_norm = plane_fixture_top_face[1]
    #top_plane_norm should be pointing up
    if top_plane_norm[2] < 0:
      top_plane_norm = -1*top_plane_norm

    '''
    Z results
    Define a line along the z points
    '''
    try:
      z_line = self.add_feature('z_line', Stages.WRITE_CALIB)
      z_line_proj = self.add_feature('z_line_proj', Stages.WRITE_CALIB)
      for i in range(0,Z_MIN,Z_STEP):
        fid = self.feature_ids['z_%+.4f' % i]
        z_pos_feat = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
        z_pos_sphere = z_pos_feat.sphere()
        z_pos = z_pos_sphere[1]
        z_line.addPoint(*z_pos)
        #also project onto top-fixture-plane
        plane_orig_to_pt = z_pos - top_plane_pt
        dist = np.dot(plane_orig_to_pt, top_plane_norm)
        proj_pt = z_pos - dist * top_plane_norm
        z_line_proj.addPoint(*proj_pt)

      z_line_partcsy_dir = -1*z_line.line()[1]
      z_line_proj_dir = z_line_proj.line()[1]
      z_vec_cross_top_plane_norm = np.cross(z_line_proj_dir,top_plane_norm)
    except Exception as ex:
      logger.error("calc_calib exception (while z results): %s" % str(ex))
      raise ex


    '''
    Construct CNC Coordinate System
    '''
    try:
      #   y_norm = top_plane_norm
      #   x_norm = np.cross(y_norm, z_line_partcsy_dir)
      #   pocket_yz_plane_norm = x_norm
      #   z_norm = np.cross(x_norm,y_norm)
      #   p2m_construct = np.array([x_norm,y_norm,z_norm,top_plane_pt])
      #   p2m = np.vstack((p2m_construct.transpose(),[0,0,0,1]))
      #   cmm2cnc = np.linalg.inv(p2m)
      #   #TODO use the plane defined by B-rotation instead of fixture top plane
      #   dir_b_norm_cnccsy = np.matmul(cmm2cnc,np.append(top_plane_norm,0))
        z_line_cnccsy_dir = np.matmul(self.cmm2cnc,np.append(z_line_partcsy_dir,0))
      #   print("pocketnc 2 machine matrix")
      #   print(p2m)
      #   print("machine 2 pocketnc matrix")
      #   print(cmm2cnc)
    except Exception as ex:
      logger.error("calc_calib exception (while constructing coordinate system): %s" % str(ex))
      raise ex

    '''
    B results
      project the b-angle lines to top plane
      for each b-axis position
        translate projected points into POCKET coord sys
        find best-fit line through projected points
      for each line
        find angle compared to 0, compare to nominal
    
    for each line, find the position of intersect with the next line
    the center of rotation is at the centroid of these intersection positions
    '''
    try:
      logger.debug('Calculating B Calib')
      b_home_err = float(self.feat_name_last_find_b_proj[len("find_b_"):-len("_proj")])
      self.offsets['b'] = self.active_offsets['b'] - b_home_err
      proj_true_b0_fid = self.feature_ids[self.feat_name_last_find_b_proj]
      feat_proj_b0 = self.metrologyManager.getActiveFeatureSet().getFeature(proj_true_b0_fid)
      dir_proj_b0 = feat_proj_b0.line()[1]
      points_proj_b0 = feat_proj_b0.points()
      #the sign of (last - first) should be same as on dir
      if points_proj_b0[0][0] > points_proj_b0[-1][0]:
        #first X greater than last, dir-X should be positive
        if dir_proj_b0[0] < 0:
          dir_proj_b0 = -1 * dir_proj_b0
      else:
        #first X less than or equal to last, dir-X should be 0 or negative
        if dir_proj_b0[0] > 0:
          dir_proj_b0 = -1 * dir_proj_b0
      dir_b0_proj_transformed = np.matmul(self.cmm2cnc,np.append(dir_proj_b0,0))
      dir_b0_proj_transformed_2d = np.array([dir_b0_proj_transformed[0],dir_b0_proj_transformed[2]])
      feat_b_circle = self.add_feature('b_circle', Stages.CALC_CALIB)
      b_results = self.calc_b_results(self.b_calib_probes, 'probe_b_', b_home_err, dir_b0_proj_transformed_2d, feat_b_circle)
      self.b_err = b_results[:-1]
      logger.debug("Got b_err_table")
      logger.debug(self.b_err)
      # b_err_table = compensation.processBData(b_err_table)
      (b_comp_table, err, bestAlgo) = compensation.calculateBCompensation(self.b_err)
      self.b_comp = b_comp_table
      logger.debug("Got b_comp_table")
      logger.debug(err)
      logger.debug(bestAlgo)
      logger.debug(self.b_comp)

    except Exception as ex:
      logger.error("calc_calib exception (in b results): %s" % str(ex))
      raise ex

    '''
    A results
      project the a-angle lines to POCKET coord sys YZ plane
      for each a-axis position
        translate projected points into POCKET space
        find best-fit line through projected points
      for each line
        find angle compared to 0, compare to nominal
    '''
    try:
      logger.debug('Calculating A Calib')
      a_home_err = float(self.feat_name_last_find_a_proj[len("find_a_"):-len("_proj")])
      self.offsets['a'] = self.active_offsets['a'] - a_home_err
      proj_true_a0_fid = self.feature_ids[self.feat_name_last_find_a_proj]
      feat_proj_a0 = self.metrologyManager.getActiveFeatureSet().getFeature(proj_true_a0_fid)
      dir_proj_a0 = feat_proj_a0.line()[1]
      points_proj_a0 = feat_proj_a0.points()
      if dir_proj_a0[2] < 0:
        #z-component for A features near A0 should be positive
        dir_proj_a0 = -1 * dir_proj_a0
      dir_a0_proj_transformed = np.matmul(self.cmm2cnc,np.append(dir_proj_a0,0))
      dir_a0_proj_transformed_2d = np.array([dir_a0_proj_transformed[1],dir_a0_proj_transformed[2]])
      feat_a_circle = self.add_feature('a_circle', Stages.CALC_CALIB)
      a_results = self.calc_a_results(self.a_calib_probes, 'probe_a_', a_home_err, dir_a0_proj_transformed_2d, feat_a_circle)
      self.a_err = a_results[:-1]

      print(feat_a_circle.points())
      print(feat_a_circle.average())
      logger.debug("Got a_err_table")
      logger.debug(self.a_err)
      (a_comp_table, err) = compensation.calculateACompensation(self.a_err)
      self.a_comp = a_comp_table
      logger.debug("Got a_comp_table")
      logger.debug(err)
      logger.debug(self.a_comp)

    except Exception as ex:
      logger.error("calc_calib exception (in a results): %s" % str(ex))
      raise ex

    '''
    Linear Axes
    The home offset values should align the axes so that 
    the axes normals intersects the center of rotation for the 
    associated rotational axis
    X HOME_OFFSET aligns Z-norm with B COR
    Y HOME_OFFSET aligns Z-norm with A COR
    Z HOME_OFFSET places tool tip TOOL_OFFSET away from B COR

    Account for homing variation 

      """account for homing variation by offsetting by the vector drawn from 
      the last Z home position to the average Z home position
      """

    First find Y offset, so that height of Z-norm relative to B-points is known
      Find center of rotation of A
      Calculate Y-pos of spindle tip when spindle Z-pos is equal to Z-pos of A-CoR 
      Difference in Y between these two positions is Y-offset err
      New Y-offset = Old - err
    Second find X offset
      Find the point where B-norm intersects the plane that is...
        parallel to XZ
        at height (Y pos) where Z-norm intersects A-CoR
      Find the X-offset needed so Z-norm intersects this point
    '''
    try:
      print("Linear Axes Offsets")
      pos_a_circle_partcsy, radius_a_circle, normal_a_circle = feat_a_circle.circle()
      print(pos_a_circle_partcsy)
      pos_a_circle_cnccsy = np.matmul(self.cmm2cnc,np.append(pos_a_circle_partcsy,1))
      print(pos_a_circle_cnccsy)
      pos_a_circle_cnccsy_y0 = pos_a_circle_cnccsy[0:3] - np.array([0,PROBING_POS_Y,0])
      print(pos_a_circle_cnccsy_y0)
      slope_z_axis_in_cnc_yz_plane = z_line_cnccsy_dir[1]/z_line_cnccsy_dir[2]
      print('slope_z_axis_in_cnc_yz_plane')
      print(slope_z_axis_in_cnc_yz_plane)
      feat_z_line = self.get_feature("z_line")
      # pos_z0_partcsy = feat_z_line.points()[0]
      # pos_z0_cnccsy = np.matmul(self.cmm2cnc,np.append(pos_z0_partcsy,1))
      pos_z_home_partcsy = self.status['z_home']['avg_pos']
      pos_z_home_cnccsy = np.matmul(self.cmm2cnc,np.append(pos_z_home_partcsy,1))
      
      print("pos_z_home_cnccsy")
      print(pos_z_home_cnccsy)
      z_travel_z0_to_acor = pos_a_circle_cnccsy_y0[2] - pos_z_home_cnccsy[2]
      print("z_travel_z0_to_acor")
      print(z_travel_z0_to_acor)
      # dist_z0_to_acor = z_travel_z0_to_acor / z_line_cnccsy_dir[2]
      y_pos_of_z_at_travel = slope_z_axis_in_cnc_yz_plane * z_travel_z0_to_acor + pos_z_home_cnccsy[1]
      print("y_pos_of_z_at_travel")
      print(y_pos_of_z_at_travel)
      # print(dist_z0_to_acor)
      y_travel_spindle_to_acor = y_pos_of_z_at_travel - pos_a_circle_cnccsy_y0[1]#pos_a_circle_cnccsy_y0[1] - (slope_z_axis_in_cnc_yz_plane * z_travel)
      print("y offset err")
      print(y_travel_spindle_to_acor)
      self.offsets['y'] = self.active_offsets['y'] - (y_travel_spindle_to_acor)/25.4

      pos_b_circle_partcsy, radius_b_circle, normal_b_circle_partcsy = feat_b_circle.circle()
      print('pos_b_circle_partcsy')
      print(pos_b_circle_partcsy)
      pos_b_circle_cnccsy = np.matmul(self.cmm2cnc,np.append(pos_b_circle_partcsy,1))
      print('pos_b_circle_cnccsy')
      print(pos_b_circle_cnccsy)
      pos_b_circle_cnccsy_y0 = pos_b_circle_cnccsy[0:3] - np.array([0,PROBING_POS_Y,0])
      print('pos_b_circle_cnccsy_y0')
      print(pos_b_circle_cnccsy_y0)
      norm_b_circle_cnccsy = np.matmul(self.cmm2cnc,np.append(normal_b_circle_partcsy,0))
      print('norm_b_circle_cnccsy')
      print(norm_b_circle_cnccsy)

      y_travel_b_cor_to_a_cor = pos_a_circle_cnccsy_y0[1] - pos_b_circle_cnccsy_y0[1]
      print('y_travel_b_cor_to_a_cor')
      print(y_travel_b_cor_to_a_cor)
      # dist_b_cor_to_a_cor_height = y_travel_b_cor_to_a_cor / norm_b_circle_cnccsy[1]
      # pos_bnorm_intersect_acor_xzplane = pos_b_circle_cnccsy_y0 + dist_b_cor_to_a_cor_height * norm_b_circle_cnccsy[0:3]
      pos_bnorm_intersect_acor_xzplane = pos_b_circle_cnccsy_y0 + y_travel_b_cor_to_a_cor * norm_b_circle_cnccsy[0:3]
      print('pos_bnorm_intersect_acor_xzplane')
      print(pos_bnorm_intersect_acor_xzplane)

      # xy_slope_b_norm = norm_b_circle_cnccsy[0]/norm_b_circle_cnccsy[1]
      # dist_z0_to_bcor = pos_a_circle_offset[2] - pos_z0_cnccsy[2]
      # x_offset_z_intersect_bcor = pos_b_circle_cnccsy_y0[0] - (xy_slope_b_norm * y_travel_b_cor_to_a_cor)
      
      slope_z_axis_in_cnc_xz_plane = z_line_cnccsy_dir[0]/z_line_cnccsy_dir[2]
      print('slope_z_axis_in_cnc_xz_plane')
      print(slope_z_axis_in_cnc_xz_plane)

      # z_diff_z0_to_pos_bnorm_intersect_acor_xzplane = pos_bnorm_intersect_acor_xzplane[2] - pos_z0_cnccsy[2]
      # print('z_diff_z0_to_pos_bnorm_intersect_acor_xzplane')
      # # print(z_diff_z0_to_pos_bnorm_intersect_acor_xzplane)
      # dist_z0_to_pos_bnorm_intersect_acor_xzplane = z_diff_z0_to_pos_bnorm_intersect_acor_xzplane / z_line_cnccsy_dir[2]
      # print('dist_z0_to_pos_bnorm_intersect_acor_xzplane')
      # print(dist_z0_to_pos_bnorm_intersect_acor_xzplane)
      # # z_travel = pos_a_circle_cnccsy_y0[2] - pos_z0_cnccsy[2]
      # x_pos_of_z_at_travel = slope_z_axis_in_cnc_xz_plane * dist_z0_to_pos_bnorm_intersect_acor_xzplane + pos_z0_cnccsy[0]
      # print('x_pos_of_z_at_travel')
      # print(x_pos_of_z_at_travel)
      # # x_offset_for_z_norm_to_intersect_pos_bnorm_intersect_acor_xzplane = pos_bnorm_intersect_acor_xzplane[0] - (slope_z_axis_in_cnc_xz_plane * y_travel_b_cor_to_a_cor)
      # x_offset_for_z_norm_to_intersect_pos_bnorm_intersect_acor_xzplane = x_pos_of_z_at_travel - pos_bnorm_intersect_acor_xzplane[0]
      # print("x offset err")
      # print(x_offset_for_z_norm_to_intersect_pos_bnorm_intersect_acor_xzplane)

      z_travel_z0_to_cor = norm_b_circle_cnccsy[2] - pos_z_home_cnccsy[2]
      print('z_travel_z0_to_cor')
      print(z_travel_z0_to_cor)
      feat_x_line = self.get_feature("x_line")
      pos_x0_partcsy = feat_x_line.points()[0]
      
      vec_last_x_home_to_avg_x_home_partcsy = pos_x0_partcsy - self.status['x_home']['avg_pos']
      pos_z_home_offset_by_x_homing_variation_partcsy = pos_z_home_partcsy + vec_last_x_home_to_avg_x_home_partcsy
      pos_z_home_offset_by_x_homing_variation_cnccsy = np.matmul(self.cmm2cnc,np.append(pos_z_home_offset_by_x_homing_variation_partcsy,1))
      x_pos_of_z_at_travel = slope_z_axis_in_cnc_xz_plane * z_travel_z0_to_cor + pos_z_home_offset_by_x_homing_variation_cnccsy[0]
      print('x_pos_of_z_at_travel')
      print(x_pos_of_z_at_travel)

      x_offset_for_dir_z_to_intersect_cor = x_pos_of_z_at_travel - pos_b_circle_cnccsy_y0[0]
      print("x offset err")
      print(x_offset_for_dir_z_to_intersect_cor)
      self.offsets['x'] = self.active_offsets['x'] - (x_offset_for_dir_z_to_intersect_cor)/25.4
    except Exception as ex:
      logger.error("calc_calib exception (in linear results): %s" % str(ex))
      raise ex

    '''
    B table offset
    Distance along y-norm, from A-center-of-rotation to top of B table
    '''
    try:
      vec_top_plane_to_a_cor = pos_a_circle_partcsy - self.fitted_features['fixture_top_face']['pt']
      dist = np.dot(vec_top_plane_to_a_cor, self.cnc_csy.y_dir)
      self.offsets['b_table'] = dist + FIXTURE_HEIGHT

    except Exception as ex:
      logger.error("calc_calib exception (in b table offset): %s" % str(ex))
      raise ex

    '''
    probe_sensor_123 offset
    Difference in CNC Z axis position between...
    -position where tool probe button is triggered
    -and position where contact would be made with a 3 inch block, mounted on B table, at A90 (i.e. B-face towards Z)
    '''
    try:
      self.offsets['probe_sensor_123'] = 0.8 #self.tool_probe_z - self.tool_Length - 3*25.4 + self.offsets['b_table']

    except Exception as ex:
      logger.error("calc_calib exception (in probe_sensor_123 results): %s" % str(ex))
      raise ex

    return True


  async def write_calib(self):
    return self.write_calib_sync()


  def write_calib_sync(self):
    '''
    B results
    '''
    try:
      with open( os.path.join(RESULTS_DIR, 'b.err'), 'w') as f:
        for p in self.b_err:
          f.write("%0.6f %0.6f %0.6f\n" % (p[0], p[1], p[1]))
      with open( os.path.join(RESULTS_DIR, 'b.comp.raw'), 'w') as f:
        for p in self.b_err:
          f.write("%0.6f %0.6f %0.6f\n" % (p[0], -p[1], -p[1]))
      with open( os.path.join(RESULTS_DIR, 'b.comp'), 'w') as f:
        #remove the 0 and 360 points from cycles because they result in duplicated values, which causes error
        #probably the 360 point should be included.
        #0 point should have negligible error (it gets accounted for in home offset error) and can be left out
        # pts = [ p for p in self.b_comp.pts[1:] ]
        #now including 360 point, with extra try-blocks around each insert
        pts = [ p for p in self.b_comp.pts[1:-1] ]
        for cycles in range(1,29):
          for p in pts:
            forward = (p[0]+360*cycles, p[1])
            reverse = (p[0]-360*cycles, p[1])
            try:
              self.b_comp.insert(forward)
            except:
              pass
            try:
              self.b_comp.insert(reverse)
            except:
              pass
        for p in self.b_comp.pts:
          f.write("%0.6f %0.6f %0.6f\n" % (p[0], p[1], p[1]))
    except Exception as ex:
      logger.error("write_calib exception (in b results): %s" % str(ex))
      #commented out until i get the range expansion above working
      # raise ex 


    '''
    A results
    '''
    try:
      with open( os.path.join(RESULTS_DIR, 'a.err'), 'w') as f:
        for p in self.a_err:
          f.write("%0.6f %0.6f %0.6f\n" % (p[0], p[1], p[1]))
      with open( os.path.join(RESULTS_DIR, 'a.comp.raw'), 'w') as f:
        for p in self.a_err:
          f.write("%0.6f %0.6f %0.6f\n" % (p[0], -p[1], -p[1]))
      with open( os.path.join(RESULTS_DIR, 'a.comp'), 'w') as f:
        for p in self.a_comp.pts:
          f.write("%0.6f %0.6f %0.6f\n" % (p[0], p[1], p[1]))
    except Exception as ex:
      logger.error("write_calib exception (in a results): %s" % str(ex))
      raise ex

    '''
    Writing CalibrationOverlay
    '''
    try:
      new_overlay_data = copy.deepcopy(self.overlay_data)
      # with open(NEW_OVERLAY_FILENAME, 'w') as f:
      if "x" in self.offsets:
          print("Writing %0.6f to X HOME_OFFSET" % self.offsets["x"])
          ini.set_parameter(new_overlay_data, "JOINT_0", "HOME_OFFSET", self.offsets["x"])

      if "y" in self.offsets:
          print("Writing %0.6f to Y HOME_OFFSET" % self.offsets["y"])
          ini.set_parameter(new_overlay_data, "JOINT_1", "HOME_OFFSET", self.offsets["y"])

      if "a" in self.offsets:
          print("Writing %0.6f to A HOME_OFFSET" % self.offsets["a"])
          ini.set_parameter(new_overlay_data, "JOINT_3", "HOME_OFFSET", self.offsets["a"])

      if "b" in self.offsets:
          print("Writing %0.6f to B HOME_OFFSET" % self.offsets["b"])
          ini.set_parameter(new_overlay_data, "JOINT_4", "HOME_OFFSET", self.offsets["b"])

      if "b_table" in self.offsets:
          print("Writing %0.6f to PROBE_B_TABLE_OFFSET" % self.offsets["b_table"])
          ini.set_parameter(new_overlay_data, "TOOL_PROBE", "PROBE_B_TABLE_OFFSET", self.offsets["b_table"])

      if "probe_sensor_123" in self.offsets:
          print("Writing %0.6f to PROBE_SENSOR_123_OFFSET" % self.offsets["probe_sensor_123"])
          ini.set_parameter(new_overlay_data, "TOOL_PROBE", "PROBE_SENSOR_123_OFFSET", self.offsets["probe_sensor_123"])
      ini.write_ini_data(new_overlay_data, NEW_OVERLAY_FILENAME)
      logger.debug(self.offsets)
    except Exception as ex:
      logger.error("write_calib exception (while writing overlay): %s" % str(ex))
      raise ex

    
    """
    Copy files to calib results dir
    """
    try:
      # for f in ['a.comp.raw', 'b.comp.raw', 'CalibrationOverlay.inc']:
      for f in ['a.comp.raw', 'b.comp', 'CalibrationOverlay.inc']:
        curr_path = os.path.join(RESULTS_DIR, f)
        dest_f = f[:-4] if f.endswith('.raw') else f
        dest_path = os.path.join(CALIB_RESULTS_DIR, dest_f)
        os.popen('cp %s %s' % (curr_path, dest_path))
    except Exception as ex:
      logger.error("write_calib exception (while copying files to POCKETNC_VAR_DIR): %s" % str(ex))
      raise ex

    """
    Copy files to POCKETNC_VAR_DIR
    """
    try:
      for f in ['a.comp.raw', 'b.comp.raw', 'CalibrationOverlay.inc']:
        curr_path = os.path.join(RESULTS_DIR, f)
        dest_f = f[:-4] if f.endswith('.raw') else f
        dest_path = os.path.join(POCKETNC_VAR_DIR, dest_f)
        os.popen('cp %s %s' % (curr_path, dest_path))
    except Exception as ex:
      logger.error("write_calib exception (while copying files to POCKETNC_VAR_DIR): %s" % str(ex))
      raise ex
    return True


  async def setup_verify(self):
    """
    Load PartCsy 
    Load CncCsy
    """
    #load data from before reboot that is needed to construct the CNC coord sys
    try:
      self.config['table_slot'] = "front_right"

      await self.load_part_csy()
      if not self.config['skip_cmm']:
        await self.set_cmm_csy(self.part_csy)

      await self.load_cnc_csy()
      self.load_stage_progress(Stages.PROBE_TOP_PLANE, is_performing_stage=False)
      fixture_top_face = self.metrologyManager.getActiveFeatureSet().getFeature( self.feature_ids['fixture_top_face'] )
      plane_fixture_top_face = fixture_top_face.plane()
      top_plane_pt = plane_fixture_top_face[0]
      top_plane_norm = plane_fixture_top_face[1]
      self.fitted_features['fixture_top_face'] = {'pt': top_plane_pt, 'norm': top_plane_norm}
      self.load_stage_progress(Stages.SETUP_CNC_CSY, is_performing_stage=False)
      return True
    except Exception as ex:
      logger.error("setup_verify exception: %s" % str(ex))
      raise ex


  async def calc_verify(self):
    '''
    A and B
      Home position <0.05 deg from true zero
      All other positions <0.02 deg from nominal
      Best-fit circle eccentricity <0.001 in
    XYZ ? Currently nothing in verify
    '''
    offsets = {}
    print(self.cnc_csy)

    '''
    B results
    '''
    try:
      logger.debug('Calculating B Verify')
      name_b0_verify = "verify_b_%+.6f" % 0
      fid_b0_verify = self.feature_ids[name_b0_verify]
      print('b0 fid')
      print(fid_b0_verify)
      feat_b0_verify = self.metrologyManager.getActiveFeatureSet().getFeature(fid_b0_verify)
      print(feat_b0_verify.points())
      [name_b0_verify_proj] = self.project_feats_to_plane([name_b0_verify], self.cnc_csy.orig, self.cnc_csy.y_dir)
      print('b0 projected')
      fid_b0_verify_proj = self.feature_ids[name_b0_verify_proj]
      feat_b0_verify_proj = self.metrologyManager.getActiveFeatureSet().getFeature(fid_b0_verify_proj)
      dir_b0_verify_proj = feat_b0_verify_proj.line()[1]
      points_b0_verify_proj = feat_b0_verify_proj.points()
      #the sign of (last - first) should be same as on dir
      if points_b0_verify_proj[0][0] > points_b0_verify_proj[-1][0]:
        #first X greater than last, dir-X should be positive
        if dir_b0_verify_proj[0] < 0:
          dir_b0_verify_proj = -1 * dir_b0_verify_proj
      else:
        #first X less than or equal to last, dir-X should be 0 or negative
        if dir_b0_verify_proj[0] > 0:
          dir_b0_verify_proj = -1 * dir_b0_verify_proj
      dir_b0_ver_proj_transformed = np.matmul(self.cmm2cnc,np.append(dir_b0_verify_proj,0))
      dir_b0_ver_proj_transformed_2d = np.array([dir_b0_ver_proj_transformed[0],dir_b0_ver_proj_transformed[2]])
      print('b0')
      print(dir_b0_ver_proj_transformed_2d)
      feat_b_verify_circle = self.add_feature('b_verify_circle', Stages.CALC_VERIFY)
      b_results = self.calc_b_results(self.b_verify_probes, 'verify_b_', 0, dir_b0_ver_proj_transformed_2d, feat_b_verify_circle)
      logger.debug(feat_b_verify_circle.points())
      logger.debug(feat_b_verify_circle.average())
      logger.debug("Got verify B err_table")
      logger.debug(b_results)
      b_max_abs_err_pos = 0
      b_max_abs_err = 0
      self.b_ver_err = b_results
      for (pos, err) in b_results:
        if abs(err) > b_max_abs_err:
          b_max_abs_err = abs(err)
          b_max_err_pos = pos
      self.spec['b_max_err']['val'] = b_max_abs_err
      b_in_spec = b_max_abs_err < SPEC_ANGULAR_ACCURACY
      self.spec['b_max_err']['pass'] = b_in_spec
    except Exception as ex:
      logger.error("calc_verify exception (b results): %s" % str(ex))
      raise ex

    '''
    A results
    '''
    try:
      logger.debug('Calculating A Verify')
      name_a0_verify = "verify_a_%+.6f" % 0
      fid_a0_verify = self.feature_ids[name_a0_verify]
      feat_a0_verify = self.metrologyManager.getActiveFeatureSet().getFeature(fid_a0_verify)
      [name_a0_verify_proj] = self.project_feats_to_plane([name_a0_verify], self.cnc_csy.orig, self.cnc_csy.x_dir)
      fid_a0_verify_proj = self.feature_ids[name_a0_verify_proj]
      feat_a0_verify_proj = self.metrologyManager.getActiveFeatureSet().getFeature(fid_a0_verify_proj)
      dir_a0_verify_proj = feat_a0_verify_proj.line()[1]
      points_a0_verify_proj = feat_a0_verify_proj.points()
      if dir_a0_verify_proj[2] < 0:
        #z-component for A features near A0 should be positive
        dir_a0_verify_proj = -1 * dir_a0_verify_proj
      dir_a0_ver_proj_transformed = np.matmul(self.cmm2cnc,np.append(dir_a0_verify_proj,0))
      dir_a0_ver_proj_transformed_2d = np.array([dir_a0_ver_proj_transformed[1],dir_a0_ver_proj_transformed[2]])
      feat_a_circle = self.add_feature('a_verify_circle', Stages.CALC_VERIFY)
      a_results = self.calc_a_results(self.a_verify_probes, 'verify_a_', 0, dir_a0_ver_proj_transformed_2d, feat_a_circle)
      logger.debug(feat_a_circle.points())
      logger.debug(feat_a_circle.average())
      logger.debug("Got verify A err_table")
      logger.debug(a_results)
      a_max_abs_err_pos = 0
      a_max_abs_err = 0
      self.a_ver_err = a_results
      for (pos, err) in a_results:
        if abs(err) > a_max_abs_err:
          a_max_abs_err = abs(err)
          a_max_err_pos = pos
      self.spec['a_max_err']['val'] = a_max_abs_err
      a_in_spec = a_max_abs_err < SPEC_ANGULAR_ACCURACY
      self.spec['a_max_err']['pass'] = a_in_spec
    except Exception as ex:
      logger.error("calc_verify exception (a results): %s" % str(ex))
      raise ex
    
    #do not return False here when spec fails
    #unlike some earlier stages that perform a spec check
    #because we want to run stage write_verify either way
    self.status['spec_failure'] = (not a_in_spec) or (not b_in_spec)
    return True


  async def write_verify(self):
    '''
    Reload any needed feature data from before the verification-reboot
    Re-setup coordinate system using XYZ data from calib
    B verify
      project the b-angle lines to top plane
      for each b-axis position
        translate projected points into POCKET coord sys
        find best-fit line through projected points
      for each line
        find angle compared to 0, compare to nominal
    A verify
      project the a-angle lines to POCKET coord sys YZ plane
      for each a-axis position
        translate projected points into POCKET space
        find best-fit line through projected points
      for each line
        find angle compared to 0, compare to nominal
    '''
    offsets = {}

    '''
    B results
    '''
    try:
      with open( os.path.join(RESULTS_DIR, 'ver_b.err'), 'w') as f:
        for p in self.b_ver_err:
          f.write("%0.6f %0.6f\n" % (p[0], p[1]))
    except Exception as ex:
      logger.error("write_verify exception (writing b): %s" % str(ex))
      raise ex

    '''
    A results
    '''
    try:
      with open( os.path.join(RESULTS_DIR, 'ver_a.err'), 'w') as f:
        for p in self.a_ver_err:
          f.write("%0.6f %0.6f\n" % (p[0], p[1]))
    except Exception as ex:
      logger.error("write_verify exception (writing a): %s" % str(ex))
      raise ex

    '''
    Linear Axes
    The home offset values should align the axes so that 
    the axes normals intersects the center of rotation for the 
    associated rotational axis
    X HOME_OFFSET aligns Z-norm with B COR
    Y HOME_OFFSET aligns Z-norm with A COR
    Z HOME_OFFSET places tool tip TOOL_OFFSET away from B COR

    First find Y offset, so that height of Z-norm relative to B-points is known
      Find center of rotation of A
      vertically offset CoR using the position Y was in while A was characterized
      find additional Y-offset needed so Z-norm intersects the CoR
    Second find X offset
      Find the point where B-norm intersects the plane that is...
        parallel to XZ
        at height (Y pos) where Z-norm intersects A-CoR
      Find the X-offset needed so Z-norm intersects this point
    '''
        '''
    This is the final CalibManager stage. Disconnect from CMM
    '''
    await self.disconnect_from_cmm()
    return True
