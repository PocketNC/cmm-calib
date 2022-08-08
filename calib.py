'''
Prototype of V2 calibration routine
'''
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


#import path to in-house modules depends on the platform
import platform
this_platform = platform.platform()
if 'x86' in this_platform:
  print('x86 os')
  sys.path.append("/Users/jakedanczyk/source/ippclient")
  from ipp import Client, TransactionCallbacks, float3, CmmException, readPointData
  import ipp_routines as routines

  sys.path.append("/Users/jakedanczyk/source/nfs/pocketnc/Settings")
  import metrology
elif 'arm' in this_platform:
  print('arm os')
  sys.path.append("/opt/ippclient")
  from ipp import Client, TransactionCallbacks, float3, CmmException, readPointData
  import ipp_routines as routines

  sys.path.append("/opt/pocketnc/Settings")
  import metrology
  sys.path.append("/opt/pocketnc/Rockhopper")
  import ini

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

def reload():
  importlib.reload(routines)
  importlib.reload(metrology)


IPADDR_CMM = "10.0.0.1"
PORT_CMM = 1294

POCKETNC_DIRECTORY = "/var/opt/pocketnc"
POCKETNC_VAR_DIRECTORY = os.environ.get('POCKETNC_VAR_DIRECTORY')
calibrationOverlayFileName = os.path.join(POCKETNC_VAR_DIRECTORY, "CalibrationOverlay.inc")
mainPocketNCINIFileName = os.path.join(POCKETNC_VAR_DIRECTORY, "PocketNC.ini")
aAngleFileName = os.path.join(POCKETNC_VAR_DIRECTORY, "calib/a-angle.txt")
bAngleFileName = os.path.join(POCKETNC_VAR_DIRECTORY, "calib/b-angle.txt")
xyzFileName = os.path.join(POCKETNC_VAR_DIRECTORY, "calib/xyz.txt")
newCalFilename = os.path.join(POCKETNC_VAR_DIRECTORY, "calib/CalibrationOverlay.inc")
aCompFileName = os.path.join(POCKETNC_VAR_DIRECTORY, "calib/a.comp")
bCompFileName = os.path.join(POCKETNC_VAR_DIRECTORY, "calib/b.comp")


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
  SETUP_CNC_CSY = auto()
  PROBE_MACHINE_POS = auto()
  PROBE_TOP_PLANE = auto()
  PROBE_X = auto() 
  PROBE_Y = auto()
  PROBE_Z = auto()
  PROBE_A = auto()
  PROBE_B = auto()
  '''
  the find_pos_ steps use the same methods as probe_ steps, 
  but use a different name for the metrology feature, 
  and return the measured value instead of returning nothing
  '''
  FIND_POS_A = auto() 
  FIND_POS_B = auto()
  WRITE_RESULTS = auto()
  VERIFY = auto()
  DISCONNECT_FROM_CMM = auto()

  @classmethod
  def has_name(cls, name):
      return name in cls._member_names_ 

'''
A stage is a progress point in the Calibration Process
The CalibManager
Steps may be repeated with (e.g. probing) or not (setting up coord systems)
'''
class Stages(Enum):
  SETUP_CMM = auto()
  PROBE_MACHINE_POS = auto()
  SETUP_PART_CSY = auto()
  # PROBE_X = auto()
  # PROBE_Y = auto()
  PROBE_Z = auto()
  PROBE_TOP_PLANE = auto()
  SETUP_CNC_CSY = auto()
  PROBE_A = auto()
  PROBE_B = auto()
  WRITE_RESULTS = auto()
  VERIFY = auto()

  @classmethod
  def has_name(cls, name):
      return name in cls._member_names_ 


'''
Some steps have prequisite stages that must be completed before they can be run
'''
STEP_PREREQS = {
  Steps.GO_TO_CLEARANCE_Z: [Stages.SETUP_CMM],
  Steps.GO_TO_CLEARANCE_Y: [Stages.SETUP_CMM],
  Steps.PROBE_MACHINE_POS: [Stages.SETUP_CMM],
  # Steps.SETUP_PART_CSY: [Stages.PROBE_MACHINE_POS],
  Steps.PROBE_TOP_PLANE: [Stages.SETUP_PART_CSY],
  # Steps.PROBE_X: [Stages.SETUP_PART_CSY],
  # Steps.PROBE_Y: [Stages.SETUP_PART_CSY],
  Steps.PROBE_Z: [Stages.SETUP_PART_CSY],
  # Steps.SETUP_CNC_CSY: [Stages.PROBE_Z, Stages.PROBE_TOP_PLANE],
  Steps.FIND_POS_A: [Stages.SETUP_CNC_CSY],
  Steps.FIND_POS_B: [Stages.SETUP_CNC_CSY],
  Steps.PROBE_A: [Stages.SETUP_CNC_CSY],
  Steps.PROBE_B: [Stages.SETUP_CNC_CSY],
  # Steps.WRITE_RESULTS: [Stages.PROBE_A, Stages.PROBE_B],
  # Steps.VERIFY: [Stages.WRITE_RESULTS],
}


STAGE_PREREQS = {
  Stages.PROBE_MACHINE_POS: [Stages.SETUP_CMM],
  Stages.SETUP_PART_CSY: [Stages.PROBE_MACHINE_POS],
  # Stages.PROBE_X: [Stages.SETUP_PARTCSY],
  # Stages.PROBE_Y: [Stages.SETUP_PARTCSY],
  Stages.PROBE_Z: [Stages.SETUP_PART_CSY],
  Stages.PROBE_TOP_PLANE: [Stages.SETUP_PART_CSY],
  Stages.SETUP_CNC_CSY: [Stages.PROBE_Z, Stages.PROBE_TOP_PLANE],
  Stages.PROBE_A: [Stages.SETUP_CNC_CSY],
  Stages.PROBE_B: [Stages.SETUP_CNC_CSY],
  # Stages.WRITE_RESULTS: [Stages.PROBE_A, Stages.PROBE_A],
}

STATE_RUN = 'RUN'
STATE_IDLE = 'IDLE'
STATE_ERROR = 'ERROR'
STATE_PAUSE = 'PAUSE'
STATE_STOP = 'STOP'

FIXTURE_SIDE = 76.2
FIXTURE_DIAG = 107.76
TOP_BACK_RIGHT_TO_ORIG = float3()
X_ORIGIN = 528
Y_ORIGIN = 238
Z_ORIGIN = 400
CMM_ORIGIN = float3(X_ORIGIN,Y_ORIGIN,Z_ORIGIN)

Z_BALL_DIA = 6.35
PROBE_DIA = 4

TOOL_3_LENGTH = 117.8
B_LINE_LENGTH = 35

Z_MIN_LIMIT_V2_10_INCHES = -3.451
Z_MIN_LIMIT_V2_50_INCHES = -3.541

Z_MIN = -26
Z_STEP = -25
B_STEP = 5
B_MIN = 0
B_MAX = 360
A_STEP = 5
A_MIN = -25
A_MAX = 135
X_PROBE_END_TRIGGER = -24
Y_PROBE_END_TRIGGER = -24
Z_PROBE_END_TRIGGER = -24
A_PROBE_END_TRIGGER = 129
B_PROBE_END_TRIGGER = 356

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
OFFSET_B_POS_REL_Z = 135
OFFSET_B_NEG_REL_Z = 225
OFFSET_A_REL_Z = 90

PROBING_POS_Y = -63.499

D_MAT = np.array([[0,1,0,0],[0,0,1,0],[0,0,0,1],[1,1,1,1]])

#z pos is screwed up because we are mixing 2 origin positions
waypoints_from_pocket_origin = {
  'origin': float3(0.0, 0.0, 0.0),
  'top_l_bracket_front_right': float3(0.0, 0.0, 0.0),
  'top_l_bracket_back_right': float3(18.4, 22.5, 0.0),
  'probe_fixture_tip': float3(-33.90, -129.5, -83.53),
  'probe_fixture_tip_from_origin': float3(-406.7, -598.5, -199.33),
  # move maybe -2 in X from b_cor 
  # have now done that (from -108.87 to -110.87)
  # well... that was Y... soooo.... lets go from (-60.33, -110.87) to (-62.33, -108.87)
  # now looks like maybe -5 in Y from b_cor, start with -3... from (-62.33, -108.87, -0.33) to (-62.33, -111.87, -0.33)

  'b_rot_cent_approx': float3(-62.33, -111.87, -0.33),
  'a_rot_cent_approx': float3(-63.10, -35.70, -6.93),
  'z_home_50': float3(42.5, -49.25, -68.83),
  'z_home_10': float3(72.0, -50.50, -68.83),
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

class CalibManager:
  def __init__(self):
    self.client = None
    self.metrologyManager = metrology.FeatureManager.getInstance()
    self.next_feature_id = 1
    self.feature_ids = {}
    # keys for fitted_features dict are the feature names, the values are tuples. 
    # you'll need to look at the metrology module to see the meaning of the tuple values
    self.fitted_features = {}

    self.run_state = None
    self.is_running = False
    self.stages_completed = {}
    for stage in Stages:
      self.stages_completed[str(stage)[len("Stages."):]] = False
    # self._stage2methods_map_ = {
    #   Stages.SETUP_CMM = self.setupCMM,
    #   Stages.VERIFY_POS = self.verifyMachinePos,
    #   Stages.SETUP_CMM = self.setupCMM,
    # }
    self.is_started_a = False
    self.is_started_b = False

    # self.model = "V2-10"
    # waypoints['z_home'] = waypoints['z_home_10']
    # self.machine_props = V2_10_PROPS
    # self.model = "V2-50"
    # waypoints['z_home'] = waypoints['z_home_50']
    # self.machine_props = V2_50_PROPS

    self.is_cmm_error = False
    self.table_slot = None
    self.part_csy_pos = None
    self.part_csy_euler = None

    self.a_probes = []
    self.b_probes = []

    self.ini_data = ini.read_ini_data(mainPocketNCINIFileName)
    self.overlay_data = ini.read_ini_data(calibrationOverlayFileName)

    self.active_offsets = {}
    self.active_offsets['x'] =  float(ini.get_parameter(self.ini_data, "JOINT_0", "HOME_OFFSET")["values"]["value"])
    self.active_offsets['y'] =  float(ini.get_parameter(self.ini_data, "JOINT_1", "HOME_OFFSET")["values"]["value"])
    self.active_offsets['z'] =  float(ini.get_parameter(self.ini_data, "JOINT_2", "HOME_OFFSET")["values"]["value"])
    self.active_offsets['a'] =  float(ini.get_parameter(self.ini_data, "JOINT_3", "HOME_OFFSET")["values"]["value"])
    self.active_offsets['b'] =  float(ini.get_parameter(self.ini_data, "JOINT_4", "HOME_OFFSET")["values"]["value"])

    # asyncio.create_task(self.zmq_listen())

    # # zmq client
    # print('zmq stuff')
    # self.zmq_context = zmq.Context()
    # self.zmq_socket = self.zmq_context.socket(zmq.REQ)
    # self.zmq_socket.connect('tcp://127.0.0.1:5555')
    # self.zmq_socket.send("yoooo")
    # msg = socket.recv()
    # print(msg)


  def getInstance():
    global calibManagerInstance
    if calibManagerInstance == None:
      calibManagerInstance = CalibManager()
    return calibManagerInstance

  async def zmq_listen(self):
    # context = zmq.Context()
    # socket = context.socket(zmq.REQ)
    socket = ctx.socket(zmq.REQ)
    socket.connect('tcp://127.0.0.1:5555')
    while True:
      print('awaiting zmq message')
      msg = await socket.recv()
      print('got zmq message')
      print(msg)
      # socket.send_string("yoooooo")
      # await socket.send_pyobj(self.stages_completed)
      # msg = socket.recv()
    socket.close()

  def zmq_report(self, why_string='UPDATE'):
    print('zmq report start')
    try:
      report = {}
      report['why'] = why_string
      report['stages'] = self.stages_completed
      report['state'] = self.run_state

      context = zmq.Context()
      socket = context.socket(zmq.PUSH)
      socket.bind('tcp://127.0.0.1:5555')
      # socket.send_string("yoooooo")
      # socket.send_pyobj(self.stages_completed)

      socket.send_json(report)
      print('zmq report sent')
      # msg = socket.recv()
    except Exception as e:
        print('exception in zmq_report')
        print(e)

  def reload_features(self):
    for f in self.metrologyManager:
      print(f)

  def add_feature(self,feature_name):
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

  def save_features(self):
    try:
      savefile_path = ''
      if os.path.isdir('/sysroot/home/pocketnc'):
        savefile_path = '/sysroot/home/pocketnc/savefile'
      else:
        savefile_path = '/home/pocketnc/savefile'
      with open(savefile_path, 'w') as f:
        for k in self.feature_ids.keys():
          if True:#k.find("proj") == -1 and k.find("b_circle") == -1:
            f.write("\n\n%s\n" % k)
            fid = self.feature_ids[k]
            feat = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
            for pt in feat.points():
              f.write("%s\n" % pt)
    except Exception as e:
      print("Exception saving features")
      print(e)
      raise e

  def load_features(self):
    try:
      savefile_path = ''
      if os.path.isdir('/sysroot/home/pocketnc'):
        savefile_path = '/sysroot/home/pocketnc/savefile'
      else:
        savefile_path = '/home/pocketnc/savefile'
      with open(savefile_path, 'r') as f:
        lines = f.readlines()
        i = 0
        while i < len(lines)-2:
          if lines[i] == '\n' and lines[i+1] == '\n':
            feat_name = lines[i+2][:-1]
            # print(feat_name)
            if feat_name.find("find-") == 0 and feat_name.rfind("-proj") == 15:
              if "find-a-" in feat_name:
                self.last_find_a_proj_name = feat_name
              if "find-b-" in feat_name:
                self.last_find_b_proj_name = feat_name
            elif feat_name.find("probe-") == 0:
              # these are the A and B characterization features
              axis = feat_name[len("probe-")]
              if feat_name[len("probe-")] == 'b':
                self.b_probes.append(feat_name)
              elif feat_name[len("probe-")] == 'a':
                self.a_probes.append(feat_name)

            feat = self.add_feature(feat_name)
            j = i + 3
            while j<len(lines) and lines[j] != '\n':
              pt = []
              for n in lines[j].split():
                # print(n)
                try:
                  num = float(n.replace('[', '').replace(']', ''))
                except ValueError:
                  continue
                # print(num)
                pt.append(num)
              feat.addPoint(*pt)
              j = j + 1
            i = j
      print("_______END___________")
      print(feat.points())
      print(self.last_find_a_proj_name)
      print(self.last_find_b_proj_name)
    except Exception as e:
      print("Exception in load_features")
      print(e)
  
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
    if step in [
      Steps.CONNECT_TO_CMM, Steps.GO_TO_CLEARANCE_Z, Steps.GO_TO_CLEARANCE_Y,
      Steps.FIND_POS_A, Steps.FIND_POS_B, Steps.DISCONNECT_FROM_CMM
    ]:
      return (False, None)
    elif step is Steps.SETUP_CMM:
      return (self.std_stage_complete_check(ret), Stages.SETUP_CMM)
    elif step is Steps.SETUP_PART_CSY:
      return (self.std_stage_complete_check(ret), Stages.SETUP_PART_CSY)
    elif step is Steps.SETUP_CNC_CSY:
      return (self.std_stage_complete_check(ret), Stages.SETUP_CNC_CSY)
    elif step is Steps.PROBE_MACHINE_POS:
      return (self.std_stage_complete_check(ret), Stages.PROBE_MACHINE_POS)
    elif step is Steps.PROBE_TOP_PLANE:
      return (self.std_stage_complete_check(ret), Stages.PROBE_TOP_PLANE)
    elif step is Steps.PROBE_A:
      print('checking if PROBE_A completed')
      print(*args)
      if ret is True:
        nominal_a_pos = args[2]
        if nominal_a_pos >= A_PROBE_END_TRIGGER:
          return(True, Stages.PROBE_A)
        else:
          return (False, Stages.PROBE_A)
      else:
        return (False, Stages.PROBE_A)
    elif step is Steps.PROBE_B:
      print('checking if PROBE_B completed')
      print(*args)
      if ret is True:
        nominal_b_pos = args[2]
        if nominal_b_pos >= B_PROBE_END_TRIGGER:
          return(True, Stages.PROBE_B)
        else:
          return (False, Stages.PROBE_B)
      else:
        return (False, Stages.PROBE_B)
    elif step is Steps.PROBE_X:
      print('checking if PROBE_X completed')
      print(*args)
      if ret is True:
        nominal_x_pos = args[1]
        if nominal_x_pos <= X_PROBE_END_TRIGGER:
          return(True, Stages.PROBE_X)
        else:
          return (False, Stages.PROBE_X)
      else:
        return (False, Stages.PROBE_X)
    elif step is Steps.PROBE_Y:
      print('checking if PROBE_Y completed')
      print(*args)
      if ret is True:
        nominal_y_pos = args[1]
        if nominal_y_pos <= Y_PROBE_END_TRIGGER:
          return(True, Stages.PROBE_Y)
        else:
          return (False, Stages.PROBE_Y)
      else:
        return (False, Stages.PROBE_Y)
    elif step is Steps.PROBE_Z:
      print('checking if PROBE_Z completed')
      print(*args)
      if ret is True:
        nominal_z_pos = args[1]
        print(nominal_z_pos)
        if nominal_z_pos <= Z_PROBE_END_TRIGGER:
          print('yes z probe complete')
          return(True, Stages.PROBE_Z)
        else:
          print('no z probe not complete')
          return (False, Stages.PROBE_Z)
      else:
        return (False, Stages.PROBE_Z)
    elif step is Steps.WRITE_RESULTS:
      return (self.std_stage_complete_check(ret), Stages.WRITE_RESULTS)
    else:
      #doubled up until I confirm logger is configured to print
      print("Ran a STEP without entry in did_step_complete_stage")
      logger.debug("Ran a STEP without entry in did_step_complete_stage")


  '''
  the find_pos_ steps use the same methods as probe_ steps, 
  but use a different name for the metrology feature, 
  and return the measured value instead of returning nothing
  '''
  def run_step(self, step, *args):
    print("Running step: %s " % step)
    # an exception will be thrown if step is not defined in the Steps enum
    if type(step) is str:
      step = Step[step.upper()]
    step in Steps
    print('step is defined')
    if not self.is_ready_for_step(step):
      print('not ready for step')
      return err_msg("1 or more prerequisite STAGES not complete before running STEP %s")
    print('ready for step')
    step_method = getattr(self, step.name.lower())
    print('step method is %s ' % str(step_method))
    try:
      self.run_state = STATE_RUN
      self.is_running = True
      self.zmq_report('UPDATE')
      step_ret = asyncio.get_event_loop().run_until_complete(step_method(*args))
      print('step ran')
      did_a_stage_complete, stage_for_step = self.did_step_complete_stage(step, step_ret, *args)
      if did_a_stage_complete:
        #putting this in an if statement because maybe some steps will be run again 
        #AFTER their 'normal' stage is completed
        self.stages_completed[str(stage_for_step)[len("Stages."):]] = did_a_stage_complete
        self.is_running = False
        self.run_state = STATE_IDLE
        logger.debug('completing step %s' % step)
      
      logger.debug('step %s return value %s' % (step, step_ret))
      self.zmq_report('STEP_COMPLETE')
      print('returning %s' % step_ret)
      return step_ret
      # self.stages_completed[stage] = isStageComplete
    except CmmException as e:
      msg = err_msg("Failed running step %s, exception message %s" % (step, str(e)))
      self.is_running = False
      self.run_state = STATE_ERROR
      self.zmq_report('ERROR')
      print(msg)
      logger.debug(msg)
      return msg

  # def run_stage(self, stage, *args):
  #   print("Running stage: %s " % stage)
  #   # an exception will be thrown if stage is not defined in the Stages enum
  #   if type(stage) is str:
  #     print("type is string")
  #     stage = Stages[stage.upper()]
  #     print("stage is ")
  #     print(stage)
  #   stage in Stages
  #   print('stage in stages')

  #   # try:
  #   #   if self.client is not None and self.client.stream is not None:
  #   #     print('trying to disconnect')
  #   #     asyncio.get_event_loop().run_until_complete(self.client.disconnect())
  #   # except Exception as e:
  #   #   print("disconnect e: %s" % e)

  #   if stage in STAGE_PREREQS:
  #     print('stage has prereqs')

  #     for prereqStage in STAGE_PREREQS[stage]:
  #       print(prereqStage)
  #       if not self.stages_completed[str(prereqStage)[len("Stages."):]]:
  #         self.run_stage(prereqStage)
    
  #   # if stage != Stages.CONNECT_TO_CMM and (self.client is None or self.client.stream is None):
  #   #   #something broken, quit
  #   #   return "FAILED"

  #   stage_method = getattr(self, stage.name.lower())
  #   try:
  #     isStageComplete = asyncio.get_event_loop().run_until_complete(stage_method(*args))
  #     print('completing stage %s' % stage)
  #     self.stages_completed[str(stage)[len("Stages."):]] = isStageComplete
  #   except CmmException as e:
  #     print("Failed running stage %s" % stage)
  #     return "FAILED"


  async def connect_to_cmm(self):
    try:
      if self.client.stream is not None:
        #assume we are connected if we have a stream?
        return True
        # print('trying to disconnect')
        # asyncio.get_event_loop().run_until_complete(self.client.disconnect())
    except Exception as e:
      print("disconnect e: %s" % e)
    
    try:
      self.client = None
      self.client = Client(IPADDR_CMM, PORT_CMM)
      await self.client.connect()
      return True
    except Exception as e:
      print("Exception while connecting")

  async def setup_cmm(self):
    await self.client.ClearAllErrors().complete()
    await routines.ensure_homed(self.client)
    await routines.ensure_tool_loaded(self.client, "Component_3.1.50.4.A0.0-B0.0")
    await self.client.SetProp("Tool.GoToPar.Speed(500)").ack()
    await self.client.SetProp("Tool.GoToPar.Accel(250)").ack()
    await self.client.SetCsyTransformation("PartCsy, 0,0,0,0,0,0").complete()
    # await self.client.SetCsyTransformation("MachineCsy, 0,0,0,0,0,0").complete()
    await self.client.SetCoordSystem("MachineCsy").complete()
    await self.set_table_slot("front_right")
    return True

  async def go_to_clearance_z(self):
    await self.client.GoTo("Z(250)").complete()
    return False

  async def go_to_clearance_y(self):
    await self.client.GoTo("Y(-200)").complete()
    return False

  async def set_table_slot(self, slot):
    self.table_slot = slot
    self.part_csy_pos = waypoints_table[slot + '_slot_origin']
    self.part_csy_euler = table_slot_euler_angles[slot]
    # await self.set_part_csy(waypoints_table['front_right_slot_origin'], table_slot_euler_angles['front_right'])

  async def set_part_csy(self,csy_pos,csy_euler):
    self.part_csy_pos = csy_pos
    self.part_csy_euler = csy_euler
    await self.client.SetCsyTransformation("PartCsy, %s, %s, %s, %s, %s, %s" % (self.part_csy_pos.x, self.part_csy_pos.y, self.part_csy_pos.z, self.part_csy_euler[0], self.part_csy_euler[1], self.part_csy_euler[2])).complete()
    await self.client.SetCoordSystem("PartCsy").complete()
    return True

  async def probe_machine_pos(self):
    '''
    Locate machine and verify it is in home position
    '''
    try:
      print("Checking machine position")
      if self.table_slot is None:
        #table slot has not been set, raise exception,
        raise CalibException("Quitting VERIFY_MACHINE_POS, table slot has not been set")
      await self.set_part_csy(self.part_csy_pos, self.part_csy_euler)
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
      L_bracket_top_face = self.add_feature('L_bracket_top_face')
      ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % (measPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      L_bracket_top_face.addPoint(pt.x, pt.y, pt.z)

      measPos = float3(-20,0,0)
      ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % (measPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      L_bracket_top_face.addPoint(pt.x, pt.y, pt.z)

      measPos = float3(-35,12,0)
      ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % (measPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      L_bracket_top_face.addPoint(pt.x, pt.y, pt.z)

      # L-bracket back face
      L_bracket_back_face = self.add_feature('L_bracket_back_face')
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
      L_bracket_right_face = self.add_feature('L_bracket_right_face')
      for i in range(3):
        meas_pos = approach_pos + float3(-15, i * -8, 0)
        pt_meas = await self.client.PtMeas("%s,IJK(1,0,0)" % (meas_pos.ToXYZString())).complete()
        pt = float3.FromXYZString(pt_meas.data_list[0])
        L_bracket_right_face.addPoint(pt.x, pt.y, pt.z)

      # #locate the probe fixtures vertical fin
      #
      # self.next_feature_id = self.next_feature_id + 1
      # self.feature_ids['probeFixtureFinRightFace'] = self.next_feature_id
      # self.metrologyManager.getActiveFeatureSet().setActiveFeatureID(self.next_feature_id)
      # probeFixtureFinRightFace = self.metrologyManager.getActiveFeatureSet().getActiveFeature()

      # currOrig = ORIGIN + waypoints['probe_fixture_tip_from_origin']
      # await self.client.SetProp("Tool.PtMeasPar.HeadTouch(1)").complete()
      # debugWait()
      # nextPos = currOrig + float3(20,0,20)
      # await self.client.GoTo("%s,Tool.A(0),Tool.B(-90)" % (nextPos.ToXYZString())).complete()
      # debugWait()
      # nextPos = currOrig + float3(20,0,-10)
      # await self.client.GoTo("%s" % (nextPos.ToXYZString())).complete()
      # debugWait()
      # nextPos = currOrig + float3(0,0,-10)
      # ptMeas = await self.client.PtMeas("%s,IJK(1,0,0)" % (nextPos.ToXYZString())).complete()
      # pt = float3.FromXYZString(ptMeas.data_list[0])
      # probeFixtureFinRightFace.addPoint(pt.x, pt.y, pt.z)
      # nextPos = currOrig + float3(0,+3,0)
      # ptMeas = await self.client.PtMeas("%s,IJK(1,0,0)" % (nextPos.ToXYZString())).complete()
      # pt = float3.FromXYZString(ptMeas.data_list[0])
      # probeFixtureFinRightFace.addPoint(pt.x, pt.y, pt.z)
      # debugWait()
      # nextPos = currOrig + float3(0,+6,0)
      # ptMeas = await self.client.PtMeas("%s,IJK(1,0,0)" % (nextPos.ToXYZString())).complete()
      # pt = float3.FromXYZString(ptMeas.data_list[0])
      # probeFixtureFinRightFace.addPoint(pt.x, pt.y, pt.z)
      # debugWait()

      # self.next_feature_id = self.next_feature_id + 1
      # self.feature_ids['probeFixtureFinAngleFace'] = self.next_feature_id
      # self.metrologyManager.getActiveFeatureSet().setActiveFeatureID(self.next_feature_id)
      # probeFixtureFinAngleFace = self.metrologyManager.getActiveFeatureSet().getActiveFeature()
      # await self.client.GoTo("Tool.A(0),Tool.B(-135)").complete()
      # debugWait()
      # await self.client.GoTo("Tool.A(30),Tool.B(-135)").complete()
      # debugWait()
      # await self.client.GoTo("Tool.A(30),Tool.B(-90)").complete()
      # nextPos = currOrig + float3(-10,0,-10)
      # ptMeas = await self.client.PtMeas("%s,IJK(-1,0,1)" % (nextPos.ToXYZString())).complete()
      # pt = float3.FromXYZString(ptMeas.data_list[0])
      # probeFixtureFinAngleFace.addPoint(pt.x, pt.y, pt.z)
      # nextPos = currOrig + float3(-10,-2,-10)
      # ptMeas = await self.client.PtMeas("%s,IJK(-1,0,1)" % (nextPos.ToXYZString())).complete()
      # pt = float3.FromXYZString(ptMeas.data_list[0])
      # probeFixtureFinAngleFace.addPoint(pt.x, pt.y, pt.z)
      # nextPos = currOrig + float3(-10,+2,-10)
      # ptMeas = await self.client.PtMeas("%s,IJK(-1,0,1)" % (nextPos.ToXYZString())).complete()
      # pt = float3.FromXYZString(ptMeas.data_list[0])
      # probeFixtureFinAngleFace.addPoint(pt.x, pt.y, pt.z)

      # await self.client.GoTo("Z(%s)" % (nextPos.z + 150)).complete()
      # await self.client.GoTo("Tool.A(0),Tool.B(0)").complete()
      return True
    except Exception as ex:
      print("exception %s" % str(ex))
      raise ex
      # await self.client.disconnect()


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
    print('intersect_pos')
    print(intersect_pos)
    pos_to_orig = -1*waypoints_from_pocket_origin['top_l_bracket_back_right']
 
    dist = np.linalg.norm( pos_to_orig )
    print('dist')
    print(dist)
    pos_to_orig_xy_ang = math.atan2( pos_to_orig.y, pos_to_orig.x ) * 180/math.pi
    ang_true_pos_to_true_orig = pos_to_orig_xy_ang - ang_machine_z_to_partcsy_x
    origin_offset = intersect_pos + float3(1,0,0) * dist * math.cos((ang_true_pos_to_true_orig)/180*math.pi) + float3(0,1,0) * dist * math.sin((ang_true_pos_to_true_orig)/180*math.pi)

    new_euler_angles = [self.part_csy_euler[0], self.part_csy_euler[1] - ang_machine_z_to_partcsy_x, self.part_csy_euler[2]]
    print(origin_offset)
    print(new_euler_angles)
    new_orig = self.part_csy_pos - origin_offset
    print('new_orig')
    print(new_orig)
    await self.set_part_csy(new_orig, new_euler_angles)

    z_min_limit_ini = float(ini.get_parameter(self.ini_data, "JOINT_2", "MIN_LIMIT")["values"]["value"])
    if abs(z_min_limit_ini - Z_MIN_LIMIT_V2_10_INCHES) < 0.0001:
      self.model = "V2-10"
      waypoints['z_home'] = waypoints['z_home_10']
      self.machine_props = V2_10_PROPS
    elif abs(z_min_limit_ini - Z_MIN_LIMIT_V2_50_INCHES) < 0.0001:
      self.model = "V2-50"
      waypoints['z_home'] = waypoints['z_home_50']
      self.machine_props = V2_50_PROPS
    else:
      return err_msg("CMM calibration halting, Z MIN_LIMIT abnormal, doesn't correspond to known model of V2. Z MIN_LIMIT: %s" % z_min_limit_ini)

    return True


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

      fixture_top_face = self.add_feature('fixture_top_face')

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
      print("exception %s" % str(ex))
      raise ex
      # await self.client.disconnect()


  async def probe_x(self, x_pos_v2, z_pos_v2):
    '''
    Probe against the ball mounted in the spindle
    Probe should be aligned vertical (or close) before running
    The first move is to the position 25mm above the target
    '''
    orig = waypoints['z_home'] + float3(z_pos_v2,x_pos_v2 - 63.5,0)
    contact_radius = (Z_BALL_DIA+PROBE_DIA)/2
    a_angle_probe_contact = math.atan2(contact_radius,TOOL_3_LENGTH)*180/math.pi
    feature_name = "x_%.2f" % x_pos_v2
    feature_x_pos = self.add_feature(feature_name)
    try:
      await self.client.GoTo("Tool.Alignment(0,0,1,1,0,0)").complete()
      await self.client.GoTo((orig + float3(0,0,25)).ToXYZString()).complete()
      await self.client.GoTo("Tool.A(%s),Tool.B(-180)" % (a_angle_probe_contact+1)).complete()
      getCurrPosCmd = await self.client.Get("X(),Y(),Z()").complete()
      currPos = readPointData(getCurrPosCmd.data_list[0])
      await self.client.GoTo((currPos + float3(0,0,-25)).ToXYZString()).complete()
      
      await self.client.SetProp("Tool.PtMeasPar.HeadTouch(1)").complete()
      await self.client.SetProp("Tool.PtMeasPar.Search(5)").complete()

      ptMeas = await self.client.PtMeas("%s,IJK(0,1,0)" % ((orig + float3(0,contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_x_pos.addPoint(*pt)

      await self.client.GoTo("Tool.A(%s),Tool.B(-90)" % (a_angle_probe_contact+1)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(-1,0,0)" % ((orig + float3(-contact_radius,0,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_x_pos.addPoint(*pt) 

      await self.client.GoTo("Tool.A(%s),Tool.B(0)" % (a_angle_probe_contact+1)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(0,-1,0)" % ((orig + float3(0,-contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_x_pos.addPoint(*pt) 

      #rise 2mm and probe another 3 points
      getCurrPosCmd = await self.client.Get("X(),Y(),Z()").complete()
      currPos = readPointData(getCurrPosCmd.data_list[0])
      await self.client.GoTo((currPos + float3(0,0,+2)).ToXYZString()).complete()

      measPos2 = orig + float3(0,0,2)
      ptMeas = await self.client.PtMeas("%s,IJK(0,1,0)" % ((measPos2 + float3(0,contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_x_pos.addPoint(*pt)

      await self.client.GoTo("Tool.A(%s),Tool.B(-90)" % (a_angle_probe_contact+1)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(-1,0,0)" % ((measPos2 + float3(-contact_radius,0,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_x_pos.addPoint(*pt) 

      await self.client.GoTo("Tool.A(%s),Tool.B(0)" % (a_angle_probe_contact+1)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(0,-1,0)" % ((measPos2 + float3(0,-contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_x_pos.addPoint(*pt) 

      await self.client.GoTo((orig + float3(0,0,50)).ToXYZString()).complete()
      return False
    except Exception as ex:
      print("exception %s" % str(ex))
      raise ex
      # await self.client.disconnect()


  async def probe_z(self, x_pos_v2, z_pos_v2):
    '''
    Probe against the ball mounted in the spindle
    Probe should be aligned vertical (or close) before running
    The initial movement orients the tool downwards and then moves the position to 25mm above the target
    '''
    orig = waypoints['z_home'] + float3(z_pos_v2,x_pos_v2 - 63.5,0)
    contact_radius = (Z_BALL_DIA+PROBE_DIA)/2
    a_angle_probe_contact = math.atan2(contact_radius,TOOL_3_LENGTH)*180/math.pi
    feature_name = "z_%d" % z_pos_v2
    feature_z_pos = self.add_feature(feature_name)
    try:
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
      feature_z_pos.addPoint(*pt)

      # move to -X pos, probe in +X dir
      await self.client.GoTo("Tool.A(%s),Tool.B(180)" % (a_angle_probe_contact+5)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(-1,0,0)" % ((orig + float3(-contact_radius,0,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_z_pos.addPoint(*pt)

      # move to -Y pos, probe in +Y dir
      await self.client.GoTo("Tool.A(%s),Tool.B(270)" % (a_angle_probe_contact+5)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(0,-1,0)" % ((orig + float3(0,-contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_z_pos.addPoint(*pt) 

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
      feature_z_pos.addPoint(*pt)

      # move to -X pos, probe in +X dir
      await self.client.GoTo("Tool.A(%s),Tool.B(180)" % (a_angle_probe_contact+5)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(-1,0,0)" % ((measPos2 + float3(-contact_radius,0,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_z_pos.addPoint(*pt) 

      # move to +Y pos, probe in -Y dir
      await self.client.GoTo("Tool.A(%s),Tool.B(90)" % (a_angle_probe_contact+5)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(0,1,0)" % ((measPos2 + float3(0,contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_z_pos.addPoint(*pt) 

      await self.client.GoTo((orig + float3(0,0,50)).ToXYZString()).complete()
      return True
    except Exception as ex:
      print("exception %s" % str(ex))
      raise ex


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
      print("Finding A position %s" % a_pos_v2)
      a_line = self.add_feature(feat_name)
      if "probe-a" in feat_name:
        self.a_probes.append(feat_name)

      total_angle =  fixture_offset_angle - a_pos_v2
      angle_start_pos = total_angle + angle_offset_start
      print('vec_cor_to_orig_start %s' % vec_cor_to_orig_start)
      print('angle_offset_start %s' % angle_offset_start)
      print('start origin Rad %s' % dist_cor_to_start)
      print('start pos angle %s' % angle_start_pos)
      start_pos = a_cor + float3(dist_cor_to_start * math.cos(angle_start_pos*math.pi/180), vec_a_cor_to_orig_fixture_tip.y,dist_cor_to_start * math.sin(angle_start_pos*math.pi/180))
      print('start pos %s' % start_pos)
      # await self.client.GoTo((a_cor + float3(0, -100, 150)).ToXYZString()).complete()
      # await self.client.GoTo("Tool.Alignment(0,-1,0)").complete()
      # await self.client.GoTo((start_pos + float3(0, -100, 150)).ToXYZString()).complete()
      # await self.client.GoTo((start_pos + float3(0, -100, 0)).ToXYZString()).complete()
      # await self.client.GoTo((start_pos + float3(0, -25, 0)).ToXYZString()).complete()

      drive_angle = total_angle - 90
      drive_vec = float3(math.cos(drive_angle*math.pi/180), 0, math.sin(drive_angle*math.pi/180))
      face_norm = float3(-drive_vec.z, 0, drive_vec.x)
      print("drive_vec %s" % drive_vec)
      try:
        #there are issues, the yz routine assumes the CNC is aligned so that its X-axis is parallel with the CMM MachineCsy X-axis
        if self.table_slot == "front_right":
          points = await routines.headprobe_line_yz(self.client,start_pos,drive_vec,B_LINE_LENGTH,face_norm,3,1)
        else:
          points = await routines.headprobe_line_xz(self.client,start_pos,drive_vec,B_LINE_LENGTH,face_norm,3,1)
        for pt in points:
          a_line.addPoint(*pt)
      except CmmException as e:
        pass

      end_pos = start_pos + drive_vec * B_LINE_LENGTH
      retract_pos = end_pos + face_norm * 10

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
      msg = "Exception in PROBE_A %s" % str(ex)
      logger.debug(msg)
      return err_msg(msg)
      # raise ex
      # await self.client.disconnect()


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
      print("Finding B position")
      # feat_name = "b-%d" % b_pos_v2
      b_line = self.add_feature(feat_name)
      if "probe-b" in feat_name:
        self.b_probes.append(feat_name)

      ang_bcor_to_startpos = (b_pos_v2 - 45) + ang_bcor_to_startpos45
      print('ang_bcor_to_startpos %s' % ang_bcor_to_startpos)
      ang_fixture_face = b_pos_v2 + FIXTURE_OFFSET_B_ANGLE
      print('ang_fixture_face %s' % ang_fixture_face)

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
      print('start pos %s' % start_pos)
      # await self.client.GoTo("Tool.Alignment(0,0,1)").complete()
      # await self.client.GoTo((orig + float3(0, 0, 100)).ToXYZString()).complete()
      # await self.client.GoTo((start_pos + float3(0, 0, 15)).ToXYZString()).complete()

      lineAngle = totAngle + 90
      lineVec = float3(math.cos(ang_fixture_face*math.pi/180), math.sin(ang_fixture_face*math.pi/180),0)
      # lineVec = float3(math.cos(lineAngle*math.pi/180), math.sin(lineAngle*math.pi/180),0)
      print("lineVec %s" % lineVec)

      # points = await routines.probe_line(self.client,start_pos, lineVec, float3(lineVec.y, -lineVec.x,0),65,10,3,1)
      try:
        points = await routines.headprobe_line(self.client,start_pos, lineVec, 35, 15, 3, 10, -1, 5)
      except CmmException as e:
        print("CmmException in probe_b, raising")
        raise e
      # await routines.headprobe_line(client,start_pos,float3(-1,0,0),75,10,5,10,-1,15)

      for pt in points:
        b_line.addPoint(*pt)

      # await self.client.GoTo("Z(%s)" % (orig.z + 15)).ack()

      return True
    except Exception as ex:
      print("exception %s" % str(ex))
      raise ex
      # await self.client.disconnect()


  async def find_pos_a(self, y_nominal, a_nominal):
    '''
    Probe A
    Project points onto POCKET coord sys YZ plane
    Translate projected points into POCKET space
    Compare angular position to Z-norm
    '''
    try:
      probeline_name = "find-a-%.6f" % a_nominal
      await self.probe_a(probeline_name, y_nominal, a_nominal)
    except Exception as e:
      print("Exception while probing a in find_pos_a")
      print(e)
      return e

    #translate projected points into POCKET coord sys
    #find best-fit line through projected points
    #find angle in YZ plane compared to Z-norm
    try:
      probeline_id = self.feature_ids[probeline_name]
      a_line = self.metrologyManager.getActiveFeatureSet().getFeature(probeline_id)
      proj_probeline_name = 'find-a-%.6f-proj' % a_nominal
      a_line_proj = self.add_feature(proj_probeline_name)
      for pt in a_line.points():
        #using the top plane point seems a bit strange because its not related to the YZ plane
        #but it should be fine, we just need to get the line on a plane to measure the angle
        plane_orig_to_pt = pt - self.fitted_features['fixture_top_face']['pt']
        dist = np.dot(plane_orig_to_pt, self.x_norm)
        proj_pt = pt - dist * self.x_norm
        a_line_proj.addPoint(proj_pt[0], proj_pt[1], proj_pt[2])
      
      a_line_proj_line = a_line_proj.line()
      self.fitted_features[proj_probeline_name] = {'pt': a_line_proj_line[0], 'dir': a_line_proj_line[1]}
      a_line_translated = np.matmul(self.cmm_to_cnc_mat, np.append(a_line_proj_line[1], 0))
      print('a_line_translated')
      print(a_line_translated)
      # z_norm_2d_xz = [self.z_norm[0], self.z_norm[2]]
      #don't use z_norm, that is in the PartCsy. We are in the ideal CNC Csy
      a_dir_2d_yz = [a_line_translated[1], a_line_translated[2]]
      # a_pos_rel_z = angle_between_ccw_2d(z_norm_2d_xz, a_dir_2d_yz)
      
      if a_line_translated[1] >= 0:
        a_pos_rel_z = angle_between_ccw_2d([1,0], a_dir_2d_yz)
      else:
        a_pos_rel_z = angle_between_ccw_2d([-1,0], a_dir_2d_yz)
      
      a_pos = a_pos_rel_z

      print("found A pos %s" % a_pos)
      self.last_find_a_proj_name = proj_probeline_name
      self.last_find_a = a_nominal
      return a_pos
    except Exception as e:
      print("Exception while calculating a position")
      print(e)
      return e

  async def find_pos_b(self, y_nominal, b_nominal):
    '''
    Probe B
    Project points onto plane of rotation defined by top surface of fixture
    Translate projected points into POCKET space
    Compare angular position to Z-norm
    '''
    try:
      probeline_name = "find-b-%.6f" % b_nominal
      await self.probe_b(probeline_name, y_nominal, b_nominal)
    except Exception as e:
      print("Exception while probing b in find_pos_b")
      print(e)
      return e

    #translate projected points into POCKET coord sys
    #find best-fit line through projected points
    #find angle in XZ plane compared to Z-norm
    try:
      fid = self.feature_ids[probeline_name]
      b_line = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
      proj_probeline_name = 'find-b-%.6f-proj' % b_nominal
      b_line_proj = self.add_feature(proj_probeline_name)
      for pt in b_line.points():
        plane_orig_to_pt = pt - self.fitted_features['fixture_top_face']['pt']
        dist = np.dot(plane_orig_to_pt, self.fitted_features['fixture_top_face']['norm'])
        proj_pt = pt - dist *  self.fitted_features['fixture_top_face']['norm']
        # proj_pt_in_plane = np.matmul(rot_mat,np.append(proj_pt,1))
        b_line_proj.addPoint(proj_pt[0], proj_pt[1], proj_pt[2])
      
      find_b_proj_line = b_line_proj.line()
      self.fitted_features[proj_probeline_name] = {'pt': find_b_proj_line[0], 'dir': find_b_proj_line[1]}
      b_line_translated = np.matmul(self.cmm_to_cnc_mat, np.append(find_b_proj_line[1], 0))
      print('b_line_translated')
      print(b_line_translated)
      # z_norm_2d = [self.z_norm[0], self.z_norm[2]]
      #don't use z_norm, that is in the PartCsy. We are in the ideal CNC Csy
      b_dir_2d = [b_line_translated[0],b_line_translated[2]]
      b_pos_rel_z = angle_between_ccw_2d([0,1], b_dir_2d)
      print("b_pos_rel_z %s" % b_pos_rel_z)
      if b_pos_rel_z >= 0:
        b_pos = OFFSET_B_POS_REL_Z - b_pos_rel_z
      else:
        b_pos = b_pos_rel_z - OFFSET_B_POS_REL_Z
      print("found B pos %s" % b_pos)
      self.last_find_b_proj_name = proj_probeline_name
      return b_pos
    except Exception as e:
      print("Exception while calculating b position")
      print(e)
      return e


  
  async def setup_cnc_csy(self):
    '''
    define CNC coord sys from TOP_PLANE and Z_VEC
    B results
      project the b-angle lines to top plane
      for each b-axis position
        translate projected points into POCKET coord sys
        find best-fit line through projected points
      for each line
        find angle compared to 0, compare to nominal
    '''
    dir_z_partcsy = None
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
    self.fitted_features['fixture_top_face'] = {'pt': top_plane_pt, 'norm': top_plane_norm}

    '''
    Use Z positions to define a line
    '''
    try:
      z_line = self.add_feature('z_line')
      z_line_proj = self.add_feature('z_line_proj')
      for i in range(0,Z_MIN,Z_STEP):
        fid = self.feature_ids['z_%d' % i]
        z_pos_feat = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
        z_pos_sphere = z_pos_feat.sphere()
        z_pos = z_pos_sphere[1]
        z_line.addPoint(*z_pos)
        #project point onto top-fixture-plane
        plane_orig_to_pt = z_pos - top_plane_pt
        dist = np.dot(plane_orig_to_pt, top_plane_norm)
        proj_pt = z_pos - dist * top_plane_norm
        z_line_proj.addPoint(*proj_pt)

      z_norm_real = -1*z_line.line()[1]
      # z_vec_proj = z_line_proj.line()[1]
      # z_vec_cross_top_plane_norm = np.cross(z_vec_proj,top_plane_norm)
      # print('z_vec_cross_top_plane_norm')
      # print(z_vec_cross_top_plane_norm)
      # affine_mat_construct = np.array([top_plane_pt,z_vec_proj,z_vec_cross_top_plane_norm,top_plane_norm])
      # print('affine_mat_construct')
      # print(affine_mat_construct)
      # affine_mat = np.vstack((affine_mat_construct.transpose(),[1,1,1,1]))
      # print('affine_mat')
      # print(affine_mat)
      # affine_mat_inv = np.linalg.inv(affine_mat)
      # print('affine_mat_inv')
      # print(affine_mat_inv)
      # rot_mat = np.matmul(D_MAT, affine_mat_inv)
      # print('rot_mat')
      # print(rot_mat)

    except Exception as e:
      print(e)
      return e

    #use results to create coord sys and tranformation matrices
    try:
      y_norm = top_plane_norm
      x_norm = np.cross(y_norm, z_norm_real)
      pocket_yz_plane_norm = x_norm
      z_norm = np.cross(x_norm,y_norm)
      p2m_construct = np.array([x_norm,y_norm,z_norm,top_plane_pt])
      p2m = np.vstack((p2m_construct.transpose(),[0,0,0,1]))
      m2p = np.linalg.inv(p2m)
      self.x_norm = x_norm
      self.y_norm = y_norm
      self.z_norm = z_norm
      self.cmm_to_cnc_mat = m2p
      return True
    except Exception as e:
      print(e)
      return e


  async def write_results(self):
    return self.write_results_sync()


  def write_results_sync(self):
    '''
    define POCKET coord sys in from TOP_PLANE and Z_VEC
    B results
      project the b-angle lines to top plane
      for each b-axis position
        translate projected points into POCKET coord sys
        find best-fit line through projected points
      for each line
        find angle compared to 0, compare to nominal
    A results
      project the a-angle lines to POCKET coord sys YZ plane
      for each a-axis position
        translate projected points into POCKET space
        find best-fit line through projected points
      for each line
        find angle compared to 0, compare to nominal
    '''
    print('sync')
    offsets = {}
    dir_z_partcsy = None
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
    #top_plane_norm should be pointing up
    if top_plane_norm[2] < 0:
      top_plane_norm = -1*top_plane_norm
    print('top_plane_norm')
    print(top_plane_norm)

    '''
    Z results
    Define a line along the z points
    '''
    try:
      z_line = self.add_feature('z_line')
      z_line_proj = self.add_feature('z_line_proj')
      for i in range(0,Z_MIN,Z_STEP):
        print(i)
        fid = self.feature_ids['z_%d' % i]
        print('fid is %s' % fid)
        z_pos_feat = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
        z_pos_sphere = z_pos_feat.sphere()
        z_pos = z_pos_sphere[1]
        z_line.addPoint(*z_pos)
        #also project onto top-fixture-plane
        plane_orig_to_pt = z_pos - top_plane_pt
        dist = np.dot(plane_orig_to_pt, top_plane_norm)
        proj_pt = z_pos - dist * top_plane_norm
        z_line_proj.addPoint(*proj_pt)

      print(z_line_proj)
      dir_z_partcsy = -1*z_line.line()[1]
      z_vec_proj = z_line_proj.line()[1]
      z_vec_cross_top_plane_norm = np.cross(z_vec_proj,top_plane_norm)
      print('z_vec_cross_top_plane_norm')
      print(z_vec_cross_top_plane_norm)
      affine_mat_construct = np.array([top_plane_pt,z_vec_proj,z_vec_cross_top_plane_norm,top_plane_norm])
      print('affine_mat_construct')
      print(affine_mat_construct)
      affine_mat = np.vstack((affine_mat_construct.transpose(),[1,1,1,1]))
      print('affine_mat')
      print(affine_mat)
      affine_mat_inv = np.linalg.inv(affine_mat)
      print('affine_mat_inv')
      print(affine_mat_inv)
      rot_mat = np.matmul(D_MAT, affine_mat_inv)
      print('rot_mat')
      print(rot_mat)

    except Exception as e:
      print(e)


    '''
    Construct CNC Coordinate System
    '''
    try:
      y_norm = top_plane_norm
      x_norm = np.cross(y_norm, dir_z_partcsy)
      pocket_yz_plane_norm = x_norm
      z_norm = np.cross(x_norm,y_norm)
      p2m_construct = np.array([x_norm,y_norm,z_norm,top_plane_pt])
      p2m = np.vstack((p2m_construct.transpose(),[0,0,0,1]))
      m2p = np.linalg.inv(p2m)
      #TODO use the plane defined by B-rotation instead of fixture top plane
      dir_b_norm_cnccsy = np.matmul(m2p,np.append(top_plane_norm,0))
      dir_z_cnccsy = np.matmul(m2p,np.append(dir_z_partcsy,0))
      print("pocketnc 2 machine matrix")
      print(p2m)
      print("machine 2 pocketnc matrix")
      print(m2p)
    except Exception as e:
      print('Exception while constructing coordinate system')
      print(e)
      return e

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


      b_home_err = float(self.last_find_b_proj_name[7:-5])
      b_home_offset_ini = float(ini.get_parameter(self.ini_data, "JOINT_4", "HOME_OFFSET")["values"]["value"])
      b_home_offset_true = b_home_offset_ini - b_home_err
      offsets['b'] = b_home_offset_true

      feat_b_circle = self.add_feature('b_circle')

      print("b lines 3d")
      for b_probe_feat_name in self.b_probes:
        print(b_probe_feat_name)
        fid = self.feature_ids[b_probe_feat_name]
        print('fid is %s' % fid)
        b_line = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
        b_line_proj = self.add_feature('proj_' + b_probe_feat_name)
        for pt in b_line.points():
          plane_orig_to_pt = pt - top_plane_pt
          dist = np.dot(plane_orig_to_pt, top_plane_norm)
          proj_pt = pt - dist * top_plane_norm
          # proj_pt_in_plane = np.matmul(rot_mat,np.append(proj_pt,1))
          b_line_proj.addPoint(proj_pt[0], proj_pt[1], proj_pt[2])
      
      print("finished projecting b lines")
      proj_true_b0_fid = self.feature_ids[self.last_find_b_proj_name]
      vec_true_b0 = self.metrologyManager.getActiveFeatureSet().getFeature(proj_true_b0_fid).line()[1]
      vec_true_b0_translated = np.matmul(m2p,np.append(vec_true_b0,0))
      vec_true_b0_translated_2d = np.array([vec_true_b0_translated[0],vec_true_b0_translated[2]])
      # b_0_vec_fid = self.feature_ids['b_0_line_proj']
      # b_0_vec = self.metrologyManager.getActiveFeatureSet().getFeature(b_0_vec_fid).line()[1]
      # vec_b0_projected_translated = np.matmul(m2p,np.append(b_0_vec,0))
      # vec_b0_2d = np.array([vec_b0_projected_translated[0],vec_b0_projected_translated[2]])

      with open(bCompFileName, 'w') as f:
        for idx, b_probe_feat_name in enumerate(self.b_probes):
          print(b_probe_feat_name)
          fid = self.feature_ids['proj_' + b_probe_feat_name]
          print('fid is %s' % fid)
          proj_b_line = self.metrologyManager.getActiveFeatureSet().getFeature(fid).line()
          proj_b_line_translated = np.matmul(m2p,np.append(proj_b_line[1],0))
          vec_bstep_2d = np.array([proj_b_line_translated[0],proj_b_line_translated[2]])
          # proj_b_line_translated2 = np.matmul(m2p2,np.append(proj_b_line[1],1))
          # proj_b_line_translated3 = np.matmul(m2p3,np.append(proj_b_line[1],1))
          #find angle relative to Z
          # line_angle_rel_z = angle_between_ccw_2d(b_0_vec, proj_b_line[1])
          line_angle_rel_0 = angle_between_ccw_2d(vec_bstep_2d, vec_true_b0_translated_2d)
          err = 0
          nominal_pos = float(b_probe_feat_name[len('probe-b-'):]) - b_home_err
          if line_angle_rel_0 < 0:
            err = (360 + line_angle_rel_0) - nominal_pos
          else:
            err = line_angle_rel_0 - nominal_pos
          comp = -1*err
          # line_angle_rel_z_3 = angle_between_ccw_2d(proj_b0_line_translated_2, proj_b_line_translated2)
          # line_angle_rel_z_4 = angle_between_ccw_2d(proj_b0_line_translated_3, proj_b_line_translated3)
          # print(line_angle_rel_z_3  )
          # print(line_angle_rel_z_4  )
          print("COMP: %s" % comp)
          f.write("%d %s %s\n" % (nominal_pos, comp, comp))
          # f.write("%d %s %s %s %s\n" % (i, line_angle_rel_z, line_angle_rel_z_2, line_angle_rel_z_3, line_angle_rel_z_4))

          #find intersect position with next B-line
          next_line_feat_name = self.b_probes[ (idx+1) % len(self.b_probes) ]
          next_line_fid = self.feature_ids[next_line_feat_name]
          next_proj_b_line = self.metrologyManager.getActiveFeatureSet().getFeature(next_line_fid).line()
          intersect_pos = find_line_intersect(proj_b_line[0],proj_b_line[1],next_proj_b_line[0],next_proj_b_line[1])
          feat_b_circle.addPoint(intersect_pos[0], intersect_pos[1], 0)

      print(feat_b_circle.points())
      print(feat_b_circle.average())
    except Exception as e:
      print(e)
      return e

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
      a_home_err = float(self.last_find_a_proj_name[7:-5])
      print('a_home_err')
      print(a_home_err)
      a_home_offset_ini = float(ini.get_parameter(self.ini_data, "JOINT_3", "HOME_OFFSET")["values"]["value"])
      a_home_offset_true = a_home_offset_ini - a_home_err
      offsets['a'] = a_home_offset_true
      
      feat_a_circle = self.add_feature('a_circle')

      # project the a-angle lines to POCKET coord sys YZ plane
      for a_probe_feat_name in self.a_probes:
        fid = self.feature_ids[a_probe_feat_name]
        a_line = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
        a_line_proj = self.add_feature('proj_' + a_probe_feat_name)
        for pt in a_line.points():
          plane_orig_to_pt = pt - top_plane_pt
          dist = np.dot(plane_orig_to_pt, pocket_yz_plane_norm)
          proj_pt = pt - dist * pocket_yz_plane_norm
          # proj_pt_in_plane = np.matmul(rot_mat,np.append(proj_pt,1))
          a_line_proj.addPoint(proj_pt[0], proj_pt[1], proj_pt[2])
      
      print("finished projecting a lines")
      proj_true_a0_fid = self.feature_ids[self.last_find_a_proj_name]
      vec_true_a0 = self.metrologyManager.getActiveFeatureSet().getFeature(proj_true_a0_fid).line()[1]
      vec_true_a0_translated = np.matmul(m2p,np.append(vec_true_a0,0))
      vec_true_a0_translated_2d = np.array([vec_true_a0_translated[1],vec_true_a0_translated[2]])
      # a_0_vec_fid = self.feature_ids['a_0_line_proj']
      # a_0_vec = self.metrologyManager.getActiveFeatureSet().getFeature(a_0_vec_fid).line()[1]
      # vec_a0_projected_translated = np.matmul(m2p,np.append(a_0_vec,0))
      # vec_a0_2d = np.array([vec_a0_projected_translated[1],vec_a0_projected_translated[2]])

      print('calculating A positions')
      with open(aCompFileName, 'w') as f:
        for idx, a_probe_feat_name in enumerate(self.a_probes):
          print(a_probe_feat_name)
          fid = self.feature_ids['proj_' + a_probe_feat_name]
          print('fid is %s' % fid)
          proj_a_line = self.metrologyManager.getActiveFeatureSet().getFeature(fid).line()
          proj_a_line_translated = np.matmul(m2p,np.append(proj_a_line[1],0))
          vec_astep_2d = np.array([proj_a_line_translated[1],proj_a_line_translated[2]])
          # proj_b_line_translated2 = np.matmul(m2p2,np.append(proj_b_line[1],1))
          # proj_b_line_translated3 = np.matmul(m2p3,np.append(proj_b_line[1],1))
          #find angle relative to Z
          # line_angle_rel_z = angle_between_ccw_2d(b_0_vec, proj_b_line[1])
          line_angle_rel_0 = angle_between_ccw_2d(vec_astep_2d, vec_true_a0_translated_2d)
          err = 0
          nominal_pos = float(a_probe_feat_name[len('probe-a-'):])
          true_nominal_pos = nominal_pos - a_home_err
          err = line_angle_rel_0 + nominal_pos
          # if i < 0:
          # else:
          #   err = i - line_angle_rel_0
          comp = err
          # line_angle_rel_z_3 = angle_between_ccw_2d(proj_b0_line_translated_2, proj_b_line_translated2)
          # line_angle_rel_z_4 = angle_between_ccw_2d(proj_b0_line_translated_3, proj_b_line_translated3)
          # print(line_angle_rel_z_3  )
          # print(line_angle_rel_z_4  )
          print("COMP: %s" % comp)
          f.write("%d %s %s\n" % (true_nominal_pos, comp, comp))
          # f.write("%d %s %s %s %s\n" % (i, line_angle_rel_z, line_angle_rel_z_2, line_angle_rel_z_3, line_angle_rel_z_4))

          #find intersect position with next A-line
          if nominal_pos < 1:
            #'negative A' features
            if nominal_pos > -1:
              #0 is the first 'negative' probed, 5 should be next but is probed after 5 more negatives
              next_line_feat_name = self.a_probes[ idx + 6 ]
            elif nominal_pos > -24:
              #-25 is the last increment in negative direction, 
              #its neighbor is the -20 line which was the previous feature probed
              next_line_feat_name = self.a_probes[ idx - 1 ]
            else:
              #the other 'negative' features -5 through -20
              next_line_feat_name = self.a_probes[ idx + 1 ]
          elif nominal_pos < 129:
            next_line_feat_name = self.a_probes[ idx + 1 ]
          else:
            #don't use the last feature
            continue

          next_line_feat_name = self.a_probes[ (idx+1) % len(self.a_probes) ]
          next_line_fid = self.feature_ids[next_line_feat_name]
          next_proj_line = self.metrologyManager.getActiveFeatureSet().getFeature(next_line_fid).line()
          #we want to know the XZ intersect coords within PartCsy
          this_orig_xz = (proj_a_line[0][0],proj_a_line[0][2])
          this_vec_xz = (proj_a_line[1][0],proj_a_line[1][2])
          next_orig_xz = (next_proj_line[0][0],next_proj_line[0][2])
          next_vec_xz = (next_proj_line[1][0],next_proj_line[1][2])
          intersect_pos = find_line_intersect(this_orig_xz,this_vec_xz,next_orig_xz,next_vec_xz)
          # intersect_pos_2d = find_line_intersect_2d(proj_a_line[0],proj_a_line[1],next_proj_line[0],next_proj_line[1])
          feat_a_circle.addPoint(intersect_pos[0], 0, intersect_pos[1 ])
      print(feat_a_circle.points())
      print(feat_a_circle.average())
    except Exception as e:
      print(e)
      return e

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
    try:
      print("Linear Axes Offsets")
      pos_a_circle_partcsy, radius_a_circle, normal_a_circle = feat_a_circle.circle()
      print(pos_a_circle_partcsy)
      pos_a_circle = np.matmul(m2p,np.append(pos_a_circle_partcsy,1))
      print(pos_a_circle)
      pos_a_circle_offset = pos_a_circle[0:3] + np.array([0,PROBING_POS_Y,0])
      print(pos_a_circle)
      yz_slope_dir_z = dir_z_cnccsy[1]/dir_z_cnccsy[2]
      print(yz_slope_dir_z)
      feat_z_line = self.get_feature("z_line")
      z_orig_partcsy = feat_z_line.points()[0]
      z_orig_cnccsy = np.matmul(m2p,np.append(z_orig_partcsy,1))
      z_diff_z0_to_acor = pos_a_circle_offset[2] - z_orig_cnccsy[2]
      dist_z0_to_acor = z_diff_z0_to_acor / dir_z_cnccsy[2]
      y_pos_of_z_at_travel = yz_slope_dir_z * dist_z0_to_acor + z_orig_cnccsy[1]
      print("z travel acor")
      print(dist_z0_to_acor)
      y_offset_z_intersect_acor = y_pos_of_z_at_travel - pos_a_circle_offset[1]#pos_a_circle_offset[1] - (yz_slope_dir_z * z_travel)
      print("y offset err")
      print(y_offset_z_intersect_acor)
      offsets['y'] = self.active_offsets['y'] - (y_offset_z_intersect_acor)/25.4
      print(1)
      pos_b_circle_partcsy, radius_b_circle, normal_b_circle_partcsy = feat_b_circle.circle()
      pos_b_circle = np.matmul(m2p,np.append(pos_b_circle_partcsy,1))
      pos_b_circle = pos_b_circle[0:3] + np.array([0,PROBING_POS_Y,0])
      norm_b_circle = np.matmul(m2p,np.append(normal_b_circle_partcsy,0))
      print(2)

      y_offset_b_cor_to_a_cor = pos_a_circle_offset[1] - pos_b_circle[1]
      dist_b_cor_to_a_cor_height = y_offset_b_cor_to_a_cor / norm_b_circle[1]
      pos_bnorm_intersect_acor_xzplane = pos_b_circle + dist_b_cor_to_a_cor_height * norm_b_circle[0:3]
      print(3)

      # xy_slope_b_norm = norm_b_circle[0]/norm_b_circle[1]
      # dist_z0_to_bcor = pos_a_circle_offset[2] - z_orig_cnccsy[2]
      # x_offset_z_intersect_bcor = pos_b_circle[0] - (xy_slope_b_norm * y_offset_b_cor_to_a_cor)
      
      xz_slope_dir_z = dir_z_cnccsy[0]/dir_z_cnccsy[2]
      z_diff_z0_to_pos_bnorm_intersect_acor_xzplane = pos_bnorm_intersect_acor_xzplane[2] - z_orig_cnccsy[2]
      dist_z0_to_pos_bnorm_intersect_acor_xzplane = z_diff_z0_to_pos_bnorm_intersect_acor_xzplane / dir_z_cnccsy[2]
      z_travel = pos_a_circle_offset[2] - z_orig_cnccsy[2]
      x_pos_of_z_at_travel = xz_slope_dir_z * dist_z0_to_pos_bnorm_intersect_acor_xzplane + z_orig_cnccsy[0]
      print(4)

      # x_offset_for_z_norm_to_intersect_pos_bnorm_intersect_acor_xzplane = pos_bnorm_intersect_acor_xzplane[0] - (xz_slope_dir_z * y_offset_b_cor_to_a_cor)
      x_offset_for_z_norm_to_intersect_pos_bnorm_intersect_acor_xzplane = x_pos_of_z_at_travel - pos_bnorm_intersect_acor_xzplane[0]
      print("x offset err")
      print(x_offset_for_z_norm_to_intersect_pos_bnorm_intersect_acor_xzplane)
      offsets['x'] = self.active_offsets['x'] - (x_offset_for_z_norm_to_intersect_pos_bnorm_intersect_acor_xzplane)/25.4
    except Exception as e:
      print(e)
      return e

    '''
    Writing CalibrationOverlay
    '''
    try:
      new_overlay_data = copy.deepcopy(self.overlay_data)
      with open(newCalFilename, 'w') as f:
        if "x" in offsets:
            print("Writing %0.6f to X HOME_OFFSET" % offsets["x"])
            ini.set_parameter(new_overlay_data, "JOINT_0", "HOME_OFFSET", offsets["x"])

        if "y" in offsets:
            print("Writing %0.6f to Y HOME_OFFSET" % offsets["y"])
            ini.set_parameter(new_overlay_data, "JOINT_1", "HOME_OFFSET", offsets["y"])

        if "a" in offsets:
            print("Writing %0.6f to A HOME_OFFSET" % offsets["a"])
            ini.set_parameter(new_overlay_data, "JOINT_3", "HOME_OFFSET", offsets["a"])

        if "b" in offsets:
            print("Writing %0.6f to B HOME_OFFSET" % offsets["b"])
            ini.set_parameter(new_overlay_data, "JOINT_4", "HOME_OFFSET", offsets["b"])

        if "bTable" in offsets:
            print("Writing %0.6f to PROBE_B_TABLE_OFFSET" % offsets["bTable"])
            ini.set_parameter(new_overlay_data, "TOOL_PROBE", "PROBE_B_TABLE_OFFSET", offsets["bTable"])

        if "sensor123" in offsets:
            print("Writing %0.6f to PROBE_SENSOR_123_OFFSET" % offsets["sensor123"])
            ini.set_parameter(new_overlay_data, "TOOL_PROBE", "PROBE_SENSOR_123_OFFSET", offsets["sensor123"])
        ini.write_ini_data(new_overlay_data, newCalFilename)

    except Exception as e:
      print(e)
      return e

    return True

  async def disconnect_from_cmm(self):
    '''
    End
    '''
    if self.client is None or self.client.stream is None:
      print("already disconnected")
      return True

    try:
      await self.client.EndSession().send()
      await self.client.disconnect()
    except CmmException as ex:
      print("CmmExceptions %s" % ex)

  async def end(self):
    '''
    End
    '''
    if self.client is None or self.client.stream is None:
      print("already disconnected")
      return True

    try:
      await self.client.EndSession().send()
      await self.client.disconnect()
    except CmmException as ex:
      print("CmmExceptions %s" % ex)

