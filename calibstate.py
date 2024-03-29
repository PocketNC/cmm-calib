"""
State management for calibration process.
"""
import os
import json
from enum import Enum, auto
from metrology import Feature, convertJSONDataToFeatures
import logging
from copy import deepcopy
import datetime
from datetime import timezone
import traceback
import ini

logger = logging.getLogger(__name__)

class CalibStateEncoder(json.JSONEncoder):
  """
  A JSON encoder that turns Feature objects into a dictionary that is readily identifiable as a Feature object, so 
  when the JSON is decoded in Python it can automatically turn those objects into Feature objects.
  """

  def default(self, o):
    if isinstance(o, Feature):
      return o.toJSON()

    return super().default(o)

class CalibException(Exception):
  """
  An exception that can be thrown by the calibration routine to indicate a calibration failure.
  """
  pass

POCKETNC_VAR_DIR = os.environ.get('POCKETNC_VAR_DIRECTORY')
CALIB_DIR = os.path.join(POCKETNC_VAR_DIR, 'calib')
if not os.path.exists(CALIB_DIR):
  os.makedirs(CALIB_DIR)

DEFAULT_STAGES_DIR = os.path.join(CALIB_DIR, 'stages')

class Stages(Enum):
  """
  An Enum class that represents each stage of the calibration process. Data can be stored to a stage
  using a `calibstate.CalibState` object. Generally, you'll use the `calibstate.CalibState.getInstance`
  method to fetch the global singleton state object, but it can be useful to instantiate `calibstate.CalibState`
  objects when inspecting multiple machines' data.
  """

  ERASE_COMPENSATION = auto()
  """
  Stage that erases compensation files and initializes those files to default values to
  begin calibration.
  """

  SETUP_CNC = auto()
  """
  Ensures that the V2 is out of E-Stop and homed so that programs and mdi commands can be run.
  """

  SETUP_CMM = auto()
  """
  Ensures that the CMM is homed and errors are cleared.
  """

  PROBE_MACHINE_POS = auto()
  """
  Probes the top face of the L-bracket, along with a line along the right side and back side of the L-bracket
  to find the top right back corner of the L-bracket. Sets the origin of the CMM's part csy to that point and
  orients the X, Y and Z axes to mostly align with the X, Y and Z axes of the machine.

  Stores three features:

     `L_bracket_top_face` - Feature that represents the top plane of the L-bracket.

     `L_bracket_back_line` - Feature that represents a line that should mostly align with the -Z axis of the V2, probed along the back face of the L-bracket. This feature is intended to be projected to the L-bracket's top face plane.

     `L_bracket_right_line` - Feature that represents a line that should mostly align with the -X axis of the V2, probed along the right face of the L-bracket. This feature is intended to be projected to the L-bracket's top face plane.

  """

  PROBE_SPINDLE_POS = auto()
  """
  Probes the position of the spindle at X0, Z0. The routine it runs depends on the version of the machine (V2-10 vs V2-50).
  Ideally, the routine would be generic enough to handle either machine and perhaps automatically determine the type of machine,
  but for now the operator must specific which type of machine is being calibrated and the routine is chosen based on that choice.

  The V2-10 routine requires a 1/2" ball to be inserted into the spindle. The V2-50 routines requires the 6mm ball to be inserted into 
  the spindle.

  Stores a single sphere feature under the stage's key, `zero_spindle_pos`.
  """

  HOMING_X = auto()
  """
  Repeatedly commands the X axis to a random position, unhomes, then homes the X axis, commands the X axis to 0 then probes the ball in the spindle using info
  found in the PROBE_SPINDLE_POS Stage.

  Stores a list of probed sphere features under the stage's key, `features`. Also stores a list of positions that represent where V2 was commanded
  when the probing occurred (these should be X0, Z0).
  """

  CHARACTERIZE_XZ = auto()
  """
  Probe a grid of spindle positions along the full length of travel of both X and Z.
  Stores the following keys in the stage:

     `features` - List of sphere features probed.

     `positions` - The X and Z position of the spindle where each of the `features` were probed.

  """

  CHARACTERIZE_X_REVERSE = auto()
  CHARACTERIZE_X = auto()
  CHARACTERIZE_X_FIXTURE_BALL = auto()
  """
  Probes a series of spindle positions along the full length of travel of the X-axis.

  Stores the following keys in the stage:

     `features` - List of sphere features probed along the X-axis.

     `positions` - The X and Z position of the spindle where each of the `features` were probed.

  """

  HOMING_Z = auto()
  """
  Repeatedly commands the Z axis to a random position, unhomes, then homes the Z axis, commands the Z axis to 0 then probes the ball in the spindle using info
  found in the PROBE_SPINDLE_POS Stage.

  Stores a list of probed sphere features under the stage's key, `features`. Also stores a list of positions that represent where V2 was commanded
  when the probing occurred (these should be X0, Z0).
  """

  CHARACTERIZE_Z_REVERSE = auto()
  CHARACTERIZE_Z = auto()
  """
  Probes a series of spindle positions along the full length of travel of the Z-axis.

  Stores the following keys in the stage:

     `features` - List of sphere features probed along the Z-axis.

     `positions` - The X and Z position of the spindle where each of the `features` were probed.

  """

  PROBE_FIXTURE_BALL_POS = auto()
  """
  Probes a ball on the calibration fixture using a nominal value for where to locate it. Other stages
  that use the fixture ball to gather data will use the more accurate position found in this stage to
  locate it.

  Stores the following keys in the stage:

     `fixture_ball_pos` - A sphere Feature of the probed fixture ball.
  """

  HOMING_Y = auto()
  """
  Repeatedly commands the Y axis to a random position, unhomes, then homes the Y axis, commands the Y axis to 0 then probes the fixture ball using info
  found in the PROBE_FIXTURE_BALL_POS Stage.

  Stores a list of probed sphere features under the stage's key, `features`. Also stores a list of positions that represent where V2 was commanded
  when the probing occurred (these should be Y0).
  """

  CHARACTERIZE_Y_REVERSE = auto()
  CHARACTERIZE_Y = auto()
  CHARACTERIZE_Y_SPINDLE_BALL = auto()
  """
  Probes a series of fixture ball positions along the full length of travel of the Y-axis.

  Stores the following keys in the stage:

     `features` - List of sphere features probed along the Y-axis.

     `positions` - The Y position of the spindle where each of the `features` were probed.

  """

  PROBE_OFFSETS = auto()
  """
  Probes a number of features used in calculating a number of linear offsets

  Probes the top plane of the calibration fixture.
  Probes features needed to calculate PROBE_SENSOR_123_OFFSET and PROBE_B_TABLE_OFFSET
  Gathers measurements for calculating the X and Y home offsets. The V2 is commanded to A90.

  """
  

  PROBE_TOP_PLANE = auto()
  """
  Probes the top plane of the calibration fixture.

  Stores the following keys in the stage:

      `top_plane` - Feature that represents the plane of the top face of the calibration fixture.
      `y` - The y position the V2 was commanded to when probing the top face.

  """

  TOOL_PROBE_OFFSET = auto()
  """
  Probes features needed to calculate PROBE_SENSOR_123_OFFSET
  Performs a V2 tool probe, with a calibration sphere in the spindle. 
  Then withdraws Z axes, lowers Y axis, and returns Z axis to position where contact occured
  Then probes sphere in spindle.
  Then withdraws Z, rotates A-axis to 90 degrees, and probes 3 points on the face to create a plane feature
  Stores the following keys in the stage:
     `tool_probe_pos` - Sphere feature, probed against the spindle sphere at the X and Z position where tool probe button contact occurred
     `plane_a90` - Plane feature, probed against the fixture top plane at Y0A90
  """

  PROBE_HOME_OFFSETS = auto()
  """
  Gathers measurements for calculating the X and Y home offsets. The V2 is commanded to A90.
  The calibration fixture's fin is probed to ensure the fixture is as aligned as possible with
  the Z axis. B is also aligned to ensure the sides of the calibration fixture are perpendicular
  to Y. Two points on each side of the fixture are probed, then B is rotated 180 degrees and the
  two points are probed again. The average of all four points is used to determine an offset. The
  same procedure happens for X and then Y.
  """

  HOMING_A = auto()
  """
  Sets A HOME_OFFSET to 0, so the home position is the latch position
  Then repeatedly commands the A axis to a random position, unhomes then homes the A axis, and then probes a line along the fixture fin

  Stores a list of probed line features under the stage's key, `features`. 
  Also stores a list of positions that represent where V2 was commanded when the probing occurred (these should be Y0A0).
  """

  HOMING_B = auto()
  """
  Sets B HOME_OFFSET to 0, so the home position is the latch position
  Then repeatedly commands the B axis to a random position, 
    unhomes then homes the B axis, 
    and then probes a line along the fixture side

  Stores a list of probed line features under the stage's key, `features`. 
  Also stores a list of positions that represent where V2 was commanded when the probing occurred (these should be Y0B0).
  """

  CHARACTERIZE_A_SPHERE_REVERSE = auto()
  CHARACTERIZE_A_SPHERE = auto()
  """
  Probes a series of fixture ball positions along the full arc of travel of the A-axis.
  Stores the following keys in the stage:
     `features` - List of sphere features probed as the A-axis rotates.
     `positions` - The A position of the spindle where each of the `features` were probed.
  """
  
  CHARACTERIZE_B_SPHERE_REVERSE = auto()
  CHARACTERIZE_B_SPHERE = auto()
  """
  Probes a series of fixture ball positions along the full arc of travel of the B-axis.
  Stores the following keys in the stage:
     `features` - List of sphere features probed as the B-axis rotates.
     `positions` - The B position of the spindle where each of the `features` were probed.
  """

  CHARACTERIZE_A_LINE_REVERSE = auto()
  CHARACTERIZE_A_LINE = auto()
  """
  Probes a series of lines against the fixture fin along the full arc of travel of the A-axis.
  First iteratively zeroes the A-axis. A set of points for the zero position is saved.
  Stores the following keys in the stage:
     `features` - List of line features probed as the A-axis rotates.
     `positions` - The A position of the spindle where each of the `features` were probed.
     `zero` - A line feature probed after walking on to the true zero position
     `zero_a_pos` - The nominal A-axis posture at the true zero position
  """
  
  CHARACTERIZE_B_LINE_REVERSE = auto()
  CHARACTERIZE_B_LINE = auto()
  """
  Probes a series of lines against the fixture fin along the full arc of travel of the B-axis.
  First iteratively zeroes the B-axis. A set of points for the zero position is saved.
  Stores the following keys in the stage:
     `features` - List of line features probed as the B-axis rotates.
     `positions` - The B position of the spindle where each of the `features` were probed.
     `zero` - A line feature probed after walking on to the true zero position
     `zero_b_pos` - The nominal B-axis posture at the true zero position
  """

  CALIBRATE = auto()
  """
  Calculate new set of compensation data using calibstate data
  Create compensation files in POCKETNC_VAR_DIR
  Restart services (PocketNC and Rockhopper)
  Take machine out of E-stop 
  And then home
  """

  VERIFY_HOMING_X = auto()
  """
  Repeatedly commands the X axis to a random position, unhomes, then homes the X axis, commands the X axis to 0 then probes the ball in the spindle using info
  found in the PROBE_SPINDLE_POS Stage.
  Stores a list of probed sphere features under the stage's key, `features`. Also stores a list of positions that represent where V2 was commanded
  when the probing occurred (these should be X0, Z0).
  """

  VERIFY_HOMING_Y = auto()
  """
  Repeatedly commands the Y axis to a random position, unhomes, then homes the Y axis, commands the Y axis to 0 then probes the fixture ball using info
  found in the PROBE_FIXTURE_BALL_POS Stage.

  Stores a list of probed sphere features under the stage's key, `features`. Also stores a list of positions that represent where V2 was commanded
  when the probing occurred (these should be Y0).
  """

  VERIFY_HOMING_A = auto()
  """
  Repeatedly commands the A-axis to a random position, unhomes, then homes the A axis, and then probes a line along the fixture fin 
  Stores a list of probed line features under the stage's key, `features`. 
  Also stores a list of positions that represent where V2 was commanded when the probing occurred (these should be Y0A0).
  """

  VERIFY_HOMING_B = auto()
  """
  Repeatedly commands the B-axis to a random position, unhomes, then homes the B axis, and then probes a line along the fixture side 
  Stores a list of probed line features under the stage's key, `features`. 
  Also stores a list of positions that represent where V2 was commanded when the probing occurred (these should be Y0B0).
  """

  VERIFY_A_LINE = auto()
  """
  Probes a series of lines against the fixture fin along the full arc of travel of the A-axis.
  Stores the following keys in the stage:
     `features` - List of line features probed as the A-axis rotates.
     `positions` - The A position of the spindle where each of the `features` were probed.
  """

  VERIFY_B_LINE = auto()
  """
  Probes a series of lines against the fixture fin along the full arc of travel of the B-axis.
  Stores the following keys in the stage:
     `features` - List of line features probed as the B-axis rotates.
     `positions` - The B position of the spindle where each of the `features` were probed.
  """

  VERIFY_OFFSETS = auto()
  """
  Probes a series of lines against the fixture fin along the full arc of travel of the B-axis.
  Stores the following keys in the stage:
     `features` - List of line features probed as the B-axis rotates.
     `positions` - The B position of the spindle where each of the `features` were probed.
  """

  UPLOAD_FILES = auto()
  """
  Upload calibration data
  """

  VERIFY_X = auto()
  VERIFY_Y = auto()
  VERIFY_Z = auto()
  VERIFY_A = auto()
  VERIFY_B = auto()
  CALC_VERIFY = auto()
  WRITE_VERIFY = auto()

  RESTART_CNC = auto()
  
  DEVELOPMENT = auto()
  """
  A stage that can be a sandbox for testing. Can be used to store data that would normally be saved to a stage, but
  may simply be an experiment.
  """

  @classmethod
  def has_name(cls, name):
      return name in cls._member_names_ 

  def toJSON(self):
    return str(self)

calibStateInstance = None

class CalibState:
  def getInstance():
    global calibStateInstance

    if calibStateInstance == None:
      calibStateInstance = CalibState()

    return calibStateInstance

  def __init__(self, dir=DEFAULT_STAGES_DIR, enumClass=Stages):
    self.enumClass = enumClass
    self.dir = dir
    if not os.path.exists(dir):
      os.makedirs(dir)

    self.stages = {}
    self.storeStartupCalibration()

  def getStageKey(self, stage):
    if type(stage) == int:
      stage = self.enumClass(stage)
    elif type(stage) == str:
      stage = self.enumClass[stage]

    # Use enum's name as key as we often reload our Python context in 
    # development see SOFT-1033
    return stage.name
   
  def getStageAll(self, stage):
    key = self.getStageKey(stage)

    if key not in self.stages:
      with open(os.path.join(self.dir, key), 'r') as f:
        rawData = json.loads(f.read())

        self.stages[key] = rawData

    # This explicitly returns a deep copy of stages, so a user can't
    # manipulate stages, it also converts any JSON objects that look
    # like features to Feature objects.
    return convertJSONDataToFeatures(self.stages[key])

  def getStageMeta(self, stage):
    return self.getStageAll(stage)[-1]['meta']

  def getStageRun(self, stage):
    return self.getStageAll(stage)[-1]

  def getStage(self, stage):
    return self.getStageAll(stage)[-1]['data']

  def mergeIntoStage(self, stage, data):
    key = self.getStageKey(stage)
    
    self.stages[key].update(data)
    with open(os.path.join(self.dir, key), 'w') as f:
      dataStr = json.dumps(self.stages[key], cls=CalibStateEncoder)
      f.write(dataStr)


  def updateStage(self, stage, data):
    """
    Updates the last run of the provided stage with new data. Also updates meta data such as modification_time.
    Assumes that saveStage has been called prior to updateStage.
    """
    key = self.getStageKey(stage)

    # Save a deep copy of the data passed in, so the data can't
    # be modified as a side effect later, using the same reference.
    self.stages[key][-1]['meta']['modification_time'] = self.getTimestamp()
    self.stages[key][-1]['data'] = deepcopy(data)

    with open(os.path.join(self.dir, key), 'w') as f:
      dataStr = json.dumps(self.stages[key], cls=CalibStateEncoder)
      f.write(dataStr)


  def saveStage(self, stage, data):
    """
    Saves a new run of the provided stage. Includes metadata such as the active calibration and creation/modification times.
    """
    key = self.getStageKey(stage)

    try:
      # Ensure the stage is loaded from disk
      self.getStage(key)
    except:
      # if it still doesn't exist, we'll create it below
      pass

    # Save a deep copy of the data passed in, so the data can't
    # be modified as a side effect later, using the same reference.
    metadata = self.getInitialMetadata()
    if key in self.stages:
      self.stages[key].append({'data': deepcopy(data), 'meta': self.getInitialMetadata()})
    else:
      self.stages[key] = [{'data': deepcopy(data), 'meta': self.getInitialMetadata()}]
    with open(os.path.join(self.dir, key), 'w') as f:
      dataStr = json.dumps(self.stages[key], cls=CalibStateEncoder)
      f.write(dataStr)

  def getTimestamp(self):
    dt = datetime.datetime.now(timezone.utc)
    utc_time = dt.replace(tzinfo=timezone.utc)
    return utc_time.strftime("%Y-%m-%d %H:%M:%S")

  def getInitialMetadata(self):
    data = {}
    data['calibration'] = self.startupCalibration
    data['creation_time'] = self.getTimestamp()
    data['modification_time'] = data['creation_time']
    return data

  def storeStartupCalibration(self):
    self.startupCalibration = self.getCurrentCalib()

  def getCurrentCalib(self):
    curr = {}
    try:
      with open(os.path.join(POCKETNC_VAR_DIR, 'CalibrationOverlay.inc'), 'r') as f:
        curr['overlay'] = f.read()
    except:
      logger.error("Missing calib file CalibrationOverlay.inc")
    try:
      with open(os.path.join(POCKETNC_VAR_DIR, 'a.comp'), 'r') as f:
        curr['a_comp'] = f.read()
    except:
      logger.error("Missing calib file a.comp")
    try:
      with open(os.path.join(POCKETNC_VAR_DIR, 'b.comp'), 'r') as f:
        curr['b_comp'] = f.read()
    except:
      logger.error("Missing calib file b.comp")
    return curr
    
  def writeCalibration(self,data):
    logger.debug("data: %s", data)
    overlayPath = os.path.join(POCKETNC_VAR_DIR, 'CalibrationOverlay.inc')
    overlayData = ini.read_ini_data(overlayPath)
    ini.set_parameter(overlayData, "JOINT_0", "HOME_OFFSET", data["x_home_offset"])
    ini.set_parameter(overlayData, "JOINT_1", "HOME_OFFSET", data["y_home_offset"])
    ini.set_parameter(overlayData, "JOINT_3", "HOME_OFFSET", data["a_home_offset"])
    ini.set_parameter(overlayData, "JOINT_4", "HOME_OFFSET", data["b_home_offset"])

    ini.write_ini_data(overlayData, overlayPath)

    with open(os.path.join(POCKETNC_VAR_DIR, 'a.comp'), 'w') as f:
      for p in data["a_comp"]:
        f.write("%0.6f %0.6f %0.6f\n" % (p[0], p[1], p[1]))

    with open(os.path.join(POCKETNC_VAR_DIR, 'b.comp'), 'w') as f:
      NUM_CYCLES = 28
      for n in range(NUM_CYCLES):
        cycle = NUM_CYCLES-n

        for p in data["b_comp"]:
          if p[0] >= 0 and p[0] < 360:
            f.write("%0.6f %0.6f %0.6f\n" % (p[0]-360*cycle, p[1], p[1]))

      for p in data["b_comp"]:
        if p[0] >= 0 and p[0] < 360:
          f.write("%0.6f %0.6f %0.6f\n" % (p[0], p[1], p[1]))

      for n in range(NUM_CYCLES):
        cycle = n+1

        for p in data["b_comp"]:
          if p[0] >= 0 and p[0] < 360:
            f.write("%0.6f %0.6f %0.6f\n" % (p[0]+360*cycle, p[1], p[1]))

