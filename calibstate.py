"""
State management for calibration process.
"""
import os
import json
from enum import Enum, auto
from metrology import Feature, convertJSONDataToFeatures
import logging

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

  CHARACTERIZE_X_REVERSE = auto()
  CHARACTERIZE_X = auto()
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

  CHARACTERIZE_Y = auto()
  """
  Probes a series of fixture ball positions along the full length of travel of the Y-axis.

  Stores the following keys in the stage:

     `features` - List of sphere features probed along the Y-axis.

     `positions` - The Y position of the spindle where each of the `features` were probed.

  """

  PROBE_TOP_PLANE = auto()
  """
  Probes the top plane of the calibration fixture.

  Stores the following keys in the stage:

      `top_plane` - Feature that represents the plane of the top face of the calibration fixture.
      `y` - The y position the V2 was commanded to when probing the top face.

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
  HOMING_B = auto()

  CHARACTERIZE_A = auto()
  CHARACTERIZE_B = auto()

  CALC_CALIB = auto()
  WRITE_CALIB = auto()
  '''verification stages'''
  RESTART_CNC: auto()
  SETUP_VERIFY = auto()
  TOOL_PROBE_OFFSET = auto()
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

  def getStage(self, stage):
    if type(stage) == int:
      stage = self.enumClass(stage)
    elif type(stage) == str:
      stage = self.enumClass[stage]

    if stage not in self.stages:
      with open(os.path.join(self.dir, stage.name), 'r') as f:
        rawData = json.loads(f.read())

        self.stages[stage] = rawData

    return convertJSONDataToFeatures(self.stages[stage])

  def saveStage(self, stage, data):
    if type(stage) == int:
      stage = self.enumClass(stage)
    elif type(stage) == str:
      stage = self.enumClass[stage]

    logger.debug("Saving stage %s", stage)
    self.stages[stage] = data
    with open(os.path.join(self.dir, stage.name), 'w') as f:
      dataStr = json.dumps(data, cls=CalibStateEncoder)
      f.write(dataStr)
