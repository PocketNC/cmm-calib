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
  def default(self, o):
    if isinstance(o, Feature):
      return o.toJSON()

    return super().default(o)

class CalibException(Exception):
  pass

POCKETNC_VAR_DIR = os.environ.get('POCKETNC_VAR_DIRECTORY')
CALIB_DIR = os.path.join(POCKETNC_VAR_DIR, 'calib')
if not os.path.exists(CALIB_DIR):
  os.makedirs(CALIB_DIR)

DEFAULT_STAGES_DIR = os.path.join(CALIB_DIR, 'stages')

class Stages(Enum):
  """
  A stage is a progress point in the calibration process. State is recorded
  by steps during a stage and serialized to a stages/STAGE.json file. This file
  is automatically read in at start up, if it exists and can be explicitly removed
  using methods on the CalibState object.
  """
  ERASE_COMPENSATION = auto()
  SETUP_CNC = auto()
  SETUP_CMM = auto()
  PROBE_MACHINE_POS = auto()
  PROBE_SPINDLE_POS = auto()
  HOMING_X = auto()
  CHARACTERIZE_X = auto()
  HOMING_Z = auto()
  CHARACTERIZE_Z = auto()
  PROBE_FIXTURE_BALL_POS = auto()
  HOMING_Y = auto()
  CHARACTERIZE_Y = auto()
  PROBE_TOP_PLANE = auto()
  PROBE_HOME_OFFSETS = auto()

  HOMING_A = auto()
  HOMING_B = auto()
  SETUP_CNC_CSY = auto()
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
