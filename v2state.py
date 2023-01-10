from calibstate import CalibState, Stages
import v2calculations
from metrology import Feature, angle_between_ccw_2d, intersectLines, angle_between


def getFixtureBallPos(state):
  return state.getStage(Stages.PROBE_FIXTURE_BALL_POS)["fixture_ball_pos"]

def getZeroSpindlePos(state):
  return state.getStage(Stages.PROBE_SPINDLE_POS)["zero_spindle_pos"]

def getToolProbePos(state):
  return state.getStage(Stages.TOOL_PROBE_OFFSET)["tool_probe_pos"]

def getPlaneA90(state):
  return state.getStage(Stages.TOOL_PROBE_OFFSET)["plane_a90"]

def getFeaturesX(state):  
  return state.getStage(Stages.CHARACTERIZE_X)['features']

def getFeaturesY(state):  
  return state.getStage(Stages.CHARACTERIZE_Y)['features']

def getFeaturesZ(state):  
  return state.getStage(Stages.CHARACTERIZE_Z)['features']

def getFeaturesA(state):  
  return state.getStage(Stages.CHARACTERIZE_A)['features']

def getFeaturesB(state):  
  return state.getStage(Stages.CHARACTERIZE_B)['features']

def getXDirection(state):
  feats = getFeaturesX(state)

  # Points are collected from positive to negative, so reverse the direction
  # so we are returning the positive X direction

  spheres = v2calculations.calc_sphere_centers(reversed(feats))
  (pt,dir) = spheres.line()
  return dir

def getYDirection(state):
  feats = getFeaturesY(state)
  spheres = v2calculations.calc_sphere_centers(feats)
  (pt,dir) = spheres.line()
  return dir

def getZDirection(state):
  feats = getFeaturesZ(state)

  # Points are collected from positive to negative, so reverse the direction
  # so we are returning the positive Z direction

  spheres = v2calculations.calc_sphere_centers(reversed(feats))
  (pt,dir) = spheres.line()
  return dir

def getAxisDirections(state):
  x = getXDirection(state)
  y = getYDirection(state)
  z = getZDirection(state)
  return (x,y,z)

def getAZeroPt(state):
  feat = state.getStage(Stages.CHARACTERIZE_A)['zero']
  (rad, pt) = feat.sphere()
  return pt

def getAPositions(state):
  stage = state.getStage(Stages.CHARACTERIZE_A)
  zero_pt = getAZeroPt(state)
  feats = getFeaturesA(stage)
  pts = v2calculations.calc_sphere_centers(feats)
  (rad, cor, norm) = Feature(pts).circle()
  zero_line = zero_pt - cor
  positions = []
  for pt in pts:
    a_line = pt - cor
    a_pos = angle_between_ccw_2d(zero_line, a_line)
    positions.append(a_pos)

def getAErrors(state):
  positions = getAPositions(state)
  nominal_positions = state.getStage(Stages.CHARACTERIZE_A)['positions']
  #TODO

