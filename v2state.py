from calibstate import CalibState, Stages
import v2calculations

def getFixtureBallPos(state):
  return state.getStage(Stages.PROBE_FIXTURE_BALL_POS)["fixture_ball_pos"]

def getZeroSpindlePos(state):
  return state.getStage(Stages.PROBE_SPINDLE_POS)["zero_spindle_pos"]

def getFeaturesX(state):  
  return state.getStage(Stages.CHARACTERIZE_X)['features']

def getFeaturesY(state):  
  return state.getStage(Stages.CHARACTERIZE_Y)['features']

def getFeaturesZ(state):  
  return state.getStage(Stages.CHARACTERIZE_Z)['features']

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
