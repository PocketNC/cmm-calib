from calibstate import CalibState, Stages
import v2calculations

def getFeaturesX(state):  
  return state.getStage(Stages.CHARACTERIZE_X)['features']

def getFeaturesY(state):  
  return state.getStage(Stages.CHARACTERIZE_Y)['features']

def getFeaturesZ(state):  
  return state.getStage(Stages.CHARACTERIZE_Z)['features']

def getLineX(state):
  feats = getFeaturesX(state)
  spheres = v2calculations.calc_sphere_centers(feats)
  (pt,line) = spheres.line()
  return line

def getLineY(state):
  feats = getFeaturesY(state)
  spheres = v2calculations.calc_sphere_centers(feats)
  (pt,line) = spheres.line()
  return line

def getLineZ(state):
  feats = getFeaturesZ(state)
  spheres = v2calculations.calc_sphere_centers(feats)
  (pt,line) = spheres.line()
  return line

def getAxisLines(state):
  x = getLineX(state)
  y = getLineY(state)
  z = getLineZ(state)
  return (x,y,z)