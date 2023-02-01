import logging
from calibstate import CalibState, Stages
import v2calculations
from v2routines import V2_10, V2_50, PROBE_DIA, SPINDLE_BALL_DIA_10, SPINDLE_BALL_DIA_50, APPROX_FIXTURE_BALL_HOME, FIXTURE_BALL_DIA, APPROX_COR
from metrology import Feature, angle_between_ccw_2d, intersectLines, angle_between, projectPointToPlane, projectDirectionToPlane
import numpy as np
logger = logging.getLogger(__name__)

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

def getFeaturesXFixtureBall(state):  
  return state.getStage(Stages.CHARACTERIZE_X_FIXTURE_BALL)['features']

def getFeaturesY(state):  
  return state.getStage(Stages.CHARACTERIZE_Y)['features']

def getFeaturesZ(state):  
  return state.getStage(Stages.CHARACTERIZE_Z)['features']

def getFeaturesA(state):  
  return state.getStage(Stages.CHARACTERIZE_A)['features']

def getFeaturesB(state):  
  return state.getStage(Stages.CHARACTERIZE_B)['features']

def getFeaturesHomingA(state):  
  return state.getStage(Stages.HOMING_A)['features']

def getFeaturesHomingB(state):  
  return state.getStage(Stages.HOMING_B)['features']

def getXDirection(state):
  feats = getFeaturesX(state)

  # Points are collected from positive to negative, so reverse the direction
  # so we are returning the positive X direction

  spheres = v2calculations.calc_sphere_centers(reversed(feats))
  (pt,dir) = spheres.line()
  return dir

def getXDirectionOOS(state):
  x_stage = state.getStage(Stages.CHARACTERIZE_X)
  x_stage_fixture = state.getStage(Stages.CHARACTERIZE_X_FIXTURE_BALL)

  spheres = v2calculations.calc_sphere_centers(reversed(x_stage["features"][0:-1]))
  spheres_fixture = v2calculations.calc_sphere_centers(reversed(x_stage_fixture["features"][0:-1]))

  spindle_ball_points = spheres.points()
  fixture_ball_points = spheres_fixture.points()

  fixture_ref_pt = x_stage_fixture["features"][-1].sphere()[1]

  line_feature = Feature()
  for (spindle_ball_pt,fixture_ball_pt) in zip(spindle_ball_points, fixture_ball_points):
    pt = spindle_ball_pt + (fixture_ref_pt - fixture_ball_pt)
    line_feature.addPoint(*pt)

  (pt,dir) = line_feature.line()
  return dir

def getYDirectionOOS(state):
  y_stage = state.getStage(Stages.CHARACTERIZE_Y)
  y_stage_fixture = state.getStage(Stages.CHARACTERIZE_Y_SPINDLE_BALL)

  spheres = v2calculations.calc_sphere_centers(y_stage["features"][0:-1])
  spheres_spindle = v2calculations.calc_sphere_centers(y_stage_fixture["features"][0:-1])

  fixture_ball_points = spheres.points()
  spindle_ball_points = spheres_spindle.points()

  spindle_ref_pt = y_stage_fixture["features"][-1].sphere()[1]

  line_feature = Feature()
  for (fixture_ball_pt,spindle_ball_pt) in zip(fixture_ball_points, spindle_ball_points):
    pt = fixture_ball_pt + (spindle_ref_pt - spindle_ball_pt)
    line_feature.addPoint(*pt)

  (pt,dir) = line_feature.line()
  return dir

def getYDirection(state):
  feats = getFeaturesY(state)

  spheres = v2calculations.calc_sphere_centers(feats)
  (pt,dir) = spheres.line()
  logger.debug('y spheres %s', spheres.points())
  logger.debug('y dir %s', dir)

  if dir[1] < 0:
    dir = -1*dir

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

def getBZeroPt(state):
  feat = state.getStage(Stages.CHARACTERIZE_B)['zero']
  (rad, pt) = feat.sphere()
  return pt

def getAPositionsLine(state):
  (x_dir,y_dir,z_dir) = v2state.getAxisDirections(state)
  characterize_stage = state.getStage(Stages.CHARACTERIZE_A_LINE)
  
  zero_feat = characterize_stage['zero']
  zero_pos = v2calculations.calc_pos_a(zero_feat, x_dir, y_dir, z_dir, APPROX_COR)
  nom_zero_pos = characterize_stage['zero_a_pos']

  positions = []

  for (feat, nom_pos) in zip(characterize_stage["features"],characterize_stage["positions"]):
    zeroed_nom_pos = nom_pos - nom_zero_pos
    a_pos = v2calculations.calc_pos_a(feat, x_dir, y_dir, z_dir, APPROX_COR)
    positions.append((zeroed_nom_pos, a_pos))

  return positions

def getBPositionsLine(state):
  (x_dir,y_dir,z_dir) = v2state.getAxisDirections(state)
  characterize_stage = state.getStage(Stages.CHARACTERIZE_B_LINE)
  
  zero_feat = characterize_stage['zero']
  zero_pos = v2calculations.calc_pos_a(zero_feat, x_dir, y_dir, z_dir, APPROX_COR)
  nom_zero_pos = characterize_stage['zero_a_pos']

  positions = []

  for (feat, nom_pos) in zip(characterize_stage["features"],characterize_stage["positions"]):
    zeroed_nom_pos = nom_pos - nom_zero_pos
    b_pos = v2calculations.calc_pos_b(feat, x_dir, y_dir, z_dir, APPROX_COR)
    if zeroed_nom_pos <= 90 and (360-b_pos) <= 5:
      #needed because B is continuous, near-0 nominal positions have near-0 true positions, not near-360
      zeroed_nom_pos = zeroed_nom_pos - 360
    positions.append((zeroed_nom_pos, b_pos))

  return positions

def getBErrorLineAndSphere(state):
  (x_dir,y_dir,z_dir) = getAxisDirections(state)
  b_stage = state.getStage(Stages.CHARACTERIZE_B_SPHERE)
  features = b_stage["features"]
  positions = b_stage["positions"]

  centers = v2calculations.calc_sphere_centers(features)

  circle = centers.circle()
  cor_pt = circle[1]
  zero_pt = b_stage["zero"].sphere()[1]

  axis_of_rotation = circle[2]
  if np.dot(y_dir,axis_of_rotation) < 0:
    axis_of_rotation *= -1

  # Each point needs to be projected to the plane perpendicular
  # to the axis of rotation.
  plane = (cor_pt,axis_of_rotation)

  b_line_stage = state.getStage(Stages.CHARACTERIZE_B_LINE)
  b_line_features = b_line_stage["features"]
  b_line_positions = b_line_stage["positions"]
  zero_b_pos = b_line_stage["zero_b_pos"]
  zero_line = b_line_stage["zero"].projectToPlane(plane)
  unit_zero_vec = zero_line.line()[1]

  # We need to convert all out points into a 2D space for calculating a ccw rotation,
  # so we're going to define an x_vec and y_vec that define that 2D space.

  x_vec = -unit_zero_vec
  y_vec = np.cross(axis_of_rotation, x_vec)

  b_err = []
  for (feature,pos) in zip(b_line_features,b_line_positions):
    line = feature.projectToPlane(plane).line()
    dir = -1*projectDirectionToPlane(line[1], plane)
    vec = dir/np.linalg.norm(dir)
    x = np.dot(x_vec, vec)
    y = np.dot(y_vec, vec)

    angle = angle_between_ccw_2d([1,0], [x,y])
    if angle < 0:
      angle += 360
    commanded_angle = pos["b"]-zero_b_pos
    error = commanded_angle-angle
    if error > 300:
      error -= 360
    elif error < -300:
      error += 360


    b_err.append((commanded_angle, error))

  return b_err


def getAErrorSphere(state):
  (x_dir,y_dir,z_dir) = getAxisDirections(state)
  a_stage = state.getStage(Stages.CHARACTERIZE_A_SPHERE)
  zero_a_pos = a_stage["zero_a_pos"]
  features = a_stage["features"]
  positions = a_stage["positions"]

  centers = v2calculations.calc_sphere_centers(features)

  circle = centers.circle()
  cor_pt = circle[1]
  zero_pt = a_stage["zero"].sphere()[1]

  axis_of_rotation = circle[2]
  if np.dot(x_dir,axis_of_rotation) < 0:
    axis_of_rotation *= -1

  # We're defining the zero point that we measured to be A0.
  # Each point needs to be projected to the plane perpendicular
  # to the axis of rotation.
  plane = (cor_pt,axis_of_rotation)
  zero_pt_proj = projectPointToPlane(zero_pt, plane)

  zero_vec = zero_pt_proj-cor_pt
  unit_zero_vec = zero_vec/np.linalg.norm(zero_vec)

  # We need to convert all out points into a 2D space for calculating a ccw rotation,
  # so we're going to define an x_vec and y_vec that define that 2D space.

  y_vec = np.cross(axis_of_rotation,unit_zero_vec)
  x_vec = unit_zero_vec

  a_err = []
  for (feature,pos) in zip(features,positions):
    pt = feature.sphere()[1]
    pt_proj = projectPointToPlane(pt, plane)
    vec = pt_proj-cor_pt
    r = np.linalg.norm(vec)
    x = np.dot(x_vec, vec)
    y = np.dot(y_vec, vec)

    angle = angle_between_ccw_2d([1,0], [x,y])
    commanded_angle = pos["a"]-zero_a_pos
    print(f"{commanded_angle}\t{r-circle[0]}")
    error = commanded_angle-angle

    a_err.append((commanded_angle, error))

  return a_err

def getBErrorSphere(state):
  (x_dir,y_dir,z_dir) = getAxisDirections(state)
  b_stage = state.getStage(Stages.CHARACTERIZE_B_SPHERE)
  zero_b_pos = b_stage["zero_b_pos"]
  features = b_stage["features"]
  positions = b_stage["positions"]

  centers = v2calculations.calc_sphere_centers(features)

  circle = centers.circle()
  cor_pt = circle[1]
  zero_pt = b_stage["zero"].sphere()[1]

  axis_of_rotation = circle[2]
  if np.dot(y_dir,axis_of_rotation) < 0:
    axis_of_rotation *= -1

  # We're defining the zero point that we measured to be B0.
  # Each point needs to be projected to the plane perpendicular
  # to the axis of rotation.
  plane = (cor_pt,axis_of_rotation)
  zero_pt_proj = projectPointToPlane(zero_pt, plane)

  zero_vec = zero_pt_proj-cor_pt
  unit_zero_vec = zero_vec/np.linalg.norm(zero_vec)

  # We need to convert all out points into a 2D space for calculating a ccw rotation,
  # so we're going to define an x_vec and y_vec that define that 2D space.

  y_vec = np.cross(axis_of_rotation,unit_zero_vec)
  x_vec = unit_zero_vec

  b_err = []
  for (feature,pos) in zip(features,positions):
    pt = feature.sphere()[1]
    pt_proj = projectPointToPlane(pt, plane)
    vec = pt_proj-cor_pt
    x = np.dot(x_vec, vec)
    y = np.dot(y_vec, vec)

    angle = angle_between_ccw_2d([1,0], [x,y])
    if angle < 0:
      angle += 360

    commanded_angle = pos["b"]-zero_b_pos
    error = commanded_angle-angle
    if error > 300:
      error -= 360
    elif error < -300:
      error += 360

    r = np.linalg.norm(vec)
    print(f"{commanded_angle}\t{r-circle[0]}")

    b_err.append((commanded_angle, error))

  print("Circle", circle)

  return b_err

def getAPositionsSphere(state):
  stage = state.getStage(Stages.CHARACTERIZE_A_SPHERE)
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


def getAErrorLineAndSphere(state):
  (x_dir,y_dir,z_dir) = getAxisDirections(state)
  a_stage = state.getStage(Stages.CHARACTERIZE_A_SPHERE)
  features = a_stage["features"]
  positions = a_stage["positions"]

  centers = v2calculations.calc_sphere_centers(features)

  circle = centers.circle()
  cor_pt = circle[1]
  zero_pt = a_stage["zero"].sphere()[1]

  axis_of_rotation = circle[2]
  if np.dot(x_dir,axis_of_rotation) < 0:
    axis_of_rotation *= -1

  # We're defining the zero point that we measured to be A0.
  # Each point needs to be projected to the plane perpendicular
  # to the axis of rotation.
  plane = (cor_pt,axis_of_rotation)

  a_line_stage = state.getStage(Stages.CHARACTERIZE_A_LINE)
  a_line_features = a_line_stage["features"]
  a_line_positions = a_line_stage["positions"]
  zero_a_pos = a_line_stage["zero_a_pos"]
  zero_line = a_line_stage["zero"].projectToPlane(plane)
  unit_zero_vec = zero_line.line()[1]

  # We need to convert all out points into a 2D space for calculating a ccw rotation,
  # so we're going to define an x_vec and y_vec that define that 2D space.

  y_vec = -unit_zero_vec
  x_vec = np.cross(axis_of_rotation, y_vec)

  a_err = []
  for (feature,pos) in zip(a_line_features,a_line_positions):
    line = feature.projectToPlane(plane).line()
    dir = -1*projectDirectionToPlane(line[1], plane)
    vec = dir/np.linalg.norm(dir)
    x = np.dot(x_vec, vec)
    y = np.dot(y_vec, vec)

    angle = -angle_between_ccw_2d([0,1], [x,y])
    commanded_angle = pos["a"]-zero_a_pos
    error = commanded_angle-angle

    a_err.append((commanded_angle, error))

  return a_err

def getBErrorLineAndSphere(state):
  (x_dir,y_dir,z_dir) = getAxisDirections(state)
  b_stage = state.getStage(Stages.CHARACTERIZE_B_SPHERE)
  features = b_stage["features"]
  positions = b_stage["positions"]

  centers = v2calculations.calc_sphere_centers(features)

  circle = centers.circle()
  cor_pt = circle[1]
  zero_pt = b_stage["zero"].sphere()[1]

  axis_of_rotation = circle[2]
  if np.dot(y_dir,axis_of_rotation) < 0:
    axis_of_rotation *= -1

  # Each point needs to be projected to the plane perpendicular
  # to the axis of rotation.
  plane = (cor_pt,axis_of_rotation)

  b_line_stage = state.getStage(Stages.CHARACTERIZE_B_LINE)
  b_line_features = b_line_stage["features"]
  b_line_positions = b_line_stage["positions"]
  zero_b_pos = b_line_stage["zero_b_pos"]
  zero_line = b_line_stage["zero"].projectToPlane(plane)
  unit_zero_vec = zero_line.line()[1]

  # We need to convert all out points into a 2D space for calculating a ccw rotation,
  # so we're going to define an x_vec and y_vec that define that 2D space.

  x_vec = -unit_zero_vec
  y_vec = np.cross(axis_of_rotation, x_vec)

  b_err = []
  for (feature,pos) in zip(b_line_features,b_line_positions):
    line = feature.projectToPlane(plane).line()
    dir = -1*projectDirectionToPlane(line[1], plane)
    vec = dir/np.linalg.norm(dir)
    x = np.dot(x_vec, vec)
    y = np.dot(y_vec, vec)

    angle = angle_between_ccw_2d([1,0], [x,y])
    if angle < 0:
      angle += 360
    commanded_angle = pos["b"]-zero_b_pos
    error = commanded_angle-angle
    if error > 300:
      error -= 360
    elif error < -300:
      error += 360


    b_err.append((commanded_angle, error))

  return b_err



def getYHomeOffsetFromASpheres(state):
  a_stage = state.getStage(Stages.CHARACTERIZE_A_SPHERE)
  features = a_stage["features"]
  positions = a_stage["positions"]

  centers = v2calculations.calc_sphere_centers(features)

  circle = centers.circle()

  z_stage = state.getStage(Stages.HOMING_Z)
  z_features = z_stage["features"]
  z_centers = v2calculations.calc_sphere_centers(z_features)
  z0 = z_centers.average()

  z_char_stage = state.getStage(Stages.CHARACTERIZE_Z)
  z_3 = z_char_stage["features"][-1].sphere()[1]

  y_stage = state.getStage(Stages.HOMING_Y)
  y_features = y_stage["features"]
  y_centers = v2calculations.calc_sphere_centers(y_features)
  y0 = y_centers.average()

  vec = circle[1]-z0

  y_dir = getYDirectionOOS(state)

  return np.dot(y_dir, vec)

def getXHomeOffsetFromBSpheres(state):
  b_stage = state.getStage(Stages.CHARACTERIZE_B_SPHERE)
  features = b_stage["features"]
  positions = b_stage["positions"]

  centers = v2calculations.calc_sphere_centers(features)

  circle = centers.circle()

  x_stage = state.getStage(Stages.HOMING_X)
  x_features = x_stage["features"]
  x_centers = v2calculations.calc_sphere_centers(x_features)
  x0 = x_centers.average()

  vec = circle[1]-x0

  x_dir = getXDirectionOOS(state)

  return np.dot(x_dir, vec)

