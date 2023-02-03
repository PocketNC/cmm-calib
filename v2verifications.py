"""
Helper functions that raise errors if a verification fails.
"""
import v2calculations
import logging

logger = logging.getLogger(__name__)

BEST_FIT_SPHERE_ERROR = 0.0254
LINEAR_HOMING_REPEATABILITY = 0.001 * 25.4 # 0.001 inches, 0.0254 mm
B_HOMING_REPEATABILITY = 0.04 # degrees
A_HOMING_REPEATABILITY = 0.08 # degrees

class CalibException(Exception):
  pass

def verify_sphere_diameters_within_tolerance(features, expected_dia):
  """Verifies that all best fit sphere diameters in features are within BEST_FIT_SPHERE_ERROR of expected_dia."""
  dia_error = v2calculations.calc_max_sphere_diameter_deviation(features, expected_dia)
  if dia_error > BEST_FIT_SPHERE_ERROR:
    raise CalibException("Deviation in best-fit sphere diameter. Expected <= %s, found %s" % (BEST_FIT_SPHERE_ERROR, dia_error))

def verify_linear_homing_repeatability(features, axis):
  centers = v2calculations.calc_sphere_centers(features)
  max_dist = v2calculations.calc_max_dist_between_points(centers)
  if max_dist > LINEAR_HOMING_REPEATABILITY:
    raise CalibException("%s repeatability failure, expected <= %s, found %s" % (axis, LINEAR_HOMING_REPEATABILITY, max_dist))

  return (max_dist, LINEAR_HOMING_REPEATABILITY)

def verify_x_homing_accuracy(homing_features, offsets_features, x_dir, y_dir, z_dir, origin):
  home_offset_error = v2calculations.calc_home_offset_x_error(x_dir, y_dir, homing_features, offsets_features)
  if home_offset_error > LINEAR_HOMING_REPEATABILITY:
    raise CalibException("X homing accuracy failure, expected error <= %s, found %s" % (LINEAR_HOMING_REPEATABILITY, home_offset_error))  
  return (home_offset_error, LINEAR_HOMING_REPEATABILITY)

def verify_y_homing_accuracy(homing_features, offsets_features, x_dir, y_dir, z_dir, origin):
  home_offset_error = v2calculations.calc_home_offset_y_error(x_dir, y_dir, homing_features, offsets_features)
  if home_offset_error > LINEAR_HOMING_REPEATABILITY:
    raise CalibException("Y homing accuracy failure, expected error <= %s, found %s" % (LINEAR_HOMING_REPEATABILITY, home_offset_error))  
  return (home_offset_error, LINEAR_HOMING_REPEATABILITY)

def verify_a_homing_repeatability(features, x_dir, y_dir, z_dir, origin):
  positions = []
  for a_line in features:
    a_pos = v2calculations.calc_pos_a(a_line, x_dir, y_dir, z_dir, origin)
    positions.append(a_pos)
  max_diff = v2calculations.calc_max_diff(positions)
  if max_diff > A_HOMING_REPEATABILITY:
    raise CalibException("A repeatability failure, expected <= %s, found %s" % (A_HOMING_REPEATABILITY, max_diff))

  return (max_diff, A_HOMING_REPEATABILITY)

def verify_b_homing_repeatability(features, x_dir, y_dir, z_dir, origin):
  positions = []
  for b_line in features:
    b_pos = v2calculations.calc_pos_b(b_line, x_dir, y_dir, z_dir, origin)
    positions.append(b_pos)
  max_diff = v2calculations.calc_max_diff(positions)
  if max_diff > B_HOMING_REPEATABILITY:
    raise CalibException("B repeatability failure, expected <= %s, found %s" % (B_HOMING_REPEATABILITY, max_diff))

  return (max_diff, B_HOMING_REPEATABILITY)

def verify_linear_axis_direction(dir1,dir2):
  """
  Sanity check that linear axes are reasonably parallel with the L bracket.
  The L bracket surfaces are not precision, so this is only a sanity check,
  not any kind of spec.
  """

  parallelism = v2calculations.calc_parallelism(dir1,dir2,6)
  # error beyond above .05" over 6" (~.5 degrees)
  LIMIT = .05
  if parallelism > LIMIT:
    raise CalibError("Linear axis parallelism exceeds reasonable limits: %s, %s, %s, expected <= %s" % (dir1,dir2,parallelism, LIMIT))
  else:
    logger.debug("Linear axis parallelism: %s, %s, %s, expected <= %s", dir1, dir2, parallelism, LIMIT)
