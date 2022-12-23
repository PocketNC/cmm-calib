"""
Helper functions that raise errors if a verification fails.
"""
import v2calculations
import logging

logger = logging.getLogger(__name__)

BEST_FIT_SPHERE_ERROR = 0.0254
LINEAR_HOMING_REPEATABILITY = 0.001 * 25.4 # 0.001 inches, 0.0254 mm

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
