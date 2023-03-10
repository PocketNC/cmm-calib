"""
Helper functions that raise errors if a verification fails.
"""
import v2calculations
import logging
from v2calculations import calc_sphere_centers
from metrology import linePerpendicularity, angle_between, straightness

logger = logging.getLogger(__name__)

BEST_FIT_SPHERE_ERROR = 0.0254
LINEAR_HOMING_REPEATABILITY = 0.001 * 25.4 # 0.001 inches, 0.0254 mm
B_HOMING_REPEATABILITY = 0.04 # degrees
A_HOMING_REPEATABILITY = 0.08 # degrees
ANGULAR_ACCURACY = 0.05 # degrees

PERPENDICULARITY_SPEC = .002 # inches

class CalibException(Exception):
  pass

def verify_axes(xFeatures, yFeatures, zFeatures):
  xLineFeature = calc_sphere_centers(xFeatures)
  yLineFeature = calc_sphere_centers(yFeatures)
  zLineFeature = calc_sphere_centers(zFeatures)

  xLine = xLineFeature.line()
  yLine = yLineFeature.line()
  zLine = zLineFeature.line()

  xyPerpendicularity = linePerpendicularity(yLineFeature, xLineFeature.line())/25.4
  zyPerpendicularity = linePerpendicularity(yLineFeature, zLineFeature.line())/25.4
  zxPerpendicularity = linePerpendicularity(xLineFeature, zLineFeature.line())/25.4

  yxPerpendicularity = linePerpendicularity(xLineFeature, yLineFeature.line())/25.4
  yzPerpendicularity = linePerpendicularity(zLineFeature, yLineFeature.line())/25.4
  xzPerpendicularity = linePerpendicularity(zLineFeature, xLineFeature.line())/25.4

  xStraightness = straightness(xLineFeature)/25.4
  yStraightness = straightness(yLineFeature)/25.4
  zStraightness = straightness(zLineFeature)/25.4

  xyPerp = xyPerpendicularity if xStraightness < yStraightness else yxPerpendicularity
  zxPerp = zxPerpendicularity if zStraightness < xStraightness else xzPerpendicularity
  zyPerp = zyPerpendicularity if zStraightness < yStraightness else yzPerpendicularity

  xyAngle = angle_between(xLine[1], yLine[1])
  zxAngle = angle_between(zLine[1], xLine[1])
  zyAngle = angle_between(zLine[1], xLine[1])

  xyPass = xyPerp < PERPENDICULARITY_SPEC
  zxPass = zxPerp < PERPENDICULARITY_SPEC
  zyPass = zyPerp < PERPENDICULARITY_SPEC

  overallPass = all([ xyPass, zxPass, zyPass ])

  logger.info(f"XY Perpendicularity: {xyPerp:.6f} {'PASS' if xyPass else 'FAIL'}")
  logger.info(f"XY Angle: {xyAngle}")
  logger.info(f"ZX Perpendicularity: {zxPerp:.6f} {'PASS' if zxPass else 'FAIL'}")
  logger.info(f"ZX Angle: {zxAngle}")
  logger.info(f"ZY Perpendicularity: {zyPerp:.6f} {'PASS' if zyPass else 'FAIL'}")
  logger.info(f"ZY Angle: {zyAngle}")
  
  if not overallPass:
    err = f"One or more axes out of square by more than {PERPENDICULARITY_SPEC}. XY Perpendicularity: {xyPerp:.6f} inches, {xyAngle} degrees. ZX Perpendicularity: {zxPerp:.6f} inches, {zxAngle} degrees. ZY Perpendicularity: {zyPerp:.6f}, {zyAngle} degrees."
    logger.error(err)
    raise CalibException(err)


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

def verify_x_homing_accuracy(homing_features, offsets_features, x_dir, y_dir):
  homing_error = v2calculations.calc_home_offset_x_error(x_dir, y_dir, homing_features, offsets_features)
  if abs(homing_error) > LINEAR_HOMING_REPEATABILITY:
    raise CalibException("X homing accuracy failure, expected error <= %s, found %s" % (LINEAR_HOMING_REPEATABILITY, homing_error))  
  return (homing_error, LINEAR_HOMING_REPEATABILITY)

def verify_y_homing_accuracy(homing_features, offsets_features, x_dir, y_dir, z_dir, origin):
  homing_error = v2calculations.calc_home_offset_y_error(x_dir, y_dir, homing_features, offsets_features)
  if abs(homing_error) > LINEAR_HOMING_REPEATABILITY:
    raise CalibException("Y homing accuracy failure, expected error <= %s, found %s" % (LINEAR_HOMING_REPEATABILITY, homing_error))  
  return (homing_error, LINEAR_HOMING_REPEATABILITY)

def verify_a_homing_accuracy(homing_features, x_dir, y_dir, z_dir, origin):
  homing_error = v2calculations.calc_homing_error_a(homing_features, x_dir, y_dir, z_dir)
  if abs(homing_error) > A_HOMING_REPEATABILITY:
    raise CalibException("A homing accuracy failure, expected error <= %s, found %s" % (A_HOMING_REPEATABILITY, homing_error))  
  return (homing_error, LINEAR_HOMING_REPEATABILITY)

def verify_b_homing_accuracy(homing_features, x_dir, y_dir, z_dir, origin):
  homing_error = v2calculations.calc_homing_error_b(homing_features, x_dir, y_dir, z_dir)
  if abs(homing_error) > B_HOMING_REPEATABILITY:
    raise CalibException("B homing accuracy failure, expected error <= %s, found %s" % (B_HOMING_REPEATABILITY, homing_error))  
  return (homing_error, LINEAR_HOMING_REPEATABILITY)

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

def verify_rotary_accuracy(errors, axis):
  """
  Check if error exceeded spec at any position for a set of rotary axis features
  """
  max_error_pair = (0,0)
  failures = []
  for (pos, err) in errors:
    if abs(err) > abs(max_error_pair[1]):
      max_error_pair = (pos,err)
    if abs(err) > ANGULAR_ACCURACY:
      failures.append((pos,err))

  if len(failures) > 0:
    raise CalibException("%s accuracy failure. Expected max error <= %s exceeded at following points (pos, err): %s" % (axis, ANGULAR_ACCURACY, str(failures).strip('[]')))

  return (max_error_pair,ANGULAR_ACCURACY)
