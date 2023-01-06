"""
Helper functions for performing calculations on probed `metrology.Feature`s and other data.
"""
import numpy as np
import math
from metrology import Feature, angle_between_ccw_2d, intersectLines, angle_between
from ipp import Csy
import logging

logger = logging.getLogger(__name__)

def calc_part_csy(csy, L_bracket_top_face, L_bracket_back_line, L_bracket_right_line):
  """
  Caclulates a new Csy using the probed L bracket features that align the Csy axes
  to match the direction of the V2 axes. The origin is set to the top back right corner
  of the L bracket. The L bracket features are in the original csy space and a new,
  combined Csy is return.
  """
  top_plane = L_bracket_top_face.plane()

  right_line = L_bracket_right_line.reverse().projectToPlane(top_plane).line()
  back_line = L_bracket_back_line.reverse().projectToPlane(top_plane).line()

  intersections = intersectLines(right_line, back_line)

  origin = intersections[0]
  y = top_plane[1]
  x = np.cross(back_line[1], y)
  z = np.cross(x, y)

  mat0 = np.array([
    [ x[0], y[0], z[0], origin[0] ],
    [ x[1], y[1], z[1], origin[1] ],
    [ x[2], y[2], z[2], origin[2] ],
    [    0,    0,    0,         1 ]
  ])

  mat1 = csy.toMatrix4()

  newmat = np.matmul(mat1, mat0)

  return Csy.fromMatrix4(newmat)


def calc_pos_a(a_line, x_dir, y_dir, z_dir, origin):
  '''
  Calculates an A position of the machine using the a_line feature returned by probe_a, projecting
  the points gathered to the YZ plane (defined by the +X axis), converting those points to a 2D
  coordinate to calculate a ccw angle.
  '''

#  a_line is sampled from top to bottom, so going in the -Y direction when A is 0. Reverse the order
#  to make it go in the +Y direction, then project it to the YZ plane.
  a_line_proj = a_line.reverse().projectToPlane((origin,x_dir))
  a_line_proj_line = a_line_proj.line()
  a_proj_line_dir = a_line_proj_line[1]

  y = np.dot(a_proj_line_dir, y_dir)
  z = np.dot(a_proj_line_dir, z_dir)

  logger.debug('line dir y %s' % (y,))
  logger.debug('line dir z %s' % (z,))
  
  # A0 sample starts along +Y axis, so get angle relative to [0, 1] where 2D coordinate is [ z, y ]
  a_pos = -angle_between_ccw_2d([0, 1], [z, y])

  logger.debug('a_pos %s', a_pos)

  return a_pos

def calc_pos_b(b_line, x_dir, y_dir, z_dir, origin):
  '''
  Calculates a B position of the machine using the b_line feature returned by probe_b, projecting
  the points gathered to the XZ plane (defined by the +Y axis), converting those points to a 2D
  coordinate to calculate a ccw angle.
  '''

#  b_line is sampled going in the -X direction. Reverse the order
#  to make it go in the +X direction, then project it to the XZ plane.
  b_line_proj = b_line.reverse().projectToPlane((origin,y_dir))
  
  line = b_line_proj.line()
  b_line_dir = line[1]

  z = np.dot(b_line_dir, z_dir)
  x = np.dot(b_line_dir, x_dir)

  # B0 sample starts along +X axis, so get angle relative to [0, 1] where 2D coordinate is [ z, x ]
  b_pos = angle_between_ccw_2d([0, 1],[z, x])

  #offset by 45 degrees because thats the fixture face angle at B0
  b_pos = b_pos + 45

  # put b angle in 0 to 360 range rather than -180 to 180
  if b_pos < 0:
    b_pos += 360
  
  logger.debug('b_pos %s', b_pos)

  return b_pos
    
def calc_ccw_angle_from_y(line, x_dir, y_dir, z_dir, origin):
  '''
  Calculates a rotation about +Z (with +Y at 0, going ccw) by projecting
  the points in the line feature the XY plane (defined by the +Z axis), 
  converting the direction of that projected line to a 2D coordinate 
  to calculate an angle.
  '''

  line = line.reverse().projectToPlane((origin,z_dir)).line()
  line_dir = line[1]

  x = np.dot(line_dir, x_dir)
  y = np.dot(line_dir, y_dir)

  angle = angle_between_ccw_2d([0, 1], [x, y])

  return angle

def calc_ccw_angle_from_x(line, x_dir, y_dir, z_dir, origin):
  '''
  Calculates a rotation about +Z (with +X at 0, going ccw) by projecting
  the points in the line feature the XY plane (defined by the +Z axis), 
  converting the direction of that projected line to a 2D coordinate 
  to calculate an angle.
  '''

  line = line.projectToPlane((origin,z_dir)).line()
  line_dir = line[1]

  x = np.dot(line_dir, x_dir)
  y = np.dot(line_dir, y_dir)

  angle = angle_between_ccw_2d([1, 0], [x, y])
  logger.debug('calc_ccw_angle_from_x %s', angle)

  return angle

def calc_sphere_centers(feats):
  positions = Feature()
  for feat in feats:
    (rad, pos) = feat.sphere()
    positions.addPoint(*pos)
  return positions

def calc_max_dist_between_points(feat):
  max_dist = 0
  pts = feat.points()
  for (i,pt) in enumerate(pts):
    for (j,pt2) in enumerate(pts):
      if i != j:
        dist = np.linalg.norm(pt - pt2)
        if dist > max_dist:
          max_dist = dist
  return max_dist

def calc_max_dist_from_pos(pos,feat):
  max_dist = 0
  for pt in feat.points():
    dist = np.linalg.norm(pt - pos)
    if dist > max_dist:
      max_dist = dist
  return max_dist

def calc_max_diff(numbers):
  max_diff = 0
  for n in numbers:
    for m in numbers:
      diff = abs(n - m)
      if diff > max_diff:
        max_diff = diff
  return max_diff

def calc_home_offset_x(self, points, x_dir, y_dir, z_dir, origin, x0, x_home_offset):
  """
  """
  x_vals = []
  for pt in points():
    pt_cnccsy = np.matmul(self.cmm2cnc,np.append(pt,1))
    x_vals.append(pt_cnccsy[0])
  x_avg = np.array(x_vals).mean()
  x_offset_err = x_avg - x0
  return x_home_offset + x_offset_err

def calc_home_offset_y(self, points, x_dir, y_dir, z_dir, origin, y0, y_home_offset):
  """
  """
  y_vals = []
  for pt in points():
    pt_cnccsy = np.matmul(self.cmm2cnc,np.append(pt,1))
    y_vals.append(pt_cnccsy[1])
  y_avg = np.array(y_vals).mean()
  y_offset_err = y_avg - y0
  return y_home_offset + y_offset_err

def calc_b_table_offset(self, pos_a_cor, pos_top_plane, y_dir, probe_dia):
  vec_top_plane_to_a_cor = pos_a_cor - pos_top_plane
  dist_top_plane_to_a_cor = np.dot(vec_top_plane_to_a_cor, y_dir) + PROBE_DIA/2
  logger.debug('dist_top_plane_to_a_cor %s' % dist_top_plane_to_a_cor)
  b_table_offset = (dist_top_plane_to_a_cor + FIXTURE_HEIGHT) / 25.4
  #TODO return b_table_offset


def calc_tool_probe_offset(self, tool_probe_pos, plane_a90, x_dir, y_dir, z_dir, origin):
  pass #TODO

def calc_sphere_diameter_deviation(sphere_feature, expected_dia):
  """Returns the absolute value   of the difference in diameter between expected_dia and the best fit sphere diameter of sphere_feature."""
  (rad,pos) = sphere_feature.sphere()
  return abs(rad*2-expected_dia)

def calc_max_sphere_diameter_deviation(sphere_features, expected_dia):
  """Returns the max difference in diameter between expected_dia and the best fit sphere diameters of each of the sphere_features."""
  max_dev = 0
  for f in sphere_features:
    dev = calc_sphere_diameter_deviation(f, expected_dia)
    if dev > max_dev:
      max_dev = dev
  return max_dev

def calc_parallelism(dir1,dir2,run):
  """
  Calculates the pallelism over a distance of run of two direction vectors.
  """
  dir1 = np.array(dir1)
  dir2 = np.array(dir2)
  dot = np.dot(dir1,dir2)

  return np.linalg.norm(run*dir1-run*dot*dir2)

