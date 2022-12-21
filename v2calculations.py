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
  a_pos = angle_between_ccw_2d([0, 1], [z, y])

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
  b_pos = angle_between_ccw_2d([0, 1], [z, x])

  # put b angle in 0 to 360 range rather than -180 to 180
  if b_pos < 0:
    b_pos += 360

  return b_pos
    
def calc_ccw_angle_from_y(line, x_dir, y_dir, z_dir, origin):
  '''
  Calculates a rotation about +Z (with +Y at 0, going ccw) by projecting
  the points in the line feature the XY plane (defined by the +Z axis), 
  converting the direction of that projected line to a 2D coordinate 
  to calculate an angle.
  '''

  line = line.reverse().projectOntoPlane((origin,z_dir)).line()
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

  line = line.reverse().projectOntoPlane((origin,z_dir)).line()
  line_dir = line[1]

  x = np.dot(line_dir, x_dir)
  y = np.dot(line_dir, y_dir)

  angle = angle_between_ccw_2d([1, 0], [x, y])

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
