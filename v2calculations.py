import numpy as np
import math
from metrology import Feature, angle_between_ccw_2d, intersectLines, angle_between
import logging

logger = logging.getLogger(__name__)

class Csy:
  def __init__(self, origin, euler):
    self.origin = origin
    self.euler = euler

APPROX_CSY = Csy([ 653.0, 134.0, 126.5 ], [0, -90, 0])

def calc_part_csy(L_bracket_top_face, L_bracket_back_line, L_bracket_right_line):
  """
  Caclulates a more accurate origin and rotation about Z using the features probed using
  the provided origin and euler angles.
  """
  top_plane = L_bracket_top_face.plane()

  logger.debug("plane pt %s", top_plane[0])
  logger.debug("plane n %s", top_plane[1])
  
  back_line = L_bracket_back_line.projectToPlane(top_plane).line()

  logger.debug("back line pt %s", back_line[0])
  logger.debug("back line dir %s", back_line[1])

  right_line = L_bracket_right_line.reverse().projectToPlane(top_plane).line()

  logger.debug("right line pt %s", right_line[0])
  logger.debug("right line dir %s", right_line[1])

  intersections = intersectLines(back_line, right_line)

  corner_pt = intersections[0]
  back_line_z = L_bracket_back_line.projectToPlane((corner_pt, (0,0,1))).line()

  angle = angle_between_ccw_2d([1, 0], [back_line_z[1][0], back_line_z[1][1]])

# This is not very generic. Ideally, Csy could handle combining multiple Csy objects
# by converting to 4x4 tranformation matrices, multiplying those matrices and then
# converting back to a Csy object, but we haven't been able to successfully represent
# how the CMM uses euler angles. For now we assume the euler angles are (0, -90, 0)
# so we can add the y-component of corner_pt to the global x machine coordinate and
# the negative x-component or corner_pt to the global y machine coordinate. We also
# only adjust the second euler angle, which is a rotation about the CMM +Z axis.

  return Csy((APPROX_CSY.origin[0]+corner_pt[1],
              APPROX_CSY.origin[1]-corner_pt[0],
              APPROX_CSY.origin[2]+corner_pt[2]), (APPROX_CSY.euler[0], APPROX_CSY.euler[1]+angle, APPROX_CSY.euler[2]))




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
