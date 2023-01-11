"""
Functions used to perform calibration routines on the cmm that return `metrology.Feature`s and other data that will be stored using `calibstate`.
"""
import logging
import math
from ipp import float3, CmmException, readPointData
import ipp_routines as routines
from metrology import Feature
from scipy.spatial.transform import Rotation
import numpy as np

logger = logging.getLogger(__name__)

Y_CLEARANCE_PART_CSY = 250
X_CLEARANCE_PART_CSY = -250

Z_CLEARANCE_TMP_PART_CSY = 250

FIXTURE_HEIGHT = 25.2476
FIXTURE_SIDE = 76.2
FIXTURE_DIAG = 107.76
FIXTURE_BALL_DIA = 6.35
SPINDLE_BALL_DIA_10 = 12.69792
SPINDLE_BALL_DIA_50 = 6.35
FIXTURE_OFFSET_B_ANGLE = 225
PROBE_DIA = 4
TOOL_3_LENGTH = 117.8

B_LINE_LENGTH = 35

APPROX_FIXTURE_BALL_HOME = float3(-133.18, -58.3, -111.0)

APPROX_COR = float3(-134.9,  -69.32358094, -78.46214211)
APPROX_COR_B = float3(-133.0, -60.1, -78.5)
APPROX_FIXTURE_TOP_PLANE_CENTER = float3(-134.37, -65.5,-80.73)
ORIGIN_A_START_PROBE_POS = float3(-68.5, -73.0, -80.2)
PROBE_FIXTURE_PLANE_A90_WAYPOINT = float3(-116.0, -41.5, -74.9)
APPROX_FIN_TIP = float3(-152.27,-15,-53.28)
V2_10 = "V2_10"
V2_50 = "V2_50"

async def probe_spindle_tip(client, spindle_home, ball_dia, x_pos_v2, z_pos_v2):
  """
  Probe against the ball mounted in the spindle
  Probe should be aligned vertical (or close) before running
  The first move is to the position 25mm above the target

  Find the actual spindle position given where we probed the spindle's home position and the x and z location of the V2 machine.
  This routine uses head touches and trusts that the spindle is very close to the provided coordinates. probe_spindle_pos should
  already have been run and the resulting center of the best fit sphere should be passed in as spindle_home.

  ball_dia needs to be provided as it differs between V2-10 and V2-50 machines.

  spindle_home is a float3 that is the center of the ball at X0Z0. This should be the center
  of the ball that we probed in the PROBE_SPINDLE_POS stage.
  """
  pts = Feature()
  orig = float3(spindle_home) + float3(x_pos_v2, 0, z_pos_v2)
  tool_orig = orig + TOOL_3_LENGTH*float3(0,1,0)
  contact_radius = (ball_dia+PROBE_DIA)/2
  clearance_radius = contact_radius + 2
  a_angle_probe_contact = math.atan2(contact_radius,TOOL_3_LENGTH)*180/math.pi

  #orient tool down and place tip 25 mm above target
  # await client.GoTo("Tool.Alignment(0,0,1,1,0,0)").complete()
  # await client.GoTo((orig + float3(0,25,0)).ToXYZString()).complete()
  # getCurrPosCmd = await client.Get("X(),Y(),Z()").data()
  # currPos = readPointData(getCurrPosCmd.data_list[0])
  
  approach_pos = orig + float3(0,50,0) 
  await client.GoTo("%s,Tool.Alignment(0,1,0)" % (approach_pos.ToXYZString())).ack()
  await client.SetProp("Tool.PtMeasPar.HeadTouch(1)").send()
  await client.SetProp("Tool.PtMeasPar.Search(6)").complete()
  
  #place tip pos +X from target
  start_pos = orig + float3(clearance_radius,0,0) 
  tool_alignment = np.array((tool_orig - start_pos).normalize())
  await client.AlignTool("%s,%s,%s,0" %(tool_alignment[0],tool_alignment[1],tool_alignment[2])).complete()
  await client.GoTo("%s" % (start_pos.ToXYZString())).complete()

  # probe in -X dir
  contact_pos = orig + float3(contact_radius,0,0) 
  ptMeas = await client.PtMeas("%s,IJK(1,0,0)" % (contact_pos.ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  # move to -Z pos, probe in +Z dir
  start_pos = orig + float3(0,0,-clearance_radius)
  tool_alignment = np.array((tool_orig - start_pos).normalize())
  await client.AlignTool("%s,%s,%s,0" %(tool_alignment[0],tool_alignment[1],tool_alignment[2])).complete()
  await client.GoTo("%s" % (start_pos.ToXYZString())).complete()
  ptMeas = await client.PtMeas("%s,IJK(0,0,-1)" % ((orig + float3(0,0,-contact_radius)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  # move to -X pos, probe in +X dir
  start_pos = orig + float3(-clearance_radius,0,0) 
  tool_alignment = np.array((tool_orig - start_pos).normalize())
  await client.AlignTool("%s,%s,%s,0" %(tool_alignment[0],tool_alignment[1],tool_alignment[2])).complete()
  await client.GoTo("%s" % (start_pos.ToXYZString())).complete()
  ptMeas = await client.PtMeas("%s,IJK(-1,0,0)" % ((orig + float3(-contact_radius,0,0)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  #rise 2mm and probe another 3 points
  orig = orig + float3(0,ball_dia/3,0)
  tool_orig = tool_orig + float3(0,ball_dia/3,0)
  start_pos = orig + float3(-clearance_radius,0,0) 
  tool_alignment = np.array((tool_orig - start_pos).normalize())
  await client.AlignTool("%s,%s,%s,0" %(tool_alignment[0],tool_alignment[1],tool_alignment[2])).complete()
  await client.GoTo("%s" % (start_pos.ToXYZString())).complete()
  ptMeas = await client.PtMeas("%s,IJK(-1,0,0)" % ((orig + float3(-contact_radius,0,0)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  # move to -Z pos, probe in +Z dir
  start_pos = orig + float3(0,0,-clearance_radius)
  tool_alignment = np.array((tool_orig - start_pos).normalize())
  await client.AlignTool("%s,%s,%s,0" %(tool_alignment[0],tool_alignment[1],tool_alignment[2])).complete()
  await client.GoTo("%s" % (start_pos.ToXYZString())).complete()
  ptMeas = await client.PtMeas("%s,IJK(0,0,-1)" % ((orig + float3(0,0,-contact_radius)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  # move to +X pos, probe in -X dir
  start_pos = orig + float3(clearance_radius,0,0) 
  tool_alignment = np.array((tool_orig - start_pos).normalize())
  await client.AlignTool("%s,%s,%s,0" %(tool_alignment[0],tool_alignment[1],tool_alignment[2])).complete()
  await client.GoTo("%s" % (start_pos.ToXYZString())).complete()
  ptMeas = await client.PtMeas("%s,IJK(1,0,0)" % ((orig + float3(contact_radius,0,0)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt) 

  await client.GoTo((orig + float3(0,50,0)).ToXYZString()).complete()

  return pts

async def probe_spindle_pos(client, machine, x_pos_v2, z_pos_v2):
  '''
  Find the spindle position using long search distance probes to accomodate variation.
  Use this to initially find the spindle position after homing. It checks either 
  side of the spindle to find the center and uses linear head touches to avoid
  collisions with the geometry of the machine.

  machine: either V2_10 or V2_50
  x_pos_v2: float3 - The X position of the V2.
  z_pos_v2: float3 - The Z position of the V2.

  Returns a list of probed points.
  '''
  pts = Feature()

  #we will be probing against spindle shaft, approx. 45 mm +Z direction from typical spindle tip position
  Z_OFFSET = 45
  Y_SPINDLE_CLEARANCE = 25

  await client.GoTo("Y(%s)" % Y_CLEARANCE_PART_CSY).ack()
  await client.SetProp("Tool.PtMeasPar.Approach(10)").send()
  await client.SetProp("Tool.PtMeasPar.Search(100)").send()
  await client.SetProp("Tool.PtMeasPar.Retract(-1)").send()
  await client.SetProp("Tool.PtMeasPar.HeadTouch(0)").ack()
  await client.GoTo("Tool.A(0),Tool.B(0)").ack()

  if machine == V2_10:
    z_home = float3(-73.0, -68.83, 36.2)
    spindle_side_clearance = 22.5
    ball_dia = SPINDLE_BALL_DIA_10
  elif machine == V2_50:
    z_home = float3(-71.75, -68.83, 24.1) (24.1, -71.75, -68.83)
    spindle_side_clearance = 30
    ball_dia = SPINDLE_BALL_DIA_50
  else:
    raise CmmException("Unknown machine")

  dia = ball_dia + PROBE_DIA
  clearance_radius = dia/2 + 5
  contact_radius = dia/2
  orig = z_home + float3(x_pos_v2 - 63.5, 0, z_pos_v2)

  #probe against spindle shaft from +X position, probing direction -X
  await client.GoTo((orig + float3(spindle_side_clearance, Y_SPINDLE_CLEARANCE, Z_OFFSET)).ToXYZString()).ack()
  await client.GoTo((orig + float3(spindle_side_clearance, 0, Z_OFFSET)).ToXYZString()).ack()
  pt_meas = await client.PtMeas("%s,IJK(1,0,0)" % ((orig + float3(10, 0, Z_OFFSET)).ToXYZString())).data()
  spindle_shaft_pt_1 = float3.FromXYZString(pt_meas.data_list[0])

  #up and over shaft
  await client.GoTo((orig + float3(spindle_side_clearance, Y_SPINDLE_CLEARANCE, Z_OFFSET)).ToXYZString()).ack()
  await client.GoTo((orig + float3(-spindle_side_clearance, Y_SPINDLE_CLEARANCE, Z_OFFSET)).ToXYZString()).ack()

  #probe spindle shaft from -X side, probing in +X direction
  await client.GoTo((orig + float3(-spindle_side_clearance, 0, Z_OFFSET)).ToXYZString()).ack()
  pt_meas = await client.PtMeas("%s,IJK(-1,0,0)" % ((orig + float3(-10, 0, Z_OFFSET)).ToXYZString())).data()
  spindle_shaft_pt_2 = float3.FromXYZString(pt_meas.data_list[0])

  # spindle tip X position should be at mid point of previous two probes
  spindle_shaft_x = (spindle_shaft_pt_1.x + spindle_shaft_pt_2.x) / 2

  #now probe in +Z along the spindle center line until we hit the tip
  await client.GoTo((orig + float3(-30, 0, -50)).ToXYZString()).ack()
  await client.GoTo(float3(spindle_shaft_x, orig.y, orig.z - 50).ToXYZString()).ack()
  meas_pos = float3(spindle_shaft_x, orig.y, orig.z)
  pt_meas = await client.PtMeas("%s,IJK(0,0,-1)" % (meas_pos.ToXYZString())).data()
  tip_pt = float3.FromXYZString(pt_meas.data_list[0])

  pts.addPoint(*tip_pt)

  #take additional points on spindle tip sphere
  #from +X (probe in -X)
  await client.GoTo((tip_pt + float3(clearance_radius, 0, 0)).ToXYZString()).ack()
  await client.GoTo((tip_pt + float3(clearance_radius, 0, contact_radius)).ToXYZString()).ack()

  meas_pos = tip_pt + float3(contact_radius, 0, contact_radius)
  pt_meas = await client.PtMeas("%s,IJK(1,0,0)" % (meas_pos.ToXYZString())).data()
  pt = float3.FromXYZString(pt_meas.data_list[0])
  pts.addPoint(*pt)

  #from +Y (probe in -Y)
  await client.GoTo((tip_pt + float3(clearance_radius, clearance_radius, contact_radius)).ToXYZString()).ack()
  await client.GoTo((tip_pt + float3(0, clearance_radius, contact_radius)).ToXYZString()).ack()
  meas_pos = tip_pt + float3(0, contact_radius, contact_radius)
  pt_meas = await client.PtMeas("%s,IJK(0,1,0)" % (meas_pos.ToXYZString())).data()
  pt = float3.FromXYZString(pt_meas.data_list[0])
  pts.addPoint(*pt)

  #from -X (probe in +X)
  await client.GoTo((tip_pt + float3(-clearance_radius, clearance_radius, contact_radius)).ToXYZString()).ack()
  await client.GoTo((tip_pt + float3(-clearance_radius, 0, contact_radius)).ToXYZString()).ack()
  meas_pos = tip_pt + float3(-contact_radius, 0, contact_radius)
  pt_meas = await client.PtMeas("%s,IJK(-1,0,0)" % (meas_pos.ToXYZString())).data()
  pt = float3.FromXYZString(pt_meas.data_list[0])
  pts.addPoint(*pt)

  return pts

async def go_to_clearance_x(client, x=-250):
  await client.GoTo("X(%s)" % x).complete()

async def go_to_clearance_y(client, y=250):
  await client.GoTo("Y(%s)" % y).complete()

async def go_to_clearance_z(client, z=-250):
  await client.GoTo("Z(%s)" % z).complete()

async def align_tool(client, i, j, k):
  await client.AlignTool("%s,%s,%s,0" % (i,j,k)).complete()

async def probe_machine_pos(client):
  '''
  Routine for gathering point data about the back right corner of the L-bracket
  for precisely locating the Part CSY.

  Returns a 3-tuple of features: (L bracket top face, line on L bracket back face and line on L bracket right face)
  The first list can be used to calculate a best fit plane for the top, then two best fit lines for the back and right face after
  projecting them up to the top face.
  '''
  await client.GoTo("Z(%s)" % Z_CLEARANCE_TMP_PART_CSY).ack()
  await client.SetProp("Tool.PtMeasPar.Approach(10)").send()
  await client.SetProp("Tool.PtMeasPar.Search(15)").send()
  await client.SetProp("Tool.PtMeasPar.Retract(-1)").send()
  await client.SetProp("Tool.PtMeasPar.HeadTouch(0)").send()
  await client.GoTo("Tool.A(0),Tool.B(0)").complete()

  #ensure A-backing plate and B-trunnion are not raised above L-bracket by sweeping probe tip through area
  #sweeping potential location of B-motor when A approx 90 and Y raised near limit
  await client.GoTo(float3(-180, -155, 90).ToXYZString()).ack()
  await client.GoTo(float3(-40, -155, 90).ToXYZString()).ack()
  await client.GoTo(float3(-40, -155, 20).ToXYZString()).ack()
  await client.GoTo(float3(-40, 40, 20).ToXYZString()).ack()

  # L-bracket top face
  approachPos = float3(0,0,10)
  await client.GoTo(approachPos.ToXYZString()).ack()

  measPos = float3(0,7,0)
  L_bracket_top_face = Feature()
  ptMeas = await client.PtMeas("%s,IJK(0,0,1)" % (measPos.ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  L_bracket_top_face.addPoint(*pt)

  measPos = float3(-20,0,0)
  ptMeas = await client.PtMeas("%s,IJK(0,0,1)" % (measPos.ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  L_bracket_top_face.addPoint(*pt)

  measPos = float3(-30,12,0)
  ptMeas = await client.PtMeas("%s,IJK(0,0,1)" % (measPos.ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  L_bracket_top_face.addPoint(*pt)

  measPos = float3(-50,0,0)
  ptMeas = await client.PtMeas("%s,IJK(0,0,1)" % (measPos.ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  L_bracket_top_face.addPoint(*pt)

  measPos = float3(-60,13,0)
  ptMeas = await client.PtMeas("%s,IJK(0,0,1)" % (measPos.ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  L_bracket_top_face.addPoint(*pt)

  # L-bracket back face
  L_bracket_back_face = Feature()
  approach_pos = float3(-60, 30, 10)
  await client.GoTo(approach_pos.ToXYZString()).ack()
  approach_pos = approach_pos + float3(0, 0, -20)
  await client.GoTo(approach_pos.ToXYZString()).ack()
  numPoints = 5
  for i in range(numPoints):
    minx = 0
    maxx = 60
    t = i/(numPoints-1)
    meas_pos = approach_pos + float3(minx*(1-t)+maxx*t, -15, 0)
    pt_meas = await client.PtMeas("%s,IJK(0,1,0)" % (meas_pos.ToXYZString())).data()
    pt = float3.FromXYZString(pt_meas.data_list[0])
    L_bracket_back_face.addPoint(*pt)

  #back-to-side transition
  approach_pos = float3(25,30,-10)
  await client.GoTo(approach_pos.ToXYZString()).ack()

  # L-bracket right face
  approach_pos = float3(23,15,-10)
  await client.GoTo(approach_pos.ToXYZString()).ack()

  L_bracket_right_face = Feature()
  for i in range(3):
    meas_pos = approach_pos + float3(-15, i * -8, 0)
    pt_meas = await client.PtMeas("%s,IJK(1,0,0)" % (meas_pos.ToXYZString())).data()
    pt = float3.FromXYZString(pt_meas.data_list[0])
    L_bracket_right_face.addPoint(*pt)

  return (L_bracket_top_face, L_bracket_back_face, L_bracket_right_face)

async def probe_fixture_ball_top(client, fixture_home, y_pos_v2):
  """
  Probe the position of the ball on the calibration fixture. This should first be called with fixture_home as APPROX_FIXTURE_BALL_HOME and
  subsequent calls should use the more accurate position calculated from the best fit sphere of those points.
  """
  pts = Feature()

  logger.debug("fixture_home %s %s", fixture_home, type(fixture_home))
  orig = float3(fixture_home) + float3(0,-y_pos_v2, 0)
  tool_orig = orig + TOOL_3_LENGTH*float3(0,1,0)
  FULL_CONTACT_RADIUS = (FIXTURE_BALL_DIA+PROBE_DIA)/2
  contact_radius = FULL_CONTACT_RADIUS
  clearance_radius = contact_radius + 2
  a_angle_probe_contact = math.atan2(contact_radius,TOOL_3_LENGTH)*180/math.pi

  await client.GoTo("Tool.Alignment(0,1,0)").complete()
  await client.GoTo((orig + float3(0,25,0)).ToXYZString()).complete()
  getCurrPosCmd = await client.Get("X(),Y(),Z()").complete()
  currPos = readPointData(getCurrPosCmd.data_list[0])

  await client.SetProp("Tool.PtMeasPar.HeadTouch(1)").complete()
  await client.SetProp("Tool.PtMeasPar.Approach(2)").send()
  await client.SetProp("Tool.PtMeasPar.Search(4)").complete()

  #place tip pos +X from target
  start_pos = orig + float3(clearance_radius,0,0) 
  tool_alignment = np.array((tool_orig - start_pos).normalize())
  await client.AlignTool("%s,%s,%s,0" %(tool_alignment[0],tool_alignment[1],tool_alignment[2])).complete()
  await client.GoTo("%s" % (start_pos.ToXYZString())).complete()

  # probe in -X dir
  contact_pos = orig + float3(contact_radius,0,0) 
  ptMeas = await client.PtMeas("%s,IJK(1,0,0)" % (contact_pos.ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  # move to -Z pos, probe in +Z dir
  start_pos = orig + float3(0,0,-clearance_radius)
  tool_alignment = np.array((tool_orig - start_pos).normalize())
  await client.AlignTool("%s,%s,%s,0" %(tool_alignment[0],tool_alignment[1],tool_alignment[2])).complete()
  await client.GoTo("%s" % (start_pos.ToXYZString())).complete()
  ptMeas = await client.PtMeas("%s,IJK(0,0,-1)" % ((orig + float3(0,0,-contact_radius)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  # move to -X pos, probe in +X dir
  start_pos = orig + float3(-clearance_radius,0,0) 
  tool_alignment = np.array((tool_orig - start_pos).normalize())
  await client.AlignTool("%s,%s,%s,0" %(tool_alignment[0],tool_alignment[1],tool_alignment[2])).complete()
  await client.GoTo("%s" % (start_pos.ToXYZString())).complete()
  ptMeas = await client.PtMeas("%s,IJK(-1,0,0)" % ((orig + float3(-contact_radius,0,0)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  #rise 2mm and probe another 3 points
  orig = orig + float3(0,2,0)
  tool_orig = tool_orig + float3(0,2,0)
  RISE = 2
  contact_radius = math.sqrt(FULL_CONTACT_RADIUS*FULL_CONTACT_RADIUS-RISE*RISE)
  clearance_radius = contact_radius + 2

  start_pos = orig + float3(-clearance_radius,0,0) 
  tool_alignment = np.array((tool_orig - start_pos).normalize())
  # probe in +X dir
  await client.AlignTool("%s,%s,%s,0" %(tool_alignment[0],tool_alignment[1],tool_alignment[2])).complete()
  await client.GoTo("%s" % (start_pos.ToXYZString())).complete()
  ptMeas = await client.PtMeas("%s,IJK(-1,0,0)" % ((orig + float3(-contact_radius,0,0)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  # move to -Z pos, probe in +Z dir
  start_pos = orig + float3(0,0,-clearance_radius)
  tool_alignment = np.array((tool_orig - start_pos).normalize())
  await client.AlignTool("%s,%s,%s,0" %(tool_alignment[0],tool_alignment[1],tool_alignment[2])).complete()
  await client.GoTo("%s" % (start_pos.ToXYZString())).complete()
  ptMeas = await client.PtMeas("%s,IJK(0,0,-1)" % ((orig + float3(0,0,-contact_radius)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  # move to +X pos, probe in -X dir
  start_pos = orig + float3(clearance_radius,0,0) 
  tool_alignment = np.array((tool_orig - start_pos).normalize())
  await client.AlignTool("%s,%s,%s,0" %(tool_alignment[0],tool_alignment[1],tool_alignment[2])).complete()
  await client.GoTo("%s" % (start_pos.ToXYZString())).complete()
  ptMeas = await client.PtMeas("%s,IJK(1,0,0)" % ((orig + float3(contact_radius,0,0)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt) 

  await client.GoTo((orig + float3(0,100,0)).ToXYZString()).complete()

  return pts

async def probe_home_offset_y(client, y_pos_v2, a_pos_v2, b_pos_v2):
  '''
  Probe top and bottom faces of fixture 
  with A at ~90 and B rotated so top/bottom faces are perpendicular to Y-axis
  '''
  feat = Feature()
  await client.GoTo("Tool.Alignment(0,0,1)").complete()

  orig = APPROX_COR + float3(0,-(y_pos_v2),0) 
  start_pos = orig + float3(FIXTURE_SIDE/2-15,FIXTURE_SIDE/2,-4)
  await client.GoTo((start_pos + float3(0,25,25)).ToXYZString()).complete()

  drive_vec = float3(-1,0,0)
  face_norm = float3(0,1,0)
  points = await routines.headline(client,start_pos,drive_vec,B_LINE_LENGTH,face_norm,2,-1,25)
  for pt in points:
    feat.addPoint(*pt)

  await client.GoTo("%s,Tool.A(90),Tool.B(120)" % ((orig + float3(-150,FIXTURE_SIDE/2 + 10,0)).ToXYZString())).complete()
  await client.GoTo("%s,Tool.A(90),Tool.B(120)" % ((orig + float3(-150,-(FIXTURE_SIDE/2 + 10),0)).ToXYZString())).complete()
  meas_pos = orig + float3(-(FIXTURE_SIDE/2 - 15),-(FIXTURE_SIDE/2),0)
  await client.GoTo("%s,Tool.A(90),Tool.B(120)" % ((meas_pos + float3(0,-5,0)).ToXYZString())).complete()
  ptMeas = await client.PtMeas("%s,IJK(0,-1,0)" % (meas_pos.ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  feat.addPoint(*pt)
  
  meas_pos = orig + float3((FIXTURE_SIDE/2 - 15),-(FIXTURE_SIDE/2),0)
  await client.GoTo("%s,Tool.A(90),Tool.B(120)" % ((meas_pos + float3(0,-5,0)).ToXYZString())).complete()
  ptMeas = await client.PtMeas("%s,IJK(0,-1,0)" % (meas_pos.ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  feat.addPoint(*pt)
  
  await client.GoTo("X(%s)" % X_CLEARANCE_PART_CSY).complete()

  return feat

async def probe_top_plane(client, y_pos_v2):
  orig = APPROX_FIXTURE_TOP_PLANE_CENTER + float3(0,-y_pos_v2,0)
  await client.GoTo("Tool.Alignment(0,1,0)").send()
  await client.SetProp("Tool.PtMeasPar.Approach(10)").send()
  await client.SetProp("Tool.PtMeasPar.Search(12)").send()
  await client.SetProp("Tool.PtMeasPar.HeadTouch(0)").send()
  await client.GoTo((orig + float3(0,100,0)).ToXYZString()).ack()
  await client.GoTo((orig + float3(0,10,0)).ToXYZString()).ack()

  fixture_top_face = Feature()

  ptMeas = await client.PtMeas("%s,IJK(0,1,0)" % ((orig).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  fixture_top_face.addPoint(*pt)

  ptMeas = await client.PtMeas("%s,IJK(0,1,0)" % ((orig + float3(-5,0,-20)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  fixture_top_face.addPoint(*pt)

  ptMeas = await client.PtMeas("%s,IJK(0,1,0)" % ((orig + float3(15,0,-20)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  fixture_top_face.addPoint(*pt)

  ptMeas = await client.PtMeas("%s,IJK(0,1,0)" % ((orig + float3(5,0,25)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  fixture_top_face.addPoint(*pt)
  
  ptMeas = await client.PtMeas("%s,IJK(0,1,0)" % ((orig + float3(0,0,45)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  fixture_top_face.addPoint(*pt)

  return fixture_top_face

async def probe_fixture_plane_a90(client, y_pos_v2):
  orig = PROBE_FIXTURE_PLANE_A90_WAYPOINT
  await client.GoTo((orig + float3(0,100,100)).ToXYZString()).complete()
  await client.AlignTool("0.0,0.99,0.14,0").complete()

  await client.GoTo((orig + float3(0,0,5)).ToXYZString()).complete()
  await client.SetProp("Tool.PtMeasPar.HeadTouch(1)").complete()

  fixture_plane_a90 = Feature()

  ptMeas = await client.PtMeas("%s,IJK(0,0,1),Tool.Alignment(0.0,0.99,0.14)" % ((orig + float3(3,-3,0)).ToXYZString())).complete()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  fixture_plane_a90.addPoint(*pt)

  ptMeas = await client.PtMeas("%s,IJK(0,0,1),Tool.Alignment(0.0,0.99,0.14)" % ((orig + float3(-10,-20,0)).ToXYZString())).complete()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  fixture_plane_a90.addPoint(*pt)


  ptMeas = await client.PtMeas("%s,IJK(0,0,1),Tool.Alignment(0.0,0.99,0.14)" % ((orig + float3(-10,10,0)).ToXYZString())).complete()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  fixture_plane_a90.addPoint(*pt)

  return fixture_plane_a90

async def probe_home_offset_x(client, y_pos_v2, a_pos_v2, b_pos_v2):
  '''
  Probe side faces of fixture 
  with A at ~90 and B rotated so side faces are perpendicular to X-axis
  '''
  feat = Feature()
  await client.GoTo("Tool.Alignment(0,0,1)").complete()

  orig = APPROX_COR + float3(0,-(y_pos_v2),0)
  start_pos = orig + float3(-FIXTURE_SIDE/2,FIXTURE_SIDE/2-15,-5)
  await client.GoTo((start_pos + float3(-25,25,25)).ToXYZString()).complete()
  
  drive_vec = float3(0,-1,0)
  face_norm = float3(-1,0,0)
  points = await routines.headline(client,start_pos,drive_vec,B_LINE_LENGTH,face_norm,2,-1,25)
  for pt in points:
    feat.addPoint(*pt)

  await client.GoTo((start_pos + float3(-25,150,25)).ToXYZString()).complete()
  await client.AlignTool("-0.045,0.573,0.818,0").complete()
  await client.GoTo("%s" % ((orig + float3(FIXTURE_SIDE/2 + 5,FIXTURE_SIDE/2+5,0)).ToXYZString())).complete()
  meas_pos = orig + float3(FIXTURE_SIDE/2,FIXTURE_SIDE/2-15,2)
  await client.GoTo("%s" % ((meas_pos + float3(5,0,0)).ToXYZString())).complete()
  ptMeas = await client.PtMeas("%s,IJK(1,0,0)" % (meas_pos.ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  feat.addPoint(*pt)
  meas_pos = orig + float3(FIXTURE_SIDE/2,-(FIXTURE_SIDE/2-15),2)
  await client.GoTo("%s" % ((meas_pos + float3(5,0,0)).ToXYZString())).complete()
  ptMeas = await client.PtMeas("%s,IJK(1,0,0)" % (meas_pos.ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  feat.addPoint(*pt)
  
  await client.GoTo("Y(%s)" % Y_CLEARANCE_PART_CSY).complete()

  return feat

async def prep_probe_fixture_fin(client, y_pos_v2, a_pos_v2):
  orig = ORIGIN_A_START_PROBE_POS
  a_cor = orig + float3(0,-y_pos_v2,0)

  await client.SetProp("Tool.PtMeasPar.Search(5)").ack()
  await client.SetProp("Tool.PtMeasPar.Approach(5)").ack()
  await client.GoTo((a_cor + float3(-100, 150, 0)).ToXYZString()).complete()
  await client.GoTo("Tool.Alignment(-1,0,0)").complete()
  await client.GoTo((a_cor + float3(-200, 0, 0)).ToXYZString()).complete()

async def prep_probe_b_line(client, y_pos_v2, b_pos_v2):
  orig = APPROX_COR
  pos_bcor = orig + float3(0,-y_pos_v2,0)
  fixture_length = FIXTURE_SIDE

  await client.SetProp("Tool.PtMeasPar.Search(15)").ack()
  await client.SetProp("Tool.PtMeasPar.Approach(10)").ack()
  await client.SetProp("Tool.PtMeasPar.Retract(15)").ack()
  await client.SetProp("Tool.PtMeasPar.HeadTouch(1)").complete()
  await client.AlignTool("0,1,0,0").complete()
  vec_bcor_to_startpos45 = float3(0.5*fixture_length-20,0,0-0.5*fixture_length)
  start_pos = vec_bcor_to_startpos45 + pos_bcor 
  await client.GoTo((start_pos + float3(0, 25, 0)).ToXYZString()).complete()

async def prep_probe_fixture_ball(client, y_pos_v2, a_pos_v2):
  await client.SetProp("Tool.PtMeasPar.HeadTouch(1)").complete()
  await client.SetProp("Tool.PtMeasPar.Search(3)").ack()
  await client.SetProp("Tool.PtMeasPar.Approach(2)").ack()
  await client.SetProp("Tool.PtMeasPar.Retract(5)").ack()
  

async def prep_probe_spindle_tip(client, y_pos_v2, a_pos_v2):
  await client.SetProp("Tool.PtMeasPar.HeadTouch(1)").complete()
  await client.SetProp("Tool.PtMeasPar.Approach(2)").send()
  await client.SetProp("Tool.PtMeasPar.Search(12)").send()
  await client.SetProp("Tool.PtMeasPar.Retract(10)").ack()
  await client.AlignTool("0,1,0,0").ack()

async def probe_a_line(client, y_pos_v2, a_pos_v2):
  '''
  Probe points along the shark fin on the calibration fixture. Returns a feature with those points.
  '''
  orig = ORIGIN_A_START_PROBE_POS
  a_cor = orig + float3(0,-y_pos_v2,0)
  vec_a_cor_to_orig_fixture_tip = APPROX_FIN_TIP - ORIGIN_A_START_PROBE_POS
  vec_cor_to_orig_start = vec_a_cor_to_orig_fixture_tip + float3(0,-2,0) # down a bit from tip of fin

  # rotate nominal touch point about x by a_pos_v2
  r = Rotation.from_euler('x', a_pos_v2, degrees=True)
  vectors = r.apply([ np.array(vec_cor_to_orig_start), np.array((0,-1,0)), np.array((0,0,1)) ])
  start_pos = float3(vectors[0]) + ORIGIN_A_START_PROBE_POS
  drive_vec = float3(vectors[1])
  face_norm = float3(vectors[2])

  # Do a head-probe line against 1 face of the probing fixture
  await client.AlignTool("%s,%s,%s,0" % (-1,0,0)).ack()
  await client.GoTo((start_pos + float3(-20, 0, 0) + (face_norm*5)).ToXYZString()).ack()

  points = await routines.headline(client,start_pos,drive_vec,B_LINE_LENGTH,face_norm,3,1,10)
  a_line = Feature()
  for pt in points:
    a_line.addPoint(*pt)

  end_pos = start_pos + drive_vec * B_LINE_LENGTH
  retract_pos = end_pos + face_norm * 20
  await client.GoTo((retract_pos).ToXYZString()).complete()

  return a_line

async def probe_fixture_ball_side(client, fixture_home, y_pos_v2, a_pos_v2):
  '''
  Probe the fixture ball with stylus 'to the side', at any A-axis position
  When at A0, probe stylus is aligned approximately with Z-axis
  '''
  logger.debug("fixture_home %s %s", fixture_home, type(fixture_home))
  pts = Feature()

  FULL_CONTACT_RADIUS = (FIXTURE_BALL_DIA+PROBE_DIA)/2
  contact_radius = FULL_CONTACT_RADIUS
  clearance_radius = contact_radius + 2
  cor = APPROX_COR + float3(0,-y_pos_v2,0)
  cor_to_ball = fixture_home - cor

  r = Rotation.from_euler('x', 25, degrees=True)
  [ x_dir_pre_a_rotation, y_dir_pre_a_rotation, z_dir_pre_a_rotation ] = r.apply([ np.array((1,0,0)), np.array((0,1,0)), np.array((0,0,1)) ])

  r = Rotation.from_euler('x', a_pos_v2, degrees=True)
  vectors = r.apply([ cor_to_ball, x_dir_pre_a_rotation, y_dir_pre_a_rotation, z_dir_pre_a_rotation ])
  vec_cor_to_ball = float3(vectors[0])
  ball_pos = vec_cor_to_ball + cor
  
  x_dir = float3(vectors[1])
  y_dir = float3(vectors[2])
  z_dir = float3(vectors[3])

  vec_tool_align = -1*z_dir
  logger.debug("ball_pos %s, vec_tool_align %s, x_dir %s, y_dir %s, z_dir %s", ball_pos, vec_tool_align, x_dir, y_dir,z_dir)
  
  await client.AlignTool("%s,%s,%s,0" % (vec_tool_align[0],vec_tool_align[1],vec_tool_align[2])).complete()
  approach_pos = 2*vec_cor_to_ball + cor
  await client.GoTo("%s" % (approach_pos.ToXYZString())).complete()

  dist_to_probe_cor = math.sqrt(TOOL_3_LENGTH*TOOL_3_LENGTH - clearance_radius*clearance_radius)
  probe_cor = ball_pos + vec_tool_align * dist_to_probe_cor

  #at A0 this probe is from +X pos, in -X dir
  start_pos = ball_pos + x_dir*clearance_radius
  tool_alignment = np.array((probe_cor - start_pos).normalize())
  tool_align_string = "%s,%s,%s" % (tool_alignment[0],tool_alignment[1],tool_alignment[2])
  # await client.AlignTool("%s,%s,%s,0" % (tool_alignment[0],tool_alignment[1],tool_alignment[2])).ack()
  # await client.GoTo("%s" % (start_pos.ToXYZString())).complete()
  contact_pos = ball_pos + x_dir*contact_radius
  approach_vec = start_pos - contact_pos
  ptMeas = await client.PtMeas("%s,%s,Tool.Alignment(%s)" % (contact_pos.ToXYZString(),approach_vec.ToIJKString(),tool_align_string)).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  #at A0 this probe is from +Y pos, in -Y dir
  start_pos = ball_pos + y_dir*clearance_radius
  tool_alignment = np.array((probe_cor - start_pos).normalize())
  tool_align_string = "%s,%s,%s" % (tool_alignment[0],tool_alignment[1],tool_alignment[2])
  # await client.AlignTool("%s,%s,%s,0" % (tool_alignment[0],tool_alignment[1],tool_alignment[2])).ack()
  # await client.GoTo("%s" % (start_pos.ToXYZString())).complete()
  contact_pos = ball_pos + y_dir*contact_radius
  approach_vec = start_pos - contact_pos
  ptMeas = await client.PtMeas("%s,%s,Tool.Alignment(%s)" % (contact_pos.ToXYZString(),approach_vec.ToIJKString(),tool_align_string)).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  #at A0 this probe is from -X pos, in +X dir
  start_pos = ball_pos - x_dir*clearance_radius
  tool_alignment = np.array((probe_cor - start_pos).normalize())
  tool_align_string = "%s,%s,%s" % (tool_alignment[0],tool_alignment[1],tool_alignment[2])
  # await client.AlignTool("%s,%s,%s,0" % (tool_alignment[0],tool_alignment[1],tool_alignment[2])).ack()
  # await client.GoTo("%s" % (start_pos.ToXYZString())).complete()
  contact_pos = ball_pos - x_dir*contact_radius
  approach_vec = start_pos - contact_pos
  ptMeas = await client.PtMeas("%s,%s,Tool.Alignment(%s)" % (contact_pos.ToXYZString(),approach_vec.ToIJKString(),tool_align_string)).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  RISE = 2
  #move RISE mm in negative z_dir and probe 3 more points
  ball_pos = ball_pos - RISE*z_dir
  #since we are no longer "on an equator" of the ball, reduce contact/clearance radius
  contact_radius = math.sqrt(FULL_CONTACT_RADIUS*FULL_CONTACT_RADIUS-RISE*RISE)
  clearance_radius = contact_radius + 2 
  #also update probe_cor pos
  dist_to_probe_cor = math.sqrt(TOOL_3_LENGTH*TOOL_3_LENGTH - clearance_radius*clearance_radius)
  probe_cor = ball_pos + vec_tool_align * dist_to_probe_cor

  #at A0 this probe is from -X pos, in +X dir
  start_pos = ball_pos - x_dir*clearance_radius
  tool_alignment = np.array((probe_cor - start_pos).normalize())
  tool_align_string = "%s,%s,%s" % (tool_alignment[0],tool_alignment[1],tool_alignment[2])
  # await client.AlignTool("%s,%s,%s,0" % (tool_alignment[0],tool_alignment[1],tool_alignment[2])).ack()
  # await client.GoTo("%s" % (start_pos.ToXYZString())).complete()
  contact_pos = ball_pos - x_dir*contact_radius
  approach_vec = start_pos - contact_pos
  ptMeas = await client.PtMeas("%s,%s,Tool.Alignment(%s)" % (contact_pos.ToXYZString(),approach_vec.ToIJKString(),tool_align_string)).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  #at A0 this probe is from +Y pos, in -Y dir
  start_pos = ball_pos + y_dir*clearance_radius
  tool_alignment = np.array((probe_cor - start_pos).normalize())
  tool_align_string = "%s,%s,%s" % (tool_alignment[0],tool_alignment[1],tool_alignment[2])
  # await client.AlignTool("%s,%s,%s,0" % (tool_alignment[0],tool_alignment[1],tool_alignment[2])).ack()
  # await client.GoTo("%s" % (start_pos.ToXYZString())).complete()
  contact_pos = ball_pos + y_dir*contact_radius
  approach_vec = start_pos - contact_pos
  ptMeas = await client.PtMeas("%s,%s,Tool.Alignment(%s)" % (contact_pos.ToXYZString(),approach_vec.ToIJKString(),tool_align_string)).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  #at A0 this probe is from +X pos, in -X dir
  start_pos = ball_pos + x_dir*clearance_radius
  tool_alignment = np.array((probe_cor - start_pos).normalize())
  tool_align_string = "%s,%s,%s" % (tool_alignment[0],tool_alignment[1],tool_alignment[2])
  # await client.AlignTool("%s,%s,%s,0" % (tool_alignment[0],tool_alignment[1],tool_alignment[2])).ack()
  # await client.GoTo("%s" % (start_pos.ToXYZString())).complete()
  contact_pos = ball_pos + x_dir*contact_radius
  approach_vec = start_pos - contact_pos
  ptMeas = await client.PtMeas("%s,%s,Tool.Alignment(%s)" % (contact_pos.ToXYZString(),approach_vec.ToIJKString(),tool_align_string)).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  retreat_pos = ball_pos + vec_tool_align*50
  await client.GoTo("%s" % (retreat_pos.ToXYZString())).complete()
  return pts

async def probe_b_pos(client, fixture_home, y_pos_v2, b_pos_v2):
  '''
  Probe the fixture sphere at any given B rotation
  '''
  logger.debug("fixture_home %s %s", fixture_home, type(fixture_home))
  pts = Feature()
  
  FULL_CONTACT_RADIUS = (FIXTURE_BALL_DIA+PROBE_DIA)/2
  contact_radius = FULL_CONTACT_RADIUS
  clearance_radius = contact_radius + 2
  cor = APPROX_COR_B + float3(0,-y_pos_v2,0)
  cor_to_ball = fixture_home - cor

  r = Rotation.from_euler('y', b_pos_v2, degrees=True)
  vectors = r.apply([ cor_to_ball, np.array((1,0,0)), np.array((0,1,0)), np.array((0,0,1)) ])
  vec_cor_to_ball = float3(vectors[0])
  ball_pos = vec_cor_to_ball + cor

  x_dir = float3(vectors[1])
  y_dir = float3(vectors[2])
  z_dir = float3(vectors[3])
  vec_tool_align = y_dir
  logger.debug("ball_pos %s, vec_tool_align %s, x_dir %s, y_dir %s, z_dir %s", ball_pos, vec_tool_align, x_dir, y_dir,z_dir)


  dist_to_probe_cor = math.sqrt(TOOL_3_LENGTH*TOOL_3_LENGTH - clearance_radius*clearance_radius)
  probe_cor = ball_pos + vec_tool_align * dist_to_probe_cor

  #at B0 this probe is from +X pos, in -X dir
  start_pos = ball_pos + x_dir*clearance_radius
  tool_alignment = np.array((probe_cor - start_pos).normalize())
  tool_align_string = "%s,%s,%s" % (tool_alignment[0],tool_alignment[1],tool_alignment[2])
  contact_pos = ball_pos + x_dir*contact_radius
  approach_vec = start_pos - contact_pos
  ptMeas = await client.PtMeas("%s,%s,Tool.Alignment(%s)" % (contact_pos.ToXYZString(),approach_vec.ToIJKString(),tool_align_string)).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  #at B0 this probe is from -Z pos, in +Z dir
  start_pos = ball_pos - z_dir*clearance_radius
  tool_alignment = np.array((probe_cor - start_pos).normalize())
  tool_align_string = "%s,%s,%s" % (tool_alignment[0],tool_alignment[1],tool_alignment[2])
  contact_pos = ball_pos - z_dir*contact_radius
  approach_vec = start_pos - contact_pos
  ptMeas = await client.PtMeas("%s,%s,Tool.Alignment(%s)" % (contact_pos.ToXYZString(),approach_vec.ToIJKString(),tool_align_string)).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  #at A0 this probe is from -X pos, in +X dir
  start_pos = ball_pos - x_dir*clearance_radius
  tool_alignment = np.array((probe_cor - start_pos).normalize())
  tool_align_string = "%s,%s,%s" % (tool_alignment[0],tool_alignment[1],tool_alignment[2])
  contact_pos = ball_pos - x_dir*contact_radius
  approach_vec = start_pos - contact_pos
  ptMeas = await client.PtMeas("%s,%s,Tool.Alignment(%s)" % (contact_pos.ToXYZString(),approach_vec.ToIJKString(),tool_align_string)).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  RISE = 2
  #move RISE mm in y_dir and probe 3 more points
  ball_pos = ball_pos + RISE*y_dir
  #since we are no longer "on an equator" of the ball, reduce contact/clearance radius
  contact_radius = math.sqrt(FULL_CONTACT_RADIUS*FULL_CONTACT_RADIUS-RISE*RISE)
  clearance_radius = contact_radius + 2 
  #also update probe_cor pos
  dist_to_probe_cor = math.sqrt(TOOL_3_LENGTH*TOOL_3_LENGTH - clearance_radius*clearance_radius)
  probe_cor = ball_pos + vec_tool_align * dist_to_probe_cor

  #at A0 this probe is from -X pos, in +X dir
  start_pos = ball_pos - x_dir*clearance_radius
  tool_alignment = np.array((probe_cor - start_pos).normalize())
  tool_align_string = "%s,%s,%s" % (tool_alignment[0],tool_alignment[1],tool_alignment[2])
  contact_pos = ball_pos - x_dir*contact_radius
  approach_vec = start_pos - contact_pos
  ptMeas = await client.PtMeas("%s,%s,Tool.Alignment(%s)" % (contact_pos.ToXYZString(),approach_vec.ToIJKString(),tool_align_string)).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  #at A0 this probe is from -Z pos, in +Z dir
  start_pos = ball_pos - z_dir*clearance_radius
  tool_alignment = np.array((probe_cor - start_pos).normalize())
  tool_align_string = "%s,%s,%s" % (tool_alignment[0],tool_alignment[1],tool_alignment[2])
  contact_pos = ball_pos - z_dir*contact_radius
  approach_vec = start_pos - contact_pos
  ptMeas = await client.PtMeas("%s,%s,Tool.Alignment(%s)" % (contact_pos.ToXYZString(),approach_vec.ToIJKString(),tool_align_string)).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  #at A0 this probe is from +X pos, in -X dir
  start_pos = ball_pos + x_dir*clearance_radius
  tool_alignment = np.array((probe_cor - start_pos).normalize())
  tool_align_string = "%s,%s,%s" % (tool_alignment[0],tool_alignment[1],tool_alignment[2])
  # await client.AlignTool("%s,%s,%s,0" % (tool_alignment[0],tool_alignment[1],tool_alignment[2])).ack()
  # await client.GoTo("%s" % (start_pos.ToXYZString())).complete()
  contact_pos = ball_pos + x_dir*contact_radius
  approach_vec = start_pos - contact_pos
  ptMeas = await client.PtMeas("%s,%s,Tool.Alignment(%s)" % (contact_pos.ToXYZString(),approach_vec.ToIJKString(),tool_align_string)).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.addPoint(*pt)

  # retreat_pos = ball_pos + vec_tool_align*50
  # await client.GoTo("%s" % (retreat_pos.ToXYZString())).complete()
  return pts

async def probe_b_line(client, y_pos_v2, b_pos_v2):
  '''
  Do a head-probe line against 1 face of the probing fixture
  The fixture face being probed is on the upper rectangle, opposite from the peak of the vertical fin
  '''
  orig = APPROX_COR
  pos_bcor = orig + float3(0,-y_pos_v2,0)
  logger.debug('pos_bcor %s', pos_bcor)
  vec_cor_to_corner = float3(0.5*FIXTURE_DIAG,0,0)
  vec_corner_to_start = float3(-15,0,-15)
  vec_cor_to_start = vec_cor_to_corner + vec_corner_to_start
  logger.debug('vec_cor_to_start %s', vec_cor_to_start)
  
  # rotate nominal touch point about y by b_pos_v2
  r = Rotation.from_euler('y', b_pos_v2, degrees=True)
  vectors = r.apply([ np.array(vec_cor_to_start), np.array((-1,0,-1)), np.array((1,0,-1)) ])
  start_pos = float3(vectors[0]) + APPROX_COR
  drive_vec = float3(vectors[1])
  face_norm = float3(vectors[2])
  logger.debug("start_pos %s, drive_vec %s, face_norm %s", start_pos, drive_vec, face_norm)
  # await client.GoTo((start_pos + float3(0,25,0)).ToXYZString()).complete()

  points = await routines.headline(client,start_pos, drive_vec, B_LINE_LENGTH, face_norm,3,-1,10)
  b_line = Feature()
  for pt in points:
    b_line.addPoint(*pt)

  end_pos = start_pos + drive_vec * B_LINE_LENGTH
  retract_pos = end_pos + float3(0,25,0)
  await client.GoTo((retract_pos).ToXYZString()).complete()

  return b_line

async def probe_fixture_vertical(client, y_pos_v2):
  '''
  With A90 B45, probe a line on the face closest to the front of the machine.
  Returns a feature with the probed points which can be used for a best fit line after being projected
  to the XY plane. An angle about +Z can then be calculated.
  '''
  y_line = Feature()

  await client.GoTo("Tool.Alignment(0,0,1)").complete()

  orig = APPROX_COR + float3(0,-(y_pos_v2),0)
  start_pos = orig + float3(-FIXTURE_SIDE/2,FIXTURE_SIDE/2-15,-4)
  await client.GoTo((start_pos + float3(-25, 25, 25)).ToXYZString()).complete()

  drive_vec = float3(0,-1,0)
  face_norm = float3(-1,0,0)
  points = await routines.headline(client,start_pos,drive_vec,B_LINE_LENGTH,face_norm,3,-1,25)
  
  for pt in points:
    y_line.addPoint(*pt)

  return y_line

async def probe_fixture_horizontal(client, y_pos_v2):
  '''
  With A90 B45, probe a line on the top most face of the fixture.
  Returns a feature with the probed points which can be used for a best fit line after being projected
  to the XY plane. An angle about +Z can then be calculated.
  '''
  x_line = Feature()

  await client.GoTo("Tool.Alignment(0,0,1)").complete()

  orig = APPROX_COR + float3(0,-(y_pos_v2),0)
  start_pos = orig + float3(FIXTURE_SIDE/2-15,FIXTURE_SIDE/2,-4)
  await client.GoTo((start_pos + float3(0, 25, 25)).ToXYZString()).complete()

  drive_vec = float3(-1,0,0)
  face_norm = float3(0,1,0)

  points = await routines.headline(client,start_pos,drive_vec,B_LINE_LENGTH,face_norm,3,-1,25)
  for pt in points:
    x_line.addPoint(*pt)

  return x_line
