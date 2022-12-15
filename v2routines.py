import logging
from ipp import float3, CmmException, readPointData
import ipp_routines as routines
from metrology import Feature, FeatureSet

logger = logging.getLogger(__name__)

Y_CLEARANCE_PART_CSY = -250
Z_CLEARANCE_PART_CSY = 250
FIXTURE_HEIGHT = 25.2476
FIXTURE_SIDE = 76.2
FIXTURE_DIAG = 107.76
FIXTURE_BALL_DIA = 6.35
FIXTURE_OFFSET_B_ANGLE = 225
PROBE_DIA = 4
TOOL_3_LENGTH = 117.8

B_LINE_LENGTH = 35

APPROX_FIXTURE_BALL_HOME = float3(-94.0, -114.7, -123.4)
APPROX_COR = float3(-61.8, -112.4, -69.7)
APPROX_B_ROT_CENT = float3(-62.33, -111.87, -2.0)
APPROX_A_ROT_CENT = float3(-63.10, -46.0, -6.93)
PROBE_FIXTURE_PLANE_A90_WAYPOINT = float3(-56.5, -93.5, -41.5)
PROBE_FIXTURE_TIP_WAYPOINT = float3(-33.90, -133.0, -83.53)
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
  """
  pts = []
  orig = spindle_home + float3(z_pos_v2, x_pos_v2 - 63.5, 0)
  contact_radius = (ball_dia+PROBE_DIA)/2
  a_angle_probe_contact = math.atan2(contact_radius,TOOL_3_LENGTH)*180/math.pi

  #orient tool down and place tip 25 mm above target
  await client.GoTo("Tool.Alignment(0,0,1,1,0,0)").complete()
  await client.GoTo((orig + float3(0,0,25)).ToXYZString()).complete()
  getCurrPosCmd = await client.Get("X(),Y(),Z()").data()
  currPos = readPointData(getCurrPosCmd.data_list[0])
  #place tip pos +Y from target
  await client.GoTo("Tool.A(%s),Tool.B(90)" % (a_angle_probe_contact+5)).complete()
  await client.GoTo("Z(%s)" % (currPos.z - 25)).complete()

  await client.SetProp("Tool.PtMeasPar.HeadTouch(1)").send()
  await client.SetProp("Tool.PtMeasPar.Search(6)").complete()

  # probe in -Y dir
  ptMeas = await client.PtMeas("%s,IJK(0,1,0)" % ((orig + float3(0,contact_radius,0)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.append(pt)

  # move to -X pos, probe in +X dir
  await client.GoTo("Tool.A(%s),Tool.B(180)" % (a_angle_probe_contact+5)).complete()
  ptMeas = await client.PtMeas("%s,IJK(-1,0,0)" % ((orig + float3(-contact_radius,0,0)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.append(pt)

  # move to -Y pos, probe in +Y dir
  await client.GoTo("Tool.A(%s),Tool.B(270)" % (a_angle_probe_contact+5)).complete()
  ptMeas = await client.PtMeas("%s,IJK(0,-1,0)" % ((orig + float3(0,-contact_radius,0)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.append(pt) 

  #rise 2mm and probe another 3 points
  measPos2 = orig + float3(0,0,2)
  getCurrPosCmd = await client.Get("X(),Y(),Z()").complete()
  currPos = readPointData(getCurrPosCmd.data_list[0])
  # await client.GoTo((currPos + float3(0,0,+2)).ToXYZString()).complete()
  await client.GoTo("Tool.A(%s),Tool.B(270)" % (a_angle_probe_contact+5)).complete()
  await client.GoTo("Z(%s)" % (currPos.z + 2)).complete()

  # probe in +Y dir
  await client.GoTo("Tool.A(%s),Tool.B(270)" % (a_angle_probe_contact+5)).complete()
  ptMeas = await client.PtMeas("%s,IJK(0,-1,0)" % ((measPos2 + float3(0,-contact_radius,0)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.append(pt)

  # move to -X pos, probe in +X dir
  await client.GoTo("Tool.A(%s),Tool.B(180)" % (a_angle_probe_contact+5)).complete()
  ptMeas = await client.PtMeas("%s,IJK(-1,0,0)" % ((measPos2 + float3(-contact_radius,0,0)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.append(pt) 

  # move to +Y pos, probe in -Y dir
  await client.GoTo("Tool.A(%s),Tool.B(90)" % (a_angle_probe_contact+5)).complete()
  ptMeas = await client.PtMeas("%s,IJK(0,1,0)" % ((measPos2 + float3(0,contact_radius,0)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.append(pt) 

  await client.GoTo((orig + float3(0,0,50)).ToXYZString()).complete()

  return Feature(pts)

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
  pts = []

  await client.GoTo("Z(%s)" % Z_CLEARANCE_PART_CSY).ack()
  await client.SetProp("Tool.PtMeasPar.Approach(10)").send()
  await client.SetProp("Tool.PtMeasPar.Search(100)").send()
  await client.SetProp("Tool.PtMeasPar.Retract(-1)").send()
  await client.SetProp("Tool.PtMeasPar.HeadTouch(0)").ack()
  await client.GoTo("Tool.A(0),Tool.B(0)").ack()

  if machine == V2_10:
    z_home = float3(54.6, -50.50, -68.83),
    spindle_side_clearance = 22.5
    ball_dia = 12.69792
  elif machine == V2_50:
    z_home = float3(42.5, -49.25, -68.83),
    spindle_side_clearance = 30
    ball_dia = 6.35
  else:
    raise CmmException("Unknown machine")

  orig = z_home + float3(z_pos_v2,x_pos_v2 - 63.5,0)

  #probe against spindle shaft, approx. 45 mm in the PART_CSY +X direction from typical spindle tip position
  await client.GoTo((orig + float3(45, spindle_side_clearance, 25)).ToXYZString()).ack()
  await client.GoTo((orig + float3(45, spindle_side_clearance, 0)).ToXYZString()).ack()
  pt_meas = await client.PtMeas("%s,IJK(0,1,0)" % ((orig + float3(45, 10, 0)).ToXYZString())).data()
  spindle_shaft_pt_1 = float3.FromXYZString(pt_meas.data_list[0])

  #up and over shaft
  await client.GoTo((orig + float3(45, spindle_side_clearance, 25)).ToXYZString()).ack()
  await client.GoTo((orig + float3(45, -spindle_side_clearance, 25)).ToXYZString()).ack()

  #probe spindle shaft from other side (from PartCsy -Y side, probing in +Y direction)
  await client.GoTo((orig + float3(45, -spindle_side_clearance, 0)).ToXYZString()).ack()
  pt_meas = await client.PtMeas("%s,IJK(0,-1,0)" % ((orig + float3(45, -10, 0)).ToXYZString())).data()
  spindle_shaft_pt_2 = float3.FromXYZString(pt_meas.data_list[0])

  # spindle tip Y position should be at mid point of previous two probes
  spindle_shaft_y = (spindle_shaft_pt_1.y + spindle_shaft_pt_2.y) / 2

  #now probe along the spindle center line until we hit the tip
  await client.GoTo((orig + float3(-50, -30, 0)).ToXYZString()).ack()
  await client.GoTo(float3(orig.x - 50, spindle_shaft_y, orig.z).ToXYZString()).ack()
  meas_pos = float3(orig.x, spindle_shaft_y, orig.z)
  pt_meas = await client.PtMeas("%s,IJK(-1,0,0)" % (meas_pos.ToXYZString())).data()
  tip_pt = float3.FromXYZString(pt_meas.data_list[0])

  pts.append(tip_pt)

  #take additional points on spindle tip sphere
  #from +Y (probe in -Y)
  await client.GoTo((tip_pt + float3(0, 10, 0)).ToXYZString()).ack()
  await client.GoTo((tip_pt + float3(5, 10, 0)).ToXYZString()).ack()

  meas_pos = tip_pt + float3(5, ball_dia/2, 0)
  pt_meas = await client.PtMeas("%s,IJK(0,1,0)" % (meas_pos.ToXYZString())).data()
  pt = float3.FromXYZString(pt_meas.data_list[0])
  pts.append(pt)

  #from +Z (probe in -Z)
  await client.GoTo((tip_pt + float3(5, 10, 10)).ToXYZString()).ack()
  await client.GoTo((tip_pt + float3(5, 0, 10)).ToXYZString()).ack()
  meas_pos = tip_pt + float3(5, 0, ball_dia/2)
  pt_meas = await client.PtMeas("%s,IJK(0,0,1)" % (meas_pos.ToXYZString())).data()
  pt = float3.FromXYZString(pt_meas.data_list[0])
  pts.append(pt)

  #from -Y (probe in +Y)
  await client.GoTo((tip_pt + float3(5, -10, 5)).ToXYZString()).ack()
  await client.GoTo((tip_pt + float3(5, -10, 0)).ToXYZString()).ack()
  meas_pos = tip_pt + float3(5, -ball_dia/2, 0)
  pt_meas = await client.PtMeas("%s,IJK(0,-1,0)" % (meas_pos.ToXYZString())).data()
  pt = float3.FromXYZString(pt_meas.data_list[0])
  pts.append(pt)

  return Feature(pts)

async def probe_machine_pos(client):
  '''
  Routine for gathering point data about the back right corner of the L-bracket
  for precisely locating the Part CSY.

  Returns a 3-tuple of features: (L bracket top face, line on L bracket back face and line on L bracket right face)
  The first list can be used to calculate a best fit plane for the top, then two best fit lines for the back and right face after
  projecting them up to the top face.
  '''
  await client.GoTo("Z(%s)" % Z_CLEARANCE_PART_CSY).ack()
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

  measPos = float3(0,2,0)
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

  # L-bracket back face
  L_bracket_back_face = Feature()
  approach_pos = float3(-35, 30, 10)
  await client.GoTo(approach_pos.ToXYZString()).ack()
  approach_pos = approach_pos + float3(0, 0, -20)
  await client.GoTo(approach_pos.ToXYZString()).ack()
  for i in range(3):
    meas_pos = approach_pos + float3(i * 15, -15, 0)
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

async def probe_fixture_ball_pos(client, fixture_home, y_pos_v2):
  """
  Probe the position of the ball on the calibration fixture. This should first be called with fixture_home as APPROX_FIXTURE_BALL_HOME and
  subsequent calls should use the more accurate position calculated from the best fit sphere of those points.
  """
  pts = []

  orig = fixture_home + float3(0,0,-(y_pos_v2-63.5))
  contact_radius = (FIXTURE_BALL_DIA+PROBE_DIA)/2
  a_angle_probe_contact = math.atan2(contact_radius,TOOL_3_LENGTH)*180/math.pi

  await client.GoTo("Tool.Alignment(0,0,1,1,0,0)").complete()
  await client.GoTo((orig + float3(0,0,25)).ToXYZString()).complete()
  getCurrPosCmd = await client.Get("X(),Y(),Z()").complete()
  currPos = readPointData(getCurrPosCmd.data_list[0])

  #place tip pos +Y from target
  await client.GoTo("Tool.A(%s),Tool.B(90)" % (a_angle_probe_contact+5)).complete()
  await client.GoTo("Z(%s)" % (currPos.z - 25)).complete()
  await client.SetProp("Tool.PtMeasPar.HeadTouch(1)").complete()
  await client.SetProp("Tool.PtMeasPar.Search(6)").complete()

  # probe in -Y dir
  ptMeas = await client.PtMeas("%s,IJK(0,1,0)" % ((orig + float3(0,contact_radius,0)).ToXYZString())).complete()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.append(pt)

  # move to -X pos, probe in +X dir
  await client.GoTo("Tool.A(%s),Tool.B(180)" % (a_angle_probe_contact+5)).complete()
  ptMeas = await client.PtMeas("%s,IJK(-1,0,0)" % ((orig + float3(-contact_radius,0,0)).ToXYZString())).complete()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.append(pt)

  # move to -Y pos, probe in +Y dir
  await client.GoTo("Tool.A(%s),Tool.B(270)" % (a_angle_probe_contact+5)).complete()
  ptMeas = await client.PtMeas("%s,IJK(0,-1,0)" % ((orig + float3(0,-contact_radius,0)).ToXYZString())).complete()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.append(pt) 

  #rise 2mm and probe another 3 points
  measPos2 = orig + float3(0,0,2)
  getCurrPosCmd = await client.Get("X(),Y(),Z()").complete()
  currPos = readPointData(getCurrPosCmd.data_list[0])
  await client.GoTo("Tool.A(%s),Tool.B(270)" % (a_angle_probe_contact+5)).complete()
  await client.GoTo("Z(%s)" % (currPos.z + 2)).complete()

  # probe in +Y dir
  await client.GoTo("Tool.A(%s),Tool.B(270)" % (a_angle_probe_contact+5)).complete()
  ptMeas = await client.PtMeas("%s,IJK(0,-1,0)" % ((measPos2 + float3(0,-contact_radius,0)).ToXYZString())).complete()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.append(pt)

  # move to -X pos, probe in +X dir
  await client.GoTo("Tool.A(%s),Tool.B(180)" % (a_angle_probe_contact+5)).complete()
  ptMeas = await client.PtMeas("%s,IJK(-1,0,0)" % ((measPos2 + float3(-contact_radius,0,0)).ToXYZString())).complete()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.append(pt) 

  # move to +Y pos, probe in -Y dir
  await client.GoTo("Tool.A(%s),Tool.B(90)" % (a_angle_probe_contact+5)).complete()
  ptMeas = await client.PtMeas("%s,IJK(0,1,0)" % ((measPos2 + float3(0,contact_radius,0)).ToXYZString())).complete()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.append(pt) 

  await client.GoTo((orig + float3(0,0,50)).ToXYZString()).complete()

  return Feature(pts)

async def probe_home_offset_y(client, y_pos_v2, a_pos_v2, b_pos_v2):
  '''
  Probe top and bottom faces of fixture 
  with A at ~90 and B rotated so top/bottom faces are perpendicular to Y-axis
  '''
  pts = []
  await client.GoTo("Tool.Alignment(1,0,0)").complete()

  orig = APPROX_COR + float3(0,0,-(y_pos_v2)) 
  start_pos = orig + float3(-5,FIXTURE_SIDE/2-15,FIXTURE_SIDE/2)
  await client.GoTo((start_pos + float3(25,0,25)).ToXYZString()).complete()

  drive_vec = float3(0,-1,0)
  face_norm = float3(0,0,1)
  points = await routines.headprobe_line_xz(client,start_pos,drive_vec,B_LINE_LENGTH,face_norm,2,-1,1, 75)
  for pt in points:
    feat.addPoint(*pt)

  await client.GoTo("%s,Tool.A(90),Tool.B(120)" % ((orig + float3(0,-150,FIXTURE_SIDE/2 + 10)).ToXYZString())).complete()
  await client.GoTo("%s,Tool.A(90),Tool.B(120)" % ((orig + float3(0,-150,-(FIXTURE_SIDE/2 + 10))).ToXYZString())).complete()
  meas_pos = orig + float3(0,-(FIXTURE_SIDE/2 - 15),-(FIXTURE_SIDE/2))
  await client.GoTo("%s,Tool.A(90),Tool.B(120)" % ((meas_pos + float3(0,0,-5)).ToXYZString())).complete()
  ptMeas = await client.PtMeas("%s,IJK(0,0,-1)" % (meas_pos.ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.append(pt)
  
  meas_pos = orig + float3(0,(FIXTURE_SIDE/2 - 15),-(FIXTURE_SIDE/2))
  await client.GoTo("%s,Tool.A(90),Tool.B(120)" % ((meas_pos + float3(0,0,-5)).ToXYZString())).complete()
  ptMeas = await client.PtMeas("%s,IJK(0,0,-1)" % (meas_pos.ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  pts.append(pt)
  
  await cmm.GoTo("Y(%s)" % Y_CLEARANCE_PART_CSY).complete()

  return Feature(pts)

async def probe_top_plane(client, y_pos_v2):
  orig = APPROX_B_ROT_CENT + float3(0,0,(-y_pos_v2 - 63.5))
  await client.GoTo("Tool.Alignment(0,0,1)").send()
  await client.SetProp("Tool.PtMeasPar.HeadTouch(0)").send()
  await client.GoTo((orig + float3(0,0,100)).ToXYZString()).ack()
  await client.GoTo((orig + float3(0,0,10)).ToXYZString()).ack()

  fixture_top_face = Feature()

  ptMeas = await client.PtMeas("%s,IJK(0,0,1)" % ((orig).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  fixture_top_face.addPoint(*pt)

  ptMeas = await client.PtMeas("%s,IJK(0,0,1)" % ((orig + float3(-25,-5,0)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  fixture_top_face.addPoint(*pt)

  ptMeas = await client.PtMeas("%s,IJK(0,0,1)" % ((orig + float3(-25,5,0)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  fixture_top_face.addPoint(*pt)

  ptMeas = await client.PtMeas("%s,IJK(0,0,1)" % ((orig + float3(25,5,0)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  fixture_top_face.addPoint(*pt)
  
  ptMeas = await client.PtMeas("%s,IJK(0,0,1)" % ((orig + float3(25,-5,0)).ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  fixture_top_face.addPoint(*pt)

  return fixture_top_face

async def probe_fixture_plane_a90(client, y_pos_v2):
  orig = PROBE_FIXTURE_PLANE_A90_WAYPOINT
  await client.GoTo((orig + float3(100,0,100)).ToXYZString()).complete()
  await client.GoTo("Tool.A(8),Tool.B(180)").complete()

  await client.GoTo((orig + float3(5,0,0)).ToXYZString()).complete()
  await client.SetProp("Tool.PtMeasPar.HeadTouch(1)").complete()

  fixture_plane_a90 = Feature()

  ptMeas = await client.PtMeas("%s,IJK(1,0,0)" % ((orig).ToXYZString())).complete()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  fixture_plane_a90.addPoint(*pt)

  ptMeas = await client.PtMeas("%s,IJK(1,0,0)" % ((orig + float3(0,-3,+3)).ToXYZString())).complete()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  fixture_plane_a90.addPoint(*pt)

  await client.GoTo("%s,Tool.A(8),Tool.B(180)" % ((orig + float3(5,0,-3)).ToXYZString())).complete()
  ptMeas = await client.PtMeas("%s,IJK(1,0,0)" % ((orig + float3(0,0,-3)).ToXYZString())).complete()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  fixture_plane_a90.addPoint(*pt)

  return fixture_plane_a90

async def probe_home_offset_x(client, y_pos_v2, a_pos_v2, b_pos_v2):
  '''
  Probe side faces of fixture 
  with A at ~90 and B rotated so side faces are perpendicular to X-axis
  '''
  feat = Feature()
  await client.GoTo("Tool.Alignment(1,0,0)").complete()

  orig = APPROX_COR
  start_pos = orig + float3(-5,-FIXTURE_SIDE/2,FIXTURE_SIDE/2-15)
  await client.GoTo((start_pos + float3(25,-25,25)).ToXYZString()).complete()
  
  drive_vec = float3(0,0,-1)
  face_norm = float3(0,-1,0)
  points = await routines.headprobe_line_xz(client,start_pos,drive_vec,B_LINE_LENGTH,face_norm,2,-1,1)
  for pt in points:
    feat.addPoint(*pt)

  await client.GoTo((start_pos + float3(25,-25,150)).ToXYZString()).complete()
  await client.GoTo("Tool.A(55),Tool.B(177)").complete()
  await client.GoTo("%s,Tool.A(55),Tool.B(177)" % ((orig + float3(0,FIXTURE_SIDE/2 + 5,FIXTURE_SIDE/2+5)).ToXYZString())).complete()
  meas_pos = orig + float3(2,FIXTURE_SIDE/2,FIXTURE_SIDE/2-15)
  await client.GoTo("%s,Tool.A(55),Tool.B(177)" % ((meas_pos + float3(0,5,0)).ToXYZString())).complete()
  ptMeas = await client.PtMeas("%s,IJK(0,1,0)" % (meas_pos.ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  feat.addPoint(*pt)
  meas_pos = orig + float3(2,FIXTURE_SIDE/2,-(FIXTURE_SIDE/2-15))
  await client.GoTo("%s,Tool.A(55),Tool.B(177)" % ((meas_pos + float3(0,5,0)).ToXYZString())).complete()
  ptMeas = await client.PtMeas("%s,IJK(0,1,0)" % (meas_pos.ToXYZString())).data()
  pt = float3.FromXYZString(ptMeas.data_list[0])
  feat.addPoint(*pt)
  
  await client.GoTo("Z(%s)" % Z_CLEARANCE_PART_CSY).complete()

  return feat

async def initial_probe_a(client, y_pos_v2, a_pos_v2):
  orig = APPROX_A_ROT_CENT
  a_cor = orig + float3(0,0,(-y_pos_v2 - 63.5))

  await client.SetProp("Tool.PtMeasPar.Search(12)").ack()
  await client.GoTo((a_cor + float3(0, -100, 150)).ToXYZString()).complete()
  await client.GoTo("Tool.Alignment(0,-1,0)").complete()
  await client.GoTo((a_cor + float3(0, -200, 0)).ToXYZString()).complete()

async def probe_a(client, y_pos_v2, a_pos_v2):
  '''
  Probe points along the shark fin on the calibration fixture. Returns a feature with those points.
  '''
  orig = APPROX_A_ROT_CENT
  a_cor = orig + float3(0,0,(-y_pos_v2 - 63.5))
  vec_a_cor_to_orig_fixture_tip = (PROBE_FIXTURE_TIP_WAYPOINT+float3(0,0,(126.5))) - APPROX_A_ROT_CENT
  fixture_offset_angle = 0
  fixture_length = 50.8
  vec_cor_to_orig_start = vec_a_cor_to_orig_fixture_tip + float3(0,0,-2) # down a bit away from edge
  dist_cor_to_start = math.sqrt(vec_cor_to_orig_start.x*vec_cor_to_orig_start.x + vec_cor_to_orig_start.z*vec_cor_to_orig_start.z) + 2
  angle_offset_start = math.atan2(vec_cor_to_orig_start.z,vec_cor_to_orig_start.x)*180/math.pi

  # Do a head-probe line against 1 face of the probing fixture
  a_line = Feature()

  total_angle =  fixture_offset_angle - a_pos_v2
  angle_start_pos = total_angle + angle_offset_start
  logger.debug('vec_cor_to_orig_start %s' % vec_cor_to_orig_start)
  logger.debug('angle_offset_start %s' % angle_offset_start)
  logger.debug('start origin Rad %s' % dist_cor_to_start)
  logger.debug('start pos angle %s' % angle_start_pos)
  
  start_pos = a_cor + float3(dist_cor_to_start * math.cos(angle_start_pos*math.pi/180), vec_a_cor_to_orig_fixture_tip.y,dist_cor_to_start * math.sin(angle_start_pos*math.pi/180))
  logger.debug('start pos %s' % start_pos)
  
  drive_angle = total_angle - 90
  drive_vec = float3(math.cos(drive_angle*math.pi/180), 0, math.sin(drive_angle*math.pi/180))
  face_norm = float3(-drive_vec.z, 0, drive_vec.x)
  logger.debug("drive_vec %s" % drive_vec)
  points = await routines.headprobe_line_yz(client,start_pos,drive_vec,B_LINE_LENGTH,face_norm,3,1)
  for pt in points:
    a_line.addPoint(*pt)

  end_pos = start_pos + drive_vec * B_LINE_LENGTH
  retract_pos = end_pos + face_norm * 20

  await client.GoTo((retract_pos).ToXYZString()).ack()

  return a_line


async def initial_probe_b(client, y_pos_v2, a_pos_v2):
  orig = APPROX_B_ROT_CENT
  pos_bcor = orig + float3(0,0,(-y_pos_v2 - 63.5))
  fixtureOffsetAngle = FIXTURE_OFFSET_B_ANGLE
  fixture_length = FIXTURE_SIDE
  #calculate some offsets 
  #we'll use the nominal b-45 start-pos for reference
  #(at b-45, the fixture face is approx. parallel to PartCsy Y axis)
  vec_bcor_to_startpos45 = float3(-0.5*fixture_length,0.5*fixture_length-20,0)
  dist_bcor_to_startpos45 = math.sqrt(vec_bcor_to_startpos45.x*vec_bcor_to_startpos45.x + vec_bcor_to_startpos45.y*vec_bcor_to_startpos45.y)
  ang_bcor_to_startpos45 = math.atan2(vec_bcor_to_startpos45.y,vec_bcor_to_startpos45.x)*180/math.pi

  ang_bcor_to_startpos = (b_pos_v2 - 45) + ang_bcor_to_startpos45
  logger.debug('ang_bcor_to_startpos %s' % ang_bcor_to_startpos)
  ang_fixture_face = b_pos_v2 + FIXTURE_OFFSET_B_ANGLE
  logger.debug('ang_fixture_face %s' % ang_fixture_face)

  fixture_face_angle = b_pos_v2 + FIXTURE_OFFSET_B_ANGLE
  totAngle = b_pos_v2 + fixtureOffsetAngle
  # start_posAngle = totAngle + start_posOffsetAngle
  # print('start_posOriginVec %s' % start_posOriginVec)
  # print('start_posOffsetAngle %s' % start_posOffsetAngle)
  # print('start origin Rad %s' % start_posOriginRad)
  # print('start pos angle %s' % start_posAngle)
  start_pos = pos_bcor + float3(dist_bcor_to_startpos45 * math.cos(ang_bcor_to_startpos*math.pi/180), dist_bcor_to_startpos45 * math.sin(ang_bcor_to_startpos*math.pi/180),-6)

  await client.SetProp("Tool.PtMeasPar.Search(15)").ack()
  await client.SetProp("Tool.PtMeasPar.HeadTouch(1)").complete()
  await client.GoTo("Tool.A(0)").complete()
  await client.GoTo((start_pos + float3(0, 0, 25)).ToXYZString()).complete()

async def probe_b_pos(self, y_pos_v2, b_pos_v2):
  a_clearance = 2
  '''
  Probe the fixture sphere at any given B rotation
  '''
  try:
    feat_name = "probe_b_%+.6f" % b_pos_v2
    self.b_calib_probes.append(feat_name)
    feature = self.add_feature(feat_name, Stages.CHARACTERIZE_B)

    vec_cor_to_fixture_sphere = self.fitted_features[FEAT_FIXTURE_SPHERE]['pos'] - waypoints['cor']
    logger.debug('vec_cor_to_fixture_sphere %s' % (vec_cor_to_fixture_sphere,))
    theta = np.deg2rad(b_pos_v2)
    rot = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    vec_cor_to_fixture_sphere = np.dot(rot,np.array([vec_cor_to_fixture_sphere[0],vec_cor_to_fixture_sphere[1]]))
    logger.debug('vec_cor_to_fixture_sphere %s' % (vec_cor_to_fixture_sphere,))
    logger.debug('cor %s' % (waypoints['cor'],))
    orig = waypoints['cor'] + float3(vec_cor_to_fixture_sphere[0],vec_cor_to_fixture_sphere[1],0) + float3(0,0,-(y_pos_v2))
    #+10mm on Z because otherwise we run into fixture top plane... but the waypoint seems to be correct for other purposes... hmmm
    #-3.5mm on Y, current waypoint off center for machine 1702
    #+1mm more on Z to avoid collision with fin when B around ~245 deg
    orig = orig + float3(0,-3.5,10)
    logger.debug('orig %s' % (orig,))

    contact_radius = (FIXTURE_BALL_DIA+PROBE_DIA)/2
    a_angle_probe_contact = math.atan2(contact_radius,TOOL_3_LENGTH)*180/math.pi

    await self.client.GoTo("Tool.Alignment(0,0,1,1,0,0)").complete()
    await self.client.GoTo((orig + float3(0,0,25)).ToXYZString()).complete()
    getCurrPosCmd = await self.client.Get("X(),Y(),Z()").data()
    currPos = readPointData(getCurrPosCmd.data_list[0])

    #place tip pos +Y from target
    await self.client.GoTo("Tool.A(%s),Tool.B(%s)" % (a_angle_probe_contact+a_clearance, 90+b_pos_v2)).complete()
    await self.client.GoTo("Z(%s)" % (currPos.z - 25)).complete()
    await self.client.SetProp("Tool.PtMeasPar.HeadTouch(1)").send()
    await self.client.SetProp("Tool.PtMeasPar.Search(6)").complete()

    # probe in -Y dir
    meas_vec = np.dot(rot,np.array([0,1]))
    contact_pos = np.dot(rot,np.array([0,contact_radius]))
    contact_pos = float3(contact_pos[0],contact_pos[1],0)
    ptMeas = await self.client.PtMeas("%s,IJK(%s,%s,0)" % ((orig + contact_pos).ToXYZString(), str(meas_vec[0]), str(meas_vec[1]))).data()
    pt = float3.FromXYZString(ptMeas.data_list[0])
    feature.addPoint(*pt)

    # move to -X pos, probe in +X dir
    await self.client.GoTo("Tool.A(%s),Tool.B(%s)" % (a_angle_probe_contact+a_clearance, 180+b_pos_v2)).complete()
    meas_vec = np.dot(rot,np.array([-1,0]))
    contact_pos = np.dot(rot,np.array([-contact_radius,0]))
    contact_pos = float3(contact_pos[0],contact_pos[1],0)
    ptMeas = await self.client.PtMeas("%s,IJK(%s,%s,0)" % ((orig + contact_pos).ToXYZString(),str(meas_vec[0]),str(meas_vec[1]))).data()
    pt = float3.FromXYZString(ptMeas.data_list[0])
    feature.addPoint(*pt)

    # move to -Y pos, probe in +Y dir
    await self.client.GoTo("Tool.A(%s),Tool.B(%s)" % (a_angle_probe_contact+a_clearance, 270+b_pos_v2)).complete()
    meas_vec = np.dot(rot,np.array([0,-1]))
    contact_pos = np.dot(rot,np.array([0,-contact_radius]))
    contact_pos = float3(contact_pos[0],contact_pos[1],0)
    ptMeas = await self.client.PtMeas("%s,IJK(%s,%s,0)" % ((orig + contact_pos).ToXYZString(),str(meas_vec[0]),str(meas_vec[1]))).data()
    pt = float3.FromXYZString(ptMeas.data_list[0])
    feature.addPoint(*pt) 

    #rise 2mm and probe another 3 points
    measPos2 = orig + float3(0,0,2)
    getCurrPosCmd = await self.client.Get("X(),Y(),Z()").complete()
    currPos = readPointData(getCurrPosCmd.data_list[0])
    # await self.client.GoTo((currPos + float3(0,0,+2)).ToXYZString()).complete()
    await self.client.GoTo("Tool.A(%s),Tool.B(%s)" % (a_angle_probe_contact+a_clearance, 270+b_pos_v2)).complete()
    await self.client.GoTo("Z(%s)" % (currPos.z + 2)).complete()

    # probe in +Y dir
    await self.client.GoTo("Tool.A(%s),Tool.B(%s)" % (a_angle_probe_contact+a_clearance, 270+b_pos_v2)).complete()
    contact_pos = np.dot(rot,np.array([0,-contact_radius+.5]))
    contact_pos = float3(contact_pos[0],contact_pos[1],0)
    ptMeas = await self.client.PtMeas("%s,IJK(%s,%s,0)" % ((measPos2 + contact_pos).ToXYZString(),str(meas_vec[0]),str(meas_vec[1]))).data()
    pt = float3.FromXYZString(ptMeas.data_list[0])
    feature.addPoint(*pt)

    # move to -X pos, probe in +X dir
    await self.client.GoTo("Tool.A(%s),Tool.B(%s)" % (a_angle_probe_contact+a_clearance, 180+b_pos_v2)).complete()
    meas_vec = np.dot(rot,np.array([-1,0]))
    contact_pos = np.dot(rot,np.array([-contact_radius+.5,0]))
    contact_pos = float3(contact_pos[0],contact_pos[1],0)
    ptMeas = await self.client.PtMeas("%s,IJK(%s,%s,0)" % ((measPos2 + contact_pos).ToXYZString(),str(meas_vec[0]),str(meas_vec[1]))).data()
    pt = float3.FromXYZString(ptMeas.data_list[0])
    feature.addPoint(*pt) 

    # move to +Y pos, probe in -Y dir
    await self.client.GoTo("Tool.A(%s),Tool.B(%s)" % (a_angle_probe_contact+a_clearance, 90+b_pos_v2)).complete()
    meas_vec = np.dot(rot,np.array([0,1]))
    contact_pos = np.dot(rot,np.array([0,contact_radius-.5]))
    contact_pos = float3(contact_pos[0],contact_pos[1],0)
    ptMeas = await self.client.PtMeas("%s,IJK(%s,%s,0)" % ((measPos2 + contact_pos).ToXYZString(),str(meas_vec[0]),str(meas_vec[1]))).data()
    pt = float3.FromXYZString(ptMeas.data_list[0])
    feature.addPoint(*pt) 

    await self.client.GoTo((orig + float3(0,0,50)).ToXYZString()).complete()
    return True
  except Exception as ex:
    logger.error("probe_b_pos exception %s" % str(ex))
    raise ex

async def probe_b(client, y_pos_v2, b_pos_v2):
  '''
  Do a head-probe line against 1 face of the probing fixture
  The fixture face being probed is on the upper rectangle, opposite from the peak of the vertical fin
  '''
  orig = APPROX_B_ROT_CENT
  pos_bcor = orig + float3(0,0,(-y_pos_v2 - 63.5))
  fixtureOffsetAngle = FIXTURE_OFFSET_B_ANGLE
  fixture_length = FIXTURE_SIDE
  #calculate some offsets 
  #we'll use the nominal b-45 start-pos for reference
  #(at b-45, the fixture face is approx. parallel to PartCsy Y axis)
  vec_bcor_to_startpos45 = float3(-0.5*fixture_length,0.5*fixture_length-20,0)
  dist_bcor_to_startpos45 = math.sqrt(vec_bcor_to_startpos45.x*vec_bcor_to_startpos45.x + vec_bcor_to_startpos45.y*vec_bcor_to_startpos45.y)
  ang_bcor_to_startpos45 = math.atan2(vec_bcor_to_startpos45.y,vec_bcor_to_startpos45.x)*180/math.pi
  b_line = Feature()

  ang_bcor_to_startpos = (b_pos_v2 - 45) + ang_bcor_to_startpos45
  logger.debug('ang_bcor_to_startpos %s' % ang_bcor_to_startpos)
  ang_fixture_face = b_pos_v2 + FIXTURE_OFFSET_B_ANGLE
  logger.debug('ang_fixture_face %s' % ang_fixture_face)

  fixture_face_angle = b_pos_v2 + FIXTURE_OFFSET_B_ANGLE
  totAngle = b_pos_v2 + fixtureOffsetAngle
  # start_posAngle = totAngle + start_posOffsetAngle
  # print('start_posOriginVec %s' % start_posOriginVec)
  # print('start_posOffsetAngle %s' % start_posOffsetAngle)
  # print('start origin Rad %s' % start_posOriginRad)
  # print('start pos angle %s' % start_posAngle)
  start_pos = pos_bcor + float3(dist_bcor_to_startpos45 * math.cos(ang_bcor_to_startpos*math.pi/180), dist_bcor_to_startpos45 * math.sin(ang_bcor_to_startpos*math.pi/180),-6)
  # start_pos = orig + float3(start_posOriginRad * math.cos(start_posAngle*math.pi/180), start_posOriginRad * math.sin(start_posAngle*math.pi/180),-6)
  await client.SetProp("Tool.PtMeasPar.Search(15)").ack()

  logger.debug('start pos %s' % start_pos)

  lineAngle = totAngle + 90
  lineVec = float3(math.cos(ang_fixture_face*math.pi/180), math.sin(ang_fixture_face*math.pi/180),0)
  logger.debug("lineVec %s" % lineVec)

  points = await routines.headprobe_line(client,start_pos, lineVec, 35, 15, 3, 10, -1, 5)
  for pt in points:
    b_line.addPoint(*pt)

  return b_line

async def probe_fixture_vertical(client, y_pos_v2):
  '''
  With A90 B45, probe a line on the face closest to the front of the machine.
  Returns a feature with the probed points which can be used for a best fit line after being projected
  to the XY plane. An angle about +Z can then be calculated.
  '''

  y_line = Feature()
  await client.GoTo("Tool.Alignment(1,0,0)").complete()
  orig = APPROX_COR + float3(0,0,-(y_pos_v2)) 

  start_pos = orig + float3(-5,FIXTURE_SIDE/2-15,FIXTURE_SIDE/2)
  await client.GoTo((start_pos + float3(25, 0, 25)).ToXYZString()).complete()

  drive_vec = float3(0,-1,0)
  face_norm = float3(0,0,1)
  points = await routines.headprobe_line_xz(client,start_pos,drive_vec,B_LINE_LENGTH,face_norm,2,-1,1, 75)

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

  await client.GoTo("Tool.Alignment(1,0,0)").complete()

  orig = APPROX_COR
  start_pos = orig + float3(-5,-FIXTURE_SIDE/2,FIXTURE_SIDE/2-15)
  await client.GoTo((start_pos + float3(25, -25, 25)).ToXYZString()).complete()

  drive_vec = float3(0,0,-1)
  face_norm = float3(0,-1,0)

  points = await routines.headprobe_line_xz(client,start_pos,drive_vec,B_LINE_LENGTH,face_norm,2,-1,1)
  for pt in points:
    x_line.addPoint(*pt)

  return x_line