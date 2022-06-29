'''
Prototype of V2 calibration routine
'''
import sys
from enum import Enum, auto
import os

import asyncio
from tornado.ioloop import IOLoop
from dataclasses import dataclass
import math

#import path to in-house modules depends on the platform
import platform
this_platform = platform.platform()
if 'x86' in this_platform:
  print('x86 os')
  sys.path.append("/Users/jakedanczyk/source/ippclient")
  from ipp import Client, TransactionCallbacks, float3, CmmException, readPointData
  import ipp_routines as routines

  sys.path.append("/Users/jakedanczyk/source/nfs/pocketnc/Settings")
  import metrology
elif 'arm' in this_platform:
  print('arm os')
  sys.path.append("/home/pocketnc/ippclient")
  from ipp import Client, TransactionCallbacks, float3, CmmException, readPointData
  import ipp_routines as routines

  sys.path.append("/opt/pocketnc/Settings")
  import metrology


HOST = "10.0.0.1"
PORT = 1294

import numpy as np
def find_line_intersect(p1,v1,p2,v2):
  print(v1)
  print(v2)
  print(p1)
  print(p2)
  v1 = np.array(v1[0:2])
  v2 = np.array(v2[0:2])
  p1 = np.array(p1[0:2])
  p2 = np.array(p2[0:2])

  v12 = p1 - p2
  v1_perp = np.array([-v1[1], v1[0]])
  denom = np.dot(v1_perp, v2)
  num = np.dot(v1_perp, v12)
  return (num / denom.astype(float))*v2 + p2

'''
Following two methods (unit_vector and angle_between) adapted from:
https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python
'''
def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    print(v1)
    print(v2)
    """ Returns the angle in degrees between vectors 'v1' and 'v2'::
            >>> angle_between((1, 0, 0), (0, 1, 0))
            180/pi * 1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            180/pi * 0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            180/pi * 3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    print(v1_u)
    print(v2_u)
    return (180/math.pi)*np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def angle_between_ccw_2d(v1,v2):
  print(v1)
  print(v2)
  dot = v1[0] * v2[0] + v1[1] * v2[1]
  det = v1[0] * v2[1] - v1[1] * v2[0]
  return math.atan2(det,dot) * 180/math.pi


class Stages(Enum):
  CONNECT_TO_CMM = auto()
  SETUP_CMM = auto()
  GO_TO_CLEARANCE_Z = auto()
  GO_TO_CLEARANCE_Y = auto()
  VERIFY_MACHINE_POS = auto()
  FIND_B_COR = auto()
  FIND_TOP_PLANE = auto()
  PROBE_X = auto()
  PROBE_Y = auto()
  PROBE_Z = auto()
  PROBE_A = auto()
  PROBE_B = auto()
  WRITE_RESULTS = auto()
  END = auto()
  TEST_HEAD = auto()

  @classmethod
  def has_name(cls, name):
      return name in cls._member_names_ 


STAGE_PREREQS = {
  Stages.SETUP_CMM: [Stages.CONNECT_TO_CMM],
  Stages.GO_TO_CLEARANCE_Z: [Stages.CONNECT_TO_CMM],
  Stages.GO_TO_CLEARANCE_Y: [Stages.CONNECT_TO_CMM],
  Stages.VERIFY_MACHINE_POS: [Stages.SETUP_CMM],
  Stages.PROBE_A: [Stages.SETUP_CMM],#[Stages.VERIFY_MACHINE_POS],
  Stages.PROBE_B: [Stages.SETUP_CMM],#[Stages.VERIFY_MACHINE_POS],
}

FIXTURE_SIDE = 76.2
FIXTURE_DIAG = 107.76

X_ORIGIN = 528
Y_ORIGIN = 238
Z_ORIGIN = 400
CMM_ORIGIN = float3(X_ORIGIN,Y_ORIGIN,Z_ORIGIN)

Z_BALL_DIA = 6.35
PROBE_DIA = 4

TOOL_3_LENGTH = 117.8
B_LINE_LENGTH = 35

Z_STEP = -25
A_STEP = 5
B_STEP = 5

V2_10_PROPS = {
  'A_MIN': -25,
  'A_MAX': 135
}

V2_50_PROPS = {
  'A_MIN': -25,
  'A_MAX': 135
}


B_FIXTURE_OFFSET = 135

D_MAT = np.array([[0,1,0,0],[0,0,1,0],[0,0,0,1],[1,1,1,1]])

waypoints = {
  'origin': CMM_ORIGIN,
  'top_l_bracket_back_right': float3(371.9, 466.8, 126.33),
  'top_l_bracket_front_right': float3(358.2, 448.4, 126.33),
  'probe_fixture_tip': float3(326.0, 290.0, 50.0),
  'probe_fixture_tip2': float3(324.3, 318.9, 42.8),
  'probe_fixture_tip_from_origin': float3(-48.5, -150.1, -73.0),
  'probe_fixture_outside_face': float3(243.25, 334.9, -10),
  'b_rot_cent_approx': float3(297.87, 339.53, 126),
  'a_rot_cent_approx': float3(295.1, 412.7, 119.4),
  'z_home': float3(400.9,400.5,57.5),
}

ORIGIN = waypoints['top_l_bracket_back_right']
ORG_TO_B = float3(-3.15, -5.1, 0)

DEBUG = False
def debugWait():
  if DEBUG:
    input()
  return

'''
Ensure startup conditions: homed, tool set, errors clear
Verify machine position
  Probe against top of A-backing plate for maximum clearance/safety
Characterize A motion
Characterize B motion
'''

calibManagerInstance = None

class CalibManager:
  def __init__(self):
    self.client = None
    self.metrologyManager = metrology.FeatureManager.getInstance()
    self.next_feature_id = 1
    self.feature_ids = {}
    self.stages_completed = {}
    for stage in Stages:
      self.stages_completed[stage] = False
    # self._stage2methods_map_ = {
    #   Stages.SETUP_CMM = self.setupCMM,
    #   Stages.VERIFY_POS = self.verifyMachinePos,
    #   Stages.SETUP_CMM = self.setupCMM,
    # }
    self.isStartedA = False
    self.isStartedB = False
    self.machine_props = V2_50_PROPS
    self.isCmmError = False

  def getInstance():
    global calibManagerInstance
    if calibManagerInstance == None:
      calibManagerInstance = CalibManager()

    return calibManagerInstance

  def add_feature(self,feature_name):
      self.feature_ids[feature_name] = self.next_feature_id
      self.metrologyManager.getActiveFeatureSet().setActiveFeatureID(self.next_feature_id)
      self.next_feature_id = self.next_feature_id + 1
      return self.metrologyManager.getActiveFeatureSet().getActiveFeature()


  def save_features(self):
    with open('/sysroot/home/pocketnc/savefile', 'w') as f:
      for k in self.feature_ids.keys():
        if k.find("proj") == -1 and k.find("b_circle") == -1:
          f.write("\n\n%s\n" % k)
          fid = self.feature_ids[k]
          feat = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
          for pt in feat.points():
            f.write("%s\n" % pt)

  def load_features(self):
    with open('/sysroot/home/pocketnc/savefile', 'r') as f:
      lines = f.readlines()
      i = 0
      while i < len(lines)-2:
        if lines[i] == '\n' and lines[i+1] == '\n':
          feat_name = lines[i+2][:-1]
          print(feat_name)
          feat = self.add_feature(feat_name)
          j = i + 3
          while j<len(lines) and lines[j] != '\n':
            pt = []
            for n in lines[j].split():
              try:
                num = float(n.replace('[', '').replace(']', ''))
              except ValueError:
                continue
              # print(num)
              pt.append(num)
            feat.addPoint(*pt)
            j = j + 1
          i = j
    fid = self.feature_ids['b-90']
    feat = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
    print("_______END___________")
    print(feat.points())
      


  def runStage(self, stage, *args):
    print("Running stage: %s " % stage)
    # an exception will be thrown if stage is not defined in the Stages enum
    if type(stage) is str:
      stage = Stages[stage.upper()]
    stage in Stages

    # try:
    #   if self.client is not None and self.client.stream is not None:
    #     print('trying to disconnect')
    #     asyncio.get_event_loop().run_until_complete(self.client.disconnect())
    # except Exception as e:
    #   print("disconnect e: %s" % e)

    if stage in STAGE_PREREQS:
      for prereqStage in STAGE_PREREQS[stage]:
        if not self.stages_completed[prereqStage]:
          self.runStage(prereqStage)
    
    if stage != Stages.CONNECT_TO_CMM and (self.client is None or self.client.stream is None):
      #something broken, quit
      return "FAILED"

    stage_method = getattr(self, stage.name.lower())
    try:
      isStageComplete = asyncio.get_event_loop().run_until_complete(stage_method(*args))
      print('completing stage %s' % stage)
      self.stages_completed[stage] = isStageComplete
    except CmmException as e:
      print("Failed running stage %s" % stage)
      return "FAILED"


  def verifyMachinePos(self):
    asyncio.get_event_loop().run_until_complete(self.verifyStartPos())

  async def connect_to_cmm(self):
    try:
      if self.client.stream is not None:
        print('trying to disconnect')
        asyncio.get_event_loop().run_until_complete(self.client.disconnect())
    except Exception as e:
      print("disconnect e: %s" % e)
    
    self.client = None
    self.client = Client(HOST, PORT)
    await self.client.connect()
    return True

  async def setup_cmm(self):
    await self.client.ClearAllErrors().complete()
    await routines.ensureHomed(self.client)
    await routines.ensureToolLoaded(self.client, "Component_3.1.50.4.A0.0-B0.0")
    await self.client.SetProp("Tool.GoToPar.Speed(500)").ack()
    await self.client.SetProp("Tool.GoToPar.Accel(250)").ack()
    return True

  async def go_to_clearance_z(self):
    await self.client.GoTo("Z(400)").complete()
    return False

  async def go_to_clearance_y(self):
    await self.client.GoTo("Y(200)").complete()
    return False


  async def verify_machine_pos(self):
    '''
    Locate machine and verify it is in home position
    '''
    try:
      print("Checking machine position")
      debugWait()
      await self.client.GoTo((ORIGIN + float3(-55, 100, 100)).ToXYZString()).complete()
      debugWait()
      await self.client.SetProp("Tool.PtMeasPar.Search(25)").complete()
      await self.client.SetProp("Tool.PtMeasPar.HeadTouch(0)").complete()
      await self.client.GoTo("Tool.A(0),Tool.B(0)").complete()
      debugWait()

      #ensure A-backing plate is not raised above L-bracket by sweeping probe tip through area
      approachPos = ORIGIN + float3(-55,50,20)
      await self.client.GoTo(approachPos.ToXYZString()).complete()
      debugWait()
      approachPos = ORIGIN + float3(-55,-50,20)
      await self.client.GoTo(approachPos.ToXYZString()).complete()
      debugWait()



      # L-bracket top face
      approachPos = ORIGIN + float3(-5,-5,10)
      await self.client.GoTo(approachPos.ToXYZString()).complete()
      debugWait()

      measPos = approachPos + float3(0,0,-10)
      lBracketTopPlane = self.metrologyManager.getActiveFeatureSet().getActiveFeature()
      ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % (measPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      lBracketTopPlane.addPoint(pt.x, pt.y, pt.z)
      debugWait()

      measPos = measPos + float3(-35,-15,00)
      ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % (measPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      lBracketTopPlane.addPoint(pt.x, pt.y, pt.z)
      debugWait()

      measPos = measPos + float3(-35,+15,00)
      ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % (measPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      lBracketTopPlane.addPoint(pt.x, pt.y, pt.z)
      debugWait()
      self.feature_ids['L-bracket Top Face'] = self.next_feature_id
      self.next_feature_id = self.next_feature_id + 1


      # L-bracket back face
      self.feature_ids['L-bracket Back Face'] = self.next_feature_id
      self.metrologyManager.getActiveFeatureSet().setActiveFeatureID(self.next_feature_id)
      lBracketBackFace = self.metrologyManager.getActiveFeatureSet().getActiveFeature()

      approachPos = ORIGIN + float3(-100,10,10)
      await self.client.GoTo(approachPos.ToXYZString()).complete()
      approachPos = approachPos + float3(0,0,-20)
      await self.client.GoTo(approachPos.ToXYZString()).complete()

      await self.client.SetProp("Tool.PtMeasPar.Search(25)").complete()
      measPos = approachPos + float3(0,-10,00)
      ptMeas = await self.client.PtMeas("%s,IJK(0,1,0)" % (measPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      lBracketBackFace.addPoint(pt.x, pt.y, pt.z)
      measPos = measPos + float3(20,0,-5)
      ptMeas = await self.client.PtMeas("%s,IJK(0,1,0)" % (measPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      lBracketBackFace.addPoint(pt.x, pt.y, pt.z)
      measPos = measPos + float3(20,0,5)
      ptMeas = await self.client.PtMeas("%s,IJK(0,1,0)" % (measPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      lBracketBackFace.addPoint(pt.x, pt.y, pt.z)
      measPos = measPos + float3(20,0,-5)
      ptMeas = await self.client.PtMeas("%s,IJK(0,1,0)" % (measPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      lBracketBackFace.addPoint(pt.x, pt.y, pt.z)
      measPos = measPos + float3(20,0,5)
      ptMeas = await self.client.PtMeas("%s,IJK(0,1,0)" % (measPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      lBracketBackFace.addPoint(pt.x, pt.y, pt.z)
      measPos = measPos + float3(20,0,-5)
      ptMeas = await self.client.PtMeas("%s,IJK(0,1,0)" % (measPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      lBracketBackFace.addPoint(pt.x, pt.y, pt.z)

      # L-bracket right face
      self.next_feature_id = self.next_feature_id + 1
      self.feature_ids['L-bracket Right Face'] = self.next_feature_id
      self.metrologyManager.getActiveFeatureSet().setActiveFeatureID(self.next_feature_id)
      lBracketRightFace = self.metrologyManager.getActiveFeatureSet().getActiveFeature()
      approachPos = ORIGIN + float3(10,10,-10)
      await self.client.GoTo(approachPos.ToXYZString()).complete()
      approachPos = approachPos + float3(0,-20,0)
      await self.client.GoTo(approachPos.ToXYZString()).complete()

      measPos = approachPos + float3(-10,0,0)
      ptMeas = await self.client.PtMeas("%s,IJK(1,0,0)" % (measPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      lBracketRightFace.addPoint(pt.x, pt.y, pt.z)
      measPos = measPos + float3(0,-5,-5)
      ptMeas = await self.client.PtMeas("%s,IJK(1,0,0)" % (measPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      lBracketRightFace.addPoint(pt.x, pt.y, pt.z)
      measPos = measPos + float3(0,-5,+5)
      ptMeas = await self.client.PtMeas("%s,IJK(1,0,0)" % (measPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      lBracketRightFace.addPoint(pt.x, pt.y, pt.z)


      # #head touches against sides of L-bracket
      # await self.client.SetProp("Tool.PtMeasPar.HeadTouch(1)").send()
      # await routines.headProbeLine(self.client,backPos,float3(1,0,0),100,10,5,20,1,15)

      # #head touch right side
      # nextPos = waypoints['top_l_bracket_front_right'] + float3(10,10,10)
      # self.client.GoTo("%s,Tool.B(-90)" % (nextPos.ToXYZString())).complete()
      # debugWait()
      # nextPos = waypoints['top_l_bracket_front_right'] + float3(10,10,-10)
      # await self.client.GoTo("%s" % (nextPos.ToXYZString())).complete()
      # debugWait()
      # nextPos = waypoints['top_l_bracket_front_right'] + float3(5,10,-10)
      # ptMeas = await self.client.PtMeas("%s,IJK(1,0,0)" % (nextPos.ToXYZString())).complete()
      # pt = float3.FromXYZString(ptMeas.data_list[0])
      # lBracketTopSides.addPoint(pt.x, pt.y, pt.z)
      # debugWait()

      # #head touch far side
      # nextPos = waypoints['top_l_bracket_front_right'] + float3(10,30,-10)
      # await self.client.GoTo("%s" % (nextPos.ToXYZString())).complete()
      # debugWait()
      # nextPos = waypoints['top_l_bracket_front_right'] + float3(-47.5,30,-10)
      # await self.client.GoTo("%s,Tool.B(360)" % (nextPos.ToXYZString())).complete()
      # debugWait()
      # nextPos = waypoints['top_l_bracket_front_right'] + float3(-47.5,20,-10)
      # ptMeas = await self.client.PtMeas("%s,IJK(0,1,0)" % (nextPos.ToXYZString())).complete()
      # pt = float3.FromXYZString(ptMeas.data_list[0])
      # lBracketTopSides.addPoint(pt.x, pt.y, pt.z)
      # debugWait()

      # #head touch left side
      # nextPos = waypoints['top_l_bracket_front_right'] + float3(-110,30,-10)
      # await self.client.GoTo("%s" % (nextPos.ToXYZString())).complete()
      # debugWait()
      # nextPos = waypoints['top_l_bracket_front_right'] + float3(-110,10,-10)
      # await self.client.GoTo("%s,Tool.B(90)" % (nextPos.ToXYZString())).complete()
      # debugWait()
      # nextPos = waypoints['top_l_bracket_front_right'] + float3(-100,10,-10)
      # ptMeas = await self.client.PtMeas("%s,IJK(-1,0,0)" % (nextPos.ToXYZString())).complete()
      # pt = float3.FromXYZString(ptMeas.data_list[0])
      # lBracketTopSides.addPoint(pt.x, pt.y, pt.z)
      # debugWait()

      # #head touch near side
      # nextPos = waypoints['top_l_bracket_front_right'] + float3(-105,-20,-10)
      # await self.client.GoTo("%s" % (nextPos.ToXYZString())).complete()
      # debugWait()
      # nextPos = waypoints['top_l_bracket_front_right'] + float3(-47.5,-20,-10)
      # await self.client.GoTo("%s,Tool.B(180)" % (nextPos.ToXYZString())).complete()
      # debugWait()
      # nextPos = waypoints['top_l_bracket_front_right'] + float3(-47.5,-10,-10)
      # ptMeas = await self.client.PtMeas("%s,IJK(0,-1,0)" % (nextPos.ToXYZString())).complete()
      # pt = float3.FromXYZString(ptMeas.data_list[0])
      # lBracketTopSides.addPoint(pt.x, pt.y, pt.z)
      # debugWait()


      #locate the probe fixtures vertical fin
      self.next_feature_id = self.next_feature_id + 1
      self.feature_ids['probeFixtureFinRightFace'] = self.next_feature_id
      self.metrologyManager.getActiveFeatureSet().setActiveFeatureID(self.next_feature_id)
      probeFixtureFinRightFace = self.metrologyManager.getActiveFeatureSet().getActiveFeature()

      currOrig = ORIGIN + waypoints['probe_fixture_tip_from_origin']
      await self.client.SetProp("Tool.PtMeasPar.HeadTouch(1)").complete()
      debugWait()
      nextPos = currOrig + float3(20,0,20)
      await self.client.GoTo("%s,Tool.A(0),Tool.B(-90)" % (nextPos.ToXYZString())).complete()
      debugWait()
      nextPos = currOrig + float3(20,0,-10)
      await self.client.GoTo("%s" % (nextPos.ToXYZString())).complete()
      debugWait()
      nextPos = currOrig + float3(0,0,-10)
      ptMeas = await self.client.PtMeas("%s,IJK(1,0,0)" % (nextPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      probeFixtureFinRightFace.addPoint(pt.x, pt.y, pt.z)
      nextPos = currOrig + float3(0,+3,0)
      ptMeas = await self.client.PtMeas("%s,IJK(1,0,0)" % (nextPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      probeFixtureFinRightFace.addPoint(pt.x, pt.y, pt.z)
      debugWait()
      nextPos = currOrig + float3(0,+6,0)
      ptMeas = await self.client.PtMeas("%s,IJK(1,0,0)" % (nextPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      probeFixtureFinRightFace.addPoint(pt.x, pt.y, pt.z)
      debugWait()

      self.next_feature_id = self.next_feature_id + 1
      self.feature_ids['probeFixtureFinAngleFace'] = self.next_feature_id
      self.metrologyManager.getActiveFeatureSet().setActiveFeatureID(self.next_feature_id)
      probeFixtureFinAngleFace = self.metrologyManager.getActiveFeatureSet().getActiveFeature()
      await self.client.GoTo("Tool.A(0),Tool.B(-135)").complete()
      debugWait()
      await self.client.GoTo("Tool.A(30),Tool.B(-135)").complete()
      debugWait()
      await self.client.GoTo("Tool.A(30),Tool.B(-90)").complete()
      nextPos = currOrig + float3(-10,0,-10)
      ptMeas = await self.client.PtMeas("%s,IJK(-1,0,1)" % (nextPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      probeFixtureFinAngleFace.addPoint(pt.x, pt.y, pt.z)
      nextPos = currOrig + float3(-10,-2,-10)
      ptMeas = await self.client.PtMeas("%s,IJK(-1,0,1)" % (nextPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      probeFixtureFinAngleFace.addPoint(pt.x, pt.y, pt.z)
      nextPos = currOrig + float3(-10,+2,-10)
      ptMeas = await self.client.PtMeas("%s,IJK(-1,0,1)" % (nextPos.ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      probeFixtureFinAngleFace.addPoint(pt.x, pt.y, pt.z)

      await self.client.GoTo("Z(%s)" % (nextPos.z + 150)).complete()
      await self.client.GoTo("Tool.A(0),Tool.B(0)").complete()

    except Exception as ex:
      print("exception %s" % str(ex))
      raise ex
      # await self.client.disconnect()

  async def test_head(self, angle):
    '''
    Do a head-probe line against 1 face of the probing fixture
    '''
    try:
      print("Manually position probe to be +Y from the -X side of the XZ-aligned plane of the L-bracket")
      input()
      getBackPos = self.client.Get("X(),Y(),Z()")
      await getBackPos.complete()
      backPos = readPointData(getBackPos.data_list[0])
      await routines.headProbeLine(self.client,backPos,float3(1,0,0),100,10,5,20,1,15)
    except CmmException as ex:
      print("CmmExceptions %s" % ex)

  async def find_b_cor(self):
    orig = waypoints['b_rot_cent_approx']
    approachPos = orig + float3(0,0,100)
    await self.client.GoTo(approachPos.ToXYZString()).complete()
    await self.client.SetProp("Tool.PtMeasPar.HeadTouch(0)").complete()

    self.next_feature_id = self.next_feature_id + 1
    self.feature_ids['probeFixtureTopFace'] = self.next_feature_id
    self.metrologyManager.getActiveFeatureSet().setActiveFeatureID(self.next_feature_id)
    probeFixtureTopFace = self.metrologyManager.getActiveFeatureSet().getActiveFeature()
    ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % (orig.ToXYZString())).complete()
    pt = float3.FromXYZString(ptMeas.data_list[0])
    probeFixtureTopFace.addPoint(*pt)
    ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % ((orig + float3(3,3,0)).ToXYZString())).complete()
    pt = float3.FromXYZString(ptMeas.data_list[0])
    probeFixtureTopFace.addPoint(*pt)
    ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % ((orig + float3(-3,3,0)).ToXYZString())).complete()
    pt = float3.FromXYZString(ptMeas.data_list[0])
    probeFixtureTopFace.addPoint(*pt)

    await self.client.SetProp("Tool.PtMeasPar.Search(15)").complete()

    #back-right
    self.next_feature_id = self.next_feature_id + 1
    self.feature_ids['probeFixtureTopBackRight'] = self.next_feature_id
    self.metrologyManager.getActiveFeatureSet().setActiveFeatureID(self.next_feature_id)
    probeFixtureTopBackRight = self.metrologyManager.getActiveFeatureSet().getActiveFeature()
    x_shift = 5
    y_diff = -x_shift
    approachPos = orig + float3(x_shift,FIXTURE_DIAG/2, 10)
    await self.client.GoTo(approachPos.ToXYZString()).complete()
    approachPos = approachPos + float3(0,0,-15)
    await self.client.GoTo(approachPos.ToXYZString()).complete()
    lineStartPos = approachPos + float3(0,y_diff,0)
    backRightPoints = await routines.probeLine(self.client, lineStartPos, float3(1,-1,0), float3(1, 1, 0), 40, 10, 2, -1)
    for pt in backRightPoints:
      probeFixtureTopBackRight.addPoint(*pt)

    #front-right
    self.next_feature_id = self.next_feature_id + 1
    self.feature_ids['probeFixtureTopFrontRight'] = self.next_feature_id
    self.metrologyManager.getActiveFeatureSet().setActiveFeatureID(self.next_feature_id)
    probeFixtureTopFrontRight = self.metrologyManager.getActiveFeatureSet().getActiveFeature()
    y_shift = -5
    x_diff = y_shift
    approachPos = orig + float3(FIXTURE_DIAG/2 + 10,0, 10)
    await self.client.GoTo(approachPos.ToXYZString()).complete()
    approachPos = orig + float3(FIXTURE_DIAG/2, y_shift, -5)
    await self.client.GoTo(approachPos.ToXYZString()).complete()
    lineStartPos = approachPos + float3(x_diff,0,0)
    frontRightPoints = await routines.probeLine(self.client, lineStartPos, float3(-1,-1,0), float3(1, -1, 0), 40, 10, 2,-1)
    for pt in frontRightPoints:
      probeFixtureTopFrontRight.addPoint(*pt)


    #front-left
    self.next_feature_id = self.next_feature_id + 1
    self.feature_ids['probeFixtureTopFrontLeft'] = self.next_feature_id
    self.metrologyManager.getActiveFeatureSet().setActiveFeatureID(self.next_feature_id)
    probeFixtureTopFrontLeft = self.metrologyManager.getActiveFeatureSet().getActiveFeature()
    x_shift = -5
    y_diff = -x_shift
    approachPos = orig + float3(0,-FIXTURE_DIAG/2-10, 10)
    await self.client.GoTo(approachPos.ToXYZString()).complete()
    approachPos = orig + float3(x_shift,-FIXTURE_DIAG/2, -5)
    await self.client.GoTo(approachPos.ToXYZString()).complete()
    lineStartPos = approachPos + float3(0,y_diff,0)
    frontLeftPoints = await routines.probeLine(self.client, lineStartPos, float3(-1,1,0), float3(-1, -1, 0), 40, 10, 2, -1)
    for pt in frontLeftPoints:
      probeFixtureTopFrontLeft.addPoint(*pt)

    #rear-left
    self.next_feature_id = self.next_feature_id + 1
    self.feature_ids['probeFixtureTopRearLeft'] = self.next_feature_id
    self.metrologyManager.getActiveFeatureSet().setActiveFeatureID(self.next_feature_id)
    probeFixtureTopRearLeft = self.metrologyManager.getActiveFeatureSet().getActiveFeature()
    y_shift = 5
    x_diff = y_shift
    approachPos = orig + float3(-(FIXTURE_DIAG/2 + 10),0, 10)
    await self.client.GoTo(approachPos.ToXYZString()).complete()
    approachPos = orig + float3(-(FIXTURE_DIAG/2 + 10), y_shift, -5)
    await self.client.GoTo(approachPos.ToXYZString()).complete()
    lineStartPos = approachPos + float3(x_diff,0,0)
    rearLeftPoints = await routines.probeLine(self.client, lineStartPos, float3(1,1,0), float3(-1, 1, 0), 40, 10, 2, -1)
    for pt in rearLeftPoints:
      probeFixtureTopRearLeft.addPoint(*pt)

    #project the side lines to the top plane

  async def find_top_plane(self, y_pos_v2):
    try:
      orig = waypoints['b_rot_cent_approx'] + float3(0,0,(-y_pos_v2 - 63.5))
      await self.client.GoTo("Tool.Alignment(0,0,1)").complete()
      await self.client.SetProp("Tool.PtMeasPar.HeadTouch(0)").complete()
      await self.client.GoTo((orig + float3(0,0,100)).ToXYZString()).complete()
      await self.client.GoTo((orig + float3(0,0,10)).ToXYZString()).complete()

      fixture_top_face = self.add_feature('fixture_top_face')

      ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % ((orig).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      fixture_top_face.addPoint(*pt)

      ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % ((orig + float3(-25,-5,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      fixture_top_face.addPoint(*pt)

      ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % ((orig + float3(-25,5,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      fixture_top_face.addPoint(*pt)

      ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % ((orig + float3(25,5,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      fixture_top_face.addPoint(*pt)
      
      ptMeas = await self.client.PtMeas("%s,IJK(0,0,1)" % ((orig + float3(25,-5,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      fixture_top_face.addPoint(*pt)
    except Exception as ex:
      print("exception %s" % str(ex))
      raise ex
      # await self.client.disconnect()

  async def probe_x(self, x_pos_v2, z_pos_v2):
    '''
    Probe against the ball mounted in the spindle
    Probe should be aligned vertical (or close) before running
    The first move is to the position 25mm above the target
    '''
    orig = waypoints['z_home'] + float3(z_pos_v2,x_pos_v2 - 63.5,0)
    contact_radius = (Z_BALL_DIA+PROBE_DIA)/2
    a_angle_probe_contact = math.atan2(contact_radius,TOOL_3_LENGTH)*180/math.pi
    feature_name = "x_%.2f" % x_pos_v2
    feature_x_pos = self.add_feature(feature_name)
    try:
      await self.client.GoTo("Tool.Alignment(0,0,1,1,0,0)").complete()
      await self.client.GoTo((orig + float3(0,0,25)).ToXYZString()).complete()
      await self.client.GoTo("Tool.A(%s),Tool.B(-180)" % (a_angle_probe_contact+1)).complete()
      getCurrPosCmd = await self.client.Get("X(),Y(),Z()").complete()
      currPos = readPointData(getCurrPosCmd.data_list[0])
      await self.client.GoTo((currPos + float3(0,0,-25)).ToXYZString()).complete()
      
      await self.client.SetProp("Tool.PtMeasPar.HeadTouch(1)").complete()
      await self.client.SetProp("Tool.PtMeasPar.Search(5)").complete()

      ptMeas = await self.client.PtMeas("%s,IJK(0,1,0)" % ((orig + float3(0,contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_x_pos.addPoint(*pt)

      await self.client.GoTo("Tool.A(%s),Tool.B(-90)" % (a_angle_probe_contact+1)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(-1,0,0)" % ((orig + float3(-contact_radius,0,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_x_pos.addPoint(*pt) 

      await self.client.GoTo("Tool.A(%s),Tool.B(0)" % (a_angle_probe_contact+1)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(0,-1,0)" % ((orig + float3(0,-contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_x_pos.addPoint(*pt) 

      #rise 2mm and probe another 3 points
      getCurrPosCmd = await self.client.Get("X(),Y(),Z()").complete()
      currPos = readPointData(getCurrPosCmd.data_list[0])
      await self.client.GoTo((currPos + float3(0,0,+2)).ToXYZString()).complete()

      measPos2 = orig + float3(0,0,2)
      ptMeas = await self.client.PtMeas("%s,IJK(0,1,0)" % ((measPos2 + float3(0,contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_x_pos.addPoint(*pt)

      await self.client.GoTo("Tool.A(%s),Tool.B(-90)" % (a_angle_probe_contact+1)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(-1,0,0)" % ((measPos2 + float3(-contact_radius,0,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_x_pos.addPoint(*pt) 

      await self.client.GoTo("Tool.A(%s),Tool.B(0)" % (a_angle_probe_contact+1)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(0,-1,0)" % ((measPos2 + float3(0,-contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_x_pos.addPoint(*pt) 

      await self.client.GoTo((orig + float3(0,0,50)).ToXYZString()).complete()
      return False
    except Exception as ex:
      print("exception %s" % str(ex))
      raise ex
      # await self.client.disconnect()

  async def probe_z(self, x_pos_v2, z_pos_v2):
    '''
    Probe against the ball mounted in the spindle
    Probe should be aligned vertical (or close) before running
    The first move is to the position 25mm above the target
    '''
    orig = waypoints['z_home'] + float3(z_pos_v2,x_pos_v2 - 63.5,0)
    contact_radius = (Z_BALL_DIA+PROBE_DIA)/2
    a_angle_probe_contact = math.atan2(contact_radius,TOOL_3_LENGTH)*180/math.pi
    feature_name = "z_%d" % z_pos_v2
    feature_z_pos = self.add_feature(feature_name)
    try:
      await self.client.GoTo("Tool.Alignment(0,0,1,1,0,0)").complete()
      await self.client.GoTo((orig + float3(0,0,25)).ToXYZString()).complete()
      await self.client.GoTo("Tool.A(%s),Tool.B(-180)" % (a_angle_probe_contact+1)).complete()
      getCurrPosCmd = await self.client.Get("X(),Y(),Z()").complete()
      currPos = readPointData(getCurrPosCmd.data_list[0])
      await self.client.GoTo((currPos + float3(0,0,-25)).ToXYZString()).complete()
      
      await self.client.SetProp("Tool.PtMeasPar.HeadTouch(1)").complete()
      await self.client.SetProp("Tool.PtMeasPar.Search(5)").complete()

      ptMeas = await self.client.PtMeas("%s,IJK(0,1,0)" % ((orig + float3(0,contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_z_pos.addPoint(*pt)

      await self.client.GoTo("Tool.A(%s),Tool.B(-90)" % (a_angle_probe_contact+1)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(-1,0,0)" % ((orig + float3(-contact_radius,0,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_z_pos.addPoint(*pt) 

      await self.client.GoTo("Tool.A(%s),Tool.B(0)" % (a_angle_probe_contact+1)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(0,-1,0)" % ((orig + float3(0,-contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_z_pos.addPoint(*pt) 

      #rise 2mm and probe another 3 points
      getCurrPosCmd = await self.client.Get("X(),Y(),Z()").complete()
      currPos = readPointData(getCurrPosCmd.data_list[0])
      await self.client.GoTo((currPos + float3(0,0,+2)).ToXYZString()).complete()

      measPos2 = orig + float3(0,0,2)
      ptMeas = await self.client.PtMeas("%s,IJK(0,1,0)" % ((measPos2 + float3(0,contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_z_pos.addPoint(*pt)

      await self.client.GoTo("Tool.A(%s),Tool.B(-90)" % (a_angle_probe_contact+1)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(-1,0,0)" % ((measPos2 + float3(-contact_radius,0,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_z_pos.addPoint(*pt) 

      await self.client.GoTo("Tool.A(%s),Tool.B(0)" % (a_angle_probe_contact+1)).complete()
      ptMeas = await self.client.PtMeas("%s,IJK(0,-1,0)" % ((measPos2 + float3(0,-contact_radius,0)).ToXYZString())).complete()
      pt = float3.FromXYZString(ptMeas.data_list[0])
      feature_z_pos.addPoint(*pt) 

      await self.client.GoTo((orig + float3(0,0,50)).ToXYZString()).complete()
      return False
    except Exception as ex:
      print("exception %s" % str(ex))
      await self.client.disconnect()


  async def probe_a(self, angle, y_pos_v2=-63.5):
    '''
    Find start position and driving direction from angle
    '''
    a_cor = waypoints['a_rot_cent_approx'] + float3(0,0,0)
    a_cor = a_cor + float3(0,0,(-y_pos_v2 - 63.5))
    vec_a_cor_to_orig_fixture_tip = (waypoints['probe_fixture_tip2']+float3(0,0,(63.5 - y_pos_v2))) - (waypoints['a_rot_cent_approx'] + float3(0,0,0))
    fixture_offset_angle = 0
    fixture_length = 50.8
    vec_cor_to_orig_start = vec_a_cor_to_orig_fixture_tip + float3(0,0,-2) # down a bit away from edge
    dist_cor_to_start = math.sqrt(math.pow(vec_cor_to_orig_start.x,2) + math.pow(vec_cor_to_orig_start.z,2)) + 2
    angle_offset_start = math.atan2(vec_cor_to_orig_start.z,vec_cor_to_orig_start.x)*180/math.pi
    if not self.isStartedA:
      await self.client.SetProp("Tool.PtMeasPar.Search(12)").ack()
      await self.client.GoTo((a_cor + float3(0, -100, 150)).ToXYZString()).complete()
      await self.client.GoTo("Tool.Alignment(0,-1,0)").complete()
      await self.client.GoTo((a_cor + float3(0, -200, 0)).ToXYZString()).complete()
      self.isStartedA = True

    '''
    Do a head-probe line against 1 face of the probing fixture
    '''
    try:
      print("Finding A position")
      self.next_feature_id = self.next_feature_id + 1
      self.feature_ids["a_%d" % angle] = self.next_feature_id
      self.metrologyManager.getActiveFeatureSet().setActiveFeatureID(self.next_feature_id)
      a_line = self.metrologyManager.getActiveFeatureSet().getActiveFeature()

      total_angle =  fixture_offset_angle - angle
      angle_start_pos = total_angle + angle_offset_start
      print('vec_cor_to_orig_start %s' % vec_cor_to_orig_start)
      print('angle_offset_start %s' % angle_offset_start)
      print('start origin Rad %s' % dist_cor_to_start)
      print('start pos angle %s' % angle_start_pos)
      start_pos = a_cor + float3(dist_cor_to_start * math.cos(angle_start_pos*math.pi/180), vec_a_cor_to_orig_fixture_tip.y,dist_cor_to_start * math.sin(angle_start_pos*math.pi/180))
      print('start pos %s' % start_pos)
      # await self.client.GoTo((a_cor + float3(0, -100, 150)).ToXYZString()).complete()
      # await self.client.GoTo("Tool.Alignment(0,-1,0)").complete()
      # await self.client.GoTo((start_pos + float3(0, -100, 150)).ToXYZString()).complete()
      # await self.client.GoTo((start_pos + float3(0, -100, 0)).ToXYZString()).complete()
      # await self.client.GoTo((start_pos + float3(0, -25, 0)).ToXYZString()).complete()

      drive_angle = total_angle - 90
      drive_vec = float3(math.cos(drive_angle*math.pi/180), 0, math.sin(drive_angle*math.pi/180))
      face_norm = float3(-drive_vec.z, 0, drive_vec.x)
      print("drive_vec %s" % drive_vec)
      try:
        # points = await routines.probeLine(self.client,start_pos, drive_vec, float3(-drive_vec.z, 0, drive_vec.x), 40, 10, 3, -1)
        points = await routines.headProbeLineXZ(self.client,start_pos,drive_vec,B_LINE_LENGTH,face_norm,3,1)
        for pt in points:
          a_line.addPoint(*pt)
      except CmmException as e:
        pass

      end_pos = start_pos + drive_vec * B_LINE_LENGTH
      retract_pos = end_pos + face_norm * 10

      # face_norm_xz_angle_rad = math.atan2(face_norm.x, face_norm.z)
      # retract_a_frac = math.cos(face_norm_xz_angle_rad)
      # retract_b_frac = math.sin(face_norm_xz_angle_rad)
      # retract_a_angle_deg = 90 - 0.1*(90*retract_a_frac)
      # retract_a_angle_deg = min(115, retract_a_angle_deg)
      # retract_b_angle_deg = 180 - 30 * (1 - retract_b_frac)

      # await self.client.GoTo((start_pos + 5*float3(-drive_vec.z, 0, drive_vec.x)).ToXYZString()).complete()
      # await self.client.GoTo("Tool.A(%s),Tool.B(%s)" % (retract_a_angle_deg, retract_b_angle_deg)).complete()
      await self.client.GoTo((retract_pos).ToXYZString()).complete()

      return False
    except Exception as ex:
      print("exception %s" % str(ex))
      raise ex
      # await self.client.disconnect()


  async def probe_b(self, angle, y_pos_v2=-63.5):
    orig = waypoints['b_rot_cent_approx']
    orig = orig + float3(0,0,(-y_pos_v2 - 63.5))
    fixtureOffsetAngle = B_FIXTURE_OFFSET
    fixture_length = 76.2
    start_posOriginVec = float3(0.5*fixture_length,20 + -0.5*fixture_length,0)
    start_posOriginRad = math.sqrt(math.pow(start_posOriginVec.x,2) + math.pow(start_posOriginVec.y,2))
    start_posOffsetAngle = math.atan2(start_posOriginVec.y,start_posOriginVec.x)*180/math.pi

    '''
    Do a head-probe line against 1 face of the probing fixture
    '''
    try:
      print("Finding B position")
      feature_name = "b-%d" % angle
      b_line = self.add_feature(feature_name)

      totAngle = angle + fixtureOffsetAngle
      start_posAngle = totAngle + start_posOffsetAngle
      print('start_posOriginVec %s' % start_posOriginVec)
      print('start_posOffsetAngle %s' % start_posOffsetAngle)
      print('start origin Rad %s' % start_posOriginRad)
      print('start pos angle %s' % start_posAngle)
      start_pos = orig + float3(start_posOriginRad * math.cos(start_posAngle*math.pi/180), start_posOriginRad * math.sin(start_posAngle*math.pi/180),-6)
      if not self.isStartedB:
        await self.client.SetProp("Tool.PtMeasPar.HeadTouch(1)").complete()
        await self.client.GoTo((start_pos + float3(0, 0, 25)).ToXYZString()).complete()
        self.isStartedB = True
      print('start pos %s' % start_pos)
      # await self.client.GoTo("Tool.Alignment(0,0,1)").complete()
      # await self.client.GoTo((orig + float3(0, 0, 100)).ToXYZString()).complete()
      # await self.client.GoTo((start_pos + float3(0, 0, 15)).ToXYZString()).complete()

      lineAngle = totAngle + 90
      lineVec = float3(math.cos(lineAngle*math.pi/180), math.sin(lineAngle*math.pi/180),0)
      print("lineVec %s" % lineVec)

      # points = await routines.probeLine(self.client,start_pos, lineVec, float3(lineVec.y, -lineVec.x,0),65,10,3,1)
      try:
        points = await routines.headProbeLine(self.client,start_pos, lineVec, 35, 10, 3, 10, -1, 5)
      except CmmException as e:
        print("CmmException in probe_b, raising")
        raise e
      # await routines.headProbeLine(client,start_pos,float3(-1,0,0),75,10,5,10,-1,15)

      for pt in points:
        b_line.addPoint(*pt)

      # await self.client.GoTo("Z(%s)" % (orig.z + 15)).ack()

      return False
    except Exception as ex:
      print("exception %s" % str(ex))
      raise ex
      # await self.client.disconnect()


  async def write_results(self):
    self.write_results_sync()


  def write_results_sync(self):
    '''
    define POCKET coord sys in from TOP_PLANE and Z_VEC
    B results
      project the b-angle lines to top plane
      for each b-axis position
        translate projected points into POCKET coord sys
        find best-fit line through projected points
      for each line
        find angle compared to 0, compare to nominal
    A results
      project the a-angle lines to POCKET coord sys YZ plane
      for each a-axis position
        translate projected points into POCKET space
        find best-fit line through projected points
      for each line
        find angle compared to 0, compare to nominal
    '''
    z_norm_actual = None
    z_vec_proj = None 
    z_vec_cross_top_plane_norm = None
    rot_mat = None
    pocket_yz_plane_norm = None
    # try:
    #   with open('/sysroot/home/pocketnc/results', 'w') as f:
    #     f.write('huh\n')
    # except Exception as e:
    #   print(e)

    fixture_top_face = self.metrologyManager.getActiveFeatureSet().getFeature( self.feature_ids['fixture_top_face'] )
    plane_fixture_top_face = fixture_top_face.plane()
    top_plane_pt = plane_fixture_top_face[0]
    top_plane_norm = plane_fixture_top_face[1]
    print(top_plane_pt)
    print(top_plane_norm)

    '''
    Z results
    Define a line along the z points
    '''
    try:
      z_line = self.add_feature('z_line')
      z_line_proj = self.add_feature('z_line_proj')
      for i in range(0,-89,Z_STEP):
        print(i)
        fid = self.feature_ids['z_%d' % i]
        print('fid is %s' % fid)
        z_pos_feat = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
        z_pos_sphere = z_pos_feat.sphere()
        z_pos = z_pos_sphere[1]
        z_line.addPoint(*z_pos)
        #also project onto top-fixture-plane
        plane_orig_to_pt = z_pos - top_plane_pt
        dist = np.dot(plane_orig_to_pt, top_plane_norm)
        proj_pt = z_pos - dist * top_plane_norm
        z_line_proj.addPoint(*proj_pt)
      
      print(z_line_proj)
      z_norm_actual = -1*z_line.line()[1]
      z_vec_proj = z_line_proj.line()[1]
      z_vec_cross_top_plane_norm = np.cross(z_vec_proj,top_plane_norm)
      print('z_vec_cross_top_plane_norm')
      print(z_vec_cross_top_plane_norm)
      affine_mat_construct = np.array([top_plane_pt,z_vec_proj,z_vec_cross_top_plane_norm,top_plane_norm])
      print('affine_mat_construct')
      print(affine_mat_construct)
      affine_mat = np.vstack((affine_mat_construct.transpose(),[1,1,1,1]))
      print('affine_mat')
      print(affine_mat)
      affine_mat_inv = np.linalg.inv(affine_mat)
      print('affine_mat_inv')
      print(affine_mat_inv)
      rot_mat = np.matmul(D_MAT, affine_mat_inv)
      print('rot_mat')
      print(rot_mat)

    except Exception as e:
      print(e)


    '''
    B results
      project the b-angle lines to top plane
      for each b-axis position
        translate projected points into POCKET coord sys
        find best-fit line through projected points
      for each line
        find angle compared to 0, compare to nominal
    
    for each line, find the position of intersect with the next line
    the center of rotation is at the centroid of these intersection positions
    '''
    try:
      y_norm = top_plane_norm
      x_norm = np.cross(y_norm, z_norm_actual)
      pocket_yz_plane_norm = x_norm
      z_norm = np.cross(x_norm,y_norm)
      p2m_construct = np.array([x_norm,y_norm,z_norm,top_plane_pt])
      p2m = np.vstack((p2m_construct.transpose(),[0,0,0,1]))
      m2p = np.linalg.inv(p2m)
      print("pocketnc 2 machine matrix")
      print(p2m)
      print("machine 2 pocketnc matrix")
      print(m2p)
      b_circle = self.add_feature('b_circle')
      print("b lines 3d")
      for k in self.feature_ids.keys():
        print(k)
      for i in range(0,360,B_STEP):
        print(i)
        fid = self.feature_ids['b-%d' % i]
        print('fid is %s' % fid)
        b_line = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
        b_line_proj = self.add_feature('b_%d_line_proj' % i)
        for pt in b_line.points():
          plane_orig_to_pt = pt - top_plane_pt
          dist = np.dot(plane_orig_to_pt, top_plane_norm)
          proj_pt = pt - dist * top_plane_norm
          # proj_pt_in_plane = np.matmul(rot_mat,np.append(proj_pt,1))
          b_line_proj.addPoint(proj_pt[0], proj_pt[1], proj_pt[2])
      
      print("b lines projected")
      print("project z vec is %s " % z_vec_proj)
      b_0_vec_fid = self.feature_ids['b_0_line_proj']
      b_0_vec = self.metrologyManager.getActiveFeatureSet().getFeature(b_0_vec_fid).line()[1]
      vec_b0_projected_translated = np.matmul(m2p,np.append(b_0_vec,0))
      vec_b0_2d = np.array([vec_b0_projected_translated[0],vec_b0_projected_translated[2]])
      # proj_b0_line_translated_2 = np.matmul(m2p2,np.append(b_0_vec,1))
      # proj_b0_line_translated_3 = np.matmul(m2p3,np.append(b_0_vec,1))

      with open('/sysroot/home/pocketnc/results/b.comp', 'w') as f:
        for i in range(0,360,B_STEP):
          print(i)
          fid = self.feature_ids['b_%d_line_proj' % i]
          print('fid is %s' % fid)
          proj_b_line = self.metrologyManager.getActiveFeatureSet().getFeature(fid).line()
          proj_b_line_translated = np.matmul(m2p,np.append(proj_b_line[1],0))
          vec_bstep_2d = np.array([proj_b_line_translated[0],proj_b_line_translated[2]])
          # proj_b_line_translated2 = np.matmul(m2p2,np.append(proj_b_line[1],1))
          # proj_b_line_translated3 = np.matmul(m2p3,np.append(proj_b_line[1],1))
          #find angle relative to Z
          # line_angle_rel_z = angle_between_ccw_2d(b_0_vec, proj_b_line[1])
          line_angle_rel_0 = angle_between_ccw_2d(vec_bstep_2d, vec_b0_2d)
          err = 0
          if line_angle_rel_0 < 0:
            err = (360 + line_angle_rel_0) - i
          else:
            err = line_angle_rel_0 - i
          comp = -1*err
          # line_angle_rel_z_3 = angle_between_ccw_2d(proj_b0_line_translated_2, proj_b_line_translated2)
          # line_angle_rel_z_4 = angle_between_ccw_2d(proj_b0_line_translated_3, proj_b_line_translated3)
          # print(line_angle_rel_z_3  )
          # print(line_angle_rel_z_4  )
          print("COMP: %s" % comp)
          f.write("%d %s %s\n" % (i, comp, comp))
          # f.write("%d %s %s %s %s\n" % (i, line_angle_rel_z, line_angle_rel_z_2, line_angle_rel_z_3, line_angle_rel_z_4))

          #find intersect position with next B-line
          next_line_angle = (i + B_STEP) % 360
          next_line_fid = self.feature_ids['b_%d_line_proj' % next_line_angle]
          next_proj_b_line = self.metrologyManager.getActiveFeatureSet().getFeature(next_line_fid).line()
          intersect_pos = find_line_intersect(proj_b_line[0],proj_b_line[1],next_proj_b_line[0],next_proj_b_line[1])
          b_circle.addPoint(intersect_pos[0], intersect_pos[1], 0)

      print(b_circle.points())
      print(b_circle.average())
    except Exception as e:
      print(e)
      return e

    '''
    A results
      project the a-angle lines to POCKET coord sys YZ plane
      for each a-axis position
        translate projected points into POCKET space
        find best-fit line through projected points
      for each line
        find angle compared to 0, compare to nominal
    '''
    try:
      # project the a-angle lines to POCKET coord sys YZ plane
      for i in range( self.machine_props['A_MIN'], self.machine_props['A_MAX'] + 1, A_STEP):
        fid = self.feature_ids['a_%d' % i]
        a_line = self.metrologyManager.getActiveFeatureSet().getFeature(fid)
        a_line_proj = self.add_feature('a_%d_line_proj' % i)
        for pt in a_line.points():
          plane_orig_to_pt = pt - top_plane_pt
          dist = np.dot(plane_orig_to_pt, pocket_yz_plane_norm)
          proj_pt = pt - dist * pocket_yz_plane_norm
          # proj_pt_in_plane = np.matmul(rot_mat,np.append(proj_pt,1))
          a_line_proj.addPoint(proj_pt[0], proj_pt[1], proj_pt[2])
      
      a_0_vec_fid = self.feature_ids['a_0_line_proj']
      a_0_vec = self.metrologyManager.getActiveFeatureSet().getFeature(a_0_vec_fid).line()[1]
      vec_a0_projected_translated = np.matmul(m2p,np.append(a_0_vec,0))
      vec_a0_2d = np.array([vec_a0_projected_translated[1],vec_a0_projected_translated[2]])

      print('calculating A positions')
      with open('/sysroot/home/pocketnc/results/a.comp', 'w') as f:
        for i in range( self.machine_props['A_MIN'], self.machine_props['A_MAX'] + 1, A_STEP):
          print(i)
          fid = self.feature_ids['a_%d_line_proj' % i]
          print('fid is %s' % fid)
          proj_a_line = self.metrologyManager.getActiveFeatureSet().getFeature(fid).line()
          proj_a_line_translated = np.matmul(m2p,np.append(proj_a_line[1],0))
          vec_astep_2d = np.array([proj_a_line_translated[1],proj_a_line_translated[2]])
          # proj_b_line_translated2 = np.matmul(m2p2,np.append(proj_b_line[1],1))
          # proj_b_line_translated3 = np.matmul(m2p3,np.append(proj_b_line[1],1))
          #find angle relative to Z
          # line_angle_rel_z = angle_between_ccw_2d(b_0_vec, proj_b_line[1])
          line_angle_rel_0 = angle_between_ccw_2d(vec_astep_2d, vec_a0_2d)
          print(line_angle_rel_0)
          err = 0
          err = line_angle_rel_0 + i
          # if i < 0:
          # else:
          #   err = i - line_angle_rel_0
          comp = err
          # line_angle_rel_z_3 = angle_between_ccw_2d(proj_b0_line_translated_2, proj_b_line_translated2)
          # line_angle_rel_z_4 = angle_between_ccw_2d(proj_b0_line_translated_3, proj_b_line_translated3)
          # print(line_angle_rel_z_3  )
          # print(line_angle_rel_z_4  )
          print("COMP: %s" % comp)
          f.write("%d %s %s\n" % (i, comp, comp))
          # f.write("%d %s %s %s %s\n" % (i, line_angle_rel_z, line_angle_rel_z_2, line_angle_rel_z_3, line_angle_rel_z_4))

          #find intersect position with next B-line
          # next_line_angle = (i + A_STEP) % 360
          # next_line_fid = self.feature_ids['b_%d_line_proj' % next_line_angle]
          # next_proj_b_line = self.metrologyManager.getActiveFeatureSet().getFeature(next_line_fid).line()
          # intersect_pos = find_line_intersect(proj_b_line[0],proj_b_line[1],next_proj_b_line[0],next_proj_b_line[1])
          # b_circle.addPoint(intersect_pos[0], intersect_pos[1], 0)

      
    except Exception as e:
      print(e)
      return e

    return False

  async def end(self):
    '''
    End
    '''
    if self.client is None or self.client.stream is None:
      print("already disconnected")
      return True

    try:
      await self.client.EndSession().complete()
      await self.client.disconnect()
    except CmmException as ex:
      print("CmmExceptions %s" % ex)

