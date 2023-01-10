import math

# Read the calibration data from file. Each line consists of:
# <commanded angle>\t<measured angle>
# return it as an array of (x,y) tuples with the following format:
# ( commanded angle, error ( commanded angle - measured angled ) )

# values with a commanded angle of less than 0 are shifted to be in the range 0-360
# and sorted into their corresponding position in the list
def readBData(file):
  with open(file, 'r') as f:
    strdata = f.read().rstrip()
    data = [ tuple([ float(dd) for dd in d.split('\t') ]) for d in strdata.split('\n') ]

    return data

def processBData(data):
  ret = []
  for d in data:
    ret.append(( float(d[0]) if float(d[0]) >= 0 else float(d[0])+360, float(d[0])-float(d[1]) ))

  ret.sort(key=lambda x: x[0])

#  if ret[0][0] < .000001 and ret[0][0] > -.000001:
#    zeroValue = ret[0][1]
#    ret = [ (d[0], d[1]-zeroValue) for d in ret ]

  return ret

# Read the calibration data from file. Each line consists of:
# <commanded angle>\t<measured angle>
# return it as an array of (x,y) tuples with the following format:
# ( commanded angle, error ( commanded angle - measured angled ) )
def readAData(file):
  with open(file, 'r') as f:
    strdata = f.read().rstrip()
    data = [ tuple([ float(dd) for dd in d.split('\t') ]) for d in strdata.split('\n') ]

  return data

def processAData(data):
  ret = []
  for d in data:
    ret.append(( float(d[0]), float(d[0])-float(d[1])))

  return ret

# Class that represents a series of linear segments that approximate a function (in the math sense of the word).
# Any given x value should compute a single y value.
# Has a sample method for use by the calculateMaxError function.
# This class is used to represent a compensation table with convenience methods
# for sampling at any point and adding additional points.
class Lines:
  def __init__(self, *args):
    self.pts = [ ]

    for p in args:
      self.insert(p)

  # removes all points that aren't in the domain [x1,x2) (includes x1, but doesn't not include x2)
  def removePointsNotInDomain(self, x1, x2):
    self.pts = [ p for p in self.pts if p[0] >= x1 and p[0] < x2 ]
    
  def offset(self, y):
    self.pts = [ (p[0], p[1]+y) for p in self.pts ]

  def insert(self, pt):
    if next((False for x in self.pts if pt[0] == x[0]), True):
      self.pts.append(pt)
      self.pts.sort(key=lambda pt: pt[0])
    else:
      raise ValueError("attempted to insert a duplicate x value: %s" % (pt,))

  def remove(self, pt):
    if pt in self.pts:
      self.pts.remove(pt)

  def __repr__(self):
    return "{ Lines - %s }" % (", ".join([ str(p) for p in self.pts]))

  def length(self):
    return len(self.pts)

  def sample(self, x):
    i = 0
    numPts = len(self.pts)
    if numPts == 0:
      return (x,0)

    while i < numPts and x > self.pts[i][0]:
      i += 1

    if i >= numPts:
      return self.pts[-1]
    elif i == 0:
      return self.pts[0]

    t = (x-self.pts[i-1][0])/(self.pts[i][0]-self.pts[i-1][0])

    return (x,self.pts[i-1][1]*(1-t)+self.pts[i][1]*t)

# class that represents an infinite line with a slope, m, and y-intercept b
# Has a sample method for use in the calculateMaxError function
class Line:
  def __init__(self, m, b):
    self.m = m
    self.b = b

  def __repr__(self):
    return "{ Line - slope: %s, y-intercept: %s }" % (self.m, self.b)

  def intersects(self, line):
    x = (line.b-self.b)/(self.m-line.m)
    y = self.sample(x)[1]
    return (x,y)

  def sample(self, x):
    return (x, self.m*x+self.b)

  @staticmethod
  def fromTwoPoints(pt1, pt2):
    m = (pt2[1]-pt1[1])/(pt2[0]-pt1[0])
    b = pt1[1]-m*pt1[0]
    return Line(m, b)

# Calculates the max error a sampler predicts given a set of (x,y) points. 
# A sampler has a sample method that returns an (x,y) tuple given an x value so
# the error is the difference between the sampled y value and a point's y value.
def calculateMaxError(sampler, pts):
  maxErr = (0,0)
  for p in pts:
    err = (p[0], abs(p[1]-sampler.sample(p[0])[1]))

    if err[1] > maxErr[1]:
      maxErr = err

  return maxErr


# pt - (x,y) point that the line must pass through, for a simple regression line, pass in the average of all points in pts
# pts - array of (x,y) points to fit a line to
# returns a Line object that best fits the points in pts and passes through the point, pt.
def bestFitLine(pt, pts):
  sumDXDY = 0
  sumDXDX = 0

  for p in pts:
    dx = p[0]-pt[0]
    dy = p[1]-pt[1]

    sumDXDY += dx*dy
    sumDXDX += dx*dx

  m = sumDXDY/sumDXDX
  b = pt[1]-pt[0]*m

  return Line(m,b)

def averagePoint(pts):
  sumX = 0
  sumY = 0
  numPts = len(pts)

  for p in pts:
    sumX += p[0]
    sumY += p[1]

  return (sumX/numPts, sumY/numPts)

def calculateACompensation(allPts):
  (greedyCompensation, greedyError) = doGreedyACompensationIncrementally(allPts, .01, .001)

  return (greedyCompensation, greedyError)

def doGreedyACompensationIncrementally(allPts, fromError, errorIncr):
  errorThreshold = fromError
  aCompensation = doGreedyACompensationCalculation(allPts, errorThreshold)
  # Don't look at error of first point, because its taken at the limit of motion and irrelevant for meeting spec
  error = calculateMaxError(aCompensation, allPts[1:])

  while error[1] > errorThreshold:
    errorThreshold += errorIncr
    aCompensation = doGreedyACompensationCalculation(allPts, errorThreshold)
    # Don't look at error of first point, because its taken at the limit of motion and irrelevant for meeting spec
    error = calculateMaxError(aCompensation, allPts[1:])

  return (aCompensation, error)

def doGreedyACompensationCalculation(allPts, errorThreshold):
  numPts = len(allPts)
  endPoint = (0,0)
  aCompensation = Lines()

  maxError = calculateMaxError(aCompensation,allPts)
  currentLine = None

  startIndex = 0

  while maxError[1] > errorThreshold:
    for i in range(startIndex+2, numPts):
      pts = allPts[startIndex:i+1]
      line = bestFitLine(endPoint, pts)
      error = calculateMaxError(line,pts)
      if error[1] <= errorThreshold:
        currentLine = line
        currentIndex = i

    if currentLine == None:
      break

    startIndex = currentIndex
    if aCompensation.length() == 0: 
      aCompensation.insert(currentLine.sample(-45))
    endPoint = currentLine.sample(allPts[currentIndex][0])
    aCompensation.insert(endPoint)

    maxError = calculateMaxError(currentLine, allPts)

    currentLine = None

  return aCompensation

def doGreedyCompensationIncrementally(allPts, fromError, errorIncr):
  errorThreshold = fromError
  compTable = doGreedyCompensationCalculation(allPts, errorThreshold)
  error = calculateMaxError(compTable, allPts)

  while error[1] > errorThreshold:
    errorThreshold += errorIncr
    compTable = doGreedyCompensationCalculation(allPts, errorThreshold)
    error = calculateMaxError(compTable, allPts)

  return (compTable, error)

def doGreedyCompensationCalculation(allPts, errorThreshold):
  numPts = len(allPts)
  endPoint = (0,0)
  compTable = Lines()

  maxError = calculateMaxError(compTable,allPts)
  currentLine = None

  startIndex = 0

  while maxError[1] > errorThreshold:
    for i in range(startIndex+2, numPts):
      pts = allPts[startIndex:i+1]
      line = bestFitLine(endPoint, pts)
      error = calculateMaxError(line,pts)
      if error[1] <= errorThreshold:
        currentLine = line
        currentIndex = i

    if currentLine == None:
      break

    startIndex = currentIndex
    endPoint = currentLine.sample(allPts[currentIndex][0])
    compTable.insert(endPoint)

    maxError = calculateMaxError(currentLine, allPts)

    currentLine = None

  return compTable

def calculateBCompensation(allPts):
  print(1)
  (greedyCompensation, greedyError) = doGreedyBCompensationIncrementally(allPts, .01, .001)
  print(2)
  (simplifyCompensation, simplifyError) = doSimplifyBCompensation(allPts)
  print(3)

  (greedyBestFitPartitionCompensation, greedyBestFitError) = bestFitPartitionCompensation(allPts, greedyCompensation.pts[1:], firstPt=(0.0,0.0), lastPt=(360.0,0.0))
  print(4)
  (simplifyBestFitPartitionCompensation, simplifyBestFitError) = bestFitPartitionCompensation(allPts, simplifyCompensation.pts[1:], firstPt=(0.0,0.0), lastPt=(360.0,0.0))
  print(5)

  (bestFitCompensation, bestFitError) = doBestFitLinesIncrementally(allPts, .01, .001)

  print("max greedy compensation error: %s" % (greedyError,))
  print("max simplify compensation error: %s" % (simplifyError,))
  print("max greedy best fit partition compensation error: %s" % (greedyBestFitError,))
  print("max simplify best fit partition compensation error: %s" % (simplifyBestFitError,))
  print("max best fit error: %s " % (bestFitError,))
  print()

  allCompensations = [ (greedyCompensation, greedyError, "greedy"), 
                       (simplifyCompensation, simplifyError, "simplify"), 
                       (greedyBestFitPartitionCompensation, greedyBestFitError, "greedyPartitionBestFit"), 
                       (simplifyBestFitPartitionCompensation, simplifyBestFitError, "simplifyPartitionBestFit"),
                       (bestFitCompensation, bestFitError, "bestFitLines") ]
  allCompensations.sort(key=lambda x: x[1][1])

  for comp in allCompensations:
    print("Compensation using %s algorithm, max error of %s:" % (comp[2],comp[1]))
    for p in comp[0].pts[0:-1]:
      print("%s %s %s" % (p[0], p[1], p[1]))

    print()

  return allCompensations[0]

def bestFitPartitionCompensation(allPts, partitionPts, firstPt=None, lastPt=None):
  dataPerSegment = []
  first = 0
  for divider in partitionPts:
    segmentData = []
    for pt in allPts:
      if pt[0] >= first and pt[0] <= divider[0]:
        segmentData.append(pt)
        first = pt[0]
    dataPerSegment.append(segmentData)

  lineObjects = []
  numSegments = len(dataPerSegment)


  for (i,segmentData) in enumerate(dataPerSegment):
    if i == 0 and firstPt != None:
      pt = firstPt
    elif i == numSegments-1 and lastPt != None:
      pt = lastPt
    else:
      pt = averagePoint(segmentData)

    lineObjects.append(bestFitLine(pt, segmentData))

#  for (i,line) in enumerate(lineObjects):
#    print line.sample(dataPerSegment[i][0][0]), line.sample(dataPerSegment[i][-1][0])

  endPoints = []
  if firstPt != None:
    endPoints.append(firstPt)
  else:
    endPoints.append(lineObjects[0].sample(allPts[0][0]))

  if lastPt != None:
    endPoints.append(lastPt)
  else:
    endPoints.append(lineObjects[-1].sample(allPts[-1][0]))

  compensation = Lines(*endPoints)
  for i in range(numSegments-1):
    line1 = lineObjects[i]
    line2 = lineObjects[i+1]
    intersectionPt = line1.intersects(line2)
    compensation.insert(intersectionPt)

#  for p in compensation.pts:
#    print "%s %s %s" % (p[0], p[1], p[1])
  error = calculateMaxError(compensation, allPts)

  return (compensation, error)
  

def doSimplifyBCompensation(allPts):
  simplified = [ p for p in allPts ]
  simplified.insert(0, (0,0))
  simplified.append((360,0))

  errorThreshold = .01
  while len(simplified) > 17:
    simplified = simplifyLines(simplified, errorThreshold)
    errorThreshold += .001

  pts = []
  seen = set()
  for pt in simplified:
    if pt[0] not in seen:
      pts.append(pt)
      seen.add(pt[0])

  bCompensation = Lines(*pts)
  error = calculateMaxError(bCompensation, allPts)

  return (bCompensation, error)

def doGreedyBCompensationIncrementally(allPts, fromError, errorIncr):
  errorThreshold = fromError
  bCompensation = doGreedyBCompensationCalculation(allPts, errorThreshold)
  error = calculateMaxError(bCompensation, allPts)

  while error[1] > errorThreshold:
    errorThreshold += errorIncr
    newBCompensation = doGreedyBCompensationCalculation(allPts, errorThreshold)
    newError = calculateMaxError(bCompensation, allPts)

    if newError[1] < error[1]:
      bCompensation = newBCompensation
      error = newError

  return (bCompensation, error)

def doGreedyBCompensationCalculation(allPts, errorThreshold):
  numPts = len(allPts)
  endPoint = (0,0)
  bCompensation = Lines((0,0), (360,0))
  currentLine = None

  maxError = calculateMaxError(bCompensation,allPts)

  startIndex = 0

  while maxError[1] > errorThreshold and bCompensation.length() < 17:
    for i in range(startIndex+2, numPts):
      pts = allPts[startIndex:i+1]
      line = bestFitLine(endPoint, pts)
      if bCompensation.length() == 16:
        currentPt = line.sample(allPts[i][0])
        bCompensation.insert(currentPt)
        error = calculateMaxError(bCompensation,allPts)
        bCompensation.remove(currentPt)
        if error[1] <= errorThreshold:
          currentLine = line
          currentIndex = i
      else:
        error = calculateMaxError(line,pts)
        if error[1] <= errorThreshold:
          currentLine = line
          currentIndex = i

    if currentLine == None:
      return bCompensation

    startIndex = currentIndex
    endPoint = currentLine.sample(allPts[currentIndex][0])
    bCompensation.insert(endPoint)
    maxError = calculateMaxError(currentLine, allPts)

    currentLine = None

  return bCompensation

def doBestFitLinesIncrementally(allPts, fromError, errorIncr):
  errorThreshold = fromError
  (bCompensation, error) = findBestFitLinesCompensationB(allPts, errorThreshold)
  numPts = len(bCompensation.pts)

  print("doBestFitLinesIncrementally", errorThreshold, error, numPts)

  while error[1] > errorThreshold or (numPts > 18 and errorThreshold < .05):
    errorThreshold += errorIncr
    (newBCompensation, newError) = findBestFitLinesCompensationB(allPts, errorThreshold)
    if newError[1] < error[1]:
      bCompensation = newBCompensation
      error = newError
    numPts = len(bCompensation.pts)
    print("doBestFitLinesIncrementally", errorThreshold, error, numPts)

  print("Sample at zero: %s" % (bCompensation.sample(0)[1],))
#  bCompensation.offset(-bCompensation.sample(0)[1])

  return (bCompensation, error)

def findBestFitLinesCompensationB(allPts, errorThreshold):
  (initialLine, negativeIndex, positiveIndex) = findBestInitialLineB(allPts, errorThreshold)

  initialPositiveIndex = positiveIndex
  lines = [ initialLine ]

  numPts = len(allPts)
  while positiveIndex < numPts+negativeIndex:
    startIndex = positiveIndex
    for i in range(startIndex+1, numPts+negativeIndex+1):
      pts = allPts[startIndex:i+1]
      if all([ (abs(d[0]-pts[0][0]) < .00001) for d in pts ]):
        continue
      line = bestFitLine(averagePoint(pts), pts)
      error = calculateMaxError(line, pts)

      if error[1] < errorThreshold:
        positiveIndex = i
        bestLine = line
    if startIndex == positiveIndex:
      # couldn't find best fit line within threshold
      pts = allPts[startIndex:numPts+negativeIndex+1]
      if not all([ (abs(d[0]-pts[0][0]) < .00001) for d in pts ]):
        bestLine = bestFitLine(averagePoint(pts), pts)
        lines.append(bestLine)
      break
    else:
      lines.append(bestLine)

  endLine = Line(initialLine.m, initialLine.b-initialLine.m*360)
  lines.append(endLine)

  points = [ ]

  for i in range(len(lines)-1):
    line1 = lines[i]
    line2 = lines[i+1]
    try:
      intersectionPt = line1.intersects(line2)
      points.append(intersectionPt)
    except:
      pass

  points = [ pt for pt in points if pt[0] >= 0 and pt[0] <= 360 ]

  if len(points) > 0:
    points.insert(0, (points[-1][0]-360, points[-1][1]))
    points.append((points[1][0]+360, points[1][1]))

    compensation = Lines(*points)
    maxError = calculateMaxError(compensation, allPts)
  else:
    compensation = Lines((1,0), (0, 359))
    maxError = calculateMaxError(compensation,allPts)

  return (compensation, maxError)


def findBestInitialLineB(allPts, errorThreshold):
  numPts = len(allPts)
  pts = [ (-360+d[0], d[1]) for d in allPts[-1:] ] + allPts[0:1]
  bestLine = bestFitLine((0,0), pts)
  bestError = calculateMaxError(bestLine, pts)

  positiveIndex = 1
  negativeIndex = -1
  alternate = True

  bestPositive = 0
  bestNegative = -1

  while positiveIndex < len(allPts)+negativeIndex:
    if alternate:
      negativeIndex -= 1
    else:
      positiveIndex += 1

    pts = [ (-360+d[0], d[1]) for d in allPts[negativeIndex:] ] + allPts[0:positiveIndex]
    line = bestFitLine((0,0), pts)
    error = calculateMaxError(line, pts)

    if error[1] < errorThreshold:
      bestError = error
      bestLine = line
      bestNegative = negativeIndex
      bestPositive = positiveIndex-1
      alternate = not alternate

  return (bestLine, bestNegative, bestPositive)


def yDistance(startPoint, endPoint, pt):
  return calculateMaxError(Line.fromTwoPoints(startPoint, endPoint), [ pt ])[1]

def distanceToSegment(startPoint, endPoint, pt):
  dx = endPoint[0]-startPoint[0]
  dy = endPoint[1]-startPoint[1]
  segmentLength = math.sqrt(dx*dx+dy*dy)

  Vx = pt[0]-startPoint[0]
  Vy = pt[1]-startPoint[1]

  t = max(0, min(1, (Vx*dx+Vy*dy)/segmentLength))

  pointOnSegmentX = startPoint[0]+t*dx
  pointOnSegmentY = startPoint[1]+t*dy

  minVx = pt[0]-pointOnSegmentX
  minVy = pt[1]-pointOnSegmentY

  return math.sqrt(minVx*minVx+minVy*minVy)

# adapted from https://github.com/AnnaMag/Line-simplification/blob/master/code/get_douglas_peucker.py
# Modified Douglas-Peucker Simplification Algorithm
# Rather than calculating an actual distance to each point, use the y distance so we're only evaluating the error.
def simplifyLines(allPts, errorThreshold):
  numPts = len(allPts)
  simplified = [ False for i in range(numPts) ]

  first = 0
  last = numPts-1

  simplified[first] = True
  simplified[last] = True

  firstList = []
  lastList = []

  while last:
    maxDist = 0
    for i in range(first+1, last):
      dist = yDistance(allPts[first], allPts[last], allPts[i])

      if dist > maxDist:
        index = i
        maxDist = dist

    if maxDist > errorThreshold:
      simplified[index] = True

      firstList.append(first)
      lastList.append(index)

      firstList.append(index)
      lastList.append(last)

    if len(firstList) == 0:
      first = None
    else:
      first = firstList.pop()

    if len(lastList) == 0:
      last = None
    else:
      last = lastList.pop()

  return [ allPts[i] for i in range(numPts) if simplified[i] ]

if __name__ == "__main__":
#  bData = processBData(readBData("1350b.csv"))
#  bData = processBData(readBData("1335b.csv"))
#  bData = processBData(readBData("1332b.csv"))
#  bData = processBData(readBData("1328b.csv"))
#  bData = processBData(readBData("1347b.csv"))
#  bData = processBData(readBData("1349b.csv"))
#  bData = processBData(readBData("1340b.csv"))
#  bData = processBData(readBData("1351b.csv"))
#  bData = processBData(readBData("1366b.csv"))
#  bData = processBData(readBData("1361b.csv"))
#  bData = processBData(readBData("1367b.csv"))
#  bData = processBData(readBData("1378b.csv"))
#  bData = processBData(readBData("1525b.csv"))
#  bData = processBData(readBData("1702b_cmm.csv"))
#  bData = processBData(readBData("1815b_cmm.csv"))
  bData = processBData(readBData("2052b.csv"))

# 1350
#  shippedCompensation = Lines((-20, .05), (85, -0.24), (195, .13), (285,.15), (340, .05), (445,-.24))

# 1335
#  shippedCompensation = Lines((-20.0, 0.05), (75, -0.18), (195, .19), (295, .12), (340, 0.05), (435, -.18))

# 1332
#  shippedCompensation = Lines((-20.0, 0.05), (80, -0.18), (195, .175), (320, .13), (340, 0.05), (440, -.18))

# 1328
#  shippedCompensation = Lines((-20, .05), (70, -0.16), (185, .23), (290, .2), (340, .05), (430, -0.16))

# 1347
#  shippedCompensation = Lines((-85, .22), (65, -0.17), (150, 0), (175,.19), (275, .22), (425,-.17))

# 1349
#  shippedCompensation = Lines((-20, .05), (75, -0.195), (185, .165), (280, .167), (340, .05), (435, -0.195))

# 1340
#  shippedCompensation = Lines((0, 0), (85, -0.159231423729), (195, 0.272716218835), (285, 0.203151808458), (360, 0))

# 1361
#  shippedCompensation = Lines((-20, 0.04), (85, -0.17), (270, 0.195), (340, 0.04))

  (bCompensation, error, bestAlgorithm) = calculateBCompensation(bData)

#  print "max error of shipped compensation table: %s" % (calculateMaxError(shippedCompensation, bData),)

  print()
  print("Best B compensation table calculated has max error of %s, using %s algorithm" % (error, bestAlgorithm))

  bCompensation.removePointsNotInDomain(0,360)
  for p in bCompensation.pts:
    print("%s %s %s" % (p[0], p[1], p[1]))

  # repeat the B compensation table from -9999 to 9999
  pts = [ p for p in bCompensation.pts ]
  for cycles in range(1,29):
    for p in pts:
      f = (p[0]+360*cycles, p[1])
      r = (p[0]-360*cycles, p[1])
      bCompensation.insert(f)
      bCompensation.insert(r)

  with open("b.comp", 'w') as bCompFile:
    for p in bCompensation.pts:
      bCompFile.write("%0.6f %0.6f %0.6f\n" % (p[0], p[1], p[1]))

#  aData = processAData(readAData("1378a.csv"))
#  (aCompensation, error) = calculateACompensation(aData)
#  print
#  print "Best A compensation table calculated has max error of %s" % (error,)
#
#  for p in aCompensation.pts:
#    print "%s %s %s" % (p[0], p[1], p[1])
