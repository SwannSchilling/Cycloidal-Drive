import adsk.core
import adsk.fusion
import traceback
import math

# All units cm

# Rotor extrude thickness
rotorThickness = 0.5 
# Rotor nominal diameter
rotorDiameter = 5.1
# Rotor bearing hole diameter
rotorBearingHoleDiameter = 2.406
# Spacing between opposing rotors
rotorSpacing = 0.1

# Camshaft cam diameter
camDiameter = 0.4

# Camshaft face diameter
camshaftDiameter = 5.96

# Camshaft reinforcement screw diameter
camshaftScrewHoleDiameter = 0.3 

# Ring gear extrude thickness
ringGearThickness = rotorThickness * 2 + rotorSpacing
# Ring gear outer diameter
ringGearOuterDiameter = rotorDiameter + 0.5
# Ring gear margin around nominal rotor diameter
ringGearMargin = 0.02
# Number of pins in ring gear
ringGearPins = 40
# Number of lobes on cycloidal rotor
rotorLobes = ringGearPins - 1
# Ring gear pin radius (ideally circumference / n_pins / 4)
ringGearPinRadius = rotorDiameter * math.pi / ringGearPins / 4
# Eccentric offset
eccentricOffset = 0.5 * ringGearPinRadius

# Rotor output hole diameter
outputHoleDiameter = 0.708
# Rotor output hole count
outputHoleCount = 8
# Rotor output hole circle diameter
outputCircleDiameter = (rotorDiameter + rotorBearingHoleDiameter) / 2 - ringGearPinRadius * 1.5

# Output pin diameter
outputPinDiameter = outputHoleDiameter - ringGearPinRadius;
# Output pin reinforcement screw diameter
outputPinScrewHoleDiameter = 0.3
# Output body thickness
outputPlateThickness = 0.3

# Calculated
rotorRadius = rotorDiameter / 2
# Maximum allowed distance between spline points
maxDist = 0.25 * ringGearPinRadius
minDist = 0.5 * maxDist # Minimum allowed distance between spline points

def getPoint(theta, rMajor, rMinor, e, n):
  psi = math.atan2(math.sin((1 - n) * theta), ((rMajor / (e * n)) - math.cos((1 - n) * theta)))
  x = (rMajor * math.cos(theta)) - (rMinor * math.cos(theta + psi)) - (e * math.cos(n * theta))
  y = (-rMajor * math.sin(theta)) + (rMinor * math.sin(theta + psi)) + (e * math.sin(n * theta))
  return (x, y)

def distance(xa, ya, xb, yb):
  return math.hypot((xa - xb), (ya - yb))

def combineFeatures_add(rootComp: adsk.fusion.Design.rootComponent, targetBody, toolBody):
    combineFeatures = rootComp.features.combineFeatures
    tools = adsk.core.ObjectCollection.create()
    tools.add(toolBody)
    input: adsk.fusion.CombineFeatureInput = combineFeatures.createInput(targetBody, tools)
    input.isNewComponent = False
    input.isKeepToolBodies = False
    input.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
    combineFeature = combineFeatures.add(input)

def rotor(design, root, invert, zOffset):
  newEccentricOffset = eccentricOffset
  offsetAngle = 0
  if invert: 
    newEccentricOffset *= -1
    offsetAngle = math.pi / rotorLobes

  rotorOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
  rotor = rotorOcc.component
  rotor.name = 'Rotor'

  planes = rotor.constructionPlanes
  planeInput = planes.createInput()
  offsetValue = adsk.core.ValueInput.createByReal(zOffset)
  planeInput.setByOffset(root.xYConstructionPlane, offsetValue)
  constructionPlane = planes.add(planeInput)

  sk = rotor.sketches.add(constructionPlane)
  points = adsk.core.ObjectCollection.create()

  (xs, ys) = getPoint(0, rotorRadius, ringGearPinRadius, eccentricOffset, ringGearPins)
  points.add(adsk.core.Point3D.create(xs, ys, 0))

  et = 2 * math.pi / rotorLobes
  (xe, ye) = getPoint(et, rotorRadius, ringGearPinRadius, eccentricOffset, ringGearPins)
  x = xs
  y = ys
  dist = 0
  ct = 0
  dt = math.pi / ringGearPins
  numPoints = 0

  while ((distance(x, y, xe, ye) > maxDist or ct < et / 2) and ct < et):
    (xt, yt) = getPoint(ct + dt, rotorRadius, ringGearPinRadius, eccentricOffset, ringGearPins)
    dist = distance(x, y, xt, yt)

    ddt = dt / 2
    lastTooBig = False
    lastTooSmall = False

    while (dist > maxDist or dist < minDist):
      if (dist > maxDist):
        if (lastTooSmall):
          ddt /= 2

        lastTooSmall = False
        lastTooBig = True

        if (ddt > dt / 2):
          ddt = dt / 2

        dt -= ddt

      elif (dist < minDist):
        if (lastTooBig):
          ddt /= 2

        lastTooSmall = True
        lastTooBig = False
        dt += ddt

      (xt, yt) = getPoint(ct + dt, rotorRadius, ringGearPinRadius, eccentricOffset, ringGearPins)
      dist = distance(x, y, xt, yt)

    x = xt
    y = yt
    points.add(adsk.core.Point3D.create(x, y, 0))
    numPoints += 1
    ct += dt

  points.add(adsk.core.Point3D.create(xe, ye, 0))
  curve = sk.sketchCurves.sketchFittedSplines.add(points)

  lines = sk.sketchCurves.sketchLines
  line1 = lines.addByTwoPoints(adsk.core.Point3D.create(0, 0, 0), curve.startSketchPoint)
  line2 = lines.addByTwoPoints(line1.startSketchPoint, curve.endSketchPoint)

  # Extrude
  prof = sk.profiles.item(0)
  dist = adsk.core.ValueInput.createByReal(rotorThickness)
  extrudes = rotor.features.extrudeFeatures
  extrude = extrudes.addSimple(prof, dist, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

  # Create component
  body1 = extrude.bodies.item(0)
  body1.name = "Rotor"
  inputEntities = adsk.core.ObjectCollection.create()
  inputEntities.add(body1)

  # Circular pattern
  zAxis = rotor.zConstructionAxis
  circularFeats = rotor.features.circularPatternFeatures
  circularFeatInput = circularFeats.createInput(inputEntities, zAxis)
  circularFeatInput.quantity = adsk.core.ValueInput.createByReal(rotorLobes)
  circularFeatInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
  circularFeatInput.isSymmetric = True
  circularFeat = circularFeats.add(circularFeatInput)

  # Combine pattern features
  ToolBodies = adsk.core.ObjectCollection.create()
  for b in circularFeat.bodies:
    ToolBodies.add(b)
  try:
    for b in ToolBodies:
      combineFeatures_add(rotor, body1, b)
  except:
    pass

  # Center bearing hole
  sk = rotor.sketches.add(constructionPlane)
  sketchCircles = sk.sketchCurves.sketchCircles
  sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), rotorBearingHoleDiameter / 2)

  prof = sk.profiles.item(0)
  dist = adsk.core.ValueInput.createByReal(rotorThickness)
  extrudes = rotor.features.extrudeFeatures
  extrude = extrudes.addSimple(prof, dist, adsk.fusion.FeatureOperations.CutFeatureOperation)

  # Output holes
  sk = rotor.sketches.add(constructionPlane)
  sketchCircles = sk.sketchCurves.sketchCircles
  sketchCircles.addByCenterRadius(adsk.core.Point3D.create(math.cos(-offsetAngle + math.pi / 2) * outputCircleDiameter / 2, math.sin(-offsetAngle + math.pi / 2) * outputCircleDiameter / 2, 0),outputHoleDiameter / 2)

  prof = sk.profiles.item(0)
  dist = adsk.core.ValueInput.createByReal(rotorThickness)
  extrudes = rotor.features.extrudeFeatures
  extrude = extrudes.addSimple(prof, dist, adsk.fusion.FeatureOperations.CutFeatureOperation)

  inputEntities = adsk.core.ObjectCollection.create()
  inputEntities.add(extrude)

  # Circular pattern
  circularFeats = rotor.features.circularPatternFeatures
  circularFeatInput = circularFeats.createInput(inputEntities, zAxis)
  circularFeatInput.quantity = adsk.core.ValueInput.createByReal(outputHoleCount)
  circularFeatInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
  circularFeatInput.isSymmetric = True
  circularFeat = circularFeats.add(circularFeatInput)

  # Offset the rotor to make the ring gear concentric with origin
  transform = rotorOcc.transform
  transform.setToRotation(offsetAngle, adsk.core.Vector3D.create(0, 0, 1), adsk.core.Point3D.create(0, 0, 0))
  transform.translation = adsk.core.Vector3D.create(newEccentricOffset, 0, 0)
  rotorOcc.transform = transform
  design.snapshots.add()

def run(context):
  ui = None

  try:
    app = adsk.core.Application.get()
    ui = app.userInterface
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent

    ui.messageBox('Ratio: ' + str((ringGearPins - rotorLobes) / rotorLobes))

    # Section: rotor
    rotor(des, root, False, 0)
    rotor(des, root, True, rotorThickness + rotorSpacing)

    # Section: camshaft
    camshaftOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    camshaft = camshaftOcc.component
    camshaft.name = 'Camshaft'

    # Cam
    sk = camshaft.sketches.add(root.xYConstructionPlane)
    sketchCircles = sk.sketchCurves.sketchCircles
    sketchCircles.addByCenterRadius(adsk.core.Point3D.create(eccentricOffset, 0, 0), camDiameter / 2)
    sketchCircles.addByCenterRadius(adsk.core.Point3D.create(eccentricOffset, 0, 0), camshaftScrewHoleDiameter / 2)

    prof = sk.profiles.item(0)
    dist = adsk.core.ValueInput.createByReal(rotorThickness)
    extrudes = camshaft.features.extrudeFeatures
    extrude = extrudes.addSimple(prof, dist, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

    # Section: output assembly
    outputOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    output = outputOcc.component
    output.name = 'Output'

    # Output pins
    sk = output.sketches.add(root.xYConstructionPlane)
    sketchCircles = sk.sketchCurves.sketchCircles
    sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, outputCircleDiameter / 2, 0), outputPinDiameter / 2)

    prof = sk.profiles.item(0)
    dist = adsk.core.ValueInput.createByReal(ringGearThickness)
    extrudes = output.features.extrudeFeatures
    extrude = extrudes.addSimple(prof, dist, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

    inputEntities = adsk.core.ObjectCollection.create()
    inputEntities.add(extrude)

    # Circular pattern
    zAxis = output.zConstructionAxis
    circularFeats = output.features.circularPatternFeatures
    circularFeatInput = circularFeats.createInput(inputEntities, zAxis)
    circularFeatInput.quantity = adsk.core.ValueInput.createByReal(outputHoleCount)
    circularFeatInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    circularFeatInput.isSymmetric = True
    circularFeat = circularFeats.add(circularFeatInput)

    # Output body
    sk = output.sketches.add(root.xYConstructionPlane)
    sketchCircles = sk.sketchCurves.sketchCircles
    sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), outputCircleDiameter / 2 + outputPinDiameter)

    prof = sk.profiles.item(0)
    dist = adsk.core.ValueInput.createByReal(-outputPlateThickness)
    extrudes = output.features.extrudeFeatures
    extrude = extrudes.addSimple(prof, dist, adsk.fusion.FeatureOperations.JoinFeatureOperation)

    # Screw holes
    sk = output.sketches.add(root.xYConstructionPlane)
    sketchCircles = sk.sketchCurves.sketchCircles
    sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, outputCircleDiameter / 2, 0), outputPinScrewHoleDiameter / 2)

    prof = sk.profiles.item(0)
    extrudeInput = extrudes.createInput(prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
    extentAll = adsk.fusion.ThroughAllExtentDefinition.create()
    deg0 = adsk.core.ValueInput.createByString('0 deg')
    extrudeInput.setTwoSidesExtent(extentAll, extentAll, deg0, deg0)
    extrudes = output.features.extrudeFeatures
    extrude = extrudes.add(extrudeInput)

    inputEntities = adsk.core.ObjectCollection.create()
    inputEntities.add(extrude)

    # Circular pattern
    zAxis = output.zConstructionAxis
    circularFeats = output.features.circularPatternFeatures
    circularFeatInput = circularFeats.createInput(inputEntities, zAxis)
    circularFeatInput.quantity = adsk.core.ValueInput.createByReal(outputHoleCount)
    circularFeatInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    circularFeatInput.isSymmetric = True
    circularFeat = circularFeats.add(circularFeatInput)

    # Section: ring gear
    ringGearOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    ringGear = ringGearOcc.component
    ringGear.name = 'Ring Gear'

    # Pins
    sk = ringGear.sketches.add(root.xYConstructionPlane)
    sketchCircles = sk.sketchCurves.sketchCircles
    centerPoint = adsk.core.Point3D.create(rotorRadius, 0, 0)
    sketchCircles.addByCenterRadius(centerPoint, ringGearPinRadius)

    prof = sk.profiles.item(0)
    dist = adsk.core.ValueInput.createByReal(ringGearThickness)
    extrudes = ringGear.features.extrudeFeatures
    extrude = extrudes.addSimple(prof, dist, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    
    pin = extrude.bodies.item(0)
    pin.name = "Pin"
    inputEntities = adsk.core.ObjectCollection.create()
    inputEntities.add(pin)

    # Circular pattern
    zAxis = ringGear.zConstructionAxis
    circularFeats = ringGear.features.circularPatternFeatures
    circularFeatInput = circularFeats.createInput(inputEntities, zAxis)
    circularFeatInput.quantity = adsk.core.ValueInput.createByReal(ringGearPins)
    circularFeatInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    circularFeatInput.isSymmetric = True
    circularFeat = circularFeats.add(circularFeatInput)

    # Housing
    sk = ringGear.sketches.add(root.xYConstructionPlane)
    sketchCircles = sk.sketchCurves.sketchCircles
    sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), rotorRadius + ringGearMargin)
    sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), ringGearOuterDiameter / 2)

    prof = sk.profiles.item(1)
    dist = adsk.core.ValueInput.createByReal(ringGearThickness)
    extrudes = ringGear.features.extrudeFeatures
    extrude = extrudes.addSimple(prof, dist, adsk.fusion.FeatureOperations.JoinFeatureOperation)

    # Fillets - conditional fillet on edges with length matching gear thickness
    fillets = ringGear.features.filletFeatures

    edgeCollection1 = adsk.core.ObjectCollection.create()
    faces = ringGear.bRepBodies.item(0).faces
    for face in faces:
      for edge in face.edges:
        if abs(edge.length - ringGearThickness) < 0.005:
          edgeCollection1.add(edge)

    radius1 = adsk.core.ValueInput.createByReal(ringGearPinRadius)
    input1 = fillets.createInput()
    input1.addConstantRadiusEdgeSet(edgeCollection1, radius1, True)
    input1.isG2 = False
    input1.isRollingBallCorner = True
    fillet1 = fillets.add(input1)

    return

  except:
    if ui:
      ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
