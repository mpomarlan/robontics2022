import math
import numbers


def checkSupport(relPair, symbolContext, frame, w):
    supporter, supportee = relPair
    teeApproxGeometry = w.getBodyProperty((supportee, "fn"), "approximategeometry")
    falling = checkFalling(supportee, symbolContext, frame, w)
    if falling:
        if None == supporter:
            return []
        return False
    if None == falling:
        return None
    teeContacts = frame[supportee]
    supporters = []
    for c in teeContacts:
        if belowApproximateGeometry(c, teeApproxGeometry):
            supporters.append(c['object'])
    if None == supporter:
        return supporters
    return supporter in supporters


def inRegion(regionName, symbolContext, frame, w):
    if regionName not in w:
        return None
    regApproxGeometry = w.getBodyProperty((regionName, 'fn'), "approximategeometry")
    retq = []
    for o in frame:
        if o in whitelist:
            continue
        oApproxGeometry = w.getBodyProperty((o, 'fn'), "approximategeometry")
        position = frame[o]['position']
        orientation = frame[o]['orientation']
        if (None != o) and (None != oApproxGeometry) and overlappingApproximateGeometries(positionApproximateGeometry(oApproxGeometry, position, orientation), regApproxGeometry):
            retq.append(o)
    return retq

def checkRelativeLocation(question, symbolContext, frame, w):
    if ('hasMode' not in symbolContext) or (question not in symbolContext['hasMode']['s']) or ([] == symbolContext['hasMode']['s']['question']):
        return None
    if ('about' not in symbolContext) or (question not in symbolContext['about']['s']):
        return None
    elaborations = getQuestionElaborations([question], symbolContext)
    trajectors = symbolContext['about']['s'][question]
    mode = symbolContext['hasMode']['s']['question'][0]
    aux = {'below': 'hasDownAxis', 'above': 'hasUpAxis', 'frontOf': 'hasForwardAxis', 'behind': 'hasBackAxis', 'leftOf': 'hasLeftAxis', 'rightOf': 'hasRightAxis'}
    if mode not in aux:
        return None
    relevantAxisProperty = aux[mode]
    relevantAxis = None
    relata = []
    if 'hasRelatum' in symbolContext or relevantAxisProperty in symbolContext:
        for e in elaborations:
            if e in symbolContext['hasRelatum']['s']:
                relata = relata + symbolContext['hasRelatum']['s'][e]
            if e in symbolContext[relevantAxisProperty]['s']:
                relevantAxis = interpretAxis(symbolContext[relevantAxisProperty]['s'][e], symbolContext, frame, w)
    if None == relevantAxis:
        return None
    retq = []
    if relata:
        for r in relata:
            for t in trajectors:
                retq.append((mode, t, r, checkPlacementByAxis((relevantAxis, t, r), symbolContext, frame, w)))
    else:
        for t in trajectors:
            fo = findFromObjectAndAxis((relevantAxis, t), symbolContext, frame, w)
            if None != fo:
                for foo in fo:
                    retq.append((mode, t, foo, True))
    return list(set(retq))

def isObjectMoving(objName, symbolContext, frame, w):
    aboutObj = []
    if ('about' in symbolContext) and (objName in symbolContext['about']['o']):
        candQs = symbolContext['about'][o][objName]
        aboutObj = list(set([q for q in candQs if (q in symbolContext['isA']['s']) and ('moving' in symbolContext['isA']['s'][q])]))
    elaborations = getQuestionElaborations(aboutObj, symbolContext)
    relataTowards = []
    relataFrom = []
    relataStill = []
    for e in elaborations:
        if ('hasMode' in symbolContext) and ('hasRelatum' in symbolContext) and
            (e in symbolContext['hasMode']['s']) and (e in symbolContext['hasRelatum']['s']):
            if 'towards' in symbolContext['hasMode']['s'][e]:
                relataTowards = relataTowards + symbolContext['hasRelatum']['s'][e]
            if 'from' in symbolContext['hasMode']['s'][e]:
                relataFrom = relataFrom + symbolContext['hasRelatum']['s'][e]
            if 'stillness' in symbolContext['hasMode']['s'][e]:
                relataStill = relataStill + symbolContext['hasRelatum']['s'][e]
    relataTowards = list(set(relataTowards))
    relataFrom = list(set(relataFrom))
    relataStill = list(set(relataStill))
    absoluteMovement = False
    objVel = None
    if (objName in frame) and ('velocity' in frame[objName]):
        objVel = frame[objName]['velocity']
    if None == objVel:
        return None
    absoluteMovement = (0 < abs(norm(objVel)))
    if (not relataTowards) and (not relataFrom) and (not relataStill):
        return absoluteMovement
    retq = []
    for r in relataTowards:
        retq.append(('towards', objName, r, checkMovementTowards((objName, r), symbolContext, frame, w)))
    for r in relataFrom:
        retq.append(('from', objName, r, checkMovementFrom((objName, r), symbolContext, frame, w)))
    for r in relataStill:
        retq.append(('stillness', objName, r, checkMovementAbsence((objName, r), symbolContext, frame, w)))
    return retq

def isRobotMoving(robName, symbolContext, frame, w):
    retq = None
    baseActuatorVelocity = w.getBodyProperty((robName, 'fn'), 'baseactuatorvelocity')
    if None != baseActuatorVelocity:
        threshold = 0
        if "baseActuatorVelocityThreshold" in symbolContext:
            if robName in symbolContext["baseActuatorVelocityThreshold"]["s"]:
                threshold = max([x for x in symbolContext["baseActuatorVelocityThreshold"]["s"][robName] if isinstance(x, number.Number)])
        retq = (threshold < abs(baseActuatorVelocity))
    return retq


## Auxiliary functions, should rename so that the identifiers start with _



def dotProduct(a, b):
    prod = [x*y for x,y in zip(a,b)]
    retq = 0
    for e in prod:
        retq = retq + e
    return retq

def norm(v):
    return math.sqrt(dotProduct(v,v))

def normalized(v):
    n = norm(v)
    if 0.000001 > n:
        return tuple([0]*len(v))
    return tuple([x/n for x in v])

def crossProduct(a, b):
    return (a[1]*b[2]-b[1]*a[2], a[2]*b[0]-b[2]*a[0], a[0]*b[1]-b[0]*a[1])

def checkFalling(objName, symbolContext, frame, w)
    if objName not in frame:
        return None
    teeVel = frame[objName]['velocity']
    return (0 <= teeVel[2])

def belowApproximateGeometry(contact, approxGeometry):
    point = contact['point']
    for s in approxGeometry:
        center, radius = s
        if (distance(center, point) < radius) and (point[2] < center[2]):
            return True
    return False

def positionApproximateGeometry(geometry, position, orientation):
    return [(tuple([a+b for a,b in zip(position, p.rotateVector(orientation, x[0]))]), x[1]) for x in geometry]

def overlappingApproximateGeometries(a, b):
    for sa in a:
        for sb in b:
            pa, ra = sa
            pb, rb = sb
            if distance(pa, pb) <= (ra + rb):
                return True
    return False

def getQuestionElaborations(questions, symbolContext):
    if 'elaboration' not in symbolContext:
        return None
    retq = []
    for q in questions:
        if q in symbolContext['elaboration']['o']:
            retq = retq + symbolContext['elaboration']['o'][q]
    return list(set(retq))

def _getFeatureEnds(relSpec, symbolConext, frame, w):
    endName, axisName = relSpec
    candidates = symbolContext[endName][]
    if ('hasRelatum' in symbolContext) and ('hasPropertyName' in symbolContext):
        for c in candidates:
            if (c in symbolContext['hasRelatum']['s']) and (c in symbolContext['hasPropertyName']['s']):
                relata = symbolContext['hasRelatum']['s'][c]
                propName = symbolContext['hasPropertyName']['s'][c]
                for r in relata:
                    if r not in frame:
                        continue
                    orientation = frame[r]['orientation']
                    position = frame[r]['position']
                    for pn in propName:
                        featurePos = w.getBodyProperty((r, 'fn'), pn)
                        if None != featurePos:
                            return tuple([a+b for a,b in zip(position,p.rotateVector(orientation, featurePos))])
    return None

def _basicInterpretAxis(axisName, symbolContext, frame, w):
    retq = w.getBodyProperty(('worldAxes', 'fn'), axisName)
    if None != retq:
        return retq
    if ('objectRelativeAxis' in symbolContext) and (axisName in symbolContext['objectRelativeAxis']['s']):
        if ('hasRelatum' in symbolContext) and (axisName in symbolContext['hasRelatum']['s']) and ('hasPropertyName' in symbolContext) and (axisName in symbolContext['hasPropertyName']['s']):
            relata = symbolContext['hasRelatum']['s'][axisName]
            propNames = symbolContext['hasPropertyName']['s'][axisName]
            candidates = []
            for r in relata:
                if r not in frame:
                    continue
                orientation = frame[r]['orientation']
                for pn in propNames:
                    pVal = w.getBodyProperty((r, 'fn'), pn)
                    if None != pVal:
                        candidates.append(tuple(p.rotateVector(orientation, pVal)))
            if [] != candidates:
                return candidates[0]
    if ('featurePairAxis' in symbolContext) and (axisName in symbolContext['featurePairAxis']['s']):
        if ('featureFrom' in symbolContext) and ('featureTo' in symbolContext) and (axisName in symbolContext['featureFrom']['s']) and (axisName in symbolContext['featureTo']['s'])
            fromEnds = _getFeatureEnds(('featureFrom', axisName), symbolConext, frame, w)
            toEnds = _getFeatureEnds(('featureTo', axisName), symbolConext, frame, w)
            if isinstance(fromEnds, list) and isinstance(toEnds, list) and ([] != fromEnds) and ([] != toEnds):
                fromEnd = fromEnds[0]
                toEnd = toEnds[0]
                return tuple([x-y for x,y in zip(toEnd, fromEnd)])
    return None

def interpretAxis(axisName, symbolContext, frame, w, axisStack=[]):
    retq = _basicInterpretAxis(axisName, symbolContext, frame, w)
    if None != retq:
        return retq
    if ('alignedWith' in symbolContext):
        axisStack = axisStack + [axisName]
        candidates = []
        if axisName in symbolContext['alignedWith']['s']:
            candidates = symbolContext['alignedWith']['s'][axisName]
        if axisName in symbolContext['alignedWith']['o']:
            candidates = candidates + symbolContext['alignedWith']['o'][axisName]
        for newAxis in candidates:
            if newAxis in axisStack:
                continue
            retq = _basicInterpretAxis(newAxis, symbolContext, frame, w)
            if None != retq:
                return retq
        for newAxis in candidates:
            if newAxis in axisStack:
                continue
            answers = interpretAxis(newAxis, symbolContext, frame, w, axisStack=axisStack)
            if None != answers:
                if isinstance(answers, tuple):
                    return answers
                if isinstance(answers, list) and ([] != answers):
                    return answers[0]
    return None

def checkPlacementByAxis(relSpec, symbolContext, frame, w):
    axis, t, r = relSpec
    if (t not in frame) or (r not in frame):
        return None
    tApproxGeometry = w.getBodyProperty((t, "fn"), "approximategeometry")
    rApproxGeometry = w.getBodyProperty((r, "fn"), "approximategeometry")
    if None not in [tApproxGeometry, rApproxGeometry]:
        tPos = frame[t]['position']
        tOrientation = frame[t]['orientation']
        rPos = frame[r]['position']
        rOrientation = frame[r]['orientation']
        tApproxGeometry = [(tuple([a+b for a,b in zip(tPos, p.rotateVector(tOrientation, x[0]))]), x[1]*x[1]*x[1]) for x in tApproxGeometry]
        rApproxGeometry = [(tuple([a+b for a,b in zip(rPos, p.rotateVector(rOrientation, x[0]))]), x[1]*x[1]*x[1]) for x in rApproxGeometry]
        retq = 0
        for st in tApproxGeometry:
            for sr in rApproxGeometry:
                aux = dotProduct(axis, [a-b for a,b in zip(st[0], sr[0])])
                if 0 < aux:
                    retq = retq + st[1]*sr[1]
                elif 0 > aux:
                    retq = retq - st[1]*sr[1]
        return (0 < retq)
    return None

def findFromObjectAndAxis(relSpec, symbolContext, frame, w):
    relevantAxis, t = relSpec
    tApproxGeometry = w.getBodyProperty((t, 'fn'), 'approximategeometry')
    if t not in frame:
        return None
    if None == tApproxGeometry:
        return None
    tPos = frame[t]['position']
    tOrientation = frame[t]['orientation']
    tApproxGeometry = [(tuple([a+b for a,b in zip(tPos, p.rotateVector(tOrientation, x[0]))]), x[1]*x[1]*x[1]) for x in tApproxGeometry]
    candidates = []
    for n in frame:
        if t != n:
            aux = [a-b for a,b in zip(tPos, frame[n]['position'])]
            if 0 < dotProduct(relevantAxis, aux):
                candidates.append((norm(aux), n))
    candidates = sorted(candidates)
    retq = []
    for c in candidates:
        rApproxGeometry = w.getBodyProperty((c[1], 'fn'), 'approximategeometry')
        rApproxGeometry = [(tuple([a+b for a,b in zip(rPos, p.rotateVector(rOrientation, x[0]))]), x[1]*x[1]*x[1]) for x in rApproxGeometry]
        x = 0
        for st in tApproxGeometry:
            for sr in rApproxGeometry:
                aux = dotProduct(relevantAxis, [a-b for a,b in zip(st[0], sr[0])])
                if 0 < aux:
                    x = x + st[1]*rt[1]
                elif 0 > aux:
                    x = x - st[1]*rt[1]
        if 0 < x:
            retq.append(c[1])
            break
    return retq

def getNetVolumeMovement(relPair, symbolContext, frame, w)
    trajector, relatum = relPair
    if (trajector not in frame) or (relatum not in frame):
        return None
    tAG = w.getBodyProperty((trajector, 'fn'), 'approximategeometry')
    rAG = w.getBodyProperty((relatum, 'fn'), 'approximategeometry')
    retq = None
    if None not in [tAG, rAG]:
        tPos = frame[trajector]['position']
        tOrientation = frame[trajector]['orientation']
        tVel = frame[trajector]['linearVelocity']
        taVel = frame[trajector]['angularVelocity']
        rPos = frame[relatum]['position']
        rOrientation = frame[relatum]['orientation']
        rVel = frame[relatum]['linearVelocity']
        raVel = frame[relatum]['angularVelocity']
        #### TODO: decide whether using angular velocity in body frame (then below is correct) or world frame (need an extra rotation in the crossproduct argument x[0])
        tAG = [(tuple([a+b for a,b in zip(tPos, p.rotateVector(tOrientation, x[0]))]), tuple([a+b for a,b in zip(tVel, crossProduct(taVel, x[0]))]), x[1]*x[1]*x[1]) for x in tAG]
        rAG = [(tuple([a+b for a,b in zip(rPos, p.rotateVector(rOrientation, x[0]))]), tuple([a+b for a,b in zip(rVel, crossProduct(raVel, x[0]))]), x[1]*x[1]*x[1]) for x in rAG]
        retq = 0
        for t in tAG:
            for r in rAG:
                axis = normalized([a-b for a,b in zip(r[0], t[0])])
                x = dotProduct(axis, [a-b for a, b in zip(t[1], r[1])])
                if 0 < x:
                    retq = retq + t[2]*r[2]
                elif 0 > x:
                    retq = retq - t[2]*r[2]
    return retq

def checkMovementTowards(relPair, symbolContext, frame, w):
    trajector, relatum = relPair
    netVolumeMovement = getNetVolumeMovement(relPair, symbolContext, frame, w)
    if None != netVolumeMovement:
        return (0 < netVolumeMovement)
    return None

def checkMovementFrom(relPair, symbolContext, frame, w):
    trajector, relatum = relPair
    netVolumeMovement = getNetVolumeMovement(relPair, symbolContext, frame, w)
    if None != netVolumeMovement:
        return (0 > netVolumeMovement)
    return None
	
def checkMovementAbsence(relPair, symbolContext, frame, w):
    trajector, relatum = relPair
    netVolumeMovement = getNetVolumeMovement(relPair, symbolContext, frame, w)
    if None != netVolumeMovement:
        return (0 == netVolumeMovement)
    return None


