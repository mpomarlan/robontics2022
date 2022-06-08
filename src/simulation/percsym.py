import math

import silkiewf as silkie

import pybullet as p

def _getQuestionData(qName, symbolContext, props):
    retq = {}
    for prop in props:
        if (prop in symbolContext) and (qName in symbolContext[prop]['s']):
            retq[prop] = symbolContext[prop]['s'][qName]
        else:
            retq[prop] = None
    return retq

def _getBasicQuestionData(qName, symbolContext):
    return _getQuestionData(qName, symbolContext, ['about', 'hasTheme'])

def askIsObjectMoving(qName, symbolContext, frame, w):
    questionData = _getBasicQuestionData(qName, symbolContext)
    if (None == questionData['about']):
        return None
    retq = []
    for a in questionData['about']:
        if a in frame:
            linVelocity = frame[a]['linearVelocity']
            angVel = frame[a]['angularVelocity']
            alVel = [0.1*x for x in angVel]
            n = _norm([x+y for x,y in zip(linVelocity,alVel)])
            if (0.1 < n):
                retq.append(('isMoving', a, a))
            else:
                retq.append(('-isMoving', a, a))
    return retq

def askIsBaseActuatorMoving(qName, symbolContext, frame, w):
    questionData = _getBasicQuestionData(qName, symbolContext)
    if (None == questionData['about']):
        return []
    retq = []
    for a in questionData['about']:
        if a in frame:
            leftVelocity = frame[a]['leftWheelVelocity']
            rightVelocity = frame[a]['rightWheelVelocity']
            n = abs(leftVelocity+rightVelocity)
            if (1.0 < n):
                retq.append(('isMovingBaseActuator', a, a))
            else:
                retq.append(('-isMovingBaseActuator', a, a))
    return retq

def askIsObjectMovingTowards(qName, symbolContext, frame, w):
    questionData = _getBasicQuestionData(qName, symbolContext)
    if (None == questionData['about']):
        return []
    if (None == questionData['hasTheme']):
        return []
    retq = []
    for a in questionData['about']:
        if a in frame:
            for b in questionData['hasTheme']:
                if (b in frame) and (b in w):
                    pa = frame[a]['position']
                    pb = frame[b]['position']
                    ob = frame[b]['orientation']
                    va = frame[a]['linearVelocity']
                    if 0.01 > _norm(va):
                        retq.append(('-isMovingTowards', a, b))
                        continue
                    n = _normalized(va)
                    approximateGeometry = w[b]['approximategeometry']
                    aux = False
                    for sphere in approximateGeometry:
                        center, radius = sphere
                        center = [x+y for x,y in zip(pb, p.rotateVector(ob,center))]
                        da = [x-y for x,y in zip(center,pa)]
                        aprod = _dotProduct(da, n)
                        distance = _norm([x-y for x,y in zip(da, [aprod*z for z in n])])
                        if distance <= radius:
                            aux = True
                            break
                    if aux:
                        retq.append(('isMovingTowards', a, b))
                    else:
                        retq.append(('-isMovingTowards', a, b))
    return retq

#### TODO
##def askIsRegionFreeFor(qName, symbolContext, frame, w):
##    questionData = _getBasicQuestionData(qName, symbolContext)
##    return []

def askIsInSpatialRelation(qName, symbolContext, frame, w):
    questionData = _getQuestionData(qName, symbolContext, ['about', 'hasMode', 'hasRelatum', 'hasPropertyName'])
    if (None == questionData['about']):
        return []
    if (None == questionData['hasMode']):
        return []
    if (None == questionData['hasRelatum']):
        return []
    if (None == questionData['hasPropertyName']):
        return []
    retq = []
    predMap = {'frontOf': 'inFrontOf'} ## TODO: expand number of modes
    questionData['hasMode'] = [x for x in set(questionData['hasMode']) if x in predMap.keys()]
    for a in questionData['about']:
        if (a not in frame):
            continue
        pa = frame[a]['position']
        for m in questionData['hasMode']:
            for r in questionData['hasRelatum']:
                if (r not in frame) or (r not in w):
                    continue
                pr = frame[r]['position']
                orr = frame[r]['orientation']
                for pn in questionData['hasPropertyName']:
                    if pn not in w[r]:
                        continue
                    pv = p.rotateVector(orr, w[r][pn])
                    for b in frame:
                        pb = frame[b]['position']
                        if (0 < _dotProduct([x-y for x,y in zip(pb,pr)],pv)) and (3.0 > _norm([x-y for x,y in zip(pa, pb)])) and (b not in ['enclosure', 'floor', 'plane']):
                            retq.append((predMap[m], b, a))
    return retq

def askIsInContact(qName, symbolContext, frame, w):
    questionData = _getBasicQuestionData(qName, symbolContext)
    if (None == questionData['about']):
        return []
    if (None == questionData['hasTheme']):
        return []
    retq = []
    for a in questionData['about']:
        if (a in frame) and (a in w):
            for b in questionData['hasTheme']:
                if (b in frame) and (b in w):
                    pa = frame[a]['position']
                    oa = frame[a]['orientation']
                    pb = frame[b]['position']
                    ob = frame[b]['orientation']
                    approximateGeometryA = w[a]['approximategeometry']
                    approximateGeometryB = w[b]['approximategeometry']
                    contact = False
                    for sa in approximateGeometryA:
                        ca, ra = sa
                        ca = [x+y for x,y in zip(pa,p.rotateVector(oa,ca))]
                        for sb in approximateGeometryB:
                            cb, rb = sb
                            cb = [x+y for x,y in zip(pb,p.rotateVector(ob,cb))]
                            if _norm([x-y for x,y in zip(ca,cb)]) <= ra + rb:
                                contact = True
                                break
                        if contact:
                            break
                    if contact:
                        retq.append(('inContact', a, b))
                        retq.append(('inContact', b, a))
                    else:
                        retq.append(('-inContact', a, b))
                        retq.append(('-inContact', b, a))
    return retq


fnMap = {
    'AskIsObjectMoving': askIsObjectMoving,
    'AskIsBaseActuatorMoving': askIsBaseActuatorMoving,
    'AskIsObjectMovingTowards': askIsObjectMovingTowards,
    #### TODO
    ## 'AskIsRegionFreeFor': askIsRegionFreeFor,
    'AskIsInSpatialRelation': askIsInSpatialRelation,
    'AskIsInContact': askIsInContact
}

def processQuestions(symbolContext, frame, w):
    if 'isA' not in symbolContext:
        return []
    retq = []
    isas = symbolContext['isA']
    for fn in fnMap.keys():
        if fn in isas['o']:
            for qName in isas['o'][fn]:
                retq = retq + fnMap[fn](qName, symbolContext, frame, w)
    return retq

def priorSymbolContext(triples):
    retq = {}
    for triple in triples:
        pred, s, o = triple
        if pred not in retq:
            retq[pred] = {'s': {}, 'o': {}}
        if s not in retq[pred]['s']:
            retq[pred]['s'][s] = set([])
        if o not in retq[pred]['o']:
            retq[pred]['o'][o] = set([])
        retq[pred]['s'][s].add(o)
        retq[pred]['o'][o].add(s)
    return retq

def selectionStep(triples, rules):
    facts = {}
    for t in triples:
        if t[0] not in facts:
            facts[t[0]] = silkie.PFact(t[0])
        facts[t[0]].addFact(t[1], t[2], silkie.DEFEASIBLE)
    theory, s2i, i2s, dt = silkie.buildTheory(rules, facts, {}, debugTheory=True)
    conclusions = silkie.dflInference(theory, teamDefeat=True, ambiguityPropagation=True, wellFoundedness=True)
    conclusions = silkie.idx2strConclusions(conclusions, i2s)
    conclusions = list(conclusions.defeasiblyProvable)
    return conclusions

def perceptionReasoningStep(baseTriples, priorTriples, rules, frame, w):
    questionTriples = []
    blackList = []
    for o in frame:
        questionTriples.append(('about', 'q_' + o, o))
        blackList.append('q_' + o)
    inTriples = baseTriples + priorTriples + questionTriples
    triples = selectionStep(inTriples, rules)
    ## TODO: use a more efficient indexing for the filter below
    auxiliaryNewTriples = [t for t in triples if (t not in inTriples) or ('about' == t[0])]
    symbolContext = priorSymbolContext(triples)
    postTriples = processQuestions(symbolContext, frame, w)
    return postTriples, auxiliaryNewTriples

def makeReport(perceptionTriples, auxiliaryTriples):
    retq = {'': set(perceptionTriples)}
    targets = set([])
    for t in auxiliaryTriples:
        if ('isA' == t[0]):
            targets.add(t[1])
            retq[t[1]] = set([])
    for tg in targets:
        for t in auxiliaryTriples:
            if (tg == t[1]) or (tg == t[2]):
                retq[tg].add(t)
    return retq

def getReportDiff(oldReport, newReport):
    def _infixTriples(triples):
        retq = ""
        for t in triples:
            retq = retq + t[0]+'('+t[1]+','+t[2]+'), '
        if retq:
            retq = retq[:-2]
        return retq
    ks = sorted(list(set(list(oldReport.keys()) + list(newReport.keys()))))
    retq = ""
    for k in ks:
        so = set([])
        sn = set([])
        if k in oldReport:
            so = oldReport[k]
        if k in newReport:
            sn = newReport[k]
        dAdd = sn.difference(so)
        dDel = so.difference(sn)
        if dAdd or dDel:
            if '' == k:
                retq = retq + 'PERCEPTION\n'
            else:
                retq = retq + k + '\n'
            if dAdd:
                retq = retq + "\tADD:\t" + _infixTriples(sorted(list(dAdd))) + '\n'
            if dDel:
                retq = retq + "\tDROP:\t" + _infixTriples(sorted(list(dDel))) + '\n'
    return retq

## in while frame playback:
##    postTriples, auxiliaryNewTriples = perceptionReasoningStep(baseTriples, priorTriples, rules, frame, w)
##    report {priorTriples, auxiliaryNewTriples} => postTriples in template: because priorTriples, then auxiliaryNewTriples (incl. questions) [which produce answers postTriples]
##    priorTriples = postTriples

def _dotProduct(a, b):
    prod = [x*y for x,y in zip(a,b)]
    retq = 0
    for e in prod:
        retq = retq + e
    return retq

def _norm(v):
    return math.sqrt(_dotProduct(v,v))

def _normalized(v):
    n = _norm(v)
    if 0.000001 > n:
        return tuple([0]*len(v))
    return tuple([x/n for x in v])

