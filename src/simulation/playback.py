import ast
import os
import sys

import math

import pybullet as p
import time

import silkiewf as silkie
from percsym import perceptionReasoningStep, makeReport, getReportDiff

if 1 >= len(sys.argv):
    print("Please provide a valid filename as a parameter")
    sys.exit(0)

infileName = sys.argv[1]

name2Id = {}
name2URDF = {'turtle_1': 'turtlebot.urdf', 'turtle_2': 'turtlebot_purple.urdf', 'plane': 'plane.urdf', 'enclosure': 'enclosure.urdf', 'orange_tile': 'orangetile.urdf', 'purple_tile': 'purpletile.urdf', 'wall1': 'wall.urdf', 'wall2': 'wall.urdf', 'trapwall1': 'trapwall.urdf', 'trapwall2': 'trapwall.urdf', 'box': 'box.urdf', 'ball': 'ball.urdf'}

p.connect(p.GUI, options = "--opengl2")

p.setRealTimeSimulation(1)

frames = [ast.literal_eval(x.strip()) for x in open(infileName).read().splitlines() if x.strip()]


rules = silkie.loadDFLRules('./rules.dfl')
baseTriples = [('isA', 's_1', 'SOURCE_PATH_GOAL'), ('isA', 's_2', 'SOURCE_PATH_GOAL'), ('trajector', 's_1', 'turtle_1'), ('goal', 's_1', 'orange_tile'), ('trajector', 's_2', 'turtle_2'), ('goal', 's_2', 'purple_tile')]

def getApproximateCuboidGeometry(m, M):
    dims = [a-b for a,b in zip(M,m)]
    centroid = [a+b for a,b in zip(M,m)]
    r = 0.5*min(dims)
    cs = [max(1, int(x/(2*r))) for x in dims]
    n = [a-int(b/2) for a, b in zip(centroid, cs)]
    retq = []
    for k in range(cs[0]):
        for j in range(cs[1]):
            for l in range(cs[2]):
                retq.append(((n[0] + 2*r*k, n[1] + 2*r*j, n[2] + 2*r*l), r))
    return retq

def getTrapWallApproximateGeometry():
    return getApproximateCuboidGeometry((-0.06,-0.79,0), (0.06,0.79,0.59))

def getWallApproximateGeometry():
    return getApproximateCuboidGeometry((-0.79,-0.05,0), (0.79,0.05,0.58))

def getEnclosureApproximateGeometry():
    return getApproximateCuboidGeometry((-2,-2,0), (-1.9,2,0.56)) + getApproximateCuboidGeometry((1.9,-2,0), (2,2,0.56)) + getApproximateCuboidGeometry((-1.9,-2,0), (1.9,-1.9,0.56)) + getApproximateCuboidGeometry((-1.9,1.9,0), (1.9,2,0.56))

def getFloorApproximateGeometry():
    retq = []
    cells = 10
    l = 4
    r = l/cells
    nx = -r*cells
    ny = -r*cells
    for k in range(cells):
        for j in range(cells):
            retq.append(((nx + 2*j*r, ny + 2*k*r, -0.9*r), r))
    return retq

w = {
    "ball": {"approximategeometry": [((0,0,0), 0.221)], "fn": {"centroid": (0,0,0), "up": (0,0,1), "forward": (1,0,0), "sideways": (0,1,0)}},
    "box": {"approximategeometry": [((0,0,0), math.sqrt(3*0.287*0.287))], "fn": {"centroid": (0,0,0), "up": (0,0,1), "forward": (1,0,0), "sideways": (0,1,0)}},
    "enclosure": {"approximategeometry": getEnclosureApproximateGeometry(), "fn": {"centroid": (0,0,0.291), "up": (0,0,1), "forward": (1,0,0), "sideways": (0,1,0)}},
    "floor": {"approximategeometry": getFloorApproximateGeometry(), "fn": {"centroid": (0,0,0), "up": (0,0,1), "forward": (1,0,0), "sideways": (0,1,0)}},
    "orange_tile": {"approximategeometry": [((0,0,0), math.sqrt(2*0.225*0.225))], "fn": {"centroid": (0,0,0), "up": (0,0,1), "forward": (1,0,0), "sideways": (0,1,0)}},
    "purple_tile": {"approximategeometry": [((0,0,0), math.sqrt(2*0.225*0.225))], "fn": {"centroid": (0,0,0), "up": (0,0,1), "forward": (1,0,0), "sideways": (0,1,0)}},
    "trapwall1": {"approximategeometry": getTrapWallApproximateGeometry(), "fn": {"centroid": (0,0,0.291), "up": (0,0,1), "forward": (1,0,0), "sideways": (0,1,0)}},
    "trapwall2": {"approximategeometry": getTrapWallApproximateGeometry(), "fn": {"centroid": (0,0,0.291), "up": (0,0,1), "forward": (1,0,0), "sideways": (0,1,0)}},
    "turtle_1": {"approximategeometry": [((0,0,0.0), 0.17), ((0,0,0.15), 0.17), ((0,0,0.3), 0.17)], "forward": (1,0,0), "fn": {"centroid": (0,0,0), "up": (0,0,1), "forward": (1,0,0), "sideways": (0,1,0)}},
    "turtle_2": {"approximategeometry": [((0,0,0.0), 0.17), ((0,0,0.15), 0.17), ((0,0,0.3), 0.17)], "forward": (1,0,0), "fn": {"centroid": (0,0,0), "up": (0,0,1), "forward": (1,0,0), "sideways": (0,1,0)}},
    "wall1": {"approximategeometry": getWallApproximateGeometry(), "fn": {"centroid": (0,0,0.291), "up": (0,0,1), "forward": (1,0,0), "sideways": (0,1,0)}},
    "wall2": {"approximategeometry": getWallApproximateGeometry(), "fn": {"centroid": (0,0,0.291), "up": (0,0,1), "forward": (1,0,0), "sideways": (0,1,0)}},
    "world": {"axes": {"worldUp": (0,0,1), "worldForward": (1,0,0), "worldSideways": (0,1,0)}}
}

oldReport = {}
priorTriples = []

reportStrings = []
dbgText = None

for fnum, f in enumerate(frames):
    p.setGravity(0,0,-10)
    time.sleep(1./24.)
    for k, v in f.items():
        if k not in name2Id:
            name2Id[k] = p.loadURDF(name2URDF[k])
        p.resetBasePositionAndOrientation(name2Id[k], v["position"], v["orientation"])
        p.resetBaseVelocity(name2Id[k], v["linearVelocity"], v["angularVelocity"])
    postTriples, auxNewTriples = perceptionReasoningStep(baseTriples, priorTriples, rules, f, w)
    newReport = makeReport(priorTriples, auxNewTriples)
    repDiff = getReportDiff(oldReport, newReport)
    if repDiff:
        print("====\nFRAME %d:\n%s" % (fnum, repDiff))
        reportStrings.append(repDiff)
        if 5 < len(reportStrings):
            reportStrings = reportStrings[-5:]
            if None != dbgText:
                p.removeUserDebugItem(dbgText)
            daText = ""
            for s in reportStrings:
                daText = daText + s + "\n"
            dbgText = p.addUserDebugText(daText, (-2,-2, 2), (0,0,0),1)
            print(dbgText)
    oldReport = newReport
    priorTriples = postTriples

while (1):
    p.setGravity(0,0,-10)
    time.sleep(1./24.)

