import ast
import os
import sys

import pybullet as p
import time

if 1 >= len(sys.argv):
    sys.exit(0)

infileName = sys.argv[1]

name2Id = {}
name2URDF = {'turtle_1': 'turtlebot.urdf', 'turtle_2': 'turtlebot.urdf', 'plane': 'plane.urdf', 'enclosure': 'enclosure.urdf', 'orange_tile': 'orangetile.urdf', 'purple_tile': 'purpletile.urdf', 'wall1': 'wall.urdf', 'wall2': 'wall.urdf', 'trapwall1': 'trapwall.urdf', 'trapwall2': 'trapwall.urdf', 'box': 'box.urdf', 'ball': 'ball.urdf'}

p.connect(p.GUI, options = "--opengl2")

p.setRealTimeSimulation(1)

frames = [ast.literal_eval(x.strip()) for x in open(infileName).read().splitlines() if x.strip()]


for f in frames:
    p.setGravity(0,0,-10)
    time.sleep(1./24.)
    for k, v in f.items():
        if k not in name2Id:
            name2Id[k] = p.loadURDF(name2URDF[k])
        p.resetBasePositionAndOrientation(name2Id[k], v["position"], v["orientation"])
        p.resetBaseVelocity(name2Id[k], v["linearVelocity"], v["angularVelocity"])

while (1):
    p.setGravity(0,0,-10)
    time.sleep(1./24.)

