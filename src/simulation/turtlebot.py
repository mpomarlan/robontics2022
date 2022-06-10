import os
import sys

import pybullet as p
import time

scenario = 0
if 2 <= len(sys.argv):
    scenario = int(sys.argv[1])

outfileName = 'out.frame'
if 3 <= len(sys.argv):
    outfileName = str(sys.argv[2])

id2Name = {}

p.connect(p.GUI, options = "--opengl2")
offset = [-1.5,-1.5,0]
offset2 = [-1.5,1.5,0]

offsetOrange = [1.5,-1.5,0.001]
offsetPurple = [1.5,1.5,0.001]

turtle = p.loadURDF("turtlebot.urdf",offset)
id2Name[turtle] = 'turtle_1'
turtle2 = p.loadURDF("turtlebot_purple.urdf",offset2)
id2Name[turtle2] = 'turtle_2'
plane = p.loadURDF("plane.urdf")
id2Name[plane] = 'plane'
enclosure = p.loadURDF("enclosure.urdf")
id2Name[enclosure] = 'enclosure'
orangetile = p.loadURDF("orangetile.urdf",offsetOrange)
id2Name[orangetile] = 'orange_tile'
purpletile = p.loadURDF("purpletile.urdf",offsetPurple)
id2Name[purpletile] = 'purple_tile'


# Scenarios:
#     0: enclosure area is free, robots move to the targets
#     1: one robot has a box in front that it has to veer around
#     2: one robot pushes on a box in the center that has a ball on it
#     3: one robot pushes a box into a trap in front of the other robot

box = None
ball = None
trapWall1 = None
trapWall2 = None
sepWall1 = None
sepWall2 = None

boxOffset = [1, 1, 0.22]
ballOffset = [1, 1, 0.22+0.44]



if 1 == scenario:
    box = p.loadURDF("box.urdf", [0, 1.5, 0.22])
    id2Name[box] = 'box'
elif 2 == scenario:
    box = p.loadURDF("box.urdf", [0, 0, 0.22])
    id2Name[box] = 'box'
    ball = p.loadURDF("ball.urdf",[0, 0, 0.22+0.44])
    id2Name[ball] = 'ball'
    wall1 = p.loadURDF("wall.urdf",[-1.15,0,0])
    id2Name[wall1] = 'wall1'
    wall2 = p.loadURDF("wall.urdf",[1.15,0,0])
    id2Name[wall2] = 'wall2'
elif 3 == scenario:
    box = p.loadURDF("box.urdf", [0, 0, 0.22])
    id2Name[box] = 'box'
    wall1 = p.loadURDF("wall.urdf",[-1.15,0,0])
    id2Name[wall1] = 'wall1'
    wall2 = p.loadURDF("wall.urdf",[1.15,0,0])
    id2Name[wall2] = 'wall2'
    trapwall1 = p.loadURDF("trapwall.urdf",[-0.42,1.36,0])
    id2Name[trapwall1] = 'trapwall1'
    trapwall2 = p.loadURDF("trapwall.urdf",[0.42,1.36,0])
    id2Name[trapwall2] = 'trapwall2'

p.setRealTimeSimulation(1)

for j in range (p.getNumJoints(turtle)):
    print(p.getJointInfo(turtle,j))
forward=0
turn=0
forward2=0
turn2=0

with open(outfileName,'w') as outfile:
    _ = outfile.write(' ')

fridx = 0
while (1):

        p.setGravity(0,0,-10)
        time.sleep(1./240.)
        keys = p.getKeyboardEvents()
        leftWheelVelocity=0
        rightWheelVelocity=0
        speed=10

        leftWheelVelocity2=0
        rightWheelVelocity2=0
        speed2=10

        for k,v in keys.items():

                if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                        turn = -0.5
                if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_RELEASED)):
                        turn = 0
                if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                        turn = 0.5
                if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_RELEASED)):
                        turn = 0

                if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                        forward=1
                if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_RELEASED)):
                        forward=0
                if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                        forward=-1
                if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_RELEASED)):
                        forward=0

                if (k == 107 and (v&p.KEY_WAS_TRIGGERED)):
                        turn2 = -0.5
                if (k == 107 and (v&p.KEY_WAS_RELEASED)):
                        turn2 = 0
                if (k == 104 and (v&p.KEY_WAS_TRIGGERED)):
                        turn2 = 0.5
                if (k == 104 and (v&p.KEY_WAS_RELEASED)):
                        turn2 = 0

                if (k == 117 and (v&p.KEY_WAS_TRIGGERED)):
                        forward2=1
                if (k == 117 and (v&p.KEY_WAS_RELEASED)):
                        forward2=0
                if (k == 106 and (v&p.KEY_WAS_TRIGGERED)):
                        forward2=-1
                if (k == 106 and (v&p.KEY_WAS_RELEASED)):
                        forward2=0

        rightWheelVelocity+= (forward+turn)*speed
        leftWheelVelocity += (forward-turn)*speed

        rightWheelVelocity2+= (forward2+turn2)*speed2
        leftWheelVelocity2 += (forward2-turn2)*speed2

        p.setJointMotorControl2(turtle,0,p.VELOCITY_CONTROL,targetVelocity=leftWheelVelocity,force=1000)
        p.setJointMotorControl2(turtle,1,p.VELOCITY_CONTROL,targetVelocity=rightWheelVelocity,force=1000)

        p.setJointMotorControl2(turtle2,0,p.VELOCITY_CONTROL,targetVelocity=leftWheelVelocity2,force=1000)
        p.setJointMotorControl2(turtle2,1,p.VELOCITY_CONTROL,targetVelocity=rightWheelVelocity2,force=1000)

        fridx = fridx + 1
        if 10 <= fridx:
            fridx = 0
            retq = {}

            for k, v in id2Name.items():
                position, orientation = p.getBasePositionAndOrientation(k)
                linearVelocity, angularVelocity = p.getBaseVelocity(k)
                retq[v] = {'position': position, 'orientation': orientation, 'linearVelocity': linearVelocity, 'angularVelocity': angularVelocity}
                if v == 'turtle_1':
                    retq[v]['leftWheelVelocity'] = leftWheelVelocity
                    retq[v]['rightWheelVelocity'] = rightWheelVelocity
                if v == 'turtle_2':
                    retq[v]['leftWheelVelocity'] = leftWheelVelocity2
                    retq[v]['rightWheelVelocity'] = rightWheelVelocity2

            with open(outfileName,'a') as outfile:
                outfile.write("%s\n" % str(retq))

