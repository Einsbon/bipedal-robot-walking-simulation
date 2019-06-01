"""
bipedal robot walking simulation

by Einsbon (Sunbin Kim)
- GitHub: https://github.com/Einsbon
- Youtube:  https://www.youtube.com/channel/UCt7FZ-8uzV_jHJiKp3NlHvg
- Blog: https://blog.naver.com/einsbon
"""

import pybullet as p
import time
from time import sleep
import pybullet_data
import numpy as np
import math
import os

import motorController
import walkGenerator

# motor setting
motor_kp = 0.5
motor_kd = 0.5
motor_torque = 2
motor_max_velocity = 10.0

# physics parameter setting
fixedTimeStep = 1. / 1000
numSolverIterations = 200

physicsClient = p.connect(p.GUI)
p.setTimeStep(timeStep=fixedTimeStep, physicsClientId=physicsClient)
p.setPhysicsEngineParameter(numSolverIterations=numSolverIterations)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # to load ground

p.setGravity(0, 0, 0)
p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=10, cameraPitch=-5, cameraTargetPosition=[0.3, 0.5, 0.1], physicsClientId=physicsClient)

planeId = p.loadSDF('stadium.sdf')  # or p.loadURDF('samurai.urdf')  # p.loadURDF('plane.urdf')

robot = p.loadURDF(os.path.abspath(os.path.dirname(__file__)) + '/humanoid_leg_12dof.8.urdf', [0, 0, 0.31],
                   p.getQuaternionFromEuler([0, 0, 0]),
                   useFixedBase=False)

controller = motorController.MotorController(robot, physicsClient, fixedTimeStep, motor_kp, motor_kd, motor_torque, motor_max_velocity)

walk = walkGenerator.WalkGenerator()
walk.setWalkParameter(bodyMovePoint=8,
                      legMovePoint=8,
                      height=50,
                      stride=90,
                      sit=40,
                      swayBody=30,
                      swayFoot=0,
                      bodyPositionForwardPlus=5,
                      swayShift=3,
                      liftPush=0.4,
                      landPull=0.6,
                      timeStep=0.06,
                      damping=0.0,
                      incline=0.0)
walk.generate()
walk.inverseKinematicsAll()

actionTime = walk._timeStep
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)
controller.setMotorsAngleInFixedTimestep(walk.walkAnglesStartRight[0], 1, 0)

waitTime = 1
repeatTime = int(waitTime / fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

p.setGravity(0, 0, -9.8)

# walk 8 steps
# start walking. right foot step
for i in range(np.size(walk.walkAnglesStartRight, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkAnglesStartRight[i], actionTime, 0)
for i in range(2):
    # left foot step
    for i in range(np.size(walk.walkAnglesWalkingLeft, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkAnglesWalkingLeft[i], actionTime, 0)
    # right foot step
    for i in range(np.size(walk.walkAnglesWalkingRight, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkAnglesWalkingRight[i], actionTime, 0)
# end walking. left
for i in range(np.size(walk.walkAnglesEndLeft, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkAnglesEndLeft[i], actionTime, 0)

# rest 2 seconds
waitTime = 2
repeatTime = int(waitTime / fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

########################################################
p.resetBasePositionAndOrientation(robot, [0, 0, 0.31], p.getQuaternionFromEuler([0, 0, 0]))
walk.setWalkParameter(bodyMovePoint=8,
                      legMovePoint=8,
                      height=50,
                      stride=90,
                      sit=70,
                      swayBody=30,
                      swayFoot=0,
                      bodyPositionForwardPlus=5,
                      swayShift=3,
                      liftPush=0.4,
                      landPull=0.6,
                      timeStep=0.06,
                      damping=0.0,
                      incline=0.0)
walk.generate()
walk.inverseKinematicsAll()
actionTime = walk._timeStep
controller.setMotorsAngleInFixedTimestep(walk.walkAnglesStartRight[0], 2, 0)

waitTime = 1
repeatTime = int(waitTime / fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

p.setGravity(0, 0, -9.8)

for i in range(np.size(walk.walkAnglesStartRight, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkAnglesStartRight[i], actionTime, 0)
for i in range(2):
    for i in range(np.size(walk.walkAnglesWalkingLeft, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkAnglesWalkingLeft[i], actionTime, 0)
    for i in range(np.size(walk.walkAnglesWalkingRight, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkAnglesWalkingRight[i], actionTime, 0)
for i in range(np.size(walk.walkAnglesEndLeft, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkAnglesEndLeft[i], actionTime, 0)

waitTime = 2
repeatTime = int(waitTime / fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

########################################################
p.resetBasePositionAndOrientation(robot, [0, 0, 0.31], p.getQuaternionFromEuler([0, 0, 0]))

walk.setWalkParameter(bodyMovePoint=8,
                      legMovePoint=8,
                      height=20,
                      stride=40,
                      sit=40,
                      swayBody=15,
                      swayFoot=0,
                      bodyPositionForwardPlus=5,
                      swayShift=3,
                      liftPush=0.4,
                      landPull=0.6,
                      timeStep=0.03,
                      damping=0.0,
                      incline=0.0)
walk.generate()
walk.inverseKinematicsAll()
actionTime = walk._timeStep
controller.setMotorsAngleInFixedTimestep(walk.walkAnglesStartRight[0], 2, 0)

waitTime = 1
repeatTime = int(waitTime / fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

for i in range(np.size(walk.walkAnglesStartRight, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkAnglesStartRight[i], actionTime, 0)
for i in range(2):
    for i in range(np.size(walk.walkAnglesWalkingLeft, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkAnglesWalkingLeft[i], actionTime, 0)
    for i in range(np.size(walk.walkAnglesWalkingRight, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkAnglesWalkingRight[i], actionTime, 0)
for i in range(np.size(walk.walkAnglesEndLeft, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkAnglesEndLeft[i], actionTime, 0)

waitTime = 2
repeatTime = int(waitTime / fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

########################################################
p.resetBasePositionAndOrientation(robot, [0, 0, 0.31], p.getQuaternionFromEuler([0, 0, 0]))

walk.setWalkParameter(bodyMovePoint=8,
                      legMovePoint=8,
                      height=50,
                      stride=140,
                      sit=40,
                      swayBody=35,
                      swayFoot=0,
                      bodyPositionForwardPlus=-2,
                      swayShift=3,
                      liftPush=0.4,
                      landPull=0.6,
                      timeStep=0.12,
                      damping=0.0,
                      incline=0.0)
walk.generate()
walk.inverseKinematicsAll()
actionTime = walk._timeStep
controller.setMotorsAngleInFixedTimestep(walk.walkAnglesStartRight[0], 2, 0)

waitTime = 1
repeatTime = int(waitTime / fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

for i in range(np.size(walk.walkAnglesStartRight, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkAnglesStartRight[i], actionTime, 0)
for i in range(2):
    for i in range(np.size(walk.walkAnglesWalkingLeft, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkAnglesWalkingLeft[i], actionTime, 0)
    for i in range(np.size(walk.walkAnglesWalkingRight, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkAnglesWalkingRight[i], actionTime, 0)
for i in range(np.size(walk.walkAnglesEndLeft, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkAnglesEndLeft[i], actionTime, 0)

waitTime = 2
repeatTime = int(waitTime / fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

########################################################
p.resetBasePositionAndOrientation(robot, [0, 0, 0.31], p.getQuaternionFromEuler([0, 0, 0]))

walk.setWalkParameter(bodyMovePoint=8,
                      legMovePoint=8,
                      height=40,
                      stride=70,
                      sit=40,
                      swayBody=30,
                      swayFoot=0,
                      bodyPositionForwardPlus=-35,
                      swayShift=3,
                      liftPush=0.4,
                      landPull=0.6,
                      timeStep=0.06,
                      damping=0.0,
                      incline=0.0)
walk.generate()
walk.inverseKinematicsAll()
actionTime = walk._timeStep
controller.setMotorsAngleInFixedTimestep(walk.walkAnglesStartRight[0], 2, 0)

waitTime = 1
repeatTime = int(waitTime / fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

for i in range(np.size(walk.walkAnglesStartRight, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkAnglesStartRight[i], actionTime, 0)
for i in range(2):
    for i in range(np.size(walk.walkAnglesWalkingLeft, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkAnglesWalkingLeft[i], actionTime, 0)
    for i in range(np.size(walk.walkAnglesWalkingRight, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkAnglesWalkingRight[i], actionTime, 0)
for i in range(np.size(walk.walkAnglesEndLeft, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkAnglesEndLeft[i], actionTime, 0)

waitTime = 2
repeatTime = int(waitTime / fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

########################################################
p.resetBasePositionAndOrientation(robot, [0, 0, 0.28], p.getQuaternionFromEuler([0, 0, 0]))

walk.setWalkParameter(bodyMovePoint=8,
                      legMovePoint=8,
                      height=50,
                      stride=-90,
                      sit=40,
                      swayBody=30,
                      swayFoot=0,
                      bodyPositionForwardPlus=0,
                      swayShift=3,
                      liftPush=0.4,
                      landPull=0.6,
                      timeStep=0.06,
                      damping=0.0,
                      incline=0.0)
walk.generate()
walk.inverseKinematicsAll()
actionTime = walk._timeStep
controller.setMotorsAngleInFixedTimestep(walk.walkAnglesStartRight[0], 2, 0)

waitTime = 1
repeatTime = int(waitTime / fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

for i in range(np.size(walk.walkAnglesStartRight, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkAnglesStartRight[i], actionTime, 0)
for i in range(2):  # repeat twice
    # left foot step
    for i in range(np.size(walk.walkAnglesWalkingLeft, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkAnglesWalkingLeft[i], actionTime, 0)
    for i in range(np.size(walk.walkAnglesWalkingRight, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkAnglesWalkingRight[i], actionTime, 0)
for i in range(np.size(walk.walkAnglesEndLeft, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkAnglesEndLeft[i], actionTime, 0)

waitTime = 2
repeatTime = int(waitTime / fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

########################################################
p.resetBasePositionAndOrientation(robot, [0, 0, 0.31], p.getQuaternionFromEuler([0, 0, 0]))

walk.setWalkParameter(bodyMovePoint=8,
                      legMovePoint=8,
                      height=50,
                      stride=0,
                      sit=40,
                      swayBody=30,
                      swayFoot=0,
                      bodyPositionForwardPlus=5,
                      swayShift=3,
                      liftPush=0.4,
                      landPull=0.6,
                      timeStep=0.06,
                      damping=0.0,
                      incline=0.0)
walk.generate()
walk.inverseKinematicsAll()
actionTime = walk._timeStep
controller.setMotorsAngleInFixedTimestep(walk.walkAnglesStartRight[0], 2, 0)

waitTime = 1
repeatTime = int(waitTime / fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

# Turn function is not accurate yet.
for i in range(np.size(walk.walkAnglesStartRight, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkAnglesStartRight[i] + walk.turnListUnfold[i] * 0.3, actionTime, 0)
for i in range(3):
    for i in range(np.size(walk.walkAnglesWalkingLeft, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkAnglesWalkingLeft[i] + walk.turnListFold[i] * 0.3, actionTime, 0)
    for i in range(np.size(walk.walkAnglesWalkingRight, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkAnglesWalkingRight[i] + walk.turnListUnfold[i] * 0.3, actionTime, 0)
for i in range(np.size(walk.walkAnglesEndLeft, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkAnglesEndLeft[i] + walk.turnListFold[i] * 0.3, actionTime, 0)

waitTime = 2
repeatTime = int(waitTime / fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

########################################################
p.resetBasePositionAndOrientation(robot, [0, 0, 0.31], p.getQuaternionFromEuler([0, 0, 0]))

walk.setWalkParameter(bodyMovePoint=8,
                      legMovePoint=8,
                      height=50,
                      stride=90,
                      sit=40,
                      swayBody=30,
                      swayFoot=0,
                      bodyPositionForwardPlus=5,
                      swayShift=3,
                      liftPush=0.4,
                      landPull=0.6,
                      timeStep=0.06,
                      damping=0.0,
                      incline=0.0)
walk.generate()
walk.inverseKinematicsAll()
actionTime = walk._timeStep
controller.setMotorsAngleInFixedTimestep(walk.walkAnglesStartRight[0], 2, 0)

waitTime = 1
repeatTime = int(waitTime / fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

for i in range(np.size(walk.walkAnglesStartRight, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkAnglesStartRight[i] + walk.turnListUnfold[i] * 0.3, actionTime, 0)
for i in range(3):
    for i in range(np.size(walk.walkAnglesWalkingLeft, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkAnglesWalkingLeft[i] + walk.turnListFold[i] * 0.3, actionTime, 0)
    for i in range(np.size(walk.walkAnglesWalkingRight, 0)):
        controller.setMotorsAngleInFixedTimestep(walk.walkAnglesWalkingRight[i] + walk.turnListUnfold[i] * 0.3, actionTime, 0)
for i in range(np.size(walk.walkAnglesEndLeft, 0)):
    controller.setMotorsAngleInFixedTimestep(walk.walkAnglesEndLeft[i] + walk.turnListFold[i] * 0.3, actionTime, 0)

########################################################
fixedTimeStep = 1 / 500
p.setTimeStep(fixedTimeStep)

giantRobot = p.loadURDF(os.path.abspath(os.path.dirname(__file__)) + '/humanoid_leg_12dof.8.urdf', [-4.7, 0, 3.1],
                        p.getQuaternionFromEuler([0, 0, 0]),
                        useFixedBase=False,
                        globalScaling=10)

for i in range(p.getNumJoints(giantRobot)):
    p.changeDynamics(
        giantRobot,
        i,
        lateralFriction=1,
        spinningFriction=1,
    )

waitTime = 2
repeatTime = int(waitTime / fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

controller2 = motorController.MotorController(giantRobot, physicsClient, fixedTimeStep, motor_kp, motor_kd, 50, motor_max_velocity)
walk.setWalkParameter(bodyMovePoint=12,
                      legMovePoint=12,
                      height=30,
                      stride=90,
                      sit=35,
                      swayBody=18,
                      swayFoot=0,
                      bodyPositionForwardPlus=5,
                      swayShift=4,
                      liftPush=0.7,
                      landPull=0.8,
                      timeStep=0.06,
                      damping=0.0,
                      incline=0.0)
walk.generate()
walk.inverseKinematicsAll()
walk.showGaitPoint3D()
actionTime = walk._timeStep
controller2.setMotorsAngleInFixedTimestep(walk.walkAnglesStartRight[0], 4, 0)

waitTime = 1
repeatTime = int(waitTime / fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

for i in range(np.size(walk.walkAnglesStartRight, 0)):
    controller2.setMotorsAngleInFixedTimestep(walk.walkAnglesStartRight[i], actionTime, 0)
for i in range(2):
    for i in range(np.size(walk.walkAnglesWalkingLeft, 0)):
        controller2.setMotorsAngleInFixedTimestep(walk.walkAnglesWalkingLeft[i], actionTime, 0)
    for i in range(np.size(walk.walkAnglesWalkingRight, 0)):
        controller2.setMotorsAngleInFixedTimestep(walk.walkAnglesWalkingRight[i], actionTime, 0)
for i in range(np.size(walk.walkAnglesEndLeft, 0)):
    controller2.setMotorsAngleInFixedTimestep(walk.walkAnglesEndLeft[i], actionTime, 0)

controller2.setMotorsAngleInFixedTimestep(walk.inverseKinematicsPoint([-10, -40, 30], [-10, -40, 30]), 1, 0.5)
controller2.setMotorsAngleInFixedTimestep(walk.inverseKinematicsPoint([-80, -35, 50], [-10, -40, 30]), 1.5, 0.5)
controller2.setMotorsAngleInFixedTimestep(walk.inverseKinematicsPoint([-10, -35, 45], [0, -40, 30]), 0.4, 0)
controller2.setMotorsAngleInFixedTimestep(walk.inverseKinematicsPoint([60, -35, 45], [0, -40, 30]), 0.2, 3)
controller2.setMotorsAngleInFixedTimestep(walk.inverseKinematicsPoint([0, -32, 45], [0, -32, 30]), 1, 0)
controller2.setMotorsAngleInFixedTimestep(walk.inverseKinematicsPoint([0, 0, 10], [0, 0, 10]), 0.5, 50)
