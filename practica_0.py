import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")

euler_angles = [0,0,0]
startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [0,0,1]

robotId = p.loadURDF("robotin.urdf", startPosition, startOrientation)

numJoints = p.getNumJoints(robotId)
print("NumJoints" + str(numJoints))

frictionId = p.addUserDebugParameter("jointFriction", 0, 10, 5)
torqueId = p.addUserDebugParameter("jointTorque", -10, 10, 5)
spinId = p.addUserDebugParameter("spinVelocity", -10, 10, 0)

for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robotId,j)[0],p.getJointInfo(robotId,j)[1].decode("utf-8")))


while (1):

    frictionForce = p.readUserDebugParameter(frictionId)
    jointTorque = p.readUserDebugParameter(torqueId)
    spinVelocity = p.readUserDebugParameter(spinId)

    p.setJointMotorControl2(robotId, 1, p.TORQUE_CONTROL, force=jointTorque)
    p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, targetVelocity=spinVelocity, force=frictionForce)

    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()