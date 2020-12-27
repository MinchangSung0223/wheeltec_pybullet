import pybullet as p
import time
import math
from datetime import datetime
from datetime import datetime
import pybullet_data

clid = p.connect(p.SHARED_MEMORY)


if (clid < 0):
  p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, -0.1])
husky = p.loadURDF("wheeltec.urdf")
for i in range(p.getNumJoints(husky)):
  print(p.getJointInfo(husky, i))
useSimulation = 0
useRealTimeSimulation = 1
p.setRealTimeSimulation(useRealTimeSimulation)
wheels = [0,1,2,3]
wheelVelocities = [0, 0, 0, 0]
wheelDeltasTurn = [1, -1, 1, -1]
wheelDeltasFwd = [1, 1, 1, 1]
p.setGravity(0, 0, -10)
while 1:

   wheelVelocities = [1, 1, 1, 1]
   for i in range(len(wheels)):
       p.setJointMotorControl2(husky,
                            wheels[i],
                            p.VELOCITY_CONTROL,
                            targetVelocity=wheelVelocities[i],
                            force=1000)
   if (useSimulation and useRealTimeSimulation == 0):
       p.stepSimulation()
