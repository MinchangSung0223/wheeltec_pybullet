import pybullet as p
import time
import math
import numpy as np
from datetime import datetime
from datetime import datetime
import pybullet_data
import cv2

clid = p.connect(p.SHARED_MEMORY)


if (clid < 0):
  p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, -0.1])
p.loadURDF("aruco.urdf", [-1, 0, 0.2])
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
ang = 0
p.setGravity(0, 0, -10)

fov = 60
aspect = 320/180
near = 0.02
far = 5

angle = 0.0;
prevPose = [0, 0, 0]
prevPose_view = [0, 0, 0]
while 1:
   keys = p.getKeyboardEvents()
   #print(keys)
   shift = 0.01
   wheelVelocities = [0, 0, 0, 0]
   speed = 5.0
   #print(wheelVelocities)
   temp_pose = p.getBasePositionAndOrientation(husky)
   now_xyz = temp_pose[0];
   now_orn = np.reshape(p.getMatrixFromQuaternion(temp_pose[1]),(3,3));
   print("------------------")
   print("x   :  ",now_xyz[0])
   print("y   :  ",now_xyz[1])
   print("z   :  ",now_xyz[2])
   print("------------------")
   print("R   :  ",now_orn)
   cam_xyz = np.array(now_xyz)
   cam_xyz[0] = cam_xyz[0]
   cam_xyz[2] = cam_xyz[2]+0.2

   view_pos = np.matmul(now_orn,np.array([-0.001,0,0.0]).T)
   view_pos = np.array(view_pos+cam_xyz)
   p.addUserDebugLine(prevPose, now_xyz, [0, 0, 1], 5, 20)
   p.addUserDebugLine(prevPose_view, view_pos, [1, 0, 0], 5, 20)
   prevPose = now_xyz
   prevPose_view = view_pos
   print(view_pos)
   view_matrix = p.computeViewMatrix([cam_xyz[0],cam_xyz[1],cam_xyz[2]], view_pos, [0,0,1])
   projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
   images = p.getCameraImage(640,
                          480,
                          view_matrix,
                          projection_matrix,
                          shadow=True,
                          renderer=p.ER_BULLET_HARDWARE_OPENGL)
   print(images)
   img = cv2.cvtColor(np.array(images[2],dtype=np.uint8),cv2.COLOR_RGB2BGR)
   cv2.imshow("RGB",img)
   cv2.waitKey(1)
   for k in keys:
      if ord('s') in keys:
        p.saveWorld("state.py")
      if ord('a') in keys:
        basepos = basepos = [basepos[0], basepos[1] - shift, basepos[2]]
      if ord('d') in keys:
        basepos = basepos = [basepos[0], basepos[1] + shift, basepos[2]]
      if p.B3G_RIGHT_ARROW in keys:
        for i in range(len(wheels)):
          wheelVelocities[i] = wheelVelocities[i] - speed * wheelDeltasTurn[i]
      if p.B3G_LEFT_ARROW in keys:
        for i in range(len(wheels)):
          wheelVelocities[i] = wheelVelocities[i] + speed * wheelDeltasTurn[i]
      if p.B3G_UP_ARROW in keys:
        for i in range(len(wheels)):
          wheelVelocities[i] = wheelVelocities[i] + speed * wheelDeltasFwd[i]
      if p.B3G_DOWN_ARROW in keys:
        for i in range(len(wheels)):
          wheelVelocities[i] = wheelVelocities[i] - speed * wheelDeltasFwd[i]
   for i in range(len(wheels)):
        p.setJointMotorControl2(husky,
                            wheels[i],
                            p.VELOCITY_CONTROL,
                            targetVelocity=wheelVelocities[i],
                            force=1000)
   if (useRealTimeSimulation):
      t = time.time()  #(dt, micro) = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f').split('.')
      #t = (dt.second/60.)*2.*math.pi
   else:
      t = t + 0.001
