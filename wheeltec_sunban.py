import pybullet as p
import time
import math
import sys
import numpy as np
from datetime import datetime
from datetime import datetime
import pybullet_data
import cv2.aruco as aruco
import cv2
import pkgutil
#p.connect(p.DIRECT)
#egl = pkgutil.get_loader('eglRenderer')
#pluginId = p.loadPlugin("eglRendererPlugin")
#p.setAdditionalSearchPath(pybullet_data.getDataPath())
def drawCube(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1,2)

    # draw ground floor in green
    # img = cv2.drawContours(img, [imgpts[:4]],-1,(0,255,0),-3)

    # draw pillars in blue color
    for i,j in zip(range(4),range(4,8)):
        img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)

    # draw top layer in red color
    img = cv2.drawContours(img, [imgpts[4:]],-1,(0,0,255),3)

    return img
cameraMatrix = np.load('mtx.npy')
distCoeffs = np.load('dist.npy')

ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_6X6_250)
board = aruco.GridBoard_create(
        markersX=1,
        markersY=1,
        markerLength=0.09,
        markerSeparation=0.01,
        dictionary=ARUCO_DICT)

# Create vectors we'll be using for rotations and translations for postures
rotation_vectors, translation_vectors = None, None
axis = np.float32([[-.5,-.5,0], [-.5,.5,0], [.5,.5,0], [.5,-.5,0],
                   [-.5,-.5,1],[-.5,.5,1],[.5,.5,1],[.5,-.5,1] ])

# Make output image fullscreen
cv2.namedWindow('ProjectImage',cv2.WINDOW_NORMAL)





clid = p.connect(p.SHARED_MEMORY)

if (clid < 0):
  p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, -0.0])
p.loadURDF("sunban/sunban.urdf", [2, 0, 1.9])
aruco_marker = p.loadURDF("aruco.urdf", [2, -0.1, 0.3],p.getQuaternionFromEuler([0,0,math.pi]))
husky = p.loadURDF("wheeltec.urdf",[0,0,0],p.getQuaternionFromEuler([0,0,math.pi]))
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
aspect = 640/480
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
   #p.resetBasePositionAndOrientation(aruco_marker,[2, -0.1, 1.5],p.getQuaternionFromEuler([0,0,math.pi]))
   now_xyz = temp_pose[0];
   now_orn = np.reshape(p.getMatrixFromQuaternion(temp_pose[1]),(3,3));
   #print("------------------")
   #print("x   :  ",now_xyz[0])
   #print("y   :  ",now_xyz[1])
   #print("z   :  ",now_xyz[2])
   #print("------------------")
   #print("R   :  ",now_orn)
   cam_xyz = np.array(now_xyz)
   cam_xyz[0] = cam_xyz[0]
   cam_xyz[2] = cam_xyz[2]+0.2

   view_pos = np.matmul(now_orn,np.array([-0.001,0,0.0]).T)
   view_pos = np.array(view_pos+cam_xyz)
   p.addUserDebugLine(prevPose, now_xyz, [0, 0, 1], 5, 20)
   p.addUserDebugLine(prevPose_view, view_pos, [1, 0, 0], 5, 20)
   prevPose = now_xyz
   prevPose_view = view_pos
   #print(view_pos)
   #view_matrix = p.computeViewMatrix([1,0.1,1.3], [10,0,1.5], [0,0,1])
   view_matrix = p.computeViewMatrix([cam_xyz[0],cam_xyz[1],cam_xyz[2]], view_pos, [0,0,1])
   projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
   images = p.getCameraImage(640,
                          480,
                          view_matrix,
                          projection_matrix,
                          shadow=True,
                          renderer=p.ER_BULLET_HARDWARE_OPENGL)
   #print(images)
   img = cv2.cvtColor(np.array(images[2],dtype=np.uint8),cv2.COLOR_RGB2BGR)
   color_img = img.copy()
   gray = cv2.cvtColor(color_img.copy(), cv2.COLOR_RGB2GRAY)
   corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
   corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(
                image = gray,
                board = board,
                detectedCorners = corners,
                detectedIds = ids,
                rejectedCorners = rejectedImgPoints,
                cameraMatrix = cameraMatrix,
                distCoeffs = distCoeffs)
   ProjectImage = color_img.copy()
   ProjectImage = aruco.drawDetectedMarkers(ProjectImage, corners, borderColor=(0, 0, 255))
   if ids is not None and len(ids) > 0:
            # Estimate the posture per each Aruco marker
            rotation_vectors, translation_vectors, _objPoints = aruco.estimatePoseSingleMarkers(corners, 1, cameraMatrix, distCoeffs)
            print(rotation_vectors.shape)
            ids = 1
            for rvec in rotation_vectors:
              rotation_mat = cv2.Rodrigues(rvec[0])[0]
              print("ids:",ids,rotation_mat)
              ids = ids+1
            #rotation_mat = cv2.Rodrigues(rotation_vectors)
            #print(rotation_mat)
            for rvec, tvec in zip(rotation_vectors, translation_vectors):
                if len(sys.argv) == 2 and sys.argv[1] == 'cube':
                    try:
                        imgpts, jac = cv2.projectPoints(axis, rvec, tvec, cameraMatrix, distCoeffs)
                        ProjectImage = drawCube(ProjectImage, corners, imgpts)
                    except:
                        continue
                else:    
                    ProjectImage = aruco.drawAxis(ProjectImage, cameraMatrix, distCoeffs, rvec, tvec, 1)
   cv2.imshow('ProjectImage', ProjectImage)


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
