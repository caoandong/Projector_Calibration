import numpy as np
import cv2
from cv2 import aruco
import matplotlib.pyplot as plt
import os
import sys
import signal

# Projector parameters
w_proj = 1920
h_proj = 1080
R_proj = np.array([[-0.99076131, -0.00491919, -0.13552799],
                     [ 0.02515179, -0.98866989, -0.14798394],
                     [-0.13326448, -0.15002553,  0.97965959]])
T_proj = np.array([ 0.09225259, -0.25273821,  1.1683442 ])
# Camera parameters
sx = 1920.0/1270
sy = 1080.0/720
K_cam_origin = np.array([[639.930358887,0,639.150634766],[0,639.930358887,351.240905762],[0,0,1]])
K_cam = np.array([[sx*639.930358887,0,sx*639.150634766],[0,sy*639.930358887,sy*351.240905762],[0,0,1]])
dist_coef = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
# Text parameters
font                   = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10,500)
fontScale              = 0.5
fontColor              = (255,0,255)
lineType               = 2

def plot_charuco(R, tvec, charucoCorners, charucoIds, K=K_cam, dist_coef=dist_coef, ori_idx=0):
    charucoCorners_normalized = cv2.convertPointsToHomogeneous(cv2.undistortPoints(charucoCorners, K, dist_coef))
    charucoCorners_normalized = np.squeeze(charucoCorners_normalized)
    t = np.array(tvec)
    plane_normal = R[2,:] # last row of plane rotation matrix is normal to plane
    plane_point = t.reshape(3,)     # t is a point on the plane
    charucoCorners = []
    epsilon = 1e-06
    for p in charucoCorners_normalized:
        p = p.reshape(3,)
        ray_direction = p / np.linalg.norm(p)
        ray_point = p

        ndotu = plane_normal.dot(ray_direction.T)

        if abs(ndotu) < epsilon:
            print ("no intersection or line is within plane")

        w = ray_point - plane_point
        si = -plane_normal.dot(w.T) / ndotu
        v = w + si * ray_direction
        Psi = w + si * ray_direction + plane_point
        charucoCorners.append(Psi)

    # dist = np.array([np.linalg.norm(tvec - np.array(pt)) for pt in charucoCorners])
    ori = charucoCorners[ori_idx]
    print('Origin id: ', charucoIds[ori_idx], ' origin xyz: ', ori)
    charuco_plot = []
    charuco_plot.append(mlab.points3d(ori[0], ori[1], ori[2], scale_factor=0.01, color=(1,0,0)))
    mlab.text3d(ori[0], ori[1], ori[2], str(charucoIds[ori_idx]), scale=(0.01,0.01,0.01))
    new_corners = np.delete(charucoCorners, (ori_idx), axis=0)
    for pt in new_corners:
        charuco_plot.append(mlab.points3d(pt[0], pt[1], pt[2], scale_factor=0.01, color=(0,0,1)))
    return charucoCorners, np.array(ori)

def get_charuco(frame, K_cam=K_cam, dist_coef=dist_coef):
    # detect charuco
    # K = K_cam.copy()
    K = K_cam_origin.copy()
    corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
    corners, ids, rejected, recovered = cv2.aruco.refineDetectedMarkers(frame, cb, corners, ids, rejected, cameraMatrix=K, distCoeffs=dist_coef)
    if corners == None or len(corners) == 0:
        return None
    ret, charucoCorners, charucoIds = cv2.aruco.interpolateCornersCharuco(corners, ids, frame, cb)
    cv2.aruco.drawDetectedCornersCharuco(frame, charucoCorners, charucoIds)
    # cv2.imshow('charuco',frame)
    # cv2.waitKey(0)
    print("charucoCorners")
    print("camera calib mat before\n%s"%K)
    ret, K, dist_coef, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco([charucoCorners],
                                                                       [charucoIds],
                                                                       cb,
                                                                       (1270,720),#(1920, 1080),
                                                                       K,
                                                                       dist_coef,
                                                                       # flags = cv2.CALIB_USE_INTRINSIC_GUESS)
                                                                       flags = cv2.CALIB_ZERO_TANGENT_DIST + cv2.CALIB_USE_INTRINSIC_GUESS  + cv2.CALIB_FIX_FOCAL_LENGTH + cv2.CALIB_FIX_PRINCIPAL_POINT + cv2.CALIB_FIX_K1  + cv2.CALIB_FIX_K2 + cv2.CALIB_FIX_K3 + cv2.CALIB_FIX_K4 + cv2.CALIB_FIX_K5 + cv2.CALIB_FIX_K6)

    # Plot?
    K_cam = K
    return [frame, charucoCorners, charucoIds, rvecs, tvecs]

def intersectCirclesRaysToBoard(circles, rvec, t, K, dist_coef):
    circles_normalized = cv2.convertPointsToHomogeneous(cv2.undistortPoints(circles, K, dist_coef)) # z= 1

    if not rvec.size:
        return None

    R, _ = cv2.Rodrigues(rvec)
    
    # https://stackoverflow.com/questions/5666222/3d-line-plane-intersection
    plane_x = R[0,:]
    plane_y = R[1,:]
    plane_normal = R[2,:] # last row of plane rotation matrix is normal to plane
    plane_point = t.reshape(3,)     # t is a point on the plane

    epsilon = 1e-06

    circles_2d = []
    circles_3d = []
    for p in circles_normalized:
        p = p.reshape(3,)
        ray_direction = p / np.linalg.norm(p)
        ray_point = p

        ndotu = plane_normal.dot(ray_direction.T)

        if abs(ndotu) < epsilon:
            print ("no intersection or line is within plane")

        w = ray_point - plane_point
        si = -plane_normal.dot(w.T) / ndotu
        v = w + si * ray_direction
        Psi = w + si * ray_direction + plane_point
        vx = v.dot(plane_x)
        vy = v.dot(plane_y)
        circles_2d.append(np.array([vx,vy,0.0]))
        circles_3d.append(Psi)
    return np.array(circles_2d), np.array(circles_3d)

def sort_circles(circles, ori):
    # Find the closest circle to the origin of the board

    print('circles: ', circles, ' ori: ', ori)
    min_dist = 10e10
    min_idx = None
    for i in range(len(circles)):
        circ = circles[i]
        dist = np.sqrt((circ[0] - ori[0])**2 - (circ[1] - ori[1])**2)
        if min_dist > dist:
            min_dist = dist
            min_idx = i
    print('min dist: ', min_dist, ' circle idx: ', min_idx, ' circle coord: ', circles[min_idx])

def get_circle(frame, ret_charuco, ori_idx, charucoCorners, projCirclePoints=None, K_cam=K_cam, dist_coeff=dist_coef):
    frame_charuco, charucoCorners, charucoIds, rvec, tvec = ret_charuco
    # origin = np.squeeze(charucoCorners[ori_idx])
    rvec = np.array(rvec)
    tvec = np.array(tvec)
    K = K_cam.copy()
    dist = dist_coeff.copy()
    # detect circles
    img = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, circles = cv2.findCirclesGrid(gray, circles_grid_size, flags=cv2.CALIB_CB_ASYMMETRIC_GRID + cv2.CALIB_CB_CLUSTERING)
    # Plot
    cv2.drawChessboardCorners(frame_charuco, circles_grid_size, circles, ret)
    circles_tmp = np.squeeze(circles)
    # sort_circles(circles_tmp, origin)
    # ray-plane intersection: circle-center to chessboard-plane
    circles2D, circles3D = intersectCirclesRaysToBoard(circles, rvec, tvec, K, dist)
    if projCirclePoints is not None:
        for i in range(len(circles_tmp)):
            circ = circles_tmp[i]
            cv2.putText(frame_charuco, str(projCirclePoints[i]), (circ[0],circ[1]), font, fontScale, fontColor, lineType)
        cv2.imshow('circle with charuco', frame_charuco)
        cv2.waitKey(0)
    return circles, circles2D, circles3D

def get_circle_coord(num_sets=1, top_left = [1151, 202], h_sep = 142, v_sep = 71, l_sep = 71):
    circ = []
    for y in range(11):
        p0 = [top_left[0]+y%2*l_sep, top_left[1]+y*v_sep]
        for x in range(4):
            p = [p0[0]+x*h_sep, p0[1]]
            circ.append(p)
    circ_ret = [circ for i in range(num_sets)]
    return np.array(circ_ret)

# create Aruco board
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
markerLength = 0.04 # unit: meters
markerSeparation = 0.02 # unit: meters
cb = aruco.CharucoBoard_create(5, 7, markerLength, markerSeparation, aruco_dict)
parameters = aruco.DetectorParameters_create()
circles_grid_size = (4,11)
charucoCornersAccum = []
charucoIdsAccum = []
rvecsAccum = []
tvecsAccum = []
circleWorld = []
circleCam = []


# Init visualizations
import mayavi
import mayavi.mlab as mlab

def plot_vec(tvec, direc, color=(1,1,0)):
    return mlab.quiver3d(tvec[0],tvec[1],tvec[2],direc[0],direc[1],direc[2], line_width=3, scale_factor=.1, color=color)

def plot_plane(tvec, R=None, color=(1,0,0)):
    pt1 = np.array([.1,.1,0], dtype=np.float32)
    pt2 = np.array([.1,-.1,0], dtype=np.float32)
    pt3 = np.array([-.1,.1,0], dtype=np.float32)
    pt4 = np.array([-.1,-.1,0], dtype=np.float32)
    pts = np.array([pt1,pt2,pt3,pt4])

    tvec = np.array(tvec).reshape((3))
    vecs = []
    if R is not None:
        pts_rot = [np.matmul(R, p) + np.array(tvec) for p in pts]
        pts = np.array(pts_rot)
        for i in range(3):
            mlab.text3d(tvec[0]+R[i,0]*0.1, tvec[1]+R[i,1]*0.1, tvec[2]+R[i,2]*0.1, str(i), scale=(0.01,0.01,0.01))
            vec = mlab.quiver3d(tvec[0],tvec[1],tvec[2],R[i,0],R[i,1],R[i,2], line_width=3, scale_factor=.1, color=color)
            vecs.append(vec)
    else:
        mlab.text3d(tvec[0]+1*0.1, tvec[1], tvec[2], 'x', scale=(0.01,0.01,0.01))
        mlab.text3d(tvec[0], tvec[1]+1*0.1, tvec[2], 'y', scale=(0.01,0.01,0.01))
        mlab.text3d(tvec[0], tvec[1], tvec[2]+1*0.1, 'z', scale=(0.01,0.01,0.01))
        vecs = [mlab.quiver3d(tvec[0],tvec[1],tvec[2],1,0,0, line_width=3, scale_factor=.1, color=color),
                mlab.quiver3d(tvec[0],tvec[1],tvec[2],0,1,0, line_width=3, scale_factor=.1, color=color),
                mlab.quiver3d(tvec[0],tvec[1],tvec[2],0,0,1, line_width=3, scale_factor=.1, color=color)]
    num_pts = len(pts)
    plane = None
    # mesh_pts = []
    # for i in range(3):
    #     mesh_tmp = []
    #     for j in range(num_pts):
    #         mesh_tmp.append([pts[j,i],pts[(j+1)%num_pts,i]])
    #     mesh_pts.append(mesh_tmp)
    # plane = mlab.mesh(mesh_pts[0], mesh_pts[1], mesh_pts[2], color=color)
    return vecs,plane

def plot_epipolar(tvec, R=None, color=(1,0,0)):
    pt1 = np.array([0.1,0,0], dtype=np.float32)
    pt2 = np.array([0,0.1,0], dtype=np.float32)
    pt3 = np.array([0,0,0.1], dtype=np.float32)

    tvec = np.array(tvec).reshape((3))
    vecs = []
    pts = [pt1, pt2, pt3]
    pts_rot = [np.matmul(R, p - np.array(tvec)) for p in pts]
    ori = np.matmul(R, - np.array(tvec))
    # print('pts_rot: \n', pts_rot)
    # print('ori: \n', ori)
    pts = np.array(pts_rot)
    for i in range(3):
        mlab.text3d(ori[0]+pts_rot[i][0]*0.1, ori[1]+pts_rot[i][1]*0.1, ori[2]+pts_rot[i][2]*0.1, str(i), scale=(0.01,0.01,0.01))
        vec = mlab.quiver3d(ori[0],ori[1],ori[2],pts_rot[i][0], pts_rot[i][1], pts_rot[i][2], line_width=3, scale_factor=.1, color=color)
        vecs.append(vec)

# frame = cv2.imread("img_calib/img_32.png")
# ret_charuco = get_charuco(frame)

img_path = 'img_calib/'
files = next(os.walk(img_path))[2]
count = 0
circleBoard = []
circleWorld = []
circleCam = []
projCirclePointsAccum = []
figure = mlab.figure('visualize')
for fname in files:
    if count >= 1:
        break
    if fname.endswith('.png'):
        frame = cv2.imread(img_path+fname)
        print(fname)
        ret_charuco = get_charuco(frame.copy())

        if ret_charuco is not None:
            frame_charuco, charucoCorners, charucoIds, rvecs, tvecs = ret_charuco
            R, _ = cv2.Rodrigues(rvecs[0])
            charucoCorners_world, origin = plot_charuco(R, tvecs[0], charucoCorners, charucoIds)
            boad_vecs,board = plot_plane(origin, R, color=(1,0,0))
            cam_vec,cam_plane = plot_plane([0,0,0], color=(0,1,0))
            # Find circle
            projCirclePoints = get_circle_coord()
            circle_cam, ret_circle, ret_circle3d = get_circle(frame, [frame_charuco, charucoCorners, charucoIds, rvecs, origin], 0, charucoCorners, projCirclePoints=np.squeeze(projCirclePoints))
            circleCam.append(circle_cam)
            circleBoard.append(ret_circle)
            circleWorld.append(ret_circle3d)
            projCirclePointsAccum.append(projCirclePoints)
            # Visualize circle
            circle_plot = [mlab.points3d(circle[0], circle[1], circle[2], scale_factor=0.01) for circle in ret_circle3d]
            # circle_board_plot = [mlab.points3d(circle[0], circle[1], circle[2], scale_factor=0.01) for circle in ret_circle]
            figure.scene.disable_render = True # Super duper trick
            circle_text = [mlab.text3d(ret_circle3d[i,0], ret_circle3d[i,1], ret_circle3d[i,2], str(i), scale=(0.01,0.01,0.01)) for i in range(ret_circle3d.shape[0])]
            # circle_text = []
            for i in range(ret_circle3d.shape[0]):
                idx = mlab.text3d(ret_circle3d[i,0], ret_circle3d[i,1], ret_circle3d[i,2], str(i), scale=(0.01,0.01,0.01))
                # coord = mlab.text3d(ret_circle3d[i,0], ret_circle3d[i,1], ret_circle3d[i,2]-0.02, "("+str(projCirclePoints[0][i][0])+","+str(projCirclePoints[0][i][1])+")", scale=(0.005,0.005,0.005))
                # circle_text.append([idx, coord])
            # print('projCirclePoints: \n',projCirclePoints)
            figure.scene.disable_render = False
            count += 1

circleBoard = np.array(circleBoard).astype('float32')
circleCam = np.array(circleCam).astype('float32')
circleWorld = np.array(circleWorld).astype('float32')
print("circleBoard shape: ", circleBoard.shape, " circleCam shape: ", circleCam.shape, " circleWorld shape: ", circleWorld.shape)
projCirclePointsAccum = np.array(projCirclePointsAccum).astype('float32')
K_proj = np.array([[1500, 0, 1920/2.0],[0,1500,1080/2.0],[0,0,1]],dtype=np.float32)
dist_coef_proj = np.array([0.0, 0.0, 0.0, 0.0, 0.0],dtype=np.float32)
ret, K_proj, dist_coef_proj, rvecs, tvecs = cv2.calibrateCamera(circleBoard,
                                                                projCirclePointsAccum,
                                                                (1920, 1080),
                                                                K_proj,
                                                                dist_coef_proj,
                                                                # flags = cv2.CALIB_USE_INTRINSIC_GUESS)
                                                                flags = cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_ZERO_TANGENT_DIST + cv2.CALIB_FIX_K1  + cv2.CALIB_FIX_K2 + cv2.CALIB_FIX_K3 + cv2.CALIB_FIX_K4 + cv2.CALIB_FIX_K5 + cv2.CALIB_FIX_K6)
# proj_R, _ = cv2.Rodrigues(rvecs[0])
# proj_vecs, proj = plot_plane(tvecs[0], proj_R, color=(0,0,1))
print("proj calib mat after\n%s"%K_proj)
print("proj dist_coef %s"%dist_coef_proj.T)
print("calibration reproj err %s"%ret)
print("stereo calibration")
ret, K, dist_coef, K_proj, dist_coef_proj, proj_R, proj_T, _, _ = cv2.stereoCalibrate(
        circleWorld,
        circleCam,
        projCirclePointsAccum,
        K_cam.copy(),
        dist_coef,
        K_proj,
        dist_coef_proj,
        (1920,1080),
        flags = cv2.CALIB_FIX_INTRINSIC
        )

print("projector displacement from camera: \n",proj_T)
proj_vecs, proj = plot_plane(proj_T, proj_R, color=(0,0,1))
mayavi.mlab.show()
