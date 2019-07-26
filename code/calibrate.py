#
# To run:
#
# Install opencv modules in Anaconda environment:
#
#   conda install opencv
#   pip install --upgrade pip
#   pip install opencv-contrib-python
#
# Run calibrate.py from the commandline:
#
#   python calibrate.py

import pickle
import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt

# file names, modify as necessary
calibimgfiles = '/home/fowlkes/class/cs117/a3/calib1/*.jpg'
resultfile = 'calibration.pickle'

# checkerboard coordinates in 3D
objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = 2.8*np.mgrid[0:8, 0:6].T.reshape(-1,2)

# arrays to store object points and image points from all the images.
objpoints = [] # 3d points in real world space
imgpoints = [] # 2d points in image plane.

# Make a list of calibration images
images = glob.glob(calibimgfiles)

# Step through the list and search for chessboard corners
for idx, fname in enumerate(images):
    img = cv2.imread(fname)
    img_size = (img.shape[1], img.shape[0])
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, (8,6), None)

    # If found, add object points, image points
    if ret == True:
        objpoints.append(objp)
        imgpoints.append(corners)

        # Display image with the corners overlayed
        cv2.drawChessboardCorners(img, (8,6), corners, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# now perform the calibration
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size,None,None)

print("Estimated camera intrinsic parameter matrix K")
print(K)
print("Estimated radial distortion coefficients")
print(dist)

print("Individual intrinsic parameters")
print("fx = ",K[0][0])
print("fy = ",K[1][1])
print("cx = ",K[0][2])
print("cy = ",K[1][2])


# save the results out to a file for later use
calib = {}
calib["fx"] = K[0][0]
calib["fy"] = K[1][1]
calib["cx"] = K[0][2]
calib["cy"] = K[1][2]
calib["dist"] = dist
fid = open(resultfile, "wb" ) 
pickle.dump(calib,fid)
fid.close()

#
# optionally go through and remove radial distortion from a set of images
#
#images = glob.glob(calibimgfiles)
#for idx, fname in enumerate(images):
#    img = cv2.imread(fname)
#    img_size = (img.shape[1], img.shape[0])
#
#    dst = cv2.undistort(img, K, dist, None, K)
#    udfname = fname+'undistort.jpg'
#    cv2.imwrite(udfname,dst)
#
