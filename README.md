# 3D Scanner
This is a 3D Scanner written in Python that uses images of an object to triangulate a point cloud, form a mesh, and construct a model. 

The code uses images modified with structured light scanning as input. In this case objects must be projected with vertical and horizonal bars of light of varying degrees and images must be taken from the left and right angles consistently to achieve the image format necessary for this process. The code will produce several meshes that can be organized in MeshLab to produce the final model. MeshLab allows you to modify the meshes to clean up the final model. 

Click here to see the [code]().

## Project Overview

Structured Light Scanning is the process of projecting a set of patterns onto an object in order to retrieve the distance of each pixel relative to the camera using triangulation. This information can be used to create a point cloud of the object’s pixels in 3D space and meshing these points together will create the object model. This project uses images projected with structured light in order to produce a 3D model of the object in the image. In order to do this, the following goals have to be met:

1. Calibrate cameras to gather intrinsic and extrinsic parameters of the camera 
2. Apply box and color masks to separate the object from the background
3. Triangulate non-masked points to create a point cloud of the object
4. Correlate pixel color data with each point in point cloud
5. Generate mesh using cleanup tools (mesh smoothing and bounding box/triangle pruning)
6. Align meshes and perform poisson surface reconstruction with MeshLab
7. Generate final model renderings

## Data

This project uses 8 sets of images. All images have the resolution 1200x1920 pixels. The first set includes 20 image pairs used to calibrate the cameras. The image pairs display a checkerboard pattern taken at varying angles where each image pair represents a left and right angle of the camera (see figure 1). The next 7 sets of images include 40 image pairs of the object taken from a left and right angle in which each image pair is projected with a different barcode pattern of structured light to be used for reconstructing the image points in 3D space (see figure 2). Each of these 7 sets show different angles of the object (ie. top view, side view, etc.) and include 2 additional image pairs of left and right angles with one image pair including the object and the other image pair discarding the object. This is used to mask the background from the object as well as provide color data of the object for the final model rendering. 

![Figure 1: Pair of Checkerboard Calibration Images from Left and Right Angles]()

![Figure 2: Two Pairs of Structured Light Object Scans from Left and Right Angles]()

## Algorithms 

There are several main algorithms at play to transform a set of images into a model of an object. These algorithms focus on calibrating the camera, triangulating points from a pair of left and right images, masking unwanted features, and creating a mesh from a point cloud. These algorithms will be described here in detail with images and pseudocode. All algorithms not from a library (cv2, numpy, etc.) have been coded by myself with guidance from Charles Folkes’s previous assignments unless stated otherwise.  

To start off we must distinguish between the global coordinate system and camera coordinate system. Global coordinates constitute the physical location of an object in 3D space. When we take a photo of an object from two different viewpoints, we are retrieving the coordinates of the object relative to the camera. Since photos are a 2D rendering of a 3D space we must figure out which pixels in both photos are the same in order to triangulate points together. The first step is to find our camera locations relative to the global coordinate system. 

There will be two cameras called camL and camR from which photos from the left and right angles are taken respectively. The camera class takes in four parameters: the camera focal length, offset of principal point, camera rotation, and camera translation. The focal length allows us to transform physical coordinates to pixel coordinates. The camera coordinate system sits at the principal point. The focal length and principal point are intrinsic parameters. Camera rotation determines which direction the camera is pointing. Camera translation is the distance between the camera coordinate system and the origin of the global coordinate system. Camera rotation and translation are extrinsic parameters. 

**Intrinsic Camera parameters: cv2.calibrateCamera()**

The intrinsic parameters of the camera are found using cv2.calibrateCamera(). Before this function is called, each image of the chessboard is fed into cv2.findChessboardCorners() which return a list of 2d chessboard corner coordinates in the image plane (see figure 3 for example). These corner coordinates are mapped to a grid of points representing 3d points in real world space. The 2d and 3d points are compiled into separate lists which cv2.calibrateCamera uses to determine the intrinsic parameters of both cameras. 

![Figure 3: Corners found by cv2.findChessboardCorners() visualized]()

###### Extrinsic Camera parameters: calibratePose()
The extrinsic parameters of the camera are found using calibratePose(pts3,pts2,cam,params_init). This function takes a grid of 3d points representing the chessboard corners as pts3, a list of chessboard corners found using cv2.findChessboardCorners()  as pts2, the camera updated with the intrinsic parameters, and an array of the initial estimate of extrinsic parameters. calibratePose() calls a multitude of functions with separate jobs. residuals(pts3,pts2,cam,t) computes the difference between the projection of 3D points by the camera
with the given parameters and the observed 2D locations. To do this, after updating the extrinsic parameters with t, cam.project(pts3) is called which projects the given 3d points in world coordinates into the camera to produce a set of 2d coordinates as if the camera took a photo of the scene. This is done with the following code:

```python
  #project() function
  #self.R=rotation, self.t=translation, self.f=focal length, self.c=offset of principal point
  #get point location relative to camera
 	pcam = self.R.transpose() @ (pts3 - self.t)
  #project
  p = self.f * (pcam / pcam[2:])
  #offset principal point
  pts2 = p[0:2,:] + self.c
	return pts2
```
By changing the value of t in residuals() with a lambda function we can compute a residual function. To get the optimal extrinsic parameters we will find the least squares solution between our residual function and our estimated extrinsic parameters using scipy.optimize.leastsq(func_residual, params_init). This provides updated extrinsic parameters of the camera so that pts3 projects as close as possible to pts2. Whenever the extrinsic parameters are updated the camera translation is set and the camera rotation is updated using make_rotation(). make_rotation() sets up 3 rotation matrices representing the degrees to rotate the x, y, and z axis respectively. The dot product of these matrices results in the final rotation matrix. We can confirm we have the right intrinsic and extrinsic parameters by projecting pts3 with cam.project() and mapping its 2d points onto the chessboard.

**Create Masks: decode()**

The purpose of decode(path,start,threshold,cthreshold=0.6,frame="",color="") is to create masks that can be applied on the input image to remove unnecessary detail such as the background since we only want to make a model of the object. decode() will use the images with projected light patterns to create a box mask. Images are converted to grayscale and read in pairs where the projected light pattern is swapped (see figure 4). Each pair of images will gradually increase the number of lines projected onto the object. Each pixel coordinate in the first image that is brighter than its corresponding pixel in the second image will append a 1 (or if not brighter a 0) to its coordinate in a 2d array of the same size. This will create a box code image in binary coded decimal that will be converted to decimal values (0-1023) for each pixel with the function graytobcd(). The box code image is what we will apply our masks to. We will create a box mask and a color mask. To create the box mask we will take the absolute value of the difference between the first and second image in a pair and see if each pixel value is above a certain threshold. Pixels that do not satisfy difference threshold are placed on the box mask which starts out as a an empty 2d array of the same image size. This threshold determines how much of the image will be masked out. The box code and box mask will be modified with each pair of images to create the final 2d arrays. The color mask is made by thresholding the difference of a color image of the object with a color image of the background to determine what pixel values have not changed dramatically which will constitute the background and thus be masked out. Because each pixel has the three values blue, green, and red we take the square root of the difference of the images and sum each pixel’s colors to get a single pixel value in order to use the threshold comparison to make the color mask. See figure 5 for the box code, box mask, and color mask.

![Figure 4: Images are read in pairs to determine what pixels will be masked out]()

