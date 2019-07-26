# 3D-Scanner
This is a 3D Scanner written in Python that uses images of an object to triangulate a point cloud, form a mesh, and construct a model. 

The code applies structured light scanning. In this case objects must be projected with vertical and horizonal bars of light of varying degrees and images must be taken from the left and right angles consistently to achieve the image format necessary for this process. The code will produce several meshes that can be organized in MeshLab to produce the final model. MeshLab allows you to modify the meshes to clean up the final model. 

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

