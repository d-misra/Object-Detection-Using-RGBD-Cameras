# Object-Detection-using-RGBD-camera
A simple object detection module for use with the Xtion3 camera

Functions from point cloud library http://pointclouds.org/ are used to perform: 

1. Voxel Grid Filtering of scene
2. Plane segmentation using RANSAC model, and removing these inliers from scene's point cloud
3. Euclidean Clustering to group points belonging to each object
4. Computing centroids of clusters to detect object location

This perception module is part of my master thesis on a reinforcement learning problem in robotics, details can be found here https://github.com/d-misra/Master-Thesis
