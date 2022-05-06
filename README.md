# realsense_devel
Package for PointCloud filtering using PCL (point cloud library) and Intel RealSense camera.

pointCloud processing steps:
- Voxel grid point reduction;
- Segmentation for gorund detection (RANSAC);
- Outlier removal;
- Clustering (euclidean cluster extraction);
- 3D bounding box extraction with PCA (bbox_pca.cpp) or moment of inertia (bbox_moi.cpp) methods;
- Rviz visualization;
### Example
The launch file provides a bag file with a PointCloud used to detect two obstacles. It launches the bag file, the clustering node, the bbox_moi node and Rviz window.
```
roslaunch realsense_devel sim.launch
```
### To keep in mind
The processing algorithms are tuned to the specific application, you need to set proper parameters for your own application.
### Improvemets
Further improvement will cover fancier code structure and more filtering algorithms.
