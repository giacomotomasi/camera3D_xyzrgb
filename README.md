# realsense_devel
Package for PointCloud filtering using PCL (point cloud library) and Intel RealSense camera.

pointCloud processing steps:
- Voxel grid point reduction;
- Segmentation for gorund detection (RANSAC);
- Outlier removal;
- Clustering (euclidean cluster extraction);
- 3D bounding box extraction with PCA (bbox_pca.cpp) or moment of inertia (bbox_moi.cpp) methods;
- Rviz visualization;
