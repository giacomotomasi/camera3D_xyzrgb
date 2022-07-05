# camera3d_xyzrgb
Package for PointCloud filtering using PCL (point cloud library) and 3D camera.

pointCloud processing steps:
- Voxel Grid point reduction;
- Segmentation for ground/plane detection (RANSAC);
- Outlier removal;
- Clustering (euclidean cluster extraction);
- 3D bounding box extraction with PCA (BoungingBox_pca class) or moment of inertia (BoungingBox_,oi class) methods;
- Rviz visualization;
### Example
Download the [rs_pointcloud.bag](https://scientificnet-my.sharepoint.com/:u:/g/personal/giatomasi_unibz_it/EZAB6zj-c29Igs8RLlMhhFoBK6kkYPkcxe1sNWO1OJeyAQ?e=mMgpUq) file and copy it in realsense_devel/bagfiles.
The launch file provides a bag file with a PointCloud used to detect two obstacles. It launches the bag file, detector_node, bbox_node (modify this file to select bbox method) and Rviz window.
```
roslaunch realsense_devel sim.launch
```
#### Note: this example works with obstacle_detection_v1
### Real Camera (realsense model)
Use the package with real camera sensor. It launches the nodes listed above but it replaces data from bag file with camera data.
```
roslaunch realsense_devel object_detection.launch
```

### Result
![Result in Rviz](https://github.com/giacomotomasi/realsense_devel/blob/main/img/rviz_result.png)

### To keep in mind
The processing algorithms are tuned to this specific application example. You need to set proper parameters for your own application. You can change such parameters from */config/detector.yaml* file.

Also, the last version takes into account a camera mounted on a robot and considers a transform between frames.
### Improvemets
Further improvement will cover fancier code structure and more filtering algorithms.
