# ROS2 Playgound

Just a bunch of ros2 packages for having fun with inverse kinematics, data processing, lidar, mobility, RL and ML.

### We got:
- [.sdf files](src/lidar_planning/resource) with a lidar for gazebo simulations
- [.npy files](src/lidar_planninng/resource/clouds) with lidar data points 
- [lidar_listener.py](src/lidar_planning/lidar_planning/lidar_listener.py): Extracts gazebo lidar data (ros_ign_bridge needed)
- [cube_detector.py](cube_detector.py): Keras model for detecting cubes from clusters in point clouds
- [pangoclouds.py](pangoclouds.py): Process, renders, and detects cubes (from cube_detector) from lidar data in pangolin


### Pangocloud
pangoclouds.py loads a point cloud, removes noise, clusters the datapoints and with a neural network identifies cubes from different geometries successfully with an accuracy of 97.6%

![pangloclouds](/images/pangoclouds.png "pangoclouds result")

