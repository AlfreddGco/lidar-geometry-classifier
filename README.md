# Lidar geometry classifier

Used ros2, gazebo, keras and pangolin to gather lidar data, render it, and classify simple geometries with a custom neural network model that achieves 97.6% accuracy. Tried out some other other architectures and they were ranging from 85% to 93% accuracy.

### We got:
- [.sdf files](src/lidar_planning/resource) with a lidar for gazebo simulations
- [.npy files](src/lidar_planninng/resource/clouds) with lidar data points 
- [lidar_listener.py](src/lidar_planning/lidar_planning/lidar_listener.py): Extracts gazebo lidar data (ros_ign_bridge needed)
- [cube_detector.py](cube_detector.py): Keras model for detecting cubes from clusters in point clouds
- [pangoclouds.py](pangoclouds.py): Process, renders, and detects cubes (from cube_detector) from lidar data in pangolin


### Demo
pangoclouds.py loads a point cloud, removes noise, clusters the datapoints and with a neural network identifies cubes from different geometries successfully with an accuracy of 97.6%

![pangloclouds](/images/pangoclouds.png "pangoclouds results")

![pangloclouds](/images/pangoclouds_cubes.png "pangoclouds results")

