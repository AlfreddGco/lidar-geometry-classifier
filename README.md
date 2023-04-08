# ROS2 Playgound

Just a bunch of ros2 packages for having fun with inverse kinematics, data processing, lidar, mobility, RL and ML 

### We got:
- .sdf files with a lidar for gazebo simulations (/lidar_planning/resource)
- .npy files with lidar data points (/lidar_planninng/resource/clouds)
- lidar_listener.py: Extracts gazebo lidar data (ros_ign_bridge needed)
- cube_detector.py: Keras model for detecting cubes from clusters in point clouds
- pangoclouds.py: Process, renders, and detects cubes (from cube_detector) from lidar data in pangolin

