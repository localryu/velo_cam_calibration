# velo_cam_calibration

### Source from Autoware (https://autoware.readthedocs.io/en/feature-documentation_rtd/DevelopersGuide/PackagesAPI/sensing/autoware_camera_lidar_calibrator.html)

## Camera pick point
    rosrun velo_cam_calibration camera_click_vertex

## LiDAR pick point
   - /home/ryu/catkin_ws/src/velo_cam_calibration/launch/velodyne_vertex.launch left_line_real_length, right_line_real_length

    roslaunch velo_cam_calibration velodyne_vertex.launch


## Calculate calibration matrix
    rosrun velo_cam_calibration velo_cam_calibration
    
   - Caution: same with #of camera_.xml and #of vertices_.xml
   - If not work, throw away!!



