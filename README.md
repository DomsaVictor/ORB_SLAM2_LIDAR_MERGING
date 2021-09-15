# ORB_SLAM2_LIDAR_MERGING
Modified ORB_SLAM2_ros code that "merges" a lidar map with a visual map.  

Tutorial:


Install:


create a new catkin workspace

copy the files in the 'src' folder

build the code (catkin_make OR catkin build)

Run:


start the LiDAR node (you should see "/base_scan" in rostopic list)

start the camera streamer node (you should see "/camera/image_raw" in rostopi list)

start orb_slma2 node: roslaunch orb_slam2_ros orb_slam2_custom_camera.launch

Note: change the camera parameters in orb_slam2_custom_camera.launch
