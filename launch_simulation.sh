roscore&
rosparam set /use_sim_time true&
rviz -d `rospack find chairbot_selfdrive`/config/ndt_macher.rviz&
roslaunch chairbot_selfdrive sim.launch&
roslaunch ndt_localizer ndt_matching_test.launch&
rosbag play ~/Downloads/odom_imu_gps_velodyne__speed_cov.bag tf:=tf_old --clock&
roslaunch chairbot_selfdrive ekf_relative.launch&

roscore
rosparam set /use_sim_time true
rviz -d `rospack find chairbot_selfdrive`/config/ndt_macher.rviz&
rosbag play ~/Downloads/coi玄関-看板前.bag /mimamorukun/odom:=/odom /fix:=/navsat/fix --clock -l
roslaunch chairbot_selfdrive sim.launch
roslaunch ndt_localizer velodyne_matcher.launch
rosrun chairbot_selfdrive pose2frame pose:=ndt_pose _mode_2d:=true
rosrun chairbot_selfdrive pose2frame pose:=ndt_pose