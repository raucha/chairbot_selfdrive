roscore&
rosparam set /use_sim_time true&
rviz -d `rospack find chairbot_selfdrive`/ndt_matcher.rviz&
roslaunch chairbot_selfdrive sim.launch&
roslaunch ndt_localizer ndt_matching_test.launch&
rosbag play ~/Downloads/odom_imu_gps_velodyne__speed_cov.bag tf:=tf_old --clock&
roslaunch chairbot_selfdrive ekf_relative.launch&
