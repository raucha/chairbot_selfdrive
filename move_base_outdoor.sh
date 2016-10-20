roscore
roslaunch tms_rc_mimamorukun_control minimal_with_tf.launch
rviz -d `rospack find chairbot_selfdrive`/config/ndt_macher.rviz
roslaunch ndt_localizer velodyne_matcher.launch
rsolaunch chairbot_selfdrive pose2frame.launch
roslaunch tms_rc_mimamorukun_control keyop.launch
