# B-senから廊下への自律移動, ポータブルGO８番岳を利用
roslaunch tms_rc_mimamorukun_control minimal_with_tf.launch
roslaunch chairbot_selfdrive pot10_lrf_reflectance.launch
roslaunch chairbot_selfdrive move_base_coi_f2.launch
rviz /initialpose:=/mimamorukun/initialpose /move_base_simple/goal:=/mimamorukun/move_base_simple/goal
rosrun tms_ss_vicon vicon_stream
rosrun tf static_transform_publisher 0.973  -0.333  0 -2.542 0 0 map pot_laser8 100

# 屋内外を通した移動
roslaunch tms_rc_mimamorukun_control minimal_with_tf.launch
roslaunch chairbot_selfdrive move_base.launch
export ROS_NAMESPACE=mimamorukun;roslaunch velodyne_pointcloud 32e_points.launch;unset ROS_NAMESPACE  # 電圧低下でよく止まるため・・・．
roslaunch chairbot_selfdrive pot10_lrf_reflectance.launch

# 屋外での移動のシミュレーション
rosbag play ~/Downloads/coi玄関-看板前_gpslink.bag /velodyne_points:=/mimamorukun/velodyne_points --clock -l
rviz -d `rospack find chairbot_selfdrive`/config/outdoor.rivz.rviz /initialpose:=/mimamorukun/initialpose /move_base_simple/goal:=/mimamorukun/move_base_simple/goal
roslaunch chairbot_selfdrive sim.launch

# Viconを使ったmove_base
## viconのPCをpot_naviのNetGearのルータに繋いでおくこと
roslaunch tms_rc_mimamorukun_control minimal_with_tf.launch
rosrun tms_ss_vicon vicon_stream
roslaunch chairbot_selfdrive move_base_coi_f2.launch
rviz -d `rospack find chairbot_selfdrive`/config/ndt_macher.rviz move_base_simple/goal:=mimamorukun/move_base_simple/goal initialpose:=mimamorukun/initialpose&

### from "move_base_outdoor.sh"
roscore
roslaunch tms_rc_mimamorukun_control minimal_with_tf.launch
rviz -d `rospack find chairbot_selfdrive`/config/ndt_macher.rviz
roslaunch ndt_localizer velodyne_matcher.launch
rsolaunch chairbot_selfdrive pose2frame.launch
roslaunch tms_rc_mimamorukun_control keyop.launch



### from "portable_navi.sh"
# W-2でのテスト？
## PFのテスト
rosbag play ~/Dropbox/my_workspace/lab/rosbag/portable_navigation/watanabe_test.bag --clock -r1
rosrun chairbot_selfdrive lrf_reflectance scan_in:=/pot4/scan
rosrun chairbot_selfdrive pot_pose_pf odom:=/mimamorukun/odom

## PFを用いたodomフレーム修正のテスト
rosbag play ~/Dropbox/my_workspace/lab/rosbag/portable_navigation/watanabe_test_fixed.bag --clock -r3
rosrun chairbot_selfdrive lrf_reflectance scan_in:=/pot4/scan
rosrun chairbot_selfdrive pot_pose_pf odom:=/mimamorukun/odom
rosrun chairbot_selfdrive pose2frame /pose:=/pf_filtered

## PFとオドメトリ生データ比較のテスト
roslaunch chairbot_selfdrive pot10_lrf_reflectance.launch
rosrun chairbot_selfdrive pot_pose_pf odom:=mimamorukun/odom
rosrun chairbot_selfdrive pose2frame pose:=pf_filtered
rosrun for_testing show_odom.py
rosrun tf static_transform_publisher 0 0 0 1.0 0.0 0.0 map for_compare 100
rosbag play ~/Dropbox/my_workspace/lab/rosbag/pot_navi_bsen/room2elevator.bag --clock -r2

## B-senでのテスト
roslaunch chairbot_selfdrive pot10_lrf_reflectance.launch       # 反射強度から車いす検出
rosrun chairbot_selfdrive pot_pose_pf odom:=mimamorukun/odom    # PF
rosrun tf static_transform_publisher 0 0 0 -0.9599310885968813 0 0 map odom 100



### from "launch_simulation.sh"
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
roslaunch chairbot_selfdrive velodyne_matcher.launch
roslaunch chairbot_selfdrive pose2frame.launch
