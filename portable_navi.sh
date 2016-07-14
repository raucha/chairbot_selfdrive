<PFのテスト>
rosbag play ~/Dropbox/my_workspace/lab/rosbag/portable_navigation/watanabe_test.bag --clock -r1
rosrun for_testing lrf_reflectance scan_in:=/pot4/scan
rosrun for_testing pot_pose_pf odom:=/mimamorukun/odom

<PFを用いたodomフレーム修正のテスト>
rosbag play ~/Dropbox/my_workspace/lab/rosbag/portable_navigation/watanabe_test_fixed.bag --clock -r3
rosrun for_testing lrf_reflectance scan_in:=/pot4/scan
rosrun for_testing pot_pose_pf odom:=/mimamorukun/odom
rosrun for_testing pose2frame /pose:=/pf_filtered
