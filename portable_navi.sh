<PFのテスト>
rosbag play ~/Dropbox/my_workspace/lab/rosbag/portable_navigation/watanabe_test.bag --clock -r1
rosrun chairbot_selfdrive lrf_reflectance scan_in:=/pot4/scan
rosrun chairbot_selfdrive pot_pose_pf odom:=/mimamorukun/odom

<PFを用いたodomフレーム修正のテスト>
rosbag play ~/Dropbox/my_workspace/lab/rosbag/portable_navigation/watanabe_test_fixed.bag --clock -r3
rosrun chairbot_selfdrive lrf_reflectance scan_in:=/pot4/scan
rosrun chairbot_selfdrive pot_pose_pf odom:=/mimamorukun/odom
rosrun chairbot_selfdrive pose2frame /pose:=/pf_filtered
