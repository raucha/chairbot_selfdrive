<PFのテスト>
rosbag play ~/Dropbox/my_workspace/lab/rosbag/portable_navigation/watanabe_test.bag --clock -r1
rosrun chairbot_selfdrive lrf_reflectance scan_in:=/pot4/scan
rosrun chairbot_selfdrive pot_pose_pf odom:=/mimamorukun/odom

<PFを用いたodomフレーム修正のテスト>
rosbag play ~/Dropbox/my_workspace/lab/rosbag/portable_navigation/watanabe_test_fixed.bag --clock -r3
rosrun chairbot_selfdrive lrf_reflectance scan_in:=/pot4/scan
rosrun chairbot_selfdrive pot_pose_pf odom:=/mimamorukun/odom
rosrun chairbot_selfdrive pose2frame /pose:=/pf_filtered

<PFとオドメトリ生データ比較のテスト>
rosrun chairbot_selfdrive lrf_reflectance  scan_in:=pot9/scan __name:=p9
rosrun chairbot_selfdrive lrf_reflectance  scan_in:=pot8/scan __name:=p8
rosrun chairbot_selfdrive lrf_reflectance  scan_in:=pot7/scan __name:=p7
rosrun chairbot_selfdrive pot_pose_pf odom:=mimamorukun/odom
rosrun chairbot_selfdrive pose2frame pose:=pf_filtered
rosrun for_testing show_odom.py
rosrun tf static_transform_publisher 0 0 0 1.0 0.0 0.0 map for_compare 100
rosbag play ~/Dropbox/my_workspace/lab/rosbag/pot_navi_bsen/room2elevator.bag --clock -r2