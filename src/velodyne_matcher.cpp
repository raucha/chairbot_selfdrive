#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud_xyz;

// ndt_pose: ndtで計算したbase_linkの座標
static geometry_msgs::PoseStamped ndt_pose;
static PointCloud_xyz map;

static bool got_3dmap = false;
static bool got_init_pose = false;

static pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
static int iter = 30;             // NDTの最大再起回数
// static float ndt_res = 0.4;    // ボクセル
static float ndt_res = 1.0;       // Resolution
static double step_size = 0.1;    // Step size
static double trans_eps = 0.01;   // Transformation epsilon

// Leaf size of VoxelGrid filter.
// static double voxel_leaf_size = 0.4;
static double voxel_leaf_size = 2.0;

static ros::Publisher ndt_pose_pub;
static ros::Publisher time_ndt_matching_pub;
static ros::Publisher filtered_scan_pub;

tf2_ros::Buffer tfBuffer_;
tf2_ros::TransformListener *tfListener_;

/**
 * @brief 姿勢を表示
 * @param arg 姿勢データ
 */
 void printTrans(tf2::Transform arg){
    double roll, pitch, yaw;
    tf2::getEulerYPR(arg.getRotation(), yaw,pitch,roll);
    ROS_INFO_STREAM("x:" << arg.getOrigin().getX() <<
                    " y:" << arg.getOrigin().getY() <<
                    " z:" << arg.getOrigin().getZ() <<
                    " yaw:" << yaw <<
                    " pitch:" << pitch <<
                    " roll:" << roll);
 }

/**
 * @brief 3次元地図を点群で購読，1回のみ
 * @param input PointCloud2,マップデータ
 */
static void map_callback(const sensor_msgs::PointCloud2::ConstPtr &input) {
  if (got_3dmap) {
    return;
  }
  ROS_INFO("map_callback");

  // ３次元点群マップ設定.
  pcl::fromROSMsg(*input, map);
  PointCloud_xyz::Ptr map_ptr(new PointCloud_xyz(map));
  ndt.setInputTarget(map_ptr);

  // NDTのパラメータ設定
  ndt.setMaximumIterations(iter);
  ndt.setResolution(ndt_res);
  ndt.setStepSize(step_size);
  ndt.setTransformationEpsilon(trans_eps);
  got_3dmap = true;
}

/**
 * @brief 初期位置の分布を購読, 何回でも実行できる
 * @param input PoseWithCovarianceStamped, 初期推定位置分布
 */
static void initialpose_callback(const nav_msgs::Odometry::ConstPtr &input) {
  if(!got_3dmap){
    return;
  }
  ///! オドメトリの周期を計算
  static ros::Time prevNavDataTime;
  if (prevNavDataTime.isZero()) {
    prevNavDataTime = input->header.stamp;
    return;
  }
  double dt = (input->header.stamp - prevNavDataTime).toSec();
  prevNavDataTime = input->header.stamp;
  if(!got_init_pose){
    // 初期姿勢設定
    ROS_INFO("initialpose_callback");
    ndt_pose.pose = input->pose.pose;
    ndt_pose.header = input->header;
    got_init_pose = true;
  }else{
    // 姿勢遷移
    geometry_msgs::Twist vel = input->twist.twist;
    tf2::Transform currentTrans, diffTrans;
    tf2::fromMsg(ndt_pose.pose, currentTrans);

    // 並進速度
    diffTrans.getOrigin().setX(vel.linear.x*dt);
    // 角速度
    tf2::Quaternion quat;
    quat.setRPY(0,0,vel.angular.z*dt);
    diffTrans.setRotation(quat.normalize());

    // //変更前姿勢
    // ROS_INFO("prevPose");
    // printTrans(currentTrans);

    // 予測姿勢を計算
    currentTrans = currentTrans*diffTrans;

    // //変更後姿勢
    // ROS_INFO("nextPose");
    // printTrans(currentTrans);

    // 予測姿勢を反映
    geometry_msgs::Transform tmp = tf2::toMsg(currentTrans);
    ndt_pose.pose.position.x = tmp.translation.x;
    ndt_pose.pose.position.y = tmp.translation.y;
    ndt_pose.pose.position.z = tmp.translation.z;
    ndt_pose.pose.orientation = tmp.rotation;
  }
}

/**
 * @brief 3次元距離センサのデータを購読
 * 各データの発行はすべてここから行われる
 * マップがロード済みかつ初期位置設定済みの時のみ実行
 * @param input PointCloud2, 計測された点群
 */
static void scan_callback(const sensor_msgs::PointCloud2::ConstPtr &input) {
  if (!got_3dmap || !got_init_pose) {
    return;
  }
  ROS_INFO("scan_callback");

  PointCloud_xyz scan;  //! センサの入力データ
  pcl::fromROSMsg(*input, scan);

  // // velodyne特化の点群に変換
  // pcl::PointCloud<velodyne_pointcloud::PointXYZIR> tmp;
  // pcl::fromROSMsg(*input, tmp);
  // scan.points.clear();
  // for (pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::const_iterator item = tmp.begin();
  //      item != tmp.end(); item++) {
  //   pcl::PointXYZ p;
  //   p.x = (double)item->x;
  //   p.y = (double)item->y;
  //   p.z = (double)item->z;
  //   if (item->ring >= 0 && item->ring <= 63) {
  //     scan.points.push_back(p);
  //   }
  // }

  PointCloud_xyz::Ptr scan_ptr(new PointCloud_xyz(scan));
  PointCloud_xyz::Ptr filtered_scan_ptr(new PointCloud_xyz());


  // 観測点群のダウンサンプリング
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);
  pcl_conversions::toPCL(input->header, filtered_scan_ptr->header);
  ROS_INFO_STREAM("filtered_scan size" << filtered_scan_ptr->size());
  filtered_scan_pub.publish(*filtered_scan_ptr);

  // Setting point cloud to be aligned.
  ndt.setInputSource(filtered_scan_ptr);

  tf2::Transform baseSensorTrans;
  try {
    tf2::fromMsg(tfBuffer_.lookupTransform("base_footprint", input->header.frame_id,
                                           /*ros::Time(0)*/ input->header.stamp,
                                           ros::Duration(3.0)).transform,
                 baseSensorTrans);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("Could NOT transform : %s", ex.what());
    return;
  }


  // 点群センサの現在位置を予測，NDTの初期位置として利用
  tf2::Transform ndtPoseTrans;
  tf2::fromMsg(ndt_pose.pose,ndtPoseTrans);
  tf2::Transform predictTrans;
  predictTrans = ndtPoseTrans * baseSensorTrans;

  double yaw, pitch, roll;
  // tf2::Matrix3x3(predictTrans.getRotation()).getRPY(roll, pitch, yaw);
  tf2::getEulerYPR(predictTrans.getRotation(), yaw, pitch, roll);
  Eigen::Translation3f init_trans(predictTrans.getOrigin().x(), predictTrans.getOrigin().y(),
                                  predictTrans.getOrigin().z());
  Eigen::AngleAxisf init_rot_x(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rot_y(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rot_z(yaw, Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f init_guess =
      (init_trans * init_rot_z * init_rot_y * init_rot_x) * Eigen::Matrix4f::Identity();

  PointCloud_xyz::Ptr output_cloud(new PointCloud_xyz);
  ndt.align(*output_cloud, init_guess);

  // スキャンマッチング実行,結果を保存
  Eigen::Matrix4f t(Eigen::Matrix4f::Identity());
  t = ndt.getFinalTransformation();
  int iteration = ndt.getFinalNumIteration();
  double trans_probability = ndt.getTransformationProbability();
  tf2::Matrix3x3 mat_l;
  mat_l.setValue(
      static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
      static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
      static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));

  // ndt_poseを更新
  geometry_msgs::PoseStamped new_sensor_pose;
  new_sensor_pose.pose.position.x = t(0, 3);
  new_sensor_pose.pose.position.y = t(1, 3);
  new_sensor_pose.pose.position.z = t(2, 3);
  //! TODO: fix angle
  mat_l.getRPY(roll, pitch, yaw, 1);
  tf2::Quaternion tmp_q;
  tmp_q.setRPY(roll, pitch, yaw);
  new_sensor_pose.pose.orientation.x = tmp_q.x();
  new_sensor_pose.pose.orientation.y = tmp_q.y();
  new_sensor_pose.pose.orientation.z = tmp_q.z();
  new_sensor_pose.pose.orientation.w = tmp_q.w();

  //! base_footprintの座標を取得
  tf2::Transform sensorBaseTrans, mapNDTSensorPoseTrans;
  try {
    tf2::fromMsg(tfBuffer_.lookupTransform(input->header.frame_id, "base_footprint",
                                           input->header.stamp, ros::Duration(3.0)).transform,
                 // tf2::fromMsg(tfBuffer_.lookupTransform(input->header.frame_id,
                 // "base_footprint", ros::Time(0)).transform,
                 sensorBaseTrans);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("Could NOT transform : %s", ex.what());
  }
  tf2::fromMsg(new_sensor_pose.pose, mapNDTSensorPoseTrans);
  tf2::Transform mapNDTBasePoseTrans;
  mapNDTBasePoseTrans.mult(mapNDTSensorPoseTrans, sensorBaseTrans);

  // NDTに時間がかかっているので，算出された絶対姿勢を姿勢遷移に変換し，最新の姿勢に適用する
  tf2::Transform diffTrans = ndtPoseTrans.inverse()*mapNDTBasePoseTrans;
  tf2::Transform latestNdtPoseTrans;
  tf2::fromMsg(ndt_pose.pose,latestNdtPoseTrans);
  latestNdtPoseTrans = latestNdtPoseTrans*diffTrans;

  geometry_msgs::Transform tmp = tf2::toMsg(latestNdtPoseTrans);
  // geometry_msgs::Transform tmp = tf2::toMsg(mapNDTBasePoseTrans);
  ndt_pose.pose.position.x = tmp.translation.x;
  ndt_pose.pose.position.y = tmp.translation.y;
  ndt_pose.pose.position.z = tmp.translation.z;
  ndt_pose.pose.orientation = tmp.rotation;

  // ndt_q.setRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw);
  nav_msgs::Odometry ndt_pose_msg;
  ndt_pose_msg.header.frame_id = "map";
  ndt_pose_msg.header.stamp = input->header.stamp;
  ndt_pose_msg.pose.pose = ndt_pose.pose;

  ndt_pose_pub.publish(ndt_pose_msg);

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence: " << input->header.seq << std::endl;
  std::cout << "Timestamp: " << input->header.stamp << std::endl;
  std::cout << "Frame ID: " << input->header.frame_id << std::endl;
  std::cout << "Number of Scan Points: " << scan_ptr->size() << " points." << std::endl;
  std::cout << "Number of Filtered Scan Points: " << filtered_scan_ptr->size() << " points."
            << std::endl;
  std::cout << "Leaf Size: " << voxel_leaf_size << std::endl;
  std::cout << "NDT has converged: " << ndt.hasConverged() << std::endl;
  std::cout << "Fitness Score: " << ndt.getFitnessScore() << std::endl;
  std::cout << "Transformation Probability: " << ndt.getTransformationProbability() << std::endl;
  std::cout << "Number of Iterations: " << ndt.getFinalNumIteration() << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw): " << std::endl;
  std::cout << "Transformation Matrix: " << std::endl;
  std::cout << t << std::endl;
  std::cout << "Initial GuessTransformation Matrix: " << std::endl;
  std::cout << init_guess << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ndt_matching");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // tf
  tf2_ros::TransformListener tmp_listener(tfBuffer_);
  tfListener_ = &tmp_listener;

  // Publishers
  // ndtでの推定座標
  ndt_pose_pub = nh.advertise<nav_msgs::Odometry>("ndt_pose", 1000);
  // ndtでの推定座標．座標系がこれだけ違う．多分base_linkではない何か
  time_ndt_matching_pub = nh.advertise<std_msgs::Float32>("time_ndt_matching", 1000);
  filtered_scan_pub = nh.advertise<PointCloud_xyz>("filtered_scan", 10);

  // Subscribers
  ros::Subscriber map_sub = nh.subscribe("points_map", 10, map_callback);
  ros::Subscriber initialpose_sub = nh.subscribe("initialpose", 1000, initialpose_callback);
  ros::Subscriber scan_sub = nh.subscribe("points_raw", 10, scan_callback);

  ros::spin();

  return 0;
}
