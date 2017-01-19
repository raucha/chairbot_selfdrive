#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

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

#include <cassert>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud_xyz;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;
typedef Cloud::Ptr CloudPtr;
typedef pcl::VoxelGridCovariance<PointType> Voxel;

// ndt_pose: ndtで計算したbase_linkの座標
static geometry_msgs::PoseStamped ndt_pose;
static PointCloud_xyz map;

static bool got_3dmap = false;
static bool got_init_pose = false;

// static pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
static int iter = 30;  // NDTの最大再起回数
// static float ndt_res = 0.4;    // ボクセル
static float ndt_res = 1.0;      // Resolution
static double step_size = 0.1;   // Step size
static double trans_eps = 0.01;  // Transformation epsilon
// パーティクルフィルタの設定
const double cell_res = ndt_res;
const double DL = 0.5;     // パーティクルを修正する位置偏差
// const double DL = cell_res / 20.0;     // パーティクルを修正する位置偏差
const double DT = 2.0 * M_PI / 180.0;  // パーティクルを修正する角度偏差
// const double DT = 1.0 * M_PI / 180.0;  // パーティクルを修正する角度偏差
bool UseSigmaPoint = true;  // Sigma点を使う場合　KL距離の時はfalse
int model_voxel_num = 0;
int target_voxel_num = 0;
pcl::VoxelGridCovariance<PointType> cell_model;
int particles = 50;
int iteration = 10;


// Leaf size of VoxelGrid filter.
// static double voxel_leaf_size = 0.4;
static double voxel_leaf_size = 2.0;

static ros::Publisher ndt_pose_pub;
static ros::Publisher time_ndt_matching_pub;
static ros::Publisher filtered_scan_pub;
static ros::Publisher trans_pub;
static ros::Publisher converge_pub;
static ros::Publisher score_pub;

tf2_ros::Buffer tfBuffer_;
tf2_ros::TransformListener *tfListener_;

/**
 * @brief ガウス分布のランダム
 * @param vd_stdev 標準偏差
 * @param vd_meanValue 平均
 */
double gaussianRand(double vd_stdev, double vd_meanValue) {
  // return 0.0;
  double rtnNum = 0.0;
  static int sw = 0;
  double randNum_1;
  double randNum_2;
  double whiteGaussianNoise_1;
  static double whiteGaussianNoise_2;

  if (sw == 0) {
    sw = 1;
    randNum_1 = ((double)rand()) / RAND_MAX;
    randNum_2 = ((double)rand()) / RAND_MAX;

    whiteGaussianNoise_1 =
        vd_stdev * sqrt(-2.0 * log(randNum_1)) * cos(2.0 * M_PI * randNum_2) + vd_meanValue;
    whiteGaussianNoise_2 =
        vd_stdev * sqrt(-2.0 * log(randNum_1)) * sin(2.0 * M_PI * randNum_2) + vd_meanValue;

    rtnNum = whiteGaussianNoise_1;
  } else {
    sw = 0;
    rtnNum = whiteGaussianNoise_2;
  }
  return (rtnNum);
}

/**
 * @brief 一様分布乱数(0.0~1.0)
 */
double uniform_random() { return (double)rand() / (double)RAND_MAX; }

/**
 * @brief 各姿勢の重みを評価
 * @param cell_model マップの点群
 * @param target_model 観測の点群
 * @param trans 入力姿勢
 */
double Evaluate(pcl::VoxelGridCovariance<PointType> *cell_model,
                pcl::VoxelGridCovariance<PointType> *cell_target, Eigen::Matrix4f &trans) {
  const std::map<size_t, Voxel::Leaf> &leaves_model = cell_model->getLeaves();
  const std::map<size_t, Voxel::Leaf> &leaves_target = cell_target->getLeaves();

  Eigen::Matrix3f R = trans.block(0, 0, 3, 3);
  Eigen::Vector3f T = trans.block(0, 3, 3, 1);

  // NDボクセルでマッチング
  double total_dist = 0;   // 代表面までの距離誤差
  double total_angl = 0;   // 代表面法線の角度誤差
  double total_score = 0;  // KL divergence
  Eigen::Vector3f size_v = cell_model->getLeafSize();
  double cell_size = (size_v[0] + size_v[1] + size_v[2]) / 3.0;
  for (std::map<size_t, Voxel::Leaf>::const_iterator it = leaves_target.begin();
       it != leaves_target.end(); ++it) {
    // targetのボクセルを選択
    const Voxel::Leaf &leaf_target = it->second;

    // 点の数が6以下なら誤差を計算しないで無視
    if (leaf_target.getPointCount() < 6) continue;

    // targetのcentroidに対応するmodelのボクセルを選択
    Eigen::Vector3f p(leaf_target.centroid[0], leaf_target.centroid[1], leaf_target.centroid[2]);
    // std::cout << " target " << leaf_target.centroid[0] << " " << leaf_target.centroid[1] << " "
    // << leaf_target.centroid[2] << std::endl;

    // 姿勢変換
    p = R * p + T;

    if (cell_model->IsOverlapped()) {
      // オーバーラップはしていないはず
      assert(false);
      // // Overlap化した場合
      // // modelにおいて、pを含むすべて（8個）のオーバーラップボクセルを選択
      // std::vector<Voxel::Leaf> Overleaves = cell_model->getOverlappedLeaves(p);
      // if (Overleaves.size() > 0) {
      //   // 誤差が最少となるオーバーラップボクセルを検索

      //   if (UseSigmaPoint) {
      //     // ヨンジンの手法
      //     double max_dist = -1e10;
      //     double max_ang = -1e10;
      //     bool hit = false;  // 誤差が計算されたかどうかのフラグ
      //     for (std::vector<Voxel::Leaf>::iterator it = Overleaves.begin(); it != Overleaves.end();
      //          ++it) {
      //       // std::cout << " model  " << it->centroid[0] << " " << it->centroid[1] << " " <<
      //       // it->centroid[2] << std::endl;
      //       // 誤差を計算
      //       if (!(it->getPointCount() < 6)) {
      //         // 距離誤差
      //         double sum = 0;
      //         Eigen::Vector3f h(it->evecs_(0, 0), it->evecs_(1, 0), it->evecs_(2, 0));
      //         for (std::vector<Eigen::Vector3d>::const_iterator its =
      //                  leaf_target.sigmapoints_.begin();
      //              its != leaf_target.sigmapoints_.end(); ++its) {
      //           Eigen::Vector3f p(its->x(), its->y(), its->z());
      //           Eigen::Vector3f c(it->centroid.x(), it->centroid.y(), it->centroid.z());

      //           // 姿勢変換
      //           p = R * p + T;

      //           double d = (c - p).dot(h);
      //           sum += cell_size - fabs(d);
      //         }
      //         if (max_dist < sum) max_dist = sum;

      //         // 角度誤差
      //         Eigen::Vector3f t(leaf_target.evecs_(0, 0), leaf_target.evecs_(1, 0),
      //                           leaf_target.evecs_(2, 0));

      //         // 姿勢変換
      //         t = R * t + T;

      //         if (max_ang < fabs(h.dot(t))) max_ang = fabs(h.dot(t));

      //         hit = true;
      //       }
      //     }
      //     // 誤差を積算
      //     if (hit) {
      //       total_dist += max_dist;
      //       total_angl += max_ang;
      //       total_score += max_dist * max_ang;
      //     }
      //   } else {
      //     assert(false);
      //     // KL divergence
      //     double minEvaluate = 1e10;
      //     bool hit = false;  // 誤差が計算されたかどうかのフラグ
      //     for (std::vector<Voxel::Leaf>::iterator it = Overleaves.begin(); it != Overleaves.end();
      //          ++it) {
      //       // std::cout << " model  " << it->centroid[0] << " " << it->centroid[1] << " " <<
      //       // it->centroid[2] << std::endl;
      //       // 誤差を計算
      //       if (!(it->getPointCount() < 6)) {
      //         Eigen::Matrix3d c0 = leaf_target.getCov();
      //         Eigen::Vector3d m0 = leaf_target.getMean();

      //         Eigen::Matrix3d c1 = it->getCov();
      //         Eigen::Vector3d m1 = it->getMean();

      //         // 姿勢変換
      //         Eigen::Matrix3d Rd = R.cast<double>();
      //         Eigen::Vector3d Td = T.cast<double>();
      //         c0 = Rd * c0 * Rd.transpose();
      //         m0 = Rd * m0 + Td;

      //         double evaluate =
      //             (log(c1.determinant() / c0.determinant()) + (c1.inverse() * c0).trace() +
      //              (m1 - m0).transpose() * c1.inverse() * (m1 - m0) - 3.0) /
      //             2.0;
      //         // これが小さいほどいい

      //         if (evaluate < minEvaluate) {
      //           minEvaluate = evaluate;
      //         }

      //         // std::cout << evaluate << std::endl;

      //         hit = true;
      //       }
      //     }
      //     if (hit) {
      //       total_score += 1.0 / minEvaluate;
      //     }
      //   }
      // }
    } else {
      // Overlap化しない場合
      const pcl::VoxelGridCovariance<PointType>::Leaf *leaf_model = cell_model->getLeaf(p);

      // 誤差を計算
      if (leaf_model && !(leaf_model->getPointCount() < 6)) {
        if (UseSigmaPoint) {
          // ヨンジンの手法
          // 距離誤差
          double sum = 0;
          Eigen::Vector3f h(leaf_model->evecs_(0, 0), leaf_model->evecs_(1, 0),
                            leaf_model->evecs_(2, 0));
          // int n = 0;
          // for (std::vector<Eigen::Vector3d>::const_iterator its =
          // leaf_target.sigmapoints_.begin(); its != leaf_target.sigmapoints_.end(); ++its, ++n){
          //	if (n == 1 || n == 2) continue;
          for (std::vector<Eigen::Vector3d>::const_iterator its = leaf_target.sigmapoints_.begin();
               its != leaf_target.sigmapoints_.end(); ++its) {
            Eigen::Vector3f p(its->x(), its->y(), its->z());
            Eigen::Vector3f c(leaf_model->centroid.x(), leaf_model->centroid.y(),
                              leaf_model->centroid.z());

            // 姿勢変換
            p = R * p + T;

            double d = (c - p).dot(h);
            // todo: velodyneの観測は面ではなく直線だから、ココは最大の内積だけをりようすべきではなかろうか
            sum += cell_size - fabs(d);
          }
          double dist = sum;

          // 角度誤差
          Eigen::Vector3f t(leaf_target.evecs_(0, 0), leaf_target.evecs_(1, 0),
                            leaf_target.evecs_(2, 0));

          // 姿勢変換
          t = R * t;
          // t = R * t + T;

          double angl = fabs(h.dot(t));

          total_dist += dist;
          total_angl += angl;
          total_score += dist * angl;
        } else {
          // ここには入らないはず
          assert(false);
          // // KL divergence
          // Eigen::Matrix3d c0 = leaf_target.getCov();
          // Eigen::Vector3d m0 = leaf_target.getMean();

          // Eigen::Matrix3d c1 = leaf_model->getCov();
          // Eigen::Vector3d m1 = leaf_model->getMean();

          // // 姿勢変換
          // Eigen::Matrix3d Rd = R.cast<double>();
          // Eigen::Vector3d Td = T.cast<double>();
          // c0 = Rd * c0 * Rd.transpose();
          // m0 = Rd * m0 + Td;

          // double evaluate =
          //     (log(c1.determinant() / c0.determinant()) + (c1.inverse() * c0).trace() +
          //      (m1 - m0).transpose() * c1.inverse() * (m1 - m0) - 3.0) /
          //     2.0;
          // // これが小さいほどいい

          // total_score += 1.0 / evaluate;
        }
      }
    }
  }
  return total_score;
}

/**
 * @brief パーティクル
 */
struct Particle {
  double w;                 // 重み
  double x, y, z, a, b, g;  // 位置と姿勢
};

/**
 * @brief 姿勢を表示
 * @param arg 姿勢データ
 */
void printTrans(tf2::Transform arg) {
  double roll, pitch, yaw;
  tf2::getEulerYPR(arg.getRotation(), yaw, pitch, roll);
  ROS_INFO_STREAM("x:" << arg.getOrigin().getX() << " y:" << arg.getOrigin().getY()
                       << " z:" << arg.getOrigin().getZ() << " yaw:" << yaw << " pitch:" << pitch
                       << " roll:" << roll);
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
  // ndt.setInputTarget(map_ptr);

  // NDTのパラメータ設定
  // ndt.setMaximumIterations(iter);
  // ndt.setResolution(ndt_res);
  // ndt.setStepSize(step_size);
  // ndt.setTransformationEpsilon(trans_eps);
  got_3dmap = true;

  // ********************* NDボクセルの生成 *********************
  // モデル（比較される方）のNDTを生成するための準備
  CloudPtr model(new Cloud(map));
  // pcl::VoxelGridCovariance<PointType> cell_model;
  cell_model.setLeafSize(cell_res, cell_res, cell_res);
  cell_model.setInputCloud(model);

  // モデル（比較される方）のNDTを生成
  // この関数でNDボクセルを作成 2つめの引数は Overlapped ND Voxelを作るときに true
  // modelはOverlap化する
  // cell_model.filter(true, true);
  // modelはOverlap化しない
  cell_model.filter(true, false);

  // // ターゲット（比較する方）のNDTを生成するための準備
  // pcl::VoxelGridCovariance<PointType> cell_target;
  // cell_target.setLeafSize(cell_res, cell_res, cell_res);
  // cell_target.setInputCloud(target);

  // // ターゲット（比較する方）のNDTを生成
  // // この関数でNDボクセルを作成 2つめの引数は Overlapped ND Voxelを作るときに true
  // // targetはOverlap化しない
  // cell_target.filter(true, false);

  // ボクセル個数の格納
  model_voxel_num = cell_model.getLeaves().size();
  // target_voxel_num = cell_target.getLeaves().size();

  // // パーティクルフィルタの設定
  // const double DL = cell_res / 20.0;     // パーティクルを修正する位置偏差
  // const double DT = 1.0 * M_PI / 180.0;  // パーティクルを修正する角度偏差
}

/**
 * @brief 初期位置の分布を購読, 何回でも実行できる
 * @param input PoseWithCovarianceStamped, 初期推定位置分布
 */
static void initialpose_callback(const nav_msgs::Odometry::ConstPtr &input) {
  if (!got_3dmap) {
    return;
  }
  ///! オドメトリの周期を計算
  // static ros::Time prevNavDataTime;
  // if (prevNavDataTime.isZero()) {
  //   prevNavDataTime = input->header.stamp;
  //   return;
  // }
  // double dt = (input->header.stamp - prevNavDataTime).toSec();
  // prevNavDataTime = input->header.stamp;
  // if(!got_init_pose){
  // 初期姿勢設定
  ROS_INFO("initialpose_callback");
  ndt_pose.pose = input->pose.pose;
  ndt_pose.header = input->header;
  got_init_pose = true;
  // }else{
  //   // 姿勢遷移
  //   geometry_msgs::Twist vel = input->twist.twist;
  //   tf2::Transform currentTrans, diffTrans;
  //   tf2::fromMsg(ndt_pose.pose, currentTrans);

  //   // 並進速度
  //   diffTrans.getOrigin().setX(vel.linear.x*dt);
  //   // 角速度
  //   tf2::Quaternion quat;
  //   quat.setRPY(0,0,vel.angular.z*dt);
  //   diffTrans.setRotation(quat.normalize());

  //   // //変更前姿勢
  //   // ROS_INFO("prevPose");
  //   // printTrans(currentTrans);

  //   // 予測姿勢を計算
  //   currentTrans = currentTrans*diffTrans;

  //   // //変更後姿勢
  //   // ROS_INFO("nextPose");
  //   // printTrans(currentTrans);

  //   // 予測姿勢を反映
  //   geometry_msgs::Transform tmp = tf2::toMsg(currentTrans);
  //   ndt_pose.pose.position.x = tmp.translation.x;
  //   ndt_pose.pose.position.y = tmp.translation.y;
  //   ndt_pose.pose.position.z = tmp.translation.z;
  //   ndt_pose.pose.orientation = tmp.rotation;
  // }
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

  CloudPtr target(new Cloud(scan));
  // pcl::io::savePCDFileASCII ("/home/shigekane/velodyne_first.pcd", scan);
  // assert(false);
  PointCloud_xyz::Ptr scan_ptr(new PointCloud_xyz(scan));
  PointCloud_xyz::Ptr filtered_scan_ptr(new PointCloud_xyz());

  // ターゲット（比較する方）のNDTを生成するための準備
  pcl::VoxelGridCovariance<PointType> cell_target;
  cell_target.setLeafSize(cell_res, cell_res, cell_res);
  cell_target.setInputCloud(target);

  // ターゲット（比較する方）のNDTを生成
  // この関数でNDボクセルを作成 2つめの引数は Overlapped ND Voxelを作るときに true
  // targetはOverlap化しない
  cell_target.filter(true, false);
  // ボクセル個数の格納
  target_voxel_num = cell_target.getLeaves().size();

  // // 観測点群のダウンサンプリング
  // pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
  // voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  // voxel_grid_filter.setInputCloud(scan_ptr);
  // voxel_grid_filter.filter(*filtered_scan_ptr);
  // pcl_conversions::toPCL(input->header, filtered_scan_ptr->header);
  // ROS_INFO_STREAM("filtered_scan size" << filtered_scan_ptr->size());
  // filtered_scan_pub.publish(*filtered_scan_ptr);

  // // Setting point cloud to be aligned.
  // ndt.setInputSource(filtered_scan_ptr);

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
  tf2::fromMsg(ndt_pose.pose, ndtPoseTrans);
  tf2::Transform predictTrans;
  predictTrans = ndtPoseTrans * baseSensorTrans;

  double yaw, pitch, roll;
  // tf2::Matrix3x3(predictTrans.getRotation()).getRPY(roll, pitch, yaw);
  tf2::getEulerYPR(predictTrans.getRotation(), yaw, pitch, roll);
  /****** 正解に限らずパーティクルをばら撒く場合 *******/
  int AngleDivNum = 72;
  double AngleDiv = 360 / AngleDivNum;
  // int inum = 100;
  int pnum = 100;  // 初期パーティクルの数
  // int pnum = AngleDivNum * inum;  // 初期パーティクルの数
  std::vector<Particle> particle(pnum);

  /*
  for (int i = 0; i < inum; i++) {
    for (int j = 0; j < AngleDivNum; j++) {
      Particle p;
      p.w = 0.0;
      p.x = predictTrans.getOrigin().x() + gaussianRand(5.0, 0.0);
      p.y = predictTrans.getOrigin().y() + gaussianRand(5.0, 0.0);
      p.z = predictTrans.getOrigin().z() + gaussianRand(5.0, 0.0);
      p.a = roll + gaussianRand(DT*5.0, 0.0);
      p.b = pitch + gaussianRand(DT*5.0, 0.0);
      p.g = yaw + gaussianRand(DT*15, 0.0);
      // p.a = roll + gaussianRand(DT * 10.0, 0.0);
      // p.b = pitch + gaussianRand(DT * 10.0, 0.0);
      // p.g = yaw + gaussianRand(DT * 10.0, 0.0);
      particle.push_back(p);
    }
  }
  */
  for (int i = 0; i < pnum; i++) {
    Particle p;
    p.w = 0.0;
    p.x = predictTrans.getOrigin().x();
    p.y = predictTrans.getOrigin().y();
    p.z = predictTrans.getOrigin().z();
    p.a = roll;
    p.b = pitch;
    p.g = yaw;
    particle.push_back(p);
  }

  Eigen::Translation3f init_trans(predictTrans.getOrigin().x(), predictTrans.getOrigin().y(),
                                  predictTrans.getOrigin().z());
  Eigen::AngleAxisf init_rot_x(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rot_y(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rot_z(yaw, Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f init_guess =
      (init_trans * init_rot_z * init_rot_y * init_rot_x) * Eigen::Matrix4f::Identity();

  PointCloud_xyz::Ptr output_cloud(new PointCloud_xyz);
  // ndt.align(*output_cloud, init_guess);

  // スキャンマッチング実行,結果を保存
  Particle max_p = {0, 0, 0, 0, 0, 0, 0};
  for (int iter = 0; iter < iteration; iter++) {
    double total_weight = 0.0;
    for (std::vector<Particle>::iterator it = particle.begin(); it != particle.end(); ++it) {
      if (it != particle.begin()) {
        // パーティクルにガウス状の偏差を加える
/*
        it->x += gaussianRand(DL, 0.0);
        it->y += gaussianRand(DL, 0.0);
        it->z += gaussianRand(DL, 0.0);
        it->a += gaussianRand(DT, 0.0);
        it->b += gaussianRand(DT, 0.0);
        it->g += gaussianRand(DT, 0.0);
*/
        it->x += 0.5 * (uniform_random() - 0.5);
        it->y += 0.5 * (uniform_random() - 0.5);
        it->z += 0.2 * (uniform_random() - 0.5);
        it->a += 0.5 * M_PI / 180.0 * (uniform_random() - 0.5);
        it->b += 0.5 * M_PI / 180.0 * (uniform_random() - 0.5);
        it->g += 5.0 * M_PI / 180.0 * (uniform_random() - 0.5);
      }

      // パーティクルの保持する位置姿勢で座標変換行列を計算
      Eigen::AngleAxisf Ra(it->a, Eigen::Vector3f::UnitX());
      Eigen::AngleAxisf Rb(it->b, Eigen::Vector3f::UnitY());
      Eigen::AngleAxisf Rg(it->g, Eigen::Vector3f::UnitZ());
      Eigen::Quaternionf q = Ra * Rb * Rg;
      Eigen::Translation3f T(it->x, it->y, it->z);
      Eigen::Matrix4f trans = (T * q).matrix();

      // 誤差計算　★★★★ここが重要★★★★
      double e = Evaluate(&cell_model, &cell_target, trans);

      it->w = e;

      // std::cout << it -> w << " : " << it->x << " " << it->y << " " << it->z << "  :  " <<
      // it->a * 180.0 / M_PI << " " << it->b * 180.0 / M_PI << " " << it->g * 180.0 / M_PI <<
      // std::endl;

      total_weight += it->w;
    }

    // 重み最大のパーティクルを選択
    max_p.w = 0.0;
    for (std::vector<Particle>::iterator it = particle.begin(); it != particle.end(); ++it) {
      // std::cout << "各パーティクルの重み " << it->w << "\n";
      if (max_p.w < it->w) max_p = *it;
    }

    std::cout << "iteration: " << iter << " Particles: " << pnum << " w: " << max_p.w << " t( "
              << max_p.x << " " << max_p.y << " " << max_p.z << " ) a( " << max_p.a * 180.0 / M_PI
              << " " << max_p.b * 180.0 / M_PI << " " << max_p.g * 180.0 / M_PI << ")" << std::endl;

    // パーティクルフィルタの数を変更する場合
    // if (iter == 0) pnum = particles;

    // 次世代のパーティクルを選択して残す（一様サンプリング法）
    std::vector<Particle> new_particle;

    // 重み最大のパーティクルは確実に残す
    new_particle.push_back(max_p);

    for (int i = 1; i < pnum; i++) {
      // 0~重みの和（total_weight）までで、乱数である値を決める
      double r = (double)rand() / RAND_MAX * total_weight;

      // 重みを順々に足していき、その値になったら、その時のパーティクルを残す
      double sr = 0.0;
      for (std::vector<Particle>::iterator it = particle.begin(); it != particle.end(); ++it) {
        sr += it->w;
        if (sr > r) {
          new_particle.push_back(*it);
          break;
        }
      }
    }
    particle.clear();
    particle = new_particle;
    new_particle.clear();
  }
  std::cout << "最大重み " << max_p.w << " t( " << max_p.x << " " << max_p.y << " " << max_p.z
            << " ) a( " << max_p.a * 180.0 / M_PI << " " << max_p.b * 180.0 / M_PI << " "
            << max_p.g * 180.0 / M_PI << ")" << std::endl;
  // パーティクルの保持する位置姿勢で座標変換行列を計算
  Eigen::AngleAxisf Ra(max_p.a, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf Rb(max_p.b, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf Rg(max_p.g, Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf q = Ra * Rb * Rg;
  Eigen::Translation3f T(max_p.x, max_p.y, max_p.z);

  Eigen::Matrix4f t(Eigen::Matrix4f::Identity());
  // t = ndt.getFinalTransformation();
  // int iteration = ndt.getFinalNumIteration();
  // double trans_probability = ndt.getTransformationProbability();
  tf2::Matrix3x3 mat_l;
  mat_l.setValue(
      static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
      static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
      static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));

  // ndt_poseを更新
  geometry_msgs::PoseStamped new_sensor_pose;
  new_sensor_pose.pose.position.x = T.x();
  new_sensor_pose.pose.position.y = T.y();
  new_sensor_pose.pose.position.z = T.z();
  // new_sensor_pose.pose.position.x = t(0, 3);
  // new_sensor_pose.pose.position.y = t(1, 3);
  // new_sensor_pose.pose.position.z = t(2, 3);
  //! TODO: fix angle
  new_sensor_pose.pose.orientation.x = q.x();
  new_sensor_pose.pose.orientation.y = q.y();
  new_sensor_pose.pose.orientation.z = q.z();
  new_sensor_pose.pose.orientation.w = q.w();
  // mat_l.getRPY(roll, pitch, yaw, 1);
  // tf2::Quaternion tmp_q;
  // tmp_q.setRPY(roll, pitch, yaw);
  // new_sensor_pose.pose.orientation.x = tmp_q.x();
  // new_sensor_pose.pose.orientation.y = tmp_q.y();
  // new_sensor_pose.pose.orientation.z = tmp_q.z();
  // new_sensor_pose.pose.orientation.w = tmp_q.w();

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
  tf2::Transform diffTrans = ndtPoseTrans.inverse() * mapNDTBasePoseTrans;
  tf2::Transform latestNdtPoseTrans;
  tf2::fromMsg(ndt_pose.pose, latestNdtPoseTrans);
  latestNdtPoseTrans = latestNdtPoseTrans * diffTrans;

  geometry_msgs::TransformStamped diffTransMsg;
  diffTransMsg.transform = tf2::toMsg(diffTrans);
  diffTransMsg.header.stamp = input->header.stamp;
  trans_pub.publish(diffTransMsg);

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
  std_msgs::Bool isCoverged;
  // isCoverged.data = (uint8_t)ndt.hasConverged();
  converge_pub.publish(isCoverged);
  std_msgs::Float64 fitnessScore;
  // fitnessScore.data = ndt.getFitnessScore();
  score_pub.publish(fitnessScore);

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence: " << input->header.seq << std::endl;
  std::cout << "Timestamp: " << input->header.stamp << std::endl;
  std::cout << "Frame ID: " << input->header.frame_id << std::endl;
  std::cout << "Number of Scan Points: " << scan_ptr->size() << " points." << std::endl;
  std::cout << "Number of Filtered Scan Points: " << filtered_scan_ptr->size() << " points."
            << std::endl;
  std::cout << "Leaf Size: " << voxel_leaf_size << std::endl;
  // std::cout << "NDT has converged: " << ndt.hasConverged() << std::endl;
  // std::cout << "Fitness Score: " << ndt.getFitnessScore() << std::endl;
  // std::cout << "Transformation Probability: " << ndt.getTransformationProbability() << std::endl;
  // std::cout << "Number of Iterations: " << ndt.getFinalNumIteration() << std::endl;
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
  trans_pub = nh.advertise<geometry_msgs::TransformStamped>("trans", 10);
  converge_pub = nh.advertise<std_msgs::Bool>("is_converged", 10);
  score_pub = nh.advertise<std_msgs::Float64>("score", 10);

  // Subscribers
  ros::Subscriber map_sub = nh.subscribe("points_map", 10, map_callback);
  ros::Subscriber initialpose_sub = nh.subscribe("initialpose", 1000, initialpose_callback);
  ros::Subscriber scan_sub = nh.subscribe("points_raw", 1, scan_callback);

  ros::spin();

  return 0;
}
