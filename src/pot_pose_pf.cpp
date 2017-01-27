#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <bfl/filter/bootstrapfilter.h>
#include <bfl/model/systemmodel.h>
#include <bfl/model/measurementmodel.h>
#include <bfl/pdf/conditionalpdf.h>
#include <bfl/pdf/gaussian.h>
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Header.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

const float WHEEL_RAD = 0.27;
tf2::Transform lastCenterPose;
/**
 * @brief 姿勢を表示
 * @param arg 姿勢データ
 */
void printTrans(tf2::Transform arg) {
  double roll, pitch, yaw;
  tf2::getEulerYPR(arg.getRotation(), yaw, pitch, roll);
  ROS_INFO_STREAM("x:" << arg.getOrigin().getX() << " y:" << arg.getOrigin().getY()
                       << " z:" << arg.getOrigin().getZ() << " yaw:" << yaw * 180.0 / 3.14
                       << " pitch:" << pitch * 180.0 / 3.14 << " roll:" << roll * 180.0 / 3.14);
}

//////////////////////
//////////////////////
// nonlinearSystemPdf/
//////////////////////
#include <math.h>
#include <wrappers/rng/rng.h>  // Wrapper around several rng libraries

#define SYSMODEL_NUMCONDARGUMENTS_MOBILE 2
#define SYSMODEL_DIMENSION_MOBILE 3

namespace BFL {
using namespace MatrixWrapper;
class NonlinearSystemPdf
    : public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector> {
 public:
  // @param additiveNoise Pdf representing the additive Gaussian uncertainty
  NonlinearSystemPdf(const Gaussian& additiveNoise)
      : ConditionalPdf<ColumnVector, ColumnVector>(SYSMODEL_DIMENSION_MOBILE,
                                                   SYSMODEL_NUMCONDARGUMENTS_MOBILE) {
    _additiveNoise = additiveNoise;
  }
  virtual ~NonlinearSystemPdf(){};
  /**
 * @brief パーティクルの移動
 * @param one_sample 状態遷移の完了したパーティクル
 * @detail 要はノイズの乗った次状態予測値をone_sampleで返せればおｋ
 */  // implement this virtual function for system model of a particle filter
  virtual bool SampleFrom(Sample<MatrixWrapper::ColumnVector>& one_sample, int method = DEFAULT,
                          void* args = NULL) const {
    //! state: [x, y, rad, 左車輪定常ノイズ, 右車輪定常ノイズ, z,  x, y, z, w],
    //パーティクルの状態変数
    ColumnVector state = ConditionalArgumentGet(0);
    //! trans: [移動距離, 角度変化], オドメトリの生データ
    ColumnVector input = ConditionalArgumentGet(1);

    // 現在座標の型変換
    tf2::Transform currentTrans;
    currentTrans.getOrigin().setX(state(1));
    currentTrans.getOrigin().setY(state(2));
    currentTrans.getOrigin().setZ(state(6));
    tf2::Quaternion quat(state(7), state(8), state(9), state(10));
    currentTrans.setRotation(quat.normalize());

    const char ODOM_DIM = 2;
    const char TRANS_DIM = 7;
    // ROS_INFO_STREAM("rows:"<<input.rows());
    if (ODOM_DIM == input.rows()) {
      // sample from additive noise
      ///! noise: [左車輪白色ノイズ, 右車輪白色ノイズ],
      ///平均値0標準偏差10のガウス分布に従う乱数から値を獲得
      Sample<ColumnVector> noise;
      _additiveNoise.SampleFrom(noise, method, args);

      // 左右車輪の滑りの白色ノイズ
      const float err_l = noise.ValueGet()(1);
      const float err_r = noise.ValueGet()(2);
      // 左右車輪の直径スケールのノイズ
      ColumnVector mu(2);
      mu(1) = 0;
      mu(2) = 0;
      SymmetricMatrix cov(2);
      cov(1, 1) = pow(0.001, 2);
      cov(1, 2) = 0.0;
      cov(2, 1) = 0.0;
      cov(2, 2) = pow(0.001, 2);
      Gaussian scale_gaus(mu, cov);
      //! @todo:車輪半径の差は考慮するほどずれていないため無視
      const float scale_err_l = 0.0;
      const float scale_err_r = 0.0;
      /*Sample<ColumnVector> scale_noise;
      scale_gaus.SampleFrom(scale_noise, method);
      const float scale_err_l = state(4) + scale_noise.ValueGet()(1);
      const float scale_err_r = state(5) + scale_noise.ValueGet()(2);
      if (0.5<fabs(scale_err_l)||0.5<fabs(scale_err_r)){
        ROS_WARN_STREAM("scale_err_l:"<<scale_err_l<<"   scale_err_r:"<<scale_err_r);
      }*/
      // ノイズの乗った左右車輪速度
      const float vel_odom = input(1);
      const float omega_odom = input(2);
      const float Vl = (vel_odom - omega_odom * WHEEL_RAD) * (1.0 + err_l + scale_err_l);
      const float Vr = (vel_odom + omega_odom * WHEEL_RAD) * (1.0 + err_r + scale_err_r);
      // 移動距離，変化角度
      const float vel = (Vl + Vr) / 2.0;
      const float omega = (Vr - Vl) / (2.0 * WHEEL_RAD);

      ///! パーティクルの状態変数を更新
      tf2::Transform diffTrans;
      // オドメトリによる変移の型変換
      diffTrans.getOrigin().setX(vel);
      quat.setRPY(0, 0, omega);
      diffTrans.setRotation(quat.normalize());
      // 遷移後の姿勢を計算
      currentTrans = currentTrans * diffTrans;
      // state(4) = scale_err_l;
      // state(5) = scale_err_r;
    } else if (TRANS_DIM == input.rows()) {
      // 重心からみた該当パーティクルの姿勢
      tf2::Transform centParticleTrans = lastCenterPose.inverse() * currentTrans;
      // 重心の姿勢変化
      tf2::Transform centDiffTrans;
      centDiffTrans.getOrigin().setX(input(1));
      centDiffTrans.getOrigin().setY(input(2));
      centDiffTrans.getOrigin().setZ(input(3));
      tf2::Quaternion quat(input(4), input(5), input(6), input(7));
      centDiffTrans.setRotation(quat.normalize());
      // 重心との相対姿勢を保ったままパーティクルを移動
      ROS_INFO("before:");
      printTrans(currentTrans);
      ROS_INFO("lastCenterPose:");
      printTrans(lastCenterPose);
      ROS_INFO("centParticleTrans:");
      printTrans(centParticleTrans);
      ROS_INFO("centDiffTrans:");
      printTrans(centDiffTrans);
      currentTrans = (lastCenterPose * centDiffTrans) * centParticleTrans;
      ROS_INFO("after:");
      printTrans(currentTrans);
      ROS_INFO("a");
    }
    currentTrans.setRotation(currentTrans.getRotation().normalize());
    geometry_msgs::Transform currentTransMsg = tf2::toMsg(currentTrans);

    // 状態変数に保存
    state(1) = currentTransMsg.translation.x;
    state(2) = currentTransMsg.translation.y;
    state(6) = currentTransMsg.translation.z;
    state(7) = currentTransMsg.rotation.x;
    state(8) = currentTransMsg.rotation.y;
    state(9) = currentTransMsg.rotation.z;
    state(10) = currentTransMsg.rotation.w;

    state(3) = tf2::getYaw(currentTrans.getRotation());

    // パーティクルの新しい状態を返す
    one_sample.ValueSet(state);
    return true;
  }

 private:
  Gaussian _additiveNoise;
};
}  // End namespace BFL

///////////////////////////
///////////////////////////
// NonlinearMeasurementPdf/
///////////////////////////
#define MEASMODEL_NUMCONDARGUMENTS_MOBILE 1
#define MEASMODEL_DIMENSION_MOBILE 3
namespace BFL {
using namespace MatrixWrapper;
/// Non Linear Conditional Gaussian
class NonlinearMeasurementPdf
    : public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector> {
 public:
  //  @param additiveNoise Pdf representing the additive Gaussian uncertainty
  NonlinearMeasurementPdf(const Gaussian& measNoise)
      : ConditionalPdf<ColumnVector, ColumnVector>(MEASMODEL_DIMENSION_MOBILE,
                                                   MEASMODEL_NUMCONDARGUMENTS_MOBILE) {
    _measNoise = measNoise;
  }
  virtual ~NonlinearMeasurementPdf() {}

  /**
 * @brief パーティクルの確率取得
 * @param measurement センサで観測されたグローバル座標
 * @detail 要は確率をProbability型で返せればおｋ
 *          Probability p(0.3)とかでも作れる
 */  // implement this virtual function for measurement model of a particle filter
  virtual Probability ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const {
    //! state: [x, y, rad, 左車輪定常ノイズ, 右車輪定常ノイズ], パーティクルの状態変数
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector expected_measurement(2);
    expected_measurement(1) = state(1);
    expected_measurement(2) = state(2);
    Probability prb = _measNoise.ProbabilityGet(measurement - expected_measurement);
    return prb;
  }

 private:
  Gaussian _measNoise;
};
}  // End namespace BFL

////////////////////////
////////////////////////
// CustomParticleFilter/
////////////////////////
using namespace BFL;

class CustomParticleFilter
    : public BootstrapFilter<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector> {
 public:
  CustomParticleFilter(MCPdf<MatrixWrapper::ColumnVector>* prior, int resampleperiod = 0,
                       double resamplethreshold = 0, int resamplescheme = DEFAULT_RS);
  vector<WeightedSample<MatrixWrapper::ColumnVector> > getNewSamples() { return _new_samples; }
};

CustomParticleFilter::CustomParticleFilter(MCPdf<ColumnVector>* prior, int resampleperiod,
                                           double resamplethreshold, int resamplescheme)
    : BootstrapFilter<ColumnVector, ColumnVector>(prior, resampleperiod, resamplethreshold,
                                                  resamplescheme) {}

/////////
/////////
// main//
/////////
using namespace MatrixWrapper;
// using namespace BFL;
using namespace std;

const int NUM_SAMPLES = 200;
// const int STATE_SIZE = 3;
// const int STATE_SIZE = 5;  //! [x, y, θ, 左車輪定常ノイズ, 右車輪定常ノイズ]
const int STATE_SIZE = 10;  //! [x, y, θ, 左車輪定常ノイズ, 右車輪定常ノイズ, z  x, y, z, w]
const int INPUT_SIZE = 2;
const int MEAS_SIZE = 2;

// SystemModel<ColumnVector>* g_sys_model;
// MeasurementModel<ColumnVector, ColumnVector>* g_meas_model;
CustomParticleFilter* g_filter;

ros::Publisher pub_filtered;
ros::Publisher particle_pub;
bool is_got_initial_pose = false;
std_msgs::Header g_latest_header;
tf2_ros::Buffer tfBuffer_;
tf2_ros::TransformListener* tfListener_;
std::vector<geometry_msgs::PointStamped> observeBuffer;
std::vector<nav_msgs::Odometry> odomBuffer;

/**
 * @brief PFで推定した姿勢を発行
 */
void PublishPose() {
  if (!is_got_initial_pose) {
    return;
  }
  Pdf<ColumnVector>* posterior = g_filter->PostGet();                         // 座標計算用
  vector<WeightedSample<ColumnVector> > samples = g_filter->getNewSamples();  // 角度計用
  ColumnVector pose = posterior->ExpectedValueGet();
  SymmetricMatrix pose_cov = posterior->CovarianceGet();

  // ROS_INFO_STREAM("Err_l:" << pose(4) << " Err_r:" << pose(5));

  nav_msgs::Odometry pose_msg;
  pose_msg.header.stamp = g_latest_header.stamp;
  pose_msg.header.frame_id = "map";

  pose_msg.pose.pose.position.x = pose(1);
  pose_msg.pose.pose.position.y = pose(2);
  pose_msg.pose.pose.position.z = pose(6);

  // 四元数の加重平均を取得
  // 参考:http://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
  vector<WeightedSample<ColumnVector> >::iterator sample_it;
  Eigen::Matrix<double, 4, NUM_SAMPLES> quatMat;
  int i = 0;
  for (sample_it = samples.begin(); sample_it < samples.end(); sample_it++) {
    ColumnVector sample = (*sample_it).ValueGet();
    double w = (*sample_it).WeightGet();
    quatMat(0, i) = w * sample(7);
    quatMat(1, i) = w * sample(8);
    quatMat(2, i) = w * sample(9);
    quatMat(3, i) = w * sample(10);
    i++;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 4, 4> > es(quatMat * quatMat.transpose());
  // cout << "The eigenvalues of A are:" << endl << es.eigenvalues() << endl;
  // cout << "The matrix of eigenvectors, V, is:" << endl << es.eigenvectors() << endl << endl;
  // 最後尾の最大固有ベクトルを利用
  Eigen::Vector4d quatAve = es.eigenvectors().col(3);

  pose_msg.pose.pose.orientation.x = quatAve(0);
  pose_msg.pose.pose.orientation.y = quatAve(1);
  pose_msg.pose.pose.orientation.z = quatAve(2);
  pose_msg.pose.pose.orientation.w = quatAve(3);

  pub_filtered.publish(pose_msg);
  tf2::fromMsg(pose_msg.pose.pose, lastCenterPose);
}

/**
 * @brief パーティクル自体を発行
 */
void PublishParticles() {
  if (!is_got_initial_pose) {
    return;
  }
  geometry_msgs::PoseArray particles_msg;
  particles_msg.header.stamp = g_latest_header.stamp;
  particles_msg.header.frame_id = "map";

  vector<WeightedSample<ColumnVector> >::iterator sample_it;
  vector<WeightedSample<ColumnVector> > samples;

  samples = g_filter->getNewSamples();
  for (sample_it = samples.begin(); sample_it < samples.end(); sample_it++) {
    geometry_msgs::Pose pose;
    ColumnVector sample = (*sample_it).ValueGet();
    pose.position.x = sample(1);
    pose.position.y = sample(2);
    pose.position.z = sample(6);
    pose.orientation.x = sample(7);
    pose.orientation.y = sample(8);
    pose.orientation.z = sample(9);
    pose.orientation.w = sample(10);
    // pose.orientation = tf::createQuaternionMsgFromYaw(sample(3));
    particles_msg.poses.insert(particles_msg.poses.begin(), pose);
  }
  particle_pub.publish(particles_msg);
}

/**
 * @brief パーティクルフィルタのパーティクル群を生成
 * @detail 引数は座標，角度，座標の分散，角度の分散
 */
void GenInitialParticles(double x, double y, double rad, double cov_trans, double cov_rad) {
  //パーティクルの初期配置
  ColumnVector prior_Mu(
      STATE_SIZE);  // [x, y, θ, 左車輪定常ノイズ, 右車輪定常ノイズ, z  x, y, z, w]
  prior_Mu(1) = x;
  prior_Mu(2) = y;
  prior_Mu(3) = rad;  //角度
  prior_Mu(4) = 0.0;  //左車輪定常ノイズ
  prior_Mu(5) = 0.0;  //右車輪定常ノイズ
  prior_Mu(6) = 0.0;  // z
  tf2::Quaternion quat;
  quat.setRPY(0, 0, rad);
  quat = quat.normalize();
  geometry_msgs::Quaternion quatMsg = tf2::toMsg(quat);
  prior_Mu(7) = quatMsg.x;   // quat x
  prior_Mu(8) = quatMsg.y;   // quat y
  prior_Mu(9) = quatMsg.z;   // quat z
  prior_Mu(10) = quatMsg.w;  // quat w
  SymmetricMatrix prior_Cov(STATE_SIZE);
  prior_Cov = 0.0;
  prior_Cov(1, 1) = cov_trans;
  prior_Cov(2, 2) = cov_trans;
  prior_Cov(3, 3) = cov_rad;
  // 初期配置で利用する分布
  Gaussian prior_cont(prior_Mu, prior_Cov);

  // Discrete prior for Particle filter (using the continuous Gaussian prior)
  // 初期分布が離散化されたパーティクルを取得
  vector<Sample<ColumnVector> > prior_samples(NUM_SAMPLES);
  prior_cont.SampleFrom(prior_samples, NUM_SAMPLES, CHOLESKY, NULL);
  MCPdf<ColumnVector> prior_discr(NUM_SAMPLES, STATE_SIZE);
  prior_discr.ListOfSamplesSet(prior_samples);
  if (is_got_initial_pose) {
    // 古いパーティクル分布を消す
    delete g_filter;
  }
  // パーティクルフィルタを生成
  // g_filter = new CustomParticleFilter(&prior_discr, 0, NUM_SAMPLES / 4.0, MULTINOMIAL_RS);
  g_filter = new CustomParticleFilter(&prior_discr, 0.0, NUM_SAMPLES * 9.0 / 10.0, MULTINOMIAL_RS);
  is_got_initial_pose = true;

  ///! 推定姿勢発行
  PublishPose();
  PublishParticles();
}

ros::Time odomLast;
ros::Time observeLast;

/**
 * @brief オドメトリでパーティクルの予測（移動）
 * @param arg オドメトリ
 */
void pf_predict(const nav_msgs::Odometry::ConstPtr& arg) {
  /// LRFから初期位置が観測されるまで待つ
  if (!is_got_initial_pose) {
    return;
  }

  ///! オドメトリの周期を計算
  static ros::Time prevNavDataTime;
  if (prevNavDataTime.isZero()) {
    prevNavDataTime = arg->header.stamp;
    return;
  }
  double dt = (arg->header.stamp - prevNavDataTime).toSec();
  prevNavDataTime = arg->header.stamp;
  g_latest_header = arg->header;
  odomLast = arg->header.stamp;

  ///! 移動距離，変化角度計算
  ColumnVector input(2);  // [v, omega]
  input(1) = dt * arg->twist.twist.linear.x;
  input(2) = dt * arg->twist.twist.angular.z;

  //  /****************************
  //  * NonLinear system model      *
  //  ***************************/
  ///! オドメトリには標準偏差が速度の10%となるガウスノイズがのる
  ColumnVector mu(INPUT_SIZE);
  mu(1) = 0.0;
  mu(2) = 0.0;
  SymmetricMatrix cov(INPUT_SIZE);
  cov(1, 1) = pow(0.1, 2);
  cov(1, 2) = 0.0;
  cov(2, 1) = 0.0;
  cov(2, 2) = pow(0.1, 2);
  Gaussian gaus_noise(mu, cov);
  NonlinearSystemPdf sys_pdf(gaus_noise);
  SystemModel<ColumnVector> sys_model(&sys_pdf);
  // g_sys_model->SystemPdfSet(&sys_pdf);

  ///! パーティクルフィルタを更新
  // ROS_INFO("odom updating");
  g_filter->Update(&sys_model, input);

  ///! 推定姿勢発行
  PublishPose();
  PublishParticles();
}

/**
 * @brief 観測座標を持ちいてPFの更新（重み付け）
 * @param arg 車輪の座標
 */
void pf_update(const geometry_msgs::PointStamped::ConstPtr& arg) {
  g_latest_header = arg->header;
  observeLast = arg->header.stamp;
  geometry_msgs::PointStamped mapPoint;
  try {
    geometry_msgs::TransformStamped mapSframeTransMsg;
    mapSframeTransMsg = tfBuffer_.lookupTransform(
        "map", arg->header.frame_id, /*ros::Time(0)*/ arg->header.stamp /*ros::Time()*/,
        ros::Duration(3.0));
    tf2::doTransform(*arg, mapPoint, mapSframeTransMsg);
  } catch (tf2::TransformException& ex) {
    ROS_WARN("Could NOT transform : %s", ex.what());
    return;
  }

  if (!is_got_initial_pose) {
    GenInitialParticles(mapPoint.point.x, mapPoint.point.y, 0.0, pow(0.1, 2),
                        pow(10000000.0 * M_PI / 180.0, 2));

    return;
  }

  /// タイムスタンプ確認
  if (ros::Duration(4.0) < odomLast - observeLast) {
    ROS_INFO_STREAM("time diff:" << odomLast - observeLast);
    ROS_WARN("too large time gap. skip to update PF");
    return;
  }
  ColumnVector measurement(2);  // [x, y]
  measurement(1) = mapPoint.point.x;
  measurement(2) = mapPoint.point.y;

  // /*********************************
  //  * NonLinear Measurement model   *
  //  ********************************/
  ColumnVector mu(MEAS_SIZE);
  mu(1) = 0.0;  // E[x]
  mu(2) = 0.0;  // E[y]
  SymmetricMatrix cov(MEAS_SIZE);
  const double cov_min = 0.01;
  cov(1, 1) = 1.0;  // Var[x,x]
  cov(1, 2) = 0.0;
  cov(2, 1) = 0.0;
  cov(2, 2) = 1.0;  // Var[y,y]
  Gaussian gaus_noise(mu, cov);
  NonlinearMeasurementPdf meas_pdf(gaus_noise);
  MeasurementModel<ColumnVector, ColumnVector> meas_model(&meas_pdf);

  g_filter->Update(&meas_model, measurement);
  PublishPose();
  PublishParticles();
}

/**
 * @brief パーティクル配置の手動生成
 */
void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& arg) {
  ROS_INFO("got initila pose");
  // geometry_msgs::PointStamped mapPoint;
  geometry_msgs::PoseStamped mapPose;
  try {
    geometry_msgs::TransformStamped mapSframeTransMsg;
    mapSframeTransMsg = tfBuffer_.lookupTransform(
        "map", arg->header.frame_id, /*ros::Time(0)*/ arg->header.stamp /*ros::Time()*/,
        ros::Duration(3.0));
    geometry_msgs::PoseStamped tmp;
    tmp.pose = arg->pose.pose;
    tmp.header = arg->header;
    tf2::doTransform(tmp, mapPose, mapSframeTransMsg);
  } catch (tf2::TransformException& ex) {
    ROS_WARN("Could NOT transform : %s", ex.what());
    return;
  }
  tf2::Transform tmp;
  tf2::fromMsg(mapPose.pose, tmp);
  double roll, pitch, yaw;
  tf2::getEulerYPR(tmp.getRotation(), yaw, pitch, roll);
  if (10.0<fabs(roll*180.0/M_PI) || 10.0<fabs(pitch*180.0/M_PI)){
    ROS_WARN("Vicon has noise.");
    return;
  }
  double rad = yaw;
  // double rad = tf::getYaw(mapPose.pose.orientation);
  GenInitialParticles(mapPose.pose.position.x, mapPose.pose.position.y, rad,
                      arg->pose.covariance.at(0), arg->pose.covariance.at(35));
}

void transCallback(const geometry_msgs::TransformStamped::ConstPtr& arg) {
  /// LRFから初期位置が観測されるまで待つ
  if (!is_got_initial_pose) {
    return;
  }
  /// 屋外が遠ければNDTを無視
  geometry_msgs::TransformStamped mapBasefoot;
  // geometry_msgs::PoseStamped mapBasefoot;
  try {
    // geometry_msgs::TransformStamped mapBasefoot;
    mapBasefoot = tfBuffer_.lookupTransform(
        "map", "base_footprint", /*ros::Time(0)*/ arg->header.stamp /*ros::Time()*/,
        ros::Duration(3.0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("Could NOT transform : %s", ex.what());
    return;
  }
  ROS_INFO_STREAM("mapBasefoot Pose: "<<mapBasefoot);
  if(-10<mapBasefoot.transform.translation.x &&
      mapBasefoot.transform.translation.x < 10 &&
    mapBasefoot.transform.translation.y < -10){
    // mapBasefoot.transform.translation.y < -6){
    ROS_INFO_STREAM("In indoor and ignore NDT transform");
    return;
  }

  const char TRANS_SIZE = 7;
  ///! 移動距離，変化角度計算
  ColumnVector input(TRANS_SIZE);  // [x, y, z,  x, y, z, w]
  input(1) = arg->transform.translation.x;
  input(2) = arg->transform.translation.y;
  input(3) = arg->transform.translation.z;
  input(4) = arg->transform.rotation.x;
  input(5) = arg->transform.rotation.y;
  input(6) = arg->transform.rotation.z;
  input(7) = arg->transform.rotation.w;
  ColumnVector mu(TRANS_SIZE);
  mu = 0.0;
  SymmetricMatrix cov(TRANS_SIZE);
  cov = 0.00000000000000001;
  Gaussian gaus_noise(mu, cov);
  NonlinearSystemPdf sys_pdf(gaus_noise);
  SystemModel<ColumnVector> sys_model(&sys_pdf);

  ///! パーティクルフィルタを更新
  g_filter->Update(&sys_model, input);

  ///! 推定姿勢発行
  PublishPose();
  PublishParticles();
}

// void PushOdom(const nav_msgs::Odometry::ConstPtr& arg){
//   odomBuffer.pushback(*arg);
// }

// void PushObserve(const geometry_msgs::PointStamped::ConstPtr& arg){
//   observeBuffer.pushback(*arg);
// }

// /**
//  * @brief call PF as stamped time
//  * @param e TimerEvent status
//  */
// void pf_spin(const ros::TimerEvent& e){
//   ROS_DEBUG("Spin function at time %f", ros::Time::now().toSec());
//   ros::Time filter_stamp_ = ros::Time::now()-ros::Duration(2.0);
//   filter_stamp_ = min(filter_stamp_, odom_stamp_);
//   if (imu_active_)   filter_stamp_ = min(filter_stamp_, imu_stamp_);
// }

/**
 * @brief main
 */
int main(int argc, char** argv) {
  cout << "Hello World" << endl;

  // Gaussian useless_sys_noise(INPUT_SIZE);
  // Gaussian useless_sys_noise(STATE_SIZE);

  Gaussian useless_meas_noise(MEAS_SIZE);
  NonlinearMeasurementPdf meas_pdf(useless_meas_noise);
  MeasurementModel<ColumnVector, ColumnVector> meas_model(&meas_pdf);

  /******************************
   * Construction of the Filter *
   ******************************/
  // MULTINOMIAL_RSでしか動作しない．
  // CustomParticleFilter filter(&prior_discr, 0, NUM_SAMPLES / 4.0, MULTINOMIAL_RS);
  // g_filter = &filter;
  // NonlinearSystemPdf sys_pdf(useless_sys_noise);
  // SystemModel<ColumnVector> sys_model(&sys_pdf);
  // g_sys_model = &sys_model;
  // g_meas_model = &meas_model;

  ROS_INFO("Hello World!");
  ros::init(argc, argv, "bfl_pf");
  // tf
  tf2_ros::TransformListener tmp_listener(tfBuffer_);
  tfListener_ = &tmp_listener;

  ros::NodeHandle nh;
  // ros::NodeHandle nh_private("~");
  pub_filtered = nh.advertise<nav_msgs::Odometry>("pf_filtered", 5);
  particle_pub = nh.advertise<geometry_msgs::PoseArray>("pf_particles", 5);
  ros::Subscriber pf_predict_sub = nh.subscribe<nav_msgs::Odometry>("odom", 10, pf_predict);
  ros::Subscriber pf_update_sub =
      nh.subscribe<geometry_msgs::PointStamped>("observation_point", 10, pf_update);
  ros::Subscriber initialpose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      "initialpose", 10, initialPoseReceived);
  ros::Subscriber trans_sub =
      nh.subscribe<geometry_msgs::TransformStamped>("trans", 10, transCallback);
  // ros::Subscriber pf_predict_sub = nh.subscribe<nav_msgs::Odometry>("odom", 10, PushOdom);
  // ros::Subscriber pf_update_sub =
  //     nh.subscribe<geometry_msgs::PointStamped>("lrf_pose", 10, PushObserve);

  // 原点位置に初期値パーティクルを生成
  GenInitialParticles(0.0, 0.0, 0.0, pow(0.1, 2),
                      pow(30 * M_PI / 180.0, 2));
  ros::spin();
  delete g_filter;
}
