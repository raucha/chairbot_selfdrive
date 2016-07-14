#include <bfl/filter/bootstrapfilter.h>
#include <bfl/model/systemmodel.h>
#include <bfl/model/measurementmodel.h>
#include <bfl/pdf/conditionalpdf.h>
#include <bfl/pdf/gaussian.h>
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Header.h>
#include <tf/tf.h>

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
/// NonLinear Conditional Gaussian
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
  // implement this virtual function for system model of a particle filter
  //! 要はノイズの乗った次状態予測値をone_sampleで返せればおｋ
  virtual bool SampleFrom(Sample<MatrixWrapper::ColumnVector>& one_sample, int method = DEFAULT,
                          void* args = NULL) const {
    //! [x, y, rad]conditionalpdf
    ColumnVector state = ConditionalArgumentGet(0);
    //! [v, sigma]
    ColumnVector vel = ConditionalArgumentGet(1);

    // sample from additive noise
    Sample<ColumnVector> noise;  //! [v, sigma]
    _additiveNoise.SampleFrom(noise, method, args);

    // system update
    state(1) += cos(state(3)) * (vel(1) + noise.ValueGet()(1));
    state(2) += sin(state(3)) * (vel(1) + noise.ValueGet()(1));
    state(3) += (vel(2) + noise.ValueGet()(2));
    // std::cout << "cov: " << _additiveNoise.CovarianceGet() << std::endl;
    // std::cout << "predict: " << vel(1) << "  " << vel(2) << std::endl;

    // store results in one_sample
    one_sample.ValueSet(state);
    // one_sample.ValueSet(state + noise.ValueGet());

    return true;
    // ColumnVector state = ConditionalArgumentGet(0);
    // ColumnVector vel = ConditionalArgumentGet(1);
    //
    // // system update
    // state(1) += cos(state(3)) * vel(1);  //* 100;
    // state(2) += sin(state(3)) * vel(1);  // * 100;
    // state(3) += vel(2);
    // // std::cout << "cov: " << _additiveNoise.CovarianceGet() << std::endl;
    // // std::cout << "predict: " << vel(1) << "  " << vel(2) << std::endl;
    //
    // // sample from additive noise
    // Sample<ColumnVector> noise;
    // _additiveNoise.SampleFrom(noise, method, args);
    //
    // // store results in one_sample
    // one_sample.ValueSet(state + noise.ValueGet());
    //
    // return true;
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

  // implement this virtual function for measurement model of a particle filter
  //! 要は確率をProbability型で返せればおｋ
  //  Probability p(0.3)とかでも作れる
  virtual Probability ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const {
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

const int STATE_SIZE = 3;
const int INPUT_SIZE = 2;
const int MEAS_SIZE = 2;

SystemModel<ColumnVector>* g_sys_model;
MeasurementModel<ColumnVector, ColumnVector>* g_meas_model;
CustomParticleFilter* g_filter;

ros::Publisher pub_filtered;
ros::Publisher particle_pub;
bool is_got_initial_pose = false;
std_msgs::Header g_latest_header;

void PublishPose() {
  if (!is_got_initial_pose) {
    return;
  }
  Pdf<ColumnVector>* posterior = g_filter->PostGet();
  ColumnVector pose = posterior->ExpectedValueGet();
  SymmetricMatrix pose_cov = posterior->CovarianceGet();

  nav_msgs::Odometry pose_msg;
  // pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.stamp = g_latest_header.stamp;
  // pose_msg.header.frame_id = "odom";
  pose_msg.header.frame_id = "base_scan4";

  pose_msg.pose.pose.position.x = pose(1);
  pose_msg.pose.pose.position.y = pose(2);

  vector<WeightedSample<ColumnVector> >::iterator sample_it;
  vector<WeightedSample<ColumnVector> > samples;
  samples = g_filter->getNewSamples();
  double ang_x = 0.0;
  double ang_y = 0.0;
  for (sample_it = samples.begin(); sample_it < samples.end(); sample_it++) {
    ColumnVector sample = (*sample_it).ValueGet();
    double w = (*sample_it).WeightGet();
    ang_x += cos(sample(3)) * w;
    ang_y += sin(sample(3)) * w;
  }
  double ang = atan2(ang_y, ang_x);
  pose_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(ang);

  const float wheel2base = 0.3;
  float tmp90 = 90.0 * M_PI / 180.0;
  double ang2[] = {ang + tmp90, ang - tmp90};
  geometry_msgs::Point p2[2];
  for (int i = 0; i < 2; i++) {
    p2[i].x = pose_msg.pose.pose.position.x + wheel2base * cos(ang2[i]);
    p2[i].y = pose_msg.pose.pose.position.y + wheel2base * sin(ang2[i]);
  }
  float dist2[2];
  for (int i = 0; i < 2; i++) {
    dist2[i] = p2[i].x * p2[i].x + p2[i].y * p2[i].y;
  }
  if (dist2[0] > dist2[1]) {
    pose_msg.pose.pose.position = p2[0];
  } else {
    pose_msg.pose.pose.position = p2[1];
  }

  pub_filtered.publish(pose_msg);
}

void PublishParticles() {
  if (!is_got_initial_pose) {
    return;
  }
  geometry_msgs::PoseArray particles_msg;
  // particles_msg.header.stamp = ros::Time::now();
  particles_msg.header.stamp = g_latest_header.stamp;
  // particles_msg.header.frame_id = "odom";
  particles_msg.header.frame_id = "base_scan4";

  vector<WeightedSample<ColumnVector> >::iterator sample_it;
  vector<WeightedSample<ColumnVector> > samples;

  samples = g_filter->getNewSamples();
  for (sample_it = samples.begin(); sample_it < samples.end(); sample_it++) {
    geometry_msgs::Pose pose;
    ColumnVector sample = (*sample_it).ValueGet();

    pose.position.x = sample(1);
    pose.position.y = sample(2);
    pose.orientation = tf::createQuaternionMsgFromYaw(sample(3));

    particles_msg.poses.insert(particles_msg.poses.begin(), pose);
  }
  particle_pub.publish(particles_msg);
}

void pf_predict(const nav_msgs::Odometry::ConstPtr& arg) {
  if (!is_got_initial_pose) {
    return;
  }
  g_latest_header = arg->header;
  static ros::Time prevNavDataTime;
  if (prevNavDataTime.isZero()) {
    prevNavDataTime = arg->header.stamp;
    return;
  }
  double dt = (arg->header.stamp - prevNavDataTime).toSec();
  prevNavDataTime = arg->header.stamp;

  ColumnVector input(2);  // [v, sigma]
  input(1) = dt * arg->twist.twist.linear.x;
  input(2) = dt * arg->twist.twist.angular.z;

  //  /****************************
  //  * NonLinear system model      *
  //  ***************************/
  ColumnVector mu(INPUT_SIZE);
  mu(1) = 0.0;
  mu(2) = 0.0;
  // mu(3) = 0.0;
  SymmetricMatrix cov(INPUT_SIZE);
  const double cov_min = 0.0000001;
  // const double cov_min = 0.01;
  cov = 0.0;
  // cov(1, 1) = 0.001;
  // cov(1, 1) = max(pow(0.2*input(1),2), cov_min);
  cov(1, 1) = pow(0.1 * max(input(1), cov_min), 2);
  cov(1, 2) = 0.0;
  cov(2, 1) = 0.0;
  // cov(2, 2) = 0.001;
  cov(2, 2) = pow(0.1 * max(input(2), 5.0 * M_PI / 180.0), 2);
  // cov(2, 2) = max(pow(0.1*input(2),2), cov_min);
  Gaussian gaus_noise(mu, cov);
  NonlinearSystemPdf sys_pdf(gaus_noise);
  g_sys_model->SystemPdfSet(&sys_pdf);

  g_filter->Update(g_sys_model, input);
  PublishPose();
  PublishParticles();
}

void pf_update(const geometry_msgs::PointStamped::ConstPtr& arg) {
  if (!is_got_initial_pose) {
    //パーティクルの初期配置
    ColumnVector prior_Mu(STATE_SIZE);  // [x, y, theta]
    prior_Mu(1) = arg->point.x;
    prior_Mu(2) = arg->point.y;
    prior_Mu(3) = 0.0;
    SymmetricMatrix prior_Cov(STATE_SIZE);
    // prior_Cov(1, 1) = pow(5.0, 2);
    prior_Cov(1, 1) = pow(1.0, 2);
    prior_Cov(1, 2) = 0.0;
    prior_Cov(1, 3) = 0.0;
    prior_Cov(2, 1) = 0.0;
    // prior_Cov(2, 2) = pow(5.0, 2);
    prior_Cov(2, 2) = pow(1.0, 2);
    prior_Cov(2, 3) = 0.0;
    prior_Cov(3, 1) = 0.0;
    prior_Cov(3, 2) = 0.0;
    prior_Cov(3, 3) = pow(10000000.0 * M_PI / 180.0, 2);
    // 初期配置で利用する分布
    Gaussian prior_cont(prior_Mu, prior_Cov);

    // Discrete prior for Particle filter (using the continuous Gaussian prior)
    const int NUM_SAMPLES = 1000;
    // const int NUM_SAMPLES = 2000;
    // 初期分布が離散化されたパーティクルを取得
    vector<Sample<ColumnVector> > prior_samples(NUM_SAMPLES);
    prior_cont.SampleFrom(prior_samples, NUM_SAMPLES, CHOLESKY, NULL);
    MCPdf<ColumnVector> prior_discr(NUM_SAMPLES, STATE_SIZE);
    prior_discr.ListOfSamplesSet(prior_samples);
    // CustomParticleFilter filter(&prior_discr, 0, NUM_SAMPLES / 4.0, MULTINOMIAL_RS);
    g_filter = new CustomParticleFilter(&prior_discr, 0, NUM_SAMPLES / 4.0, MULTINOMIAL_RS);
    // g_filter = new CustomParticleFilter(&prior_discr, 0, NUM_SAMPLES / 2.0, MULTINOMIAL_RS);;
    is_got_initial_pose = true;
  }
  g_latest_header = arg->header;
  ColumnVector measurement(2);  // [x, y]
  measurement(1) = arg->point.x;
  measurement(2) = arg->point.y;

  // /*********************************
  //  * NonLinear Measurement model   *
  //  ********************************/
  ColumnVector mu(MEAS_SIZE);
  mu(1) = 0.0;
  mu(2) = 0.0;
  SymmetricMatrix cov(MEAS_SIZE);
  const double cov_min = 0.01;
  cov(1, 1) = 1.0;
  // cov(1, 1) = max(arg->pose.covariance[0], cov_min);
  cov(1, 2) = 0.0;
  cov(2, 1) = 0.0;
  cov(2, 2) = 1.0;
  // cov(2, 2) = max(arg->pose.covariance[7], cov_min);
  Gaussian gaus_noise(mu, cov);
  NonlinearMeasurementPdf meas_pdf(gaus_noise);
  g_meas_model->MeasurementPdfSet(&meas_pdf);

  g_filter->Update(g_meas_model, measurement);
  PublishPose();
  PublishParticles();
}

int main(int argc, char** argv) {
  cout << "Hello World" << endl;

  Gaussian useless_sys_noise(INPUT_SIZE);
  // Gaussian useless_sys_noise(STATE_SIZE);

  Gaussian useless_meas_noise(MEAS_SIZE);
  NonlinearMeasurementPdf meas_pdf(useless_meas_noise);
  MeasurementModel<ColumnVector, ColumnVector> meas_model(&meas_pdf);

  /****************************
 * Linear prior DENSITY     *
 ***************************/
  // //パーティクルの初期配置
  // ColumnVector prior_Mu(STATE_SIZE);  // [x, y, theta]
  // prior_Mu(1) = 0.0;
  // prior_Mu(2) = 0.0;
  // prior_Mu(3) = 0.0;
  // SymmetricMatrix prior_Cov(STATE_SIZE);
  // prior_Cov(1, 1) = pow(5.0, 2);
  // prior_Cov(1, 2) = 0.0;
  // prior_Cov(1, 3) = 0.0;
  // prior_Cov(2, 1) = 0.0;
  // prior_Cov(2, 2) = pow(5.0, 2);
  // prior_Cov(2, 3) = 0.0;
  // prior_Cov(3, 1) = 0.0;
  // prior_Cov(3, 2) = 0.0;
  // prior_Cov(3, 3) = pow(30.0 * M_PI / 180.0, 2);
  // // 初期配置で利用する分布
  // Gaussian prior_cont(prior_Mu, prior_Cov);

  // // Discrete prior for Particle filter (using the continuous Gaussian prior)
  // const int NUM_SAMPLES = 2000;
  // // 初期分布が離散化されたパーティクルを取得
  // vector<Sample<ColumnVector> > prior_samples(NUM_SAMPLES);
  // prior_cont.SampleFrom(prior_samples, NUM_SAMPLES, CHOLESKY, NULL);
  // MCPdf<ColumnVector> prior_discr(NUM_SAMPLES, STATE_SIZE);
  // prior_discr.ListOfSamplesSet(prior_samples);

  /******************************
   * Construction of the Filter *
   ******************************/
  // MULTINOMIAL_RSでしか動作しない．
  // CustomParticleFilter filter(&prior_discr, 0, NUM_SAMPLES / 4.0, MULTINOMIAL_RS);
  // g_filter = &filter;
  NonlinearSystemPdf sys_pdf(useless_sys_noise);
  SystemModel<ColumnVector> sys_model(&sys_pdf);
  g_sys_model = &sys_model;
  g_meas_model = &meas_model;

  ROS_INFO("Hello World!");
  ros::init(argc, argv, "bfl_pf");
  ros::NodeHandle nh;
  // ros::NodeHandle nh_private("~");
  pub_filtered = nh.advertise<nav_msgs::Odometry>("pf_filtered", 5);
  particle_pub = nh.advertise<geometry_msgs::PoseArray>("pf_particles", 5);
  ros::Subscriber pf_predict_sub = nh.subscribe<nav_msgs::Odometry>("odom", 10, pf_predict);
  ros::Subscriber pf_update_sub =
      nh.subscribe<geometry_msgs::PointStamped>("lrf_pose", 10, pf_update);
  ros::spin();
  delete g_filter;
}
