#include <bfl/filter/bootstrapfilter.h>
#include <bfl/model/systemmodel.h>
#include <bfl/model/measurementmodel.h>
#include <bfl/pdf/conditionalpdf.h>
#include <bfl/pdf/gaussian.h>
#include "chairbot_selfdrive/nonlinearSystemPdf.h"
#include "chairbot_selfdrive/nonlinearMeasurementPdf.h"
#include "chairbot_selfdrive/customparticlefilter.h"
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>

using namespace MatrixWrapper;
using namespace BFL;
using namespace std;

const int STATE_SIZE = 3;
const int INPUT_SIZE = 2;
const int MEAS_SIZE = 2;

SystemModel<ColumnVector>* g_sys_model;
MeasurementModel<ColumnVector, ColumnVector>* g_meas_model;
CustomParticleFilter* g_filter;

ros::Publisher pub_filtered;
ros::Publisher particle_pub;

void PublishPose() {
  Pdf<ColumnVector>* posterior = g_filter->PostGet();
  ColumnVector pose = posterior->ExpectedValueGet();
  SymmetricMatrix pose_cov = posterior->CovarianceGet();

  nav_msgs::Odometry pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = "odom";

  pose_msg.pose.pose.position.x = pose(1);
  pose_msg.pose.pose.position.y = pose(2);
  pose_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose(3));

  pub_filtered.publish(pose_msg);
}

void PublishParticles() {
  geometry_msgs::PoseArray particles_msg;
  particles_msg.header.stamp = ros::Time::now();
  particles_msg.header.frame_id = "odom";

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
  const double cov_min = 0.0001;
  cov = 0.0;
  cov(1, 1) = pow(0.2 * max(input(1), cov_min), 2);
  cov(1, 2) = 0.0;
  cov(2, 1) = 0.0;
  cov(2, 2) = pow(0.2 * max(input(2), cov_min), 2);
  Gaussian gaus_noise(mu, cov);
  NonlinearSystemPdf sys_pdf(gaus_noise);
  g_sys_model->SystemPdfSet(&sys_pdf);

  g_filter->Update(g_sys_model, input);
  PublishPose();
  PublishParticles();
}

void pf_update(const nav_msgs::Odometry::ConstPtr& arg) {
  ColumnVector measurement(2);  // [x, y]
  measurement(1) = arg->pose.pose.position.x;
  measurement(2) = arg->pose.pose.position.y;

  // /*********************************
  //  * NonLinear Measurement model   *
  //  ********************************/
  ColumnVector mu(MEAS_SIZE);
  mu(1) = 0.0;
  mu(2) = 0.0;
  SymmetricMatrix cov(MEAS_SIZE);
  const double cov_min = 0.01;
  cov(1, 1) = max(arg->pose.covariance[0], cov_min);
  cov(1, 2) = 0.0;
  cov(2, 1) = 0.0;
  cov(2, 2) = max(arg->pose.covariance[7], cov_min);
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
  NonlinearSystemPdf sys_pdf(useless_sys_noise);
  SystemModel<ColumnVector> sys_model(&sys_pdf);

  Gaussian useless_meas_noise(MEAS_SIZE);
  NonlinearMeasurementPdf meas_pdf(useless_meas_noise);
  MeasurementModel<ColumnVector, ColumnVector> meas_model(&meas_pdf);

  /****************************
 * Linear prior DENSITY     *
 ***************************/
  //パーティクルの初期配置
  ColumnVector prior_Mu(STATE_SIZE);  // [x, y, theta]
  prior_Mu(1) = 0.0;
  prior_Mu(2) = 0.0;
  prior_Mu(3) = 0.0;
  SymmetricMatrix prior_Cov(STATE_SIZE);
  prior_Cov(1, 1) = pow(5.0, 2);
  prior_Cov(1, 2) = 0.0;
  prior_Cov(1, 3) = 0.0;
  prior_Cov(2, 1) = 0.0;
  prior_Cov(2, 2) = pow(5.0, 2);
  prior_Cov(2, 3) = 0.0;
  prior_Cov(3, 1) = 0.0;
  prior_Cov(3, 2) = 0.0;
  prior_Cov(3, 3) = pow(30.0 * M_PI / 180.0, 2);
  Gaussian prior_cont(prior_Mu, prior_Cov);

  // Discrete prior for Particle filter (using the continuous Gaussian prior)
  const int NUM_SAMPLES = 2000;
  vector<Sample<ColumnVector> > prior_samples(NUM_SAMPLES);
  MCPdf<ColumnVector> prior_discr(NUM_SAMPLES, STATE_SIZE);
  prior_cont.SampleFrom(prior_samples, NUM_SAMPLES, CHOLESKY, NULL);
  prior_discr.ListOfSamplesSet(prior_samples);

  /******************************
   * Construction of the Filter *
   ******************************/
  // MULTINOMIAL_RSでしか動作しない．
  CustomParticleFilter filter(&prior_discr, 0, NUM_SAMPLES / 4.0, MULTINOMIAL_RS);

  g_sys_model = &sys_model;
  g_meas_model = &meas_model;
  g_filter = &filter;

  ROS_INFO("Hello World!");
  ros::init(argc, argv, "bfl_pf");
  ros::NodeHandle nh;
  // ros::NodeHandle nh_private("~");
  pub_filtered = nh.advertise<nav_msgs::Odometry>("pf_filtered", 5);
  particle_pub = nh.advertise<geometry_msgs::PoseArray>("pf_particles", 5);
  ros::Subscriber pf_predict_sub =
      nh.subscribe<nav_msgs::Odometry>("odom", 10, pf_predict);
  ros::Subscriber pf_update_sub = nh.subscribe<nav_msgs::Odometry>("gps_odom", 10, pf_update);
  ros::spin();
}
