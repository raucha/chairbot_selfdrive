#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/common/centroid.h>
#include <geometry_msgs/PointStamped.h>

using namespace std;

double xmax, xmin, ymax, ymin;
string ref_frame_id;
int intensity_th;
ros::Publisher pub, pub_pc, pub_pose, pub_point;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  ROS_INFO("got scan data");
  sensor_msgs::LaserScan s_in = *scan;
  // 反射率の低い点を排除
  for(int i=0;i<s_in.ranges.size();i++){
    if(intensity_th > s_in.intensities.at(i)){
      s_in.ranges.at(i) = s_in.range_max + 1.0;
    }
  }
  sensor_msgs::PointCloud2 cloud;
  string fid = s_in.header.frame_id;
  tf::TransformListener tfl;
  static laser_geometry::LaserProjection projector_;
  projector_.transformLaserScanToPointCloud(fid, s_in, cloud, tfl);

  pub_pc.publish(cloud);

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg (cloud, pcl_cloud);

  // // Create and accumulate points
  // CentroidPoint<pcl::PointXYZ> centroid;
  // for(auto iter=pcl_cloud.begin(); iter != cloud.end(); iter++){
  //   centroid.add(*iter);
  // }
  // pcl::PointXYZ center;
  // centroid.get (center);
  pcl::PointXYZ center;
  geometry_msgs::PoseWithCovarianceStamped pose;

  for(auto iter=pcl_cloud.begin(); iter != pcl_cloud.end(); iter++){
    pose.pose.pose.position.x += iter->x;
    pose.pose.pose.position.y += iter->y;
    pose.pose.pose.position.z += iter->z;
    // pose.pose.position.x += iter->x;
    // pose.pose.position.y += iter->y;
    // pose.pose.position.z += iter->z;
  }
  if(0 != pcl_cloud.size()){
    pose.pose.pose.position.x /= (float) pcl_cloud.size();
    pose.pose.pose.position.y /= (float) pcl_cloud.size();
    pose.pose.pose.position.z /= (float) pcl_cloud.size();
    // pose.pose.position.x /= (float) pcl_cloud.size();
    // pose.pose.position.y /= (float) pcl_cloud.size();
    // pose.pose.position.z /= (float) pcl_cloud.size();
    pose.header = scan->header;
    pub_pose.publish(pose);
    geometry_msgs::PointStamped p;
    p.header = scan->header;
    p.point.x = pose.pose.pose.position.x;
    p.point.y = pose.pose.pose.position.y;
    p.point.z = pose.pose.pose.position.z;
    pub_point.publish(p);
  }
}

int main(int argc, char** argv) {
  ROS_INFO("Hello World!");
  ros::init(argc, argv, "lrf_reflectance");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  nh_private.param<int>("intensity_th", intensity_th, 1000);
  ROS_INFO_STREAM("intensity_th:"<< intensity_th);

  ros::Subscriber scan_sub = nh.subscribe("scan_in", 10000, laserCallback);

  pub = nh.advertise<sensor_msgs::LaserScan>("scan_out", 5);
  pub_pc = nh.advertise<sensor_msgs::PointCloud2>("pc_out", 5);
  pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_out", 5);
  pub_point = nh.advertise<geometry_msgs::PointStamped>("lrf_pose", 5);
  ros::spin();
}
