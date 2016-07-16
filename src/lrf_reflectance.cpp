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
#include <cmath>
#include <limits>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>

using namespace std;

double xmax, xmin, ymax, ymin;
string ref_frame_id;
int intensity_th;
ros::Publisher pub, pub_pc, /*pub_pose,*/ pub_point, pub_pc_filtered;

int callbacked_num = 0;
#define INITIALS_SIZE 5
sensor_msgs::LaserScan initials[INITIALS_SIZE];


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  ROS_INFO("got scan data");
  //スキャンの初期値を記録
  if(INITIALS_SIZE > callbacked_num){
    initials[callbacked_num] = *scan;
    callbacked_num++;
    return;
  }
  // 初期スキャンの平均値取得
  sensor_msgs::LaserScan initials_ave = *scan;
  for(int lasers=0;lasers<scan->ranges.size();lasers++){
    initials_ave.ranges.at(lasers) = 0.0;
    initials_ave.intensities.at(lasers) = 0.0;
    int vailds=0;
    for (int scans=0;scans<INITIALS_SIZE;scans++){
      // ROS_INFO_STREAM("lasers:"<<lasers<<"  scans:"<<scans);
      if(!std::isfinite(initials[scans].ranges.at(lasers))){
        continue;
      }
      vailds++;
      initials_ave.ranges.at(lasers) += initials[scans].ranges.at(lasers);
    }
    initials_ave.ranges.at(lasers) /= ((float)vailds);
  }
  sensor_msgs::LaserScan s_in = *scan;
  for(int lasers=0;lasers<scan->ranges.size();lasers++){
    const float cur = s_in.ranges.at(lasers);
    const float ini = initials_ave.ranges.at(lasers);
    if(!std::isfinite(cur) || !std::isfinite(ini)){
      continue;
    }
    if(0.2>std::fabs(cur-ini)){
      s_in.ranges.at(lasers) = s_in.range_max + 1.0;
    }
  }
  // 反射率の低い点を排除
  for (int i = 0; i < s_in.ranges.size(); i++) {
    if (intensity_th > s_in.intensities.at(i)) {
      s_in.ranges.at(i) = s_in.range_max + 1.0;
    }
  }
  sensor_msgs::PointCloud2 cloud;
  string fid = s_in.header.frame_id;
  tf::TransformListener tfl;
  static laser_geometry::LaserProjection projector_;
  projector_.transformLaserScanToPointCloud(fid, s_in, cloud, tfl);

  pub_pc.publish(cloud);

  // pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud, *pcl_cloud);

  // 等間隔にするためにダウンサンプル
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_tmp2 (new pcl::PointCloud<pcl::PointXYZ>);  
  vg.setInputCloud (pcl_cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  // vg.setLeafSize (1.0f, 1.0f, 1.0f);
  vg.filter (*pc_tmp2);
  *pcl_cloud = *pc_tmp2;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (pcl_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.05); // 5cm
  ec.setMinClusterSize (5);
  ec.setMaxClusterSize (10000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (pcl_cloud);
  ec.extract (cluster_indices);

  if(0==cluster_indices.size()){return ;}
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_tmp (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (std::vector<int>::const_iterator pit = cluster_indices.at(0).indices.begin (); pit != cluster_indices.at(0).indices.end (); ++pit){
     pc_tmp->points.push_back (pcl_cloud->points[*pit]); //*
  }
  pc_tmp->width = pc_tmp->points.size();
  pc_tmp->height = 1;
  pc_tmp->is_dense = true;
  *pcl_cloud = *pc_tmp;
  // pcl_cloud->header = scan->header;
  pcl_conversions::toPCL(scan->header, pcl_cloud->header);

  pub_pc_filtered.publish(pcl_cloud);

  // // Create and accumulate points
  // CentroidPoint<pcl::PointXYZ> centroid;
  // for(auto iter=pcl_cloud.begin(); iter != cloud.end(); iter++){
  //   centroid.add(*iter);
  // }
  // pcl::PointXYZ center;
  // centroid.get (center);
  pcl::PointXYZ center;
  geometry_msgs::PoseWithCovarianceStamped pose;

  for (auto iter = pcl_cloud->begin(); iter != pcl_cloud->end(); iter++) {
    pose.pose.pose.position.x += iter->x;
    pose.pose.pose.position.y += iter->y;
    pose.pose.pose.position.z += iter->z;
    // pose.pose.position.x += iter->x;
    // pose.pose.position.y += iter->y;
    // pose.pose.position.z += iter->z;
  }
  if (0 != pcl_cloud->size()) {
    pose.pose.pose.position.x /= (float)pcl_cloud->size();
    pose.pose.pose.position.y /= (float)pcl_cloud->size();
    pose.pose.pose.position.z /= (float)pcl_cloud->size();
    // pose.pose.position.x /= (float) pcl_cloud->size();
    // pose.pose.position.y /= (float) pcl_cloud->size();
    // pose.pose.position.z /= (float) pcl_cloud->size();
    pose.header = scan->header;
    // pub_pose.publish(pose);
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
  ROS_INFO_STREAM("intensity_th:" << intensity_th);

  ros::Subscriber scan_sub = nh.subscribe("scan_in", 10, laserCallback);

  pub = nh.advertise<sensor_msgs::LaserScan>("scan_out", 5);
  pub_pc = nh.advertise<sensor_msgs::PointCloud2>("pc_out", 5);
  pub_pc_filtered = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("pc_filtered", 5);
  // pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_out", 5);
  pub_point = nh.advertise<geometry_msgs::PointStamped>("lrf_pose", 5);
  ros::spin();
}
