#include <ros/ros.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
// #include <geometry_msgs/PoseStamped.h>

// #include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

tf2_ros::Buffer tfBuffer_;
tf2_ros::TransformListener *tfListener_;
tf2_ros::TransformBroadcaster *odomTransformBroadcaster;

/**
 * @brief 3次元地図を点群で購読，1回のみ
 * @param input PointCloud2,マップデータ
 */
// void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &arg) {
void pose_callback(const nav_msgs::Odometry::ConstPtr &arg) {
  // ROS_INFO("pose_callback");

  tf2::Transform sframePoseTrans;
  tf2::fromMsg(arg->pose.pose, sframePoseTrans);
  tf2::Transform mapPoseTrans;
  tf2::Transform baseOdomTrans;
  try {
    tf2::Transform mapSframeTrans;
    tf2::fromMsg(tfBuffer_.lookupTransform("map", arg->header.frame_id, ros::Time(0) /*arg->header.stamp*/).transform,
                 mapSframeTrans);
    mapPoseTrans = mapSframeTrans * sframePoseTrans;
    mapPoseTrans.getOrigin().setZ(0);
    ROS_INFO_STREAM("mapPoseTrans x:" << mapPoseTrans.getOrigin().getX() << "y:" << mapPoseTrans.getOrigin().getY() << "z:" << mapPoseTrans.getOrigin().getZ());
    tf2::fromMsg(tfBuffer_.lookupTransform("base_footprint", "odom", ros::Time(0) /*arg->header.stamp*/).transform,
                 baseOdomTrans);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("Could NOT transform : %s", ex.what());
  }

  tf2::Transform mapOdomTrans;
  mapOdomTrans = mapPoseTrans * baseOdomTrans;
  geometry_msgs::TransformStamped mapOdomTransMsg;
  mapOdomTransMsg.transform = tf2::toMsg(mapOdomTrans);
  mapOdomTransMsg.header.stamp = ros::Time::now();  // arg->header.stamp;
  mapOdomTransMsg.header.frame_id = "map";
  mapOdomTransMsg.child_frame_id = "odom";

  //"map" -> "odom"のtfを発行
  odomTransformBroadcaster->sendTransform(mapOdomTransMsg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pose2frame");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // tf
  tf2_ros::TransformListener tmp_listener(tfBuffer_);
  tfListener_ = &tmp_listener;
  odomTransformBroadcaster = new tf2_ros::TransformBroadcaster();

  // Subscribers
  ros::Subscriber pose_sub = nh.subscribe("pose", 10, pose_callback);

  ros::spin();
  delete odomTransformBroadcaster;
  return 0;
}
