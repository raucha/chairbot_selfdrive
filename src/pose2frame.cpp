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
geometry_msgs::TransformStamped mapOdomTransMsg;
bool is_got_trans = false;

/**
 * @brief PFで得られた姿勢と"map"->"base_footprint"が一致するよう，"map"->"odom"を逆算して発行．
 * @param input PFで得られた姿勢.frame_idは"map"であることが前提．
 */
// void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &arg) {
void pose_callback(const nav_msgs::Odometry::ConstPtr &arg) {
  // ROS_INFO("pose_callback");
  // static ros::Time last_time = ros::Time::now();
  // if(ros::Duration(5) > ros::Time::now()-last_time){
  // odomTransformBroadcaster->sendTransform(mapOdomTransMsg);
  //   return;
  // }
  // last_time = ros::Time::now();

  tf2::Transform sframePoseTrans;
  tf2::fromMsg(arg->pose.pose, sframePoseTrans);
  tf2::Transform mapPoseTrans;
  tf2::Transform baseOdomTrans;
  try {
    tf2::Transform mapSframeTrans;
    tf2::fromMsg(tfBuffer_.lookupTransform("map", arg->header.frame_id,
                                           /*ros::Time::now()*/ arg->header.stamp,
                                           ros::Duration(3.0)).transform,
                 mapSframeTrans);
    mapPoseTrans = mapSframeTrans * sframePoseTrans;
    mapPoseTrans.getOrigin().setZ(0);
    ROS_INFO_STREAM("mapPoseTrans x:" << mapPoseTrans.getOrigin().getX()
                                      << "y:" << mapPoseTrans.getOrigin().getY()
                                      << "z:" << mapPoseTrans.getOrigin().getZ());
    tf2::fromMsg(
        tfBuffer_.lookupTransform("base_footprint", "odom", /*ros::Time::now()*/ arg->header.stamp,
                                  ros::Duration(3.0)).transform,
        baseOdomTrans);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("Could NOT transform : %s", ex.what());
    return;
  }

  tf2::Transform mapOdomTrans;
  mapOdomTrans = mapPoseTrans * baseOdomTrans;
  mapOdomTransMsg.transform = tf2::toMsg(mapOdomTrans);
  //! @todo: 時間が遅れすぎるので現在時刻を利用している．
  mapOdomTransMsg.header.stamp = ros::Time::now();  /*arg->header.stamp*/;
  mapOdomTransMsg.header.frame_id = "map";
  mapOdomTransMsg.child_frame_id = "odom";

  is_got_trans = true;
}

/**
 * @brief 10Hz周期で"map"->"odom"を発行．
 * @param input 実行時間の遅れとか．不要．
 */
void timer_callback(const ros::TimerEvent& event){
  if(!is_got_trans){
    return;
  }
  //"map" -> "odom"のtfを発行
  mapOdomTransMsg.header.stamp = ros::Time::now();
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
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), timer_callback);

  ros::spin();
  delete odomTransformBroadcaster;
  return 0;
}
