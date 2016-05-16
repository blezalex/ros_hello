#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  tf::Quaternion laserQuat = tf::createQuaternionFromYaw(3.14159);

  while(n.ok()){

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "base_link";
    odom_trans.child_frame_id = "laser";

    odom_trans.transform.translation.x = 0.03;
    odom_trans.transform.translation.y = 0;
    odom_trans.transform.translation.z = 0.1;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    broadcaster.sendTransform(odom_trans);


    // broadcaster.sendTransform(
    //   tf::StampedTransform(
    //     tf::Transform(laserQuat, tf::Vector3(0.0, 0.0, 0.1)),
    //     ros::Time::now(),"base_link", "laser"));

    //std::cout << "test" << std::endl;

    r.sleep();
  }
}