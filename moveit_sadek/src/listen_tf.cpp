#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PointStamped.h"
#include <time.h>
#include <sstream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/approximate_time.h> 

using namespace message_filters;
using namespace std;
using namespace geometry_msgs;

int main(int argc, char** argv){

  ros::init(argc, argv, "listen_tf");

  ros::NodeHandle node;

 // ros::NodeHandle n;

//code part of publish to pubsub

  ros::Publisher chatter_pub = node.advertise<geometry_msgs::PointStamped>("/campos", 1000);

//en of code part to publish to pubsub

  int count = 0;
  
  tf::TransformListener listener;

  ros::Rate rate(10.0);

  while (node.ok()){

    geometry_msgs::PointStamped cam;

    tf::StampedTransform transform;
    try {

    listener.waitForTransform("tag_2", "tag_3", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform("tag_2", "tag_3", ros::Time(0), transform);

    cam.point.x=transform.getOrigin().x();
    cam.point.y=transform.getOrigin().y();
    cam.point.z=transform.getOrigin().z();
    cam.header.stamp=ros::Time::now();
    //cam.header.stamp.sec=12;
    cam.header.frame_id="14";
    cam.header.seq=count;

    chatter_pub.publish(cam);

    ros::spinOnce();

    //ROS_INFO_STREAM(endl << "Targ.x: "<< transform.getOrigin().x() << endl);

    ROS_INFO_STREAM(endl << "At time " << transform.stamp_.toSec() << endl);

    double yaw, pitch, roll;
    transform.getBasis().getRPY(roll, pitch, yaw);

    ROS_INFO_STREAM(endl << "Fixed frame: tag_2 " << endl << "Moving frame: tag_3 " << endl);

    ROS_INFO_STREAM(endl << "- Translation: " << endl << "X: " << transform.getOrigin().x() << endl << "Y: " << transform.getOrigin().y() << endl << "Z: " << transform.getOrigin().z() << endl);

    ROS_INFO_STREAM(endl << "- Rotation in Quaternion: " << endl << "X: " << transform.getRotation().x() << endl << "Y: " << transform.getRotation().y() << endl << "Z: " << transform.getRotation().z() << endl << "Q: " << transform.getRotation().z() << endl);

    ROS_INFO_STREAM(endl << "- Rotation in degree: " << endl << "X: " << roll*180.0/M_PI << endl << "Y: " << pitch*180.0/M_PI << endl << "Z: " << yaw*180.0/M_PI << endl);

} catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
}
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    rate.sleep();
  }
  return 0;
};
