#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
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

  ros::Publisher chatter_pub = node.advertise<geometry_msgs::PoseStamped>("/campos", 1000);
  ros::Publisher chatter_pub1 = node.advertise<geometry_msgs::PoseStamped>("/carpos", 1000);

//en of code part to publish to pubsub

  int count = 0;
  
  tf::TransformListener listener;
  tf::TransformListener listener1;

  ros::Rate rate(10.0);

  while (node.ok()){

    geometry_msgs::PoseStamped cam;
    geometry_msgs::PoseStamped car;

    tf::StampedTransform transform;
    tf::StampedTransform transform1;
    try {

    listener.waitForTransform("tag_2", "tag_3", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform("tag_2", "tag_3", ros::Time(0), transform);

    listener1.waitForTransform("tag_2", "tag_4", ros::Time(0), ros::Duration(10.0) );
    listener1.lookupTransform("tag_2", "tag_4", ros::Time(0), transform1);

    double yaw, pitch, roll;
    transform.getBasis().getRPY(roll, pitch, yaw);

    double yaw1, pitch1, roll1;
    transform1.getBasis().getRPY(roll1, pitch1, yaw1);

    cam.pose.position.x=transform.getOrigin().x();
    cam.pose.position.y=transform.getOrigin().y();
    cam.pose.position.z=transform.getOrigin().z();

    cam.pose.orientation.x=transform.getRotation().x();
    cam.pose.orientation.y=transform.getRotation().y();
    cam.pose.orientation.z=transform.getRotation().z();    





    //cam.pose.orientation.x=yaw*180.0/M_PI;
    //cam.pose.orientation.y=pitch*180.0/M_PI;
    //cam.pose.orientation.z=roll*180.0/M_PI;    
    cam.header.stamp=ros::Time::now();
    //cam.header.stamp.sec=12;
    cam.header.frame_id="14";
    cam.header.seq=count;

    car.pose.position.x=transform1.getOrigin().x();
    car.pose.position.y=transform1.getOrigin().y();
    car.pose.position.z=transform1.getOrigin().z();

    car.pose.orientation.x=transform1.getRotation().x();
    car.pose.orientation.y=transform1.getRotation().y();
    car.pose.orientation.z=transform1.getRotation().z(); 



    //car.pose.orientation.x=yaw1*180.0/M_PI;
    //car.pose.orientation.y=pitch1*180.0/M_PI;
    //car.pose.orientation.z=roll1*180.0/M_PI;    
    car.header.stamp=ros::Time::now();
    //cam.header.stamp.sec=12;
    car.header.frame_id="14";
    car.header.seq=count;

    chatter_pub.publish(cam);
    chatter_pub1.publish(car);

    ros::spinOnce();

    //ROS_INFO_STREAM(endl << "Targ.x: "<< transform.getOrigin().x() << endl);

    ROS_INFO_STREAM(endl << "At time " << transform.stamp_.toSec() << endl);



    ROS_INFO_STREAM(endl << "Fixed frame: tag_2 " << endl << "Moving frame: tag_3 " << endl);

    ROS_INFO_STREAM(endl << "- Translation: " << endl << "X: " << transform.getOrigin().x() << endl << "Y: " << transform.getOrigin().y() << endl << "Z: " << transform.getOrigin().z() << endl);

    ROS_INFO_STREAM(endl << "- Rotation in Quaternion: " << endl << "X: " << transform.getRotation().x() << endl << "Y: " << transform.getRotation().y() << endl << "Z: " << transform.getRotation().z() << endl << "Q: " << transform.getRotation().z() << endl);

    ROS_INFO_STREAM(endl << "- Rotation in degree: " << endl << "A: " << yaw*180.0/M_PI << endl << "B: " << pitch*180.0/M_PI << endl << "C: " << roll*180.0/M_PI << endl);

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
