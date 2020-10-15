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
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include "beginner_tutorials/Bools.h"


using namespace message_filters;
using namespace std;
using namespace geometry_msgs;


int main(int argc, char** argv)

{

ros::init(argc, argv, "Transform_Listener");

ros::NodeHandle node;

ros::Rate rate(1);


ros::Publisher pub = node.advertise<geometry_msgs::PoseStamped>("/object_pos", 1000);
ros::Publisher pub1 = node.advertise<geometry_msgs::PoseStamped>("/car_pos", 1000);
ros::Publisher pub2 = node.advertise<geometry_msgs::PoseStamped>("/robot_pos", 1000);
ros::Publisher pub3 = node.advertise<std_msgs::Bool>("/belt_ready", 1000);
 
tf::TransformListener listener;
tf::TransformListener listener1;
tf::TransformListener listener2;
 std_msgs::Bool box;
    box.data=true; 

while (node.ok()){
	
    geometry_msgs::PoseStamped object;
    geometry_msgs::PoseStamped car;
    geometry_msgs::PoseStamped robot;

    tf::StampedTransform transform;
    tf::StampedTransform transform1;
    tf::StampedTransform transform2;
   

    try {



    listener.waitForTransform("correct_frame", "tag_3", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform("correct_frame", "tag_3", ros::Time(0), transform);

    listener1.waitForTransform("correct_frame", "tag_4", ros::Time(0), ros::Duration(10.0) );
    listener1.lookupTransform("correct_frame", "tag_4", ros::Time(0), transform1);

   /* listener2.waitForTransform("base_link", "beacon", ros::Time(0), ros::Duration(10.0) );
    listener2.lookupTransform("base_link", "beacon", ros::Time(0), transform2);
*/
    double yaw1, pitch1, roll1;
    transform2.getBasis().getRPY(roll1, pitch1, yaw1);


    object.header.frame_id="object_pos";
    object.header.stamp = ros::Time::now();

    object.pose.position.x=transform.getOrigin().x();
    object.pose.position.y=transform.getOrigin().y();
    object.pose.position.z=transform.getOrigin().z();

    object.pose.orientation.x=transform.getRotation().x();
    object.pose.orientation.y=transform.getRotation().y();
    object.pose.orientation.z=transform.getRotation().z();    
    object.pose.orientation.w=transform.getRotation().w();    

    /*object.pose.position.x=0;
    object.pose.position.y=0;
    object.pose.position.z=0;

    object.pose.orientation.x=0;
    object.pose.orientation.y=0;
    object.pose.orientation.z=0;    
    object.pose.orientation.w=0; */   

    car.header.frame_id="car_pos";
    car.header.stamp = ros::Time::now();

    car.pose.position.x=transform1.getOrigin().x();
    car.pose.position.y=transform1.getOrigin().y();
    car.pose.position.z=transform1.getOrigin().z();

    car.pose.orientation.x=transform1.getRotation().x();
    car.pose.orientation.y=transform1.getRotation().y();
    car.pose.orientation.z=transform1.getRotation().z(); 
    car.pose.orientation.w=transform1.getRotation().w(); 

    
    car.header.frame_id="robot_pos";
    car.header.stamp = ros::Time::now();

    robot.pose.position.x=transform2.getOrigin().x();
    robot.pose.position.y=transform2.getOrigin().y();
    robot.pose.position.z=transform2.getOrigin().z();

    robot.pose.orientation.x=roll1*180.0/M_PI;
    robot.pose.orientation.y=pitch1*180.0/M_PI;
    robot.pose.orientation.z=yaw1*180.0/M_PI; 
    robot.pose.orientation.w=0;     


    pub.publish(object);
    pub1.publish(car);
    pub2.publish(robot);
   pub3.publish(box);
    ros::spinOnce();

    
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
