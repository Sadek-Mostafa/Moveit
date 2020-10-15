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



	ros::Publisher chatter_pub;
	ros::Publisher chatter_pub1;
	ros::Publisher chatter_pub2;
	ros::Publisher chatter_pub3;
	ros::Publisher chatter_pub4;
	ros::Publisher chatter_pub5;
	geometry_msgs::PoseStamped targ;
	beginner_tutorials::Bools ros_bools;
	beginner_tutorials::Bools rob_bools;
	geometry_msgs::PoseStamped feedback;

void Callback(const geometry_msgs::PoseStamped::ConstPtr& rob, const geometry_msgs::PoseStamped::ConstPtr& cam, const beginner_tutorials::Bools::ConstPtr& rob_bool)
	{

		


		tf::Quaternion q = tf::createQuaternionFromRPY( rob->pose.orientation.z, rob->pose.orientation.y, rob->pose.orientation.x);

 		
		targ.pose.position.x=(cam->pose.position.x)*1000;
		targ.pose.position.y=(cam->pose.position.y)*1000;
		targ.pose.position.z=(cam->pose.position.z)*1000+100;
		targ.pose.orientation.x=(cam->pose.orientation.x);
		targ.pose.orientation.y=(cam->pose.orientation.y);
		targ.pose.orientation.z=(cam->pose.orientation.z);
		targ.header.stamp.sec=cam->header.stamp.sec;
		targ.header.frame_id="14";

		

		feedback.pose.position.x=1;
		feedback.pose.position.y=1;
		feedback.pose.position.z=1;
		feedback.pose.orientation.x=1;
		feedback.pose.orientation.y=1;
		feedback.pose.orientation.z=1;



		rob_bools.b1 = rob_bool->b1;
		rob_bools.b1 = rob_bool->b2;
		rob_bools.b1 = rob_bool->b3;
		rob_bools.b1 = rob_bool->b4;
		rob_bools.b1 = rob_bool->b5;
		rob_bools.b1 = rob_bool->b6;
		rob_bools.b1 = rob_bool->b7;
		rob_bools.b1 = rob_bool->b8;
		rob_bools.b1 = rob_bool->b9;
		rob_bools.b1 = rob_bool->b10;
		rob_bools.b1 = rob_bool->b11;
		rob_bools.b1 = rob_bool->b12;
		rob_bools.b1 = rob_bool->b13;
		rob_bools.b1 = rob_bool->b14;
		rob_bools.b1 = rob_bool->b15;
		rob_bools.b1 = rob_bool->b16;



		if (targ.pose.orientation.x < 0) {
		   targ.pose.orientation.x = abs (targ.pose.orientation.x);
		   ros_bools.b1=true;
		} else {
		   ros_bools.b1=false;}

		if (targ.pose.orientation.y < 0) {
		   targ.pose.orientation.y = abs (targ.pose.orientation.y);
		   ros_bools.b2=true;
		} else {
		   ros_bools.b2=false;}

		if (targ.pose.orientation.z < 0) {
		   targ.pose.orientation.z = abs (targ.pose.orientation.z);
		   ros_bools.b3=true;
		} else {
		   ros_bools.b3=false;}


		

		
		
		ROS_INFO_STREAM(endl << "RobCoor.X: "<< rob->pose.position.x << endl << "RobCoor.Y: "<< rob->pose.position.y << endl << "RobCoor.Z: "<< rob->pose.position.z << endl << "RobCoor.A: "<< rob->pose.orientation.x << endl << "RobCoor.B: "<< rob->pose.orientation.y << endl << "RobCoor.C: "<< rob->pose.orientation.z << endl << "RobCoor.A-que: "<< q.x() << endl << "RobCoor.B-que: "<< q.y() << endl << "RobCoor.C-que: "<< q.z() << endl);
		
		ROS_INFO_STREAM(endl << "Targ.x: "<< targ.pose.position.x << endl << "Targ.y: "<< targ.pose.position.y <<endl << "Targ.z: " << targ.pose.position.z << endl << "Targ.a: " << targ.pose.orientation.x << endl << "Targ.b: " << targ.pose.orientation.y << endl << "Targ.c: " << targ.pose.orientation.z << endl);


		ROS_INFO_STREAM(endl << "rob_bools"<< endl << rob_bools << endl);
		ROS_INFO_STREAM(endl << "ros_bools"<< endl << ros_bools << endl);
		
	}








int main(int argc, char** argv)

{

  ros::init(argc, argv, "listen_tf");

  ros::NodeHandle node;

 // ros::NodeHandle n;

//code part of publish to pubsub

  ros::Publisher chatter_pub = node.advertise<geometry_msgs::PoseStamped>("/object_pos", 1000);
  ros::Publisher chatter_pub1 = node.advertise<geometry_msgs::PoseStamped>("/car_pos", 1000);

	message_filters::Subscriber<geometry_msgs::PoseStamped> PoseStamped_1(node, "/robposi", 1);
	message_filters::Subscriber<geometry_msgs::PoseStamped> PoseStamped_2(node, "/object_pos", 1);
	message_filters::Subscriber<beginner_tutorials::Bools> Bools_3(node, "/rob_bools", 1);
typedef sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped,beginner_tutorials::Bools>MySyncPolicy;
		Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), PoseStamped_1, PoseStamped_2,Bools_3);
	sync.registerCallback(boost::bind(&Callback, _1,_2,_3));

	// chatter_pub3 = node.advertise<geometry_msgs::PoseStamped>("/targpos", 1000);
	 chatter_pub4 = node.advertise<geometry_msgs::PoseStamped>("/robpos", 1000);
	 chatter_pub5 = node.advertise<beginner_tutorials::Bools>("/ros_bools", 1000);
//en of code part to publish to pubsub

  int count = 0;
  
  tf::TransformListener listener;
  tf::TransformListener listener1;

  ros::Rate rate(10);

  while (node.ok()){

    geometry_msgs::PoseStamped object;
    geometry_msgs::PoseStamped car;

    tf::StampedTransform transform;
    tf::StampedTransform transform1;
	

    try {

    listener.waitForTransform("correct_frame", "tag_3", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform("correct_frame", "tag_3", ros::Time(0), transform);

    listener1.waitForTransform("correct_frame", "tag_4", ros::Time(0), ros::Duration(10.0) );
    listener1.lookupTransform("correct_frame", "tag_4", ros::Time(0), transform1);

    double yaw, pitch, roll;
    transform.getBasis().getRPY(roll, pitch, yaw);

    double yaw1, pitch1, roll1;
    transform1.getBasis().getRPY(roll1, pitch1, yaw1);

    object.pose.position.x=transform.getOrigin().x();
    object.pose.position.y=transform.getOrigin().y();
    object.pose.position.z=transform.getOrigin().z();

    object.pose.orientation.x=transform.getRotation().x();
    object.pose.orientation.y=1;
    object.pose.orientation.z=transform.getRotation().z();    
    object.pose.orientation.w=transform.getRotation().w();    
	feedback.pose.position.x=1;
		feedback.pose.position.y=1;
		feedback.pose.position.z=1;
		feedback.pose.orientation.x=1;
		feedback.pose.orientation.y=1;
		feedback.pose.orientation.z=1;



    //cam.pose.orientation.x=yaw*180.0/M_PI;
    //cam.pose.orientation.y=pitch*180.0/M_PI;
    //cam.pose.orientation.z=roll*180.0/M_PI;    
    //cam.header.stamp=ros::Time::now();
    //cam.header.stamp.sec=12;
    object.header.frame_id="14";
    object.header.seq=count;
    object.header.stamp = ros::Time::now();
    car.header.stamp = ros::Time::now();

    car.pose.position.x=transform1.getOrigin().x();
    car.pose.position.y=transform1.getOrigin().y();
    car.pose.position.z=transform1.getOrigin().z();

    car.pose.orientation.x=transform1.getRotation().x();
    car.pose.orientation.y=1;
    car.pose.orientation.z=transform1.getRotation().z(); 
    car.pose.orientation.z=transform1.getRotation().w();    


    //car.pose.orientation.x=yaw1*180.0/M_PI;
    //car.pose.orientation.y=pitch1*180.0/M_PI;
    //car.pose.orientation.z=roll1*180.0/M_PI;    
    //car.header.stamp=ros::Time::now();
    //cam.header.stamp.sec=12;
    car.header.frame_id="14";
    car.header.seq=count;
	//chatter_pub3.publish(targ);

		chatter_pub4.publish(feedback);
		chatter_pub5.publish(ros_bools);
    chatter_pub.publish(object);
    chatter_pub1.publish(car);

    ros::spinOnce();

    //ROS_INFO_STREAM(endl << "Targ.x: "<< transform.getOrigin().x() << endl);

    //ROS_INFO_STREAM(endl << "At time " << transform.stamp_.toSec() << endl);



  //  ROS_INFO_STREAM(endl << "Fixed frame: tag_2 " << endl << "Moving frame: tag_3 " << endl);

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
