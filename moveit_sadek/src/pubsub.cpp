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

class Alignment
{

private:

	ros::NodeHandle nh_;
	ros::Publisher chatter_pub;

	message_filters::Subscriber<geometry_msgs::PointStamped> PointStamped_1;
	message_filters::Subscriber<geometry_msgs::PointStamped> PointStamped_2;

	typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, geometry_msgs::PointStamped> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync;

public:

	Alignment(ros::NodeHandle &nh) :
		PointStamped_1(nh_, "/robposi", 10),
		PointStamped_2(nh_, "/campos", 10),
		sync(MySyncPolicy(10), PointStamped_1, PointStamped_2)
	  {
		  nh_=nh;
		  sync.registerCallback(boost::bind(&Alignment::Callback, this, _1,_2));
                  chatter_pub = nh_.advertise<geometry_msgs::PointStamped>("/targpos", 1000);
		  
	  }



	void Callback(const geometry_msgs::PointStamped::ConstPtr& rob, const geometry_msgs::PointStamped::ConstPtr& cam)
	{

		geometry_msgs::PointStamped targ;
  
		targ.point.x=(cam->point.x)*1000;
		targ.point.y=(cam->point.y)*1000;
		targ.point.z=(cam->point.z)*1000+100;
		targ.header.stamp.sec=cam->header.stamp.sec;
		targ.header.frame_id="14";

		chatter_pub.publish(targ);
		
		ROS_INFO_STREAM(endl << "RobCoor.x: "<< rob->point.x << endl << "RobCoor.y: "<< rob->point.y << endl << "RobCoor.z: "<< rob->point.z << endl);
		
		ROS_INFO_STREAM(endl << "Targ.x: "<< targ.point.x << endl << "Targ.y: "<< targ.point.y <<endl << "Targ.z: " << targ.point.z << endl);
		
	}


};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "pubsub");

  ros::NodeHandle nh;

  Alignment matching(nh);
//matching.home();

  ros::spin();

  return 0;
}

