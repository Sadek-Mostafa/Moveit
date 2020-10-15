
//#####################KUKA KR3 C++ interface ####################################//



#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "std_msgs/Bool.h"
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
 #include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <iostream>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "moveit_sadek/Bools.h"
#include <stdlib.h>

using namespace message_filters;
using namespace std;
namespace rvt = rviz_visual_tools;



ros::Subscriber sub_pos;
ros::Subscriber sub_object;
ros::Publisher pub;
ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
geometry_msgs::PoseStamped Pose ;
geometry_msgs::Pose target_object_pose ;
geometry_msgs::Pose target_car_pose ;
geometry_msgs::Pose target_home_pose;
geometry_msgs::PoseStamped kr3_plan;
geometry_msgs::Pose temp_trajectory;
std_msgs::Bool rob_mag;


//####################### Callback fn for case 0 #######################// 


 void Get_robot_Pose(const geometry_msgs::PoseStamped::ConstPtr& rob)
{

	cout<<"robot status";
	kr3_plan.pose.position.x=rob->pose.position.x;
	kr3_plan.pose.position.y=rob->pose.position.y;
	kr3_plan.pose.position.z=rob->pose.position.z;
	kr3_plan.pose.orientation.x=rob->pose.orientation.x;
	kr3_plan.pose.orientation.y=rob->pose.orientation.y;
	kr3_plan.pose.orientation.z=rob->pose.orientation.z;
	kr3_plan.pose.orientation.w=rob->pose.orientation.w;
	
}


//####################### callback Home_pose for case 1 #######################// 

void Home_pose(const geometry_msgs::PoseStamped::ConstPtr& rob)
{

	 if((rob->pose.position.x <=.2 && rob->pose.position.x >=0.19 && rob->pose.position.y <=.1 && rob->pose.position.y >=-0.1 &&rob->pose.position.z <=.613&&rob->pose.position.z >=0.3))
		{
		  cout<<"robot at home position "<<endl;
		}
	
		ros::Duration(1).sleep();
	   	
}
//#######################autonomus callback fn for case 2  #######################// 

void Get_Target_Pose(const geometry_msgs::PoseStamped::ConstPtr& object,const geometry_msgs::PoseStamped::ConstPtr& car)
{

//assign the incoming messages to target pose 1 and 2 
	target_object_pose.position.x = -(object->pose.position.x);
	target_object_pose.position.y = object->pose.position.y;
	target_object_pose.position.z = -(object->pose.position.z);

	target_object_pose.orientation.x = 0; 
	target_object_pose.orientation.y = 1;
	target_object_pose.orientation.z = 0;
	target_object_pose.orientation.w = 0;

	target_car_pose.position.x = -(car->pose.position.x);
	target_car_pose.position.y = car->pose.position.y;
	target_car_pose.position.z = -(car->pose.position.z);

	target_car_pose.orientation.x = 0; 
	target_car_pose.orientation.y = 1;
	target_car_pose.orientation.z = 0;
	target_car_pose.orientation.w = 0;
	
	  ros::Duration(0.5).sleep();

}


int main(int argc, char** argv)
{
 
ros::init(argc, argv, "Kuka_Autonomous_Mode");

ros::NodeHandle n;


ros::AsyncSpinner spinner(4);
spinner.start();

// Publish Target plan of the object and Car

pub = n.advertise<geometry_msgs::PoseStamped>("/targpos", 50);//publisher target trajectory
pub3 = n.advertise<std_msgs::Bool>("/indros_mag", 1000);
//subscriber for current position 


// Message Filter Definition

message_filters::Subscriber<geometry_msgs::PoseStamped> sub_object(n, "/object_pos", 10);
message_filters::Subscriber<geometry_msgs::PoseStamped> sub_car(n, "/car_pos", 10);

typedef sync_policies::ApproximateTime<geometry_msgs::PoseStamped,geometry_msgs::PoseStamped>MySyncPolicy;
Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_object,sub_car);
sync.registerCallback(boost::bind(&Get_Target_Pose, _1,_2));



//###################################### Moveit Initializations #############################################//


// planning group defination

static const string PLANNING_GROUP = "kr3";// assign the planning group comes from moveit assistant 

moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	  
//Forward kinematics definitions

robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

const robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
kinematic_state->setToDefaultValues();
const robot_state::JointModelGroup* joint_model_groups = kinematic_model->getJointModelGroup  (PLANNING_GROUP);
const vector<string>& joint_names = joint_model_groups->getVariableNames();
//const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("tool");
// RVIZ Visualization Initialization
	  

moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
visual_tools.deleteAllMarkers();

Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
text_pose.translation().z() = 1.75;
visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);


ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str()); 		// print the name of the reference frame for this robot.

ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());		//  print the name of the end-effector link for this group.

moveit_msgs::CollisionObject collision_object;
collision_object.header.frame_id = move_group.getPlanningFrame();


// Plan Object definition

moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
vector<double> joint_group_positions;
current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
moveit::planning_interface::MoveGroupInterface::Plan my_plan;




//######################## Kuka_Autonomous_Mode Node parameters initialization #######################################//
int choise=0;
float r=180/3.14;
int z=0;
rob_mag.data = false;


cout<<"------------------------------------------"<<endl;
cout<<"|1 |current position of KUKA KR3          |"<<endl;
cout<<"|2 |home position of the KUKA KR3         |"<<endl;
cout<<"|3 |autonomous mode                       |"<<endl;
cout<<"|4 | exit                                 |"<<endl;
cout<<"-------------------------------------------"<<endl;
cin>>choise;

switch (choise)
	{



//###############################################################//

//		Case (1) = current position of KUKA KR3		//
 
//###############################################################//
	case(1):
		{
		sub_pos = n.subscribe("/robpos", 1000, &Get_robot_Pose); 
		pub.publish(kr3_plan);
		ros::Duration(0.75).sleep();
		sub_pos.shutdown();

		}

		break;

//###############################################################//

//	Case (2) = Home position of KUKA KR3	  	  	 //
	 
//###############################################################//
 
case(2):

	{
	
	z=1;

	sub_pos = n.subscribe("/robpos", 1000, &Home_pose);
	pub2 = n.advertise<geometry_msgs::PoseStamped>("/targpos", 50);			//publisher target trajectory

	target_home_pose.position.x = -0.020;
	target_home_pose.position.y = 0.001;
	target_home_pose.position.z = 0.511;
	target_home_pose.orientation.x = 0;
	target_home_pose.orientation.y = 1;
	target_home_pose.orientation.z = 0;
	target_home_pose.orientation.w = 0;

	move_group.setPoseTarget(target_home_pose);
	bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


	ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal) %s", success ? "" : "FAILED");
	ROS_INFO_NAMED("tutorial", "Visualizing plan 3 as trajectory line");
	visual_tools.publishAxisLabeled(target_home_pose, "home_pose");
	visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);//visualize the plan trajectory on Rviz


	for (size_t s=1; s<my_plan.trajectory_.joint_trajectory.points.size();s+=1)
		{
		cout << "Length of array = " << s << endl;

		while(z <= s){

				double positions[]={my_plan.trajectory_.joint_trajectory.points[z].positions[0],my_plan.trajectory_.joint_trajectory.points[z].positions[1],my_plan.trajectory_.joint_trajectory.points[z].positions[2],my_plan.trajectory_.joint_trajectory.points[z].positions[3],my_plan.trajectory_.joint_trajectory.points[z].positions[4]};

				ROS_INFO("Joint pos = [ %f, %f, %f, %f, %f]", positions[0]*r, positions[1]*r, positions[2]*r, positions[3]*r, positions[4]*r);

				kinematic_state->setVariablePositions(positions);
				kinematic_state->update();
				//ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
				//ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

				kr3_plan.pose.position.x= temp_trajectory.position.x;
				kr3_plan.pose.position.y= temp_trajectory.position.y;
				kr3_plan.pose.position.z= temp_trajectory.position.z;
				kr3_plan.pose.orientation.x=temp_trajectory.orientation.x;
				kr3_plan.pose.orientation.y=temp_trajectory.orientation.y;
				kr3_plan.pose.orientation.z=temp_trajectory.orientation.z;
				kr3_plan.pose.orientation.w=temp_trajectory.orientation.w;


				pub.publish(kr3_plan);

				ros::Duration(0.5).sleep();
				z+=2;
			   
			    }
			sub_pos.shutdown();
		}

move_group.move();
break;
}							

	//###############################################################//

	//		Case (3) = Autonomous Mode			//
	 
	//###############################################################//






	case(3):

	{


	//########## Autonomous Mode >>> Go to Object Pose ##############//

	z=1;


	// Execute planning of the target object_Pose

	move_group.setPoseTarget(target_object_pose);
	bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	// Visualize the plan

	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
	visual_tools.publishAxisLabeled(target_object_pose, "object_pose");
	visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);

	//Get the planned trajectory and publish it to the IGUS robot for execution
	for (size_t s=1; s<my_plan.trajectory_.joint_trajectory.points.size();s+=2)

		{
			cout << "Length of array = " << s <<endl;

			while(z <= s)
				{

				double positions[]={my_plan.trajectory_.joint_trajectory.points[z].positions[0],my_plan.trajectory_.joint_trajectory.points[z].positions[1],my_plan.trajectory_.joint_trajectory.points[z].positions[2],my_plan.trajectory_.joint_trajectory.points[z].positions[3],my_plan.trajectory_.joint_trajectory.points[z].positions[4]};

				ROS_INFO("Joint pos = [ %f, %f, %f, %f, %f]", positions[0]*r, positions[1]*r, positions[2]*r, positions[3]*r, positions[4]*r);

				kinematic_state->setVariablePositions(positions);
				kinematic_state->update();

				//ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
				//ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

				kr3_plan.pose.position.x= temp_trajectory.position.x;
				kr3_plan.pose.position.y= temp_trajectory.position.y;
				kr3_plan.pose.position.z= temp_trajectory.position.z;
				kr3_plan.pose.orientation.x=temp_trajectory.orientation.x;
				kr3_plan.pose.orientation.y=temp_trajectory.orientation.y;
				kr3_plan.pose.orientation.z=temp_trajectory.orientation.z;
				kr3_plan.pose.orientation.w=temp_trajectory.orientation.w;

				pub.publish(kr3_plan);

				ros::Duration(0.5).sleep();
				z+=2;
			   
			    }
		
		}
	rob_mag.data=true;

	pub3.publish(rob_mag);

	move_group.move();

	ros::Duration(5).sleep();

	z=1;
		

	target_home_pose.position.x = -0.020;
	target_home_pose.position.y = 0.001;
	target_home_pose.position.z = 0.511;
	target_home_pose.orientation.x = 0;
	target_home_pose.orientation.y = 1;
	target_home_pose.orientation.z = 0;
	target_home_pose.orientation.w = 0;

	move_group.setPoseTarget(target_home_pose);
	bool successq = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


	ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal) %s", success ? "" : "FAILED");
	ROS_INFO_NAMED("tutorial", "Visualizing plan 3 as trajectory line");
	visual_tools.publishAxisLabeled(target_home_pose, "home_pose");
	visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);//visualize the plan trajectory on Rviz


	for (size_t s=1; s<my_plan.trajectory_.joint_trajectory.points.size();s+=1)
		{
		cout << "Length of array = " << s << endl;

		while(z <= s){

				double positions[]={my_plan.trajectory_.joint_trajectory.points[z].positions[0],my_plan.trajectory_.joint_trajectory.points[z].positions[1],my_plan.trajectory_.joint_trajectory.points[z].positions[2],my_plan.trajectory_.joint_trajectory.points[z].positions[3],my_plan.trajectory_.joint_trajectory.points[z].positions[4]};

				ROS_INFO("Joint pos = [ %f, %f, %f, %f, %f]", positions[0]*r, positions[1]*r, positions[2]*r, positions[3]*r, positions[4]*r);

				kinematic_state->setVariablePositions(positions);
				kinematic_state->update();

				//ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
				//ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());



				kr3_plan.pose.position.x= temp_trajectory.position.x;
				kr3_plan.pose.position.y= temp_trajectory.position.y;
				kr3_plan.pose.position.z= temp_trajectory.position.z;
				kr3_plan.pose.orientation.x=temp_trajectory.orientation.x;
				kr3_plan.pose.orientation.y=temp_trajectory.orientation.y;
				kr3_plan.pose.orientation.z=temp_trajectory.orientation.z;
				kr3_plan.pose.orientation.w=temp_trajectory.orientation.w;

			

				pub.publish(kr3_plan);

				ros::Duration(0.5).sleep();
				z+=2;
			     }
		}
	move_group.move();


	//########################## Autonomous Mode >>> Go to car Pose ###############################################/


	// Execute planning of the target car_Pose

	z=1;

	move_group.setPoseTarget(target_car_pose);


	bool successs = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
	visual_tools.publishAxisLabeled(target_car_pose, "car_pose");
	visual_tools.publishText(text_pose, "car_pose", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);

	//Get the planned trajectory and publish it to the IGUS robot for execution


	for (size_t s=1; s<my_plan.trajectory_.joint_trajectory.points.size();s+=2)

		{
			cout << "Length of array = " << s <<endl;

			while(z <= s)
				{

					double positions[]={my_plan.trajectory_.joint_trajectory.points[z].positions[0],my_plan.trajectory_.joint_trajectory.points[z].positions[1],my_plan.trajectory_.joint_trajectory.points[z].positions[2],my_plan.trajectory_.joint_trajectory.points[z].positions[3],my_plan.trajectory_.joint_trajectory.points[z].positions[4]};

					ROS_INFO("Joint pos = [ %f, %f, %f, %f, %f]", positions[0]*r, positions[1]*r, positions[2]*r, positions[3]*r, positions[4]*r);		// Print planned trajectory
				

				kr3_plan.pose.position.x= temp_trajectory.position.x;
				kr3_plan.pose.position.y= temp_trajectory.position.y;
				kr3_plan.pose.position.z= temp_trajectory.position.z;
				kr3_plan.pose.orientation.x=temp_trajectory.orientation.x;
				kr3_plan.pose.orientation.y=temp_trajectory.orientation.y;
				kr3_plan.pose.orientation.z=temp_trajectory.orientation.z;
				kr3_plan.pose.orientation.w=temp_trajectory.orientation.w;
				pub.publish(kr3_plan);
				ros::Duration(0.5).sleep();
				z+=2;

				}

		}

	rob_mag.data= false;
	pub3.publish(rob_mag);
	move_group.move();
				

	ros::Duration(5).sleep();

	z=1;
		

	target_home_pose.position.x = -0.020;
	target_home_pose.position.y = 0.001;
	target_home_pose.position.z = 0.511;
	target_home_pose.orientation.x = 0;
	target_home_pose.orientation.y = 1;
	target_home_pose.orientation.z = 0;
	target_home_pose.orientation.w = 0;

	move_group.setPoseTarget(target_home_pose);
	bool succesq = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


	ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal) %s", succesq ? "" : "FAILED");
	ROS_INFO_NAMED("tutorial", "Visualizing plan 3 as trajectory line");
	visual_tools.publishAxisLabeled(target_home_pose, "home_pose");
	visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);//visualize the plan trajectory on Rviz


	for (size_t s=1; s<my_plan.trajectory_.joint_trajectory.points.size();s+=1)
		{
		cout << "Length of array = " << s << endl;

		while(z <= s){

				double positions[]={my_plan.trajectory_.joint_trajectory.points[z].positions[0],my_plan.trajectory_.joint_trajectory.points[z].positions[1],my_plan.trajectory_.joint_trajectory.points[z].positions[2],my_plan.trajectory_.joint_trajectory.points[z].positions[3],my_plan.trajectory_.joint_trajectory.points[z].positions[4]};

				ROS_INFO("Joint pos = [ %f, %f, %f, %f, %f]", positions[0]*r, positions[1]*r, positions[2]*r, positions[3]*r, positions[4]*r);

				kinematic_state->setVariablePositions(positions);
				kinematic_state->update();
				//ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
				//ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());



				kr3_plan.pose.position.x= temp_trajectory.position.x;
				kr3_plan.pose.position.y= temp_trajectory.position.y;
				kr3_plan.pose.position.z= temp_trajectory.position.z;
				kr3_plan.pose.orientation.x=temp_trajectory.orientation.x;
				kr3_plan.pose.orientation.y=temp_trajectory.orientation.y;
				kr3_plan.pose.orientation.z=temp_trajectory.orientation.z;
				kr3_plan.pose.orientation.w=temp_trajectory.orientation.w;

			

				pub.publish(kr3_plan);

				ros::Duration(0.5).sleep();
				z+=2;
			     }
		}
	move_group.move();


break;
	}

	//########################## End the Program ###############################################/

	case(4): {
	 cout<<"terminate the program "<<endl;
	break;
	}
	default :
	printf ("\nYou entered wrong choice!! \n");

	break;

	}

	ros::shutdown();
  return 0;}
