

//#####################IGUS ROBOLINK C++ interface ####################################//


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

ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Subscriber sub_box;
geometry_msgs::PoseStamped Pose;
geometry_msgs::Pose target_object_pose;
geometry_msgs::Pose target_car_pose;
geometry_msgs::Pose target_home_pose;
geometry_msgs::PoseStamped igus_plan ;
trajectory_msgs::JointTrajectoryPoint temp_trajectory;
std_msgs::Bool rob_mag;
std_msgs::Bool box_ready;
void belt_callback(const std_msgs::Bool::ConstPtr& box)

{
//callback for box position

box_ready.data=box->data;


}
void Get_Target_Pose(const geometry_msgs::PoseStamped::ConstPtr& object,const geometry_msgs::PoseStamped::ConstPtr& car)

{

//assign the incoming messages to target pose 1 and 2 


target_object_pose.position.x = -(object->pose.position.x);//-0.10
target_object_pose.position.y = object->pose.position.y;
target_object_pose.position.z = -(object->pose.position.z);//+0.0305
target_object_pose.orientation.x = 0; 
target_object_pose.orientation.y = 1;
target_object_pose.orientation.z = 0;
target_object_pose.orientation.w = 0;




target_car_pose.position.x = -(car->pose.position.x); //+.04
target_car_pose.position.y = (car->pose.position.y);
target_car_pose.position.z = -(car->pose.position.z); //-0.15
target_car_pose.orientation.x = 0; 
target_car_pose.orientation.y = 1;
target_car_pose.orientation.z = 0;
target_car_pose.orientation.w = 0;


ros::Duration(1).sleep();

}



int main(int argc, char** argv)
{
 
ros::init(argc, argv, "Autonomous_Mode");

ros::NodeHandle n;


ros::AsyncSpinner spinner(4);
spinner.start();
box_loop_repeat:
// Publish Target plan of the object and Car

pub = n.advertise<geometry_msgs::PoseStamped>("/targpos", 50);//publisher target trajectory
pub3 = n.advertise<std_msgs::Bool>("/indros_mag", 1000);
sub_box = n.subscribe("/belt_ready", 1000, &belt_callback);
// Message Filter Definition


message_filters::Subscriber<geometry_msgs::PoseStamped> sub_object(n, "/object_pos", 10);
message_filters::Subscriber<geometry_msgs::PoseStamped> sub_car(n, "/car_pos", 10);

typedef sync_policies::ApproximateTime<geometry_msgs::PoseStamped,geometry_msgs::PoseStamped>MySyncPolicy;
Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_object,sub_car);
sync.registerCallback(boost::bind(&Get_Target_Pose, _1,_2));




//###################################### Moveit Initializations #############################################//


// planning group defination

static const string PLANNING_GROUP = "igus_plannig";// assign the planning group comes from moveit assistant 

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


//######################## Autonomous_Mode Node parameters initialization #######################################//

int choice=0;
float r=180/3.14;
int z=0;
rob_mag.data = false;


if (box_ready.data == true){
choice=3;
cout<<"autonomos mode "<<endl;
}


else {
		goto box_loop_repeat;}






cout<<"-----------------------------------------------"<<endl;
cout<<"|1 |Home position of the IGUS Robot             |"<<endl;
cout<<"|2 |Simulation home position of the IGUS Robot |"<<endl;
cout<<"|3 |autonomous mode       		      |"<<endl;
cout<<"|4 | exit        	   	              |"<<endl;
cout<<"-----------------------------------------------"<<endl;
//cin>>choice;
switch(choice)
{




	case(1):

	{

	// modify  of the joints, plan to the new joint space goal and visualize the plan.
	 
	joint_group_positions[0] = 0.349066;//-20
	joint_group_positions[1] = -0.349066;//+19
	joint_group_positions[2] = -0.436332;//+25
	joint_group_positions[3] = -1.48353;//+82
	joint_group_positions[4] = 0;


	move_group.setJointValueTarget(joint_group_positions);

	bool successss = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


	for (size_t s=0; s<my_plan.trajectory_.joint_trajectory.points.size();s+=1)
		{
			cout << "Length of array = " << s << endl;
	
			while(z <= s)
				{

					double positions[]={my_plan.trajectory_.joint_trajectory.points[z].positions[0],my_plan.trajectory_.joint_trajectory.points[z].positions[1],my_plan.trajectory_.joint_trajectory.points[z].positions	[2],my_plan.trajectory_.joint_trajectory.points[z].positions[3],my_plan.trajectory_.joint_trajectory.points[z].positions[4]};

					ROS_INFO("Joint pos = [ %f, %f, %f, %f, %f]", positions[0]*r, positions[1]*r, positions[2]*r, positions[3]*r, positions[4]*r);

					kinematic_state->setVariablePositions(positions);
					kinematic_state->update();
	
					cout << "                   "  << endl;
					cout << ">>>>Point = " << z <<endl;
					cout << "                   "  <<endl;

					temp_trajectory.positions.resize(5);
					temp_trajectory.positions[0] = (my_plan.trajectory_.joint_trajectory.points[z].positions[0]-0.3490)*r;
					temp_trajectory.positions[1] = (my_plan.trajectory_.joint_trajectory.points[z].positions[1]+0.3316)*r;
					temp_trajectory.positions[2] = (my_plan.trajectory_.joint_trajectory.points[z].positions[2]+0.4363)*r;
					temp_trajectory.positions[3] = (my_plan.trajectory_.joint_trajectory.points[z].positions[3]+1.483)*r;
					temp_trajectory.positions[4] = my_plan.trajectory_.joint_trajectory.points[z].positions[4]*r;
					igus_plan.pose.position.x=temp_trajectory.positions[0];
					igus_plan.pose.position.y=temp_trajectory.positions[1];
					igus_plan.pose.position.z=temp_trajectory.positions[2];
					igus_plan.pose.orientation.x=temp_trajectory.positions[3];
					igus_plan.pose.orientation.y=0;
					igus_plan.pose.orientation.z=0;
					igus_plan.pose.orientation.w=0;



					//ros::Duration(0.5).sleep();
					z +=2;

		}
			}

	move_group.move();
	break;
	}


	//###############################################################//

	//	Case (2) = Home position of IGUS in Simulation		//
	 
	//###############################################################//



	//cout<<"Would you like to go to our simulation home position? If yes press 2";



	case(2):

	{
	
		z=1;


		pub2 = n.advertise<geometry_msgs::PoseStamped>("/targpos", 50);			//publisher target trajectory

		target_home_pose.orientation.x = 0;
		target_home_pose.orientation.y = 1;
		target_home_pose.orientation.z = 0;
		target_home_pose.orientation.w = 0;
		target_home_pose.position.x = 0.1950;
		target_home_pose.position.y = 0;
		target_home_pose.position.z = 0.272;

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
	

					temp_trajectory.positions.resize(5);
					temp_trajectory.positions[0] = (my_plan.trajectory_.joint_trajectory.points[z].positions[0]-0.3490)*r;
					temp_trajectory.positions[1] = (my_plan.trajectory_.joint_trajectory.points[z].positions[1]+0.3316)*r;
					temp_trajectory.positions[2] = (my_plan.trajectory_.joint_trajectory.points[z].positions[2]+0.4363)*r;
					temp_trajectory.positions[3] = (my_plan.trajectory_.joint_trajectory.points[z].positions[3]+1.483)*r;
					temp_trajectory.positions[4] = my_plan.trajectory_.joint_trajectory.points[z].positions[4]*r;
					igus_plan.pose.position.x=temp_trajectory.positions[0];
					igus_plan.pose.position.y=temp_trajectory.positions[1];
					igus_plan.pose.position.z=temp_trajectory.positions[2];
					igus_plan.pose.orientation.x=temp_trajectory.positions[3];
					igus_plan.pose.orientation.y=0;
					igus_plan.pose.orientation.z=0;
					igus_plan.pose.orientation.w=0;

					pub.publish(igus_plan);

					//ros::Duration(0.5).sleep();
					z+=2;

				    }
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
		
	z=0;


	// Execute planning of the target object_Pose

	move_group.setPoseTarget(target_object_pose);
	bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	// Visualize the plan

	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
	visual_tools.publishAxisLabeled(target_object_pose, "object_pose");
	visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);

	//Get the planned trajectory and publish it to the IGUS robot for execution

	for (size_t s=0; s<my_plan.trajectory_.joint_trajectory.points.size();++s)

		{
			cout << "Length of array = " << s <<endl;

			while(z <= s)
				{

					double positions[]={my_plan.trajectory_.joint_trajectory.points[z].positions[0],my_plan.trajectory_.joint_trajectory.points[z].positions[1],my_plan.trajectory_.joint_trajectory.points[z].positions[2],my_plan.trajectory_.joint_trajectory.points[z].positions[3],my_plan.trajectory_.joint_trajectory.points[z].positions[4]};

					ROS_INFO("Joint pos = [ %f, %f, %f, %f, %f]", positions[0]*r, positions[1]*r, positions[2]*r, positions[3]*r, positions[4]*r);		// Print planned trajectory
				

					temp_trajectory.positions.resize(5);
					temp_trajectory.positions[0] = (my_plan.trajectory_.joint_trajectory.points[z].positions[0]-0.3490)*r;
					temp_trajectory.positions[1] = (my_plan.trajectory_.joint_trajectory.points[z].positions[1]+0.3316)*r;
					temp_trajectory.positions[2] = (my_plan.trajectory_.joint_trajectory.points[z].positions[2]+0.4363)*r;
					temp_trajectory.positions[3] = (my_plan.trajectory_.joint_trajectory.points[z].positions[3]+1.483)*r;
					temp_trajectory.positions[4] = (my_plan.trajectory_.joint_trajectory.points[z].positions[4])*r;

					igus_plan.pose.position.x=temp_trajectory.positions[0];
					igus_plan.pose.position.y=temp_trajectory.positions[1];
					igus_plan.pose.position.z=temp_trajectory.positions[2];
					igus_plan.pose.orientation.x=temp_trajectory.positions[3];
					igus_plan.pose.orientation.y=0;
					igus_plan.pose.orientation.z=0;
					igus_plan.pose.orientation.w=0;

					//ros::Duration(0.3).sleep();

					z+=1;
					pub.publish(igus_plan);



				}

		}
	move_group.move();
	rob_mag.data=true;
	ros::Duration(6).sleep();
	pub3.publish(rob_mag);

	//########## Autonomous Mode >>> Go to home Pose ##############//
	/*z=0;
		

		target_home_pose.orientation.x = 0;
		target_home_pose.orientation.y = 1;
		target_home_pose.orientation.z = 0;
		target_home_pose.orientation.w = 0;
		target_home_pose.position.x = 0.1950;
		target_home_pose.position.y = 0;
		target_home_pose.position.z = 0.272;

		move_group.setPoseTarget(target_home_pose);
		bool successq = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


		ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal) %s", successq ? "" : "FAILED");
		ROS_INFO_NAMED("tutorial", "Visualizing plan 3 as trajectory line");
		visual_tools.publishAxisLabeled(target_home_pose, "home_pose");
		visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
		visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);//visualize the plan trajectory on Rviz


		for (size_t s=0; s<my_plan.trajectory_.joint_trajectory.points.size();++s)
			{
				cout << "Length of array = " << s << endl;
	

			while(z <= s){

					double positions[]={my_plan.trajectory_.joint_trajectory.points[z].positions[0],my_plan.trajectory_.joint_trajectory.points[z].positions[1],my_plan.trajectory_.joint_trajectory.points[z].positions[2],my_plan.trajectory_.joint_trajectory.points[z].positions[3],my_plan.trajectory_.joint_trajectory.points[z].positions[4]};

					ROS_INFO("Joint pos = [ %f, %f, %f, %f, %f]", positions[0]*r, positions[1]*r, positions[2]*r, positions[3]*r, positions[4]*r);

					kinematic_state->setVariablePositions(positions);
					kinematic_state->update();
	

					temp_trajectory.positions.resize(5);
					temp_trajectory.positions[0] = (my_plan.trajectory_.joint_trajectory.points[z].positions[0]-0.3490)*r;
					temp_trajectory.positions[1] = (my_plan.trajectory_.joint_trajectory.points[z].positions[1]+0.3316)*r;
					temp_trajectory.positions[2] = (my_plan.trajectory_.joint_trajectory.points[z].positions[2]+0.4363)*r;
					temp_trajectory.positions[3] = (my_plan.trajectory_.joint_trajectory.points[z].positions[3]+1.483)*r;
					temp_trajectory.positions[4] = my_plan.trajectory_.joint_trajectory.points[z].positions[4]*r;
					igus_plan.pose.position.x=temp_trajectory.positions[0];
					igus_plan.pose.position.y=temp_trajectory.positions[1];
					igus_plan.pose.position.z=temp_trajectory.positions[2];
					igus_plan.pose.orientation.x=temp_trajectory.positions[3];
					igus_plan.pose.orientation.y=0;
					igus_plan.pose.orientation.z=0;
					igus_plan.pose.orientation.w=0;



					//ros::Duration(0.01).sleep();
					z+=1;
					pub.publish(igus_plan);
				    }
			}

	move_group.move();



	//########################## Autonomous Mode >>> Go to car Pose ###############################################/


	// Execute planning of the target car_Pose

	z=0;

	move_group.setPoseTarget(target_car_pose);


	bool successs = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


	// Visualize the plan

	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
	visual_tools.publishAxisLabeled(target_car_pose, "car_pose");
	visual_tools.publishText(text_pose, "car_pose", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);

	//Get the planned trajectory and publish it to the IGUS robot for execution


	for (size_t s=0; s<my_plan.trajectory_.joint_trajectory.points.size();++s)

		{
			cout << "Length of array = " << s <<endl;

			while(z <= s)
				{

					double positions[]={my_plan.trajectory_.joint_trajectory.points[z].positions[0],my_plan.trajectory_.joint_trajectory.points[z].positions[1],my_plan.trajectory_.joint_trajectory.points[z].positions[2],my_plan.trajectory_.joint_trajectory.points[z].positions[3],my_plan.trajectory_.joint_trajectory.points[z].positions[4]};

					ROS_INFO("Joint pos = [ %f, %f, %f, %f, %f]", positions[0]*r, positions[1]*r, positions[2]*r, positions[3]*r, positions[4]*r);		// Print planned trajectory
				

					temp_trajectory.positions.resize(5);
					temp_trajectory.positions[0] = (my_plan.trajectory_.joint_trajectory.points[z].positions[0]-0.3490)*r;
					temp_trajectory.positions[1] = (my_plan.trajectory_.joint_trajectory.points[z].positions[1]+0.3316)*r;
					temp_trajectory.positions[2] = (my_plan.trajectory_.joint_trajectory.points[z].positions[2]+0.4363)*r;
					temp_trajectory.positions[3] = (my_plan.trajectory_.joint_trajectory.points[z].positions[3]+1.483)*r;
					temp_trajectory.positions[4] = (my_plan.trajectory_.joint_trajectory.points[z].positions[4])*r;

					igus_plan.pose.position.x=temp_trajectory.positions[0];
					igus_plan.pose.position.y=temp_trajectory.positions[1];
					igus_plan.pose.position.z=temp_trajectory.positions[2];
					igus_plan.pose.orientation.x=temp_trajectory.positions[3];
					igus_plan.pose.orientation.y=0;
					igus_plan.pose.orientation.z=0;
					igus_plan.pose.orientation.w=0;

					z+=1;
					pub.publish(igus_plan);

				}
		}
					
	rob_mag.data= false;
	ros::Duration(12).sleep();
	pub3.publish(rob_mag);
	move_group.move();			
	//########## Autonomous Mode >>> Go to home Pose ##############//
	z=1;
		

	target_home_pose.orientation.x = 0;
	target_home_pose.orientation.y = 1;
	target_home_pose.orientation.z = 0;
	target_home_pose.orientation.w = 0;
	target_home_pose.position.x = 0.1950;
	target_home_pose.position.y = 0;
	target_home_pose.position.z = 0.272;

	move_group.setPoseTarget(target_home_pose);
	bool successsq = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


	ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal) %s", successsq ? "" : "FAILED");
	ROS_INFO_NAMED("tutorial", "Visualizing plan 3 as trajectory line");
	visual_tools.publishAxisLabeled(target_home_pose, "home_pose");
	visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);//visualize the plan trajectory on Rviz


	for (size_t s=1; s<my_plan.trajectory_.joint_trajectory.points.size();s+=1)
		{
			cout << "Length of array = " << s << endl;
	

			while(z <= s)
				{

				double positions[]={my_plan.trajectory_.joint_trajectory.points[z].positions[0],my_plan.trajectory_.joint_trajectory.points[z].positions[1],my_plan.trajectory_.joint_trajectory.points[z].positions[2],my_plan.trajectory_.joint_trajectory.points[z].positions[3],my_plan.trajectory_.joint_trajectory.points[z].positions[4]};

				ROS_INFO("Joint pos = [ %f, %f, %f, %f, %f]", positions[0]*r, positions[1]*r, positions[2]*r, positions[3]*r, positions[4]*r);

				kinematic_state->setVariablePositions(positions);
				kinematic_state->update();


				temp_trajectory.positions.resize(5);
				temp_trajectory.positions[0] = (my_plan.trajectory_.joint_trajectory.points[z].positions[0]-0.3490)*r;
				temp_trajectory.positions[1] = (my_plan.trajectory_.joint_trajectory.points[z].positions[1]+0.3316)*r;
				temp_trajectory.positions[2] = (my_plan.trajectory_.joint_trajectory.points[z].positions[2]+0.4363)*r;
				temp_trajectory.positions[3] = (my_plan.trajectory_.joint_trajectory.points[z].positions[3]+1.483)*r;
				temp_trajectory.positions[4] = my_plan.trajectory_.joint_trajectory.points[z].positions[4]*r;
				igus_plan.pose.position.x=temp_trajectory.positions[0];
				igus_plan.pose.position.y=temp_trajectory.positions[1];
				igus_plan.pose.position.z=temp_trajectory.positions[2];
				igus_plan.pose.orientation.x=temp_trajectory.positions[3];
				igus_plan.pose.orientation.y=0;
				igus_plan.pose.orientation.z=0;
				igus_plan.pose.orientation.w=0;



				//ros::Duration(0.02).sleep();
				z+=1;
				pub.publish(igus_plan);
			       }
		}

	move_group.move();*/
	choice=0;

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
return 0;

 }
