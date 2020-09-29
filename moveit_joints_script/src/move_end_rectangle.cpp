#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
// Main moveit libraries are included
int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(0);
  spinner.start(); // For moveit implementation we need AsyncSpinner, we cant use ros::spinOnce()
  static const std::string PLANNING_GROUP = "group1_controller"; /* Now we specify with what group we want work,
  here group1 is the name of my group controller*/
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP); // loading move_group

  const robot_state::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP); //For joint control

  geometry_msgs::PoseStamped current_pose;
  geometry_msgs::PoseStamped target_pose1; 
  geometry_msgs::PoseStamped target_pose2; 
  geometry_msgs::PoseStamped target_pose3; 
  geometry_msgs::PoseStamped target_pose4; 


  current_pose = move_group.getCurrentPose(); /* Retrieving the information about the current position and orientation of the end effector*/

  target_pose1 = current_pose;
  target_pose1.pose.position.x -= 1; //move backwards

  target_pose2 = target_pose1;
  target_pose2.pose.position.y += 0.7; //move left

  target_pose3 = target_pose2;
  target_pose3.pose.position.x += 1; //move ahead

  target_pose4 = target_pose3;
  target_pose4.pose.position.y -= 0.7; //move right

  ros::Rate loop_rate(50); //Frequency

  while (ros::ok())
  {
    move_group.setApproximateJointValueTarget(target_pose1); // To calculate the trajectory
    move_group.move(); // Move the robot

    move_group.setApproximateJointValueTarget(target_pose2); // To calculate the trajectory
    move_group.move(); // Move the robot

    move_group.setApproximateJointValueTarget(target_pose3); // To calculate the trajectory
    move_group.move(); // Move the robot

    move_group.setApproximateJointValueTarget(target_pose4); // To calculate the trajectory
    move_group.move(); // Move the robot

    current_pose = move_group.getCurrentPose();

    if ((abs(current_pose.pose.position.x - target_pose4.pose.position.x) < 0.1) && (abs(current_pose.pose.position.y - target_pose4.pose.position.y)  < 0.1))
    {
      break; // Basically, check if we reached the desired position
    }

    loop_rate.sleep();
  }

  ROS_INFO("Done");
  ros::shutdown();
  return 0;
}
