#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include<math.h>
#include <vector>
#include <iostream>
namespace rvt = rviz_visual_tools;
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

  geometry_msgs::PoseStamped original_pose = move_group.getCurrentPose();
  geometry_msgs::PoseStamped target_pose = original_pose;  

  std::vector<geometry_msgs::PoseStamped> waypoints;   

  float center_x, center_y, x, radius, rs;

  radius = 0.5;
  rs = radius * radius;
  center_x = target_pose.pose.position.x - radius;
  center_y = target_pose.pose.position.y;

  // right half of circle
  for (x = target_pose.pose.position.x; x >= center_x - radius; x -= 0.01)
  {
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = center_y - sqrt((rs - (x - center_x) * (x - center_x)));
    waypoints.push_back(target_pose);
  }

  // left part of circle
  for (x = target_pose.pose.position.x; x <= center_x + radius; x += 0.01)
  {
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = center_y + sqrt((rs - (x - center_x) * (x - center_x)));
    waypoints.push_back(target_pose);
  }

  ros::Rate loop_rate(10); //Frequency

  while (ros::ok())
  {

    for (std::size_t i = 0; i < waypoints.size(); i++)
    {
      move_group.setApproximateJointValueTarget(waypoints[i]); // To calculate the trajectory

      move_group.move(); // Move the robot
    }

    target_pose = move_group.getCurrentPose();

    if ((abs(original_pose.pose.position.x - target_pose.pose.position.x) < 0.1) && (abs(original_pose.pose.position.y - target_pose.pose.position.y)  < 0.1))
    {
      break; // Basically, check if we reached the desired pose.position
    }

    loop_rate.sleep();
  }

  ROS_INFO("Done");
  ros::shutdown();
  return 0;
}
