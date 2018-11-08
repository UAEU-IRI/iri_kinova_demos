#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include <moveit/move_group_interface/move_group.h>





int main(int argc, char** argv){

ros::init(argc, argv, "test01");
ros::NodeHandle nh;
ros::AsyncSpinner spinner(1);
spinner.start();
  
  
moveit::planning_interface::MoveGroup group("arm");
ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
moveit_msgs::DisplayTrajectory display_trajectory;


geometry_msgs::Pose target_pose1;
target_pose1.orientation.w = 1.0;
target_pose1.position.x = 0.0;
target_pose1.position.y = -0.5;
target_pose1.position.z = 0.5;
group.setPoseTarget(target_pose1);

moveit::planning_interface::MoveGroup::Plan my_plan;

bool success = group.plan(my_plan);

ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
/* Sleep to give Rviz time to visualize the plan. */
sleep(5.0);


if (success)
{
  ROS_INFO("Visualizing plan 1 (again)");
  display_trajectory.trajectory_start = my_plan.start_state_;
  display_trajectory.trajectory.push_back(my_plan.trajectory_);
  display_publisher.publish(display_trajectory);
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(5.0);
  group.move();
}


/* Uncomment below line when working with a real robot*/
/* group.move() */

ros::spinOnce();
return 0;}
