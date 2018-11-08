#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group.h>





int main(int argc, char** argv){

ros::init(argc, argv, "test01");
ros::NodeHandle nh;

moveit::planning_interface::MoveGroup group("arm");
ros::Rate rate(100); 

while (ros::ok())
{
std::cout<< group.getPlanningFrame().c_str()  <<std::endl;
ros::spinOnce();
rate.sleep();
}



return 0;}
