#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/StdVector>

using namespace std;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "tet");
  ros::NodeHandle n;
  ros::Rate rate(100);

  ROS_INFO("Initialization Done!");

  while (ros::ok())
  {

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
