#include "be2r_cmpc_unitree.hpp"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "unitree_ctrl");
  ros::NodeHandle n;
  ros::Rate rate(500);

  ROS_INFO("Initialization...");

  while(ros::ok())
  {

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}