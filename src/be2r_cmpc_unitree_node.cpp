#include "be2r_cmpc_unitree.hpp"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "unitree_ctrl");
  ros::NodeHandle n;
  ros::Rate rate(MAIN_LOOP_RATE);

  ROS_INFO("Initialization...");

  Body_Manager unitree;

  unitree.init();

  unsigned long tick = 0;

  while (tick < MAIN_LOOP_RATE / 3)
  {
    tick++;
    ROS_INFO_ONCE("WAIT ROS Init");
    rate.sleep();
    ros::spinOnce();
  }

  ROS_INFO("Initialization Done!");

  while (ros::ok())
  {
    // ROS_INFO("LOOP");
    ros::spinOnce();

    unitree.run();

    rate.sleep();
  }

  return 0;
}