#include "be2r_cmpc_unitree.hpp"

using namespace std;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "unitree_ctrl");
  ros::NodeHandle n;
  ros::Rate rate(MAIN_LOOP_RATE);

  ROS_INFO("Initialization...");

  Body_Manager unitree;

  unsigned long tick = 0;

  while (tick < MAIN_LOOP_RATE / 3)
  {
    tick++;
    ROS_INFO_ONCE("WAIT ROS Init");
    rate.sleep();
    ros::spinOnce();
  }
  unitree.init();

  ROS_INFO("Initialization Done!");

  while (ros::ok())
  {
    unitree.run();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
