#include "be2r_cmpc_unitree.hpp"

using namespace std;
// using namespace USDK;

static float controller_dt = 0.002;
static float controller_freq = 500;
static bool show_loop_time = false;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "unitree_ctrl");
  ros::NodeHandle n;

  readRosParam("/static_params/controller_dt", controller_dt);
  readRosParam("/static_params/show_loop_time", show_loop_time);
  controller_freq = 1.0 / controller_dt;
  cout << "[main] Controller dt: " << controller_dt << " main loop frequency: " << controller_freq << endl;

  // ros::Rate rate(MAIN_LOOP_RATE);
  ros::Rate rate(controller_freq);

  ROS_INFO("Initialization...");

  Body_Manager unitree;

  unsigned long tick = 0;

  // while (tick < MAIN_LOOP_RATE / 3)
  while (tick < controller_freq / 3.0)
  {
    tick++;
    ROS_INFO_ONCE("WAIT ROS Init");
    rate.sleep();
    ros::spinOnce();
  }

  unitree.init();

  UNITREE_LEGGED_SDK::LoopFunc* loop_udpSend = nullptr;
  UNITREE_LEGGED_SDK::LoopFunc* loop_udpRecv = nullptr;

  if (unitree.is_udp_connection)
  {
    ROS_WARN("UDP CONNECTION");
    loop_udpSend = new UNITREE_LEGGED_SDK::LoopFunc("udp_send", controller_dt, 3, boost::bind(&Body_Manager::UDPSend, &unitree));
    loop_udpRecv = new UNITREE_LEGGED_SDK::LoopFunc("udp_recv", controller_dt, 3, boost::bind(&Body_Manager::UDPRecv, &unitree));
    loop_udpSend->start();
    loop_udpRecv->start();
  }

  ROS_INFO("Initialization Done!");

  Timer t2;

  while (ros::ok())
  {
    if (show_loop_time)
    {
      t2.start();
    }

    unitree.run();

    if (show_loop_time)
    {
      cout << "Whole loop time: " << t2.getMs() << endl;
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
