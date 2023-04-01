#include "ids_peak_cam.h"
#include <signal.h>


bool sigint_received = false;

void sigsegv_handler(int sig)
{
  signal(SIGSEGV, SIG_DFL);
  fprintf(stderr, "Segementation fualt, stopping camera.\n");
  ROS_ERROR("Segementation faulat, stopping camera.");
  ros::shutdown();
}

void sigint_handler(int signum)
{
  ROS_WARN("SIGINT RECEIVED!\n");
  sigint_received = true;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "ids_peak_cam_node");
  
  ros::NodeHandle node;

  ros::NodeHandle nh("ids_peak_cam");
  signal(SIGSEGV, &sigsegv_handler);

  ids_cam cam(nh);
  signal(SIGINT, &sigint_handler);

  while(node.ok() && !sigint_received)
  {
    cam.poll();
    ros::spinOnce();
  }

  ROS_WARN("ROS SHUTDOWN!");

  cam.shutdown();

  return 0;
}

