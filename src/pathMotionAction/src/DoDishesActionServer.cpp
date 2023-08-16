#include <ros/ros.h>
#include <ros/package.h>
#include <PurePursuitServer.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pure_pursuit_server");
  PurePursuitServer server;
  ros::spin();
  return 0;
}
