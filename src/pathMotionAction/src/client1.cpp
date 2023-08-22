#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pathMotionAction/PurePursuitAction.h>
#include <moveP2PClient.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "client1");

  actionlib::SimpleActionClient<pathMotionAction::PurePursuitAction> ac("pure_pursuit", true);

  ROS_INFO("Waiting for action server to start...");
  ac.waitForServer();
  
  ROS_INFO("Action server started.");

  pathMotionAction::PurePursuitGoal goal;
  goal.start_xy = {6, 3.5}; // Example values, replace with actual data
  goal.end_xy = {1, 5};   // Example values, replace with actual data

  ac.sendGoal(goal, &doneCallback, &activeCallback, &feedbackCallback);

  ros::spin();

  return 0;
}
