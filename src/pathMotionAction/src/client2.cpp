#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pathMotionAction/PurePursuitAction.h>
#include <PurePusitClient.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "client2");

  actionlib::SimpleActionClient<pathMotionAction::PurePursuitAction> ac("pure_pursuit", true);

  ROS_INFO("Waiting for action server to start...");
  ac.waitForServer();
  
  ROS_INFO("Action server started.");

  pathMotionAction::PurePursuitGoal goal;
  goal.start_xy = {2.18, 1}; // Example values, replace with actual data
  goal.end_xy = {5.8, 3.6};   // Example values, replace with actual data
  goal.turning_radius = 0.2;
  
  ac.sendGoal(goal, &doneCallback, &activeCallback, &feedbackCallback);

  ros::spin();

  return 0;
}
