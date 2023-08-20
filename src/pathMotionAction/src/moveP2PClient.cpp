#include <moveP2PClient.h>

void doneCallback(const actionlib::SimpleClientGoalState& state,
                  const pathMotionAction::PurePursuitResultConstPtr& result)
{
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Action finished: Robot State:, %d, %s", result->status, state.toString().c_str());
    ROS_INFO("Final position: (%f, %f), final theta: %f", result->final_xy[0], result->final_xy[1], result->final_theta);
  }
  else if(state == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    ROS_INFO("Task preempted:", state.toString().c_str());
  }
}

void activeCallback()
{
  ROS_INFO("Action is active.");
}

void feedbackCallback(const pathMotionAction::PurePursuitFeedbackConstPtr& feedback)
{
  ROS_INFO("Received feedback. Robot State:, %d,  Current position: (%f, %f), current theta: %f, car head: %s",
           feedback->status,
           feedback->cur_xy[0], feedback->cur_xy[1], feedback->cur_theta, feedback->car_head ? "true" : "false");
}
