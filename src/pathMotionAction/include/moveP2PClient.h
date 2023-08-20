#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pathMotionAction/PurePursuitAction.h>

void doneCallback(const actionlib::SimpleClientGoalState& state,
                  const pathMotionAction::PurePursuitResultConstPtr& result);
void activeCallback();
void feedbackCallback(const pathMotionAction::PurePursuitFeedbackConstPtr& feedback);