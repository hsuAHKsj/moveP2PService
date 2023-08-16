#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <pathMotionAction/PurePursuitAction.h>
#include <globalPlanInterface.h>

struct robotState
{
    double x;
    double y;
    double theta;
    double headState;
};

// 对于该Action的运行执行状态
enum class stateEnum{
    MOVING = 0,                     // 正在运动中
    STOP  = 1,                      // 运动暂停
    FINISHED  = 2,                  // 等待接受目标
    ERROR  = 3                      // 模块发生异常
};

enum class headDirection{
    FORKAHEAD    = -1,                     // 正在运动中
    FORBACKWARD  = 1,                      // 运动暂停
};


typedef pathMotionAction::PurePursuitResult ppResult;
typedef pathMotionAction::PurePursuitFeedback ppFeedback;

class PurePursuitServer
{
public:
  PurePursuitServer();

  void goalCallback();

  void preemptCallback();

private:
  void executePurePursuit(const boost::array<double, 2>& start_xy, const boost::array<double, 2>& end_xy, const double blendingRaduis);

  void pubGlobalPath(const std::vector<OutputData>& outputDataList);

  ros::Publisher global_path_pub_;  // 
  ros::NodeHandle nh_;  // 节点信息
  actionlib::SimpleActionServer<pathMotionAction::PurePursuitAction> as_;  // 纯跟踪服务
  robotState m_roborState;
  std::string package_path;

  robotState vectorXd2RobState(const Eigen::VectorXd& ro)
  {
    robotState retState;
    retState.x = ro(0);
    retState.y = ro(1);
    retState.theta = ro(2);
    retState.headState = ro(3);
    return retState;
  }

  ppResult convert2PPResult(const robotState& roborState, const stateEnum& state)
  {
        ppResult result;
        result.status = (int)state; // Example status, replace with actual data
        result.final_xy[0] = roborState.x; // Final position
        result.final_xy[1] = roborState.y; // Final position
        result.final_theta = roborState.theta; // Final theta
        return result;
  }

  ppFeedback convert2PPFeedback(const robotState& roborState, const stateEnum& state)
  {
        ppFeedback feedback;
        feedback.status = (int)state;
        feedback.cur_xy[0] = roborState.x;
        feedback.cur_xy[1] = roborState.y;
        feedback.cur_theta = roborState.theta;
        if(roborState.headState == (int)headDirection::FORKAHEAD) feedback.car_head = true;
        if(roborState.headState == (int)headDirection::FORBACKWARD) feedback.car_head = false;
        return feedback;
  }

};