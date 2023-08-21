#pragma once
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <pathMotionAction/PurePursuitAction.h>
#include <globalPlanInterface.h>
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>


struct robotState
{
    double x;
    double y;
    double theta;
    int headState;
};

struct PIDParam
{
  double Velocity_KP; 
  double Velocity_KI;
  double Velocity_KD;//速度PID系数
  double Position_KP; 
  double Position_KI; 
  double Position_KD;//位置PID系数  
};

// 对于该Action的运行执行状态
enum class stateEnum{
    PURE_PURSUIT = 0,                     // 正在运动中
    CORRECT_THE_LINE,                     // 纠正线路
    CAR_STOP  = 1,                        // 运动暂停
    CIRCUMVENT = 2,                       //绕障
    FINISHED  = 3,                  // 等待接受目标
    PREEMPTED = -1,                 // 任务被抢占了
    ERROR  = -2                      // 计算错误
};

enum class headDirection{
    FORKAHEAD    = -1,                     // 向前
    FORBACKWARD  = 1,                      // 向后
};

enum class stateEnum;



typedef pathMotionAction::PurePursuitResult ppResult;
typedef pathMotionAction::PurePursuitFeedback ppFeedback;
typedef std::vector<robotState> robotStateList;

class PurePursuitServer
{
public:
  PurePursuitServer();

  void goalCallback();

  void preemptCallback();

private:
  void executePurePursuit(const boost::array<double, 2>& start_xy, const boost::array<double, 2>& end_xy);

  void pubGlobalPath(const std::vector<OutputData>& outputDataList);

  int getIndex(const robotStateList rs_list, const robotState& rs);

  void run(const robotStateList rs_list, const robotState& rs);

  inline double thetaLimit(double yaw);//角度限制 0~180度，-180度~0

  ros::Publisher global_path_pub_;  // 
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber odom_sub_;

  ros::NodeHandle nh_;  // 节点信息
  actionlib::SimpleActionServer<pathMotionAction::PurePursuitAction> as_;  // 纯跟踪服务
  robotState m_roborState;
  robotState cur_roborState;
  stateEnum car_state_;

  double blendingRaduis;        // 交融半径
  double car_angular_velocity_; // 用户设定的旋转速度
  double car_linear_velocity_;  // 用户设定的线速度

  double preview_dis_;  // 运动参数

  bool defaultHead;
  bool is_current_pose_sub_;
  bool is_use_pid_param_;

  std::string package_path;
  PIDParam pidParam;

  double Incremental_PI(double Velocity, double Target);
  double Position_PID(double Position, double target);
  double Xianfu(double value, double Amplitude);//限制最大速度幅度

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


  // 接受 odom 信息将其转化为
void odomCallback(const nav_msgs::Odometry &msg) 
{
    cur_roborState.x = msg.pose.pose.position.x;
    cur_roborState.y = msg.pose.pose.position.y;
    // if(defaultHead == true){
    //     cur_roborState.theta = M_PI + tf::getYaw(msg.pose.pose.orientation);
    // }
    // else{
    //     cur_roborState.theta = tf::getYaw(msg.pose.pose.orientation);
    // }
    cur_roborState.theta =  tf::getYaw(msg.pose.pose.orientation);
    is_current_pose_sub_ = true;
}

robotStateList outputDataList2VectorList(const vector<OutputData>& outputDataList)
{
  robotStateList rs_vec;
  for (int i = 0; i < outputDataList.size() && ros::ok(); i++) {
      const OutputData& output = outputDataList[i];
      Eigen::MatrixXd m_path = output.path;
      for(int ii = 0; ii < m_path.rows() && ros::ok(); ii++)
      {
          VectorXd v = m_path.row(ii);
          m_roborState = vectorXd2RobState(v);
          rs_vec.push_back(m_roborState);
      }
  }
  return rs_vec;
}

};
