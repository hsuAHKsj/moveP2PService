  
#include <PurePursuitServer.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <pathMotionAction/PurePursuitAction.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <globalPlanInterface.h>
#include <cmath>
#include <string>
  
  PurePursuitServer::PurePursuitServer() : as_(nh_, "pure_pursuit", false)
  {
    as_.registerGoalCallback(boost::bind(&PurePursuitServer::goalCallback, this));
    as_.registerPreemptCallback(boost::bind(&PurePursuitServer::preemptCallback, this));
    global_path_pub_ = nh_.advertise<nav_msgs::Path>("/global_path", 1);
        // ------------------ 构造轨迹 --------------------------
    package_path = ros::package::getPath("pathMotionAction") + "/config";
    as_.start();
  }

  // 只处理目标，以及抢断
  void PurePursuitServer::goalCallback()
  {
    // Get the goal
    pathMotionAction::PurePursuitGoalConstPtr goal = as_.acceptNewGoal();
    if (!goal)
    {
      ROS_ERROR("Received an error goal.");
      as_.setAborted();
      return;
    }

    // 发布全局轨迹

    // Execute the pure pursuit logic
    executePurePursuit(goal->start_xy, goal->end_xy, goal->turning_radius);
  }

  // 处理抢断
  void PurePursuitServer::preemptCallback()
  {
    ROS_INFO("Goal is preempted.");
    // Implement any necessary cleanup or interruption logic here
    // 这里我们暂停机器人，报警诸如此类
    as_.setPreempted();
  }

  // 功能函数
  void PurePursuitServer::executePurePursuit(const boost::array<double, 2>& start_xy, const boost::array<double, 2>& end_xy, const double blendingRaduis)
  {
    ROS_INFO("Received an new goal.");
    bool success = initGlobalPlanningAlg(package_path);
    settingRadius(blendingRaduis);

    Eigen::Vector2d startPoint(start_xy[0], start_xy[1]);
    Eigen::Vector2d endPoint(end_xy[0], end_xy[1]);

    // 计算参数化路径信息
    const XYThetaList& xytList = computeGloalPath(startPoint, endPoint);

    // 计算并发布一个全局路径，如果计算错误应该返回异常信息，插值后的轨迹运动
    std::vector<OutputData> outputDataList = computeReedsSheppPaths(xytList);
    pubGlobalPath(outputDataList);


    const double rate = 30.0;
    ros::Rate loop_rate(rate);

    for (int i = 0; i < outputDataList.size() && ros::ok(); i++) {
        const OutputData& output = outputDataList[i];
        Eigen::MatrixXd m_path = output.path;
        for(int ii = 0; ii < m_path.rows() && ros::ok(); ii++)
        {
            if (as_.isPreemptRequested()){
                ROS_ERROR("exit the looping for preempted.");
                // 发布执行完成的信息
                ppResult result = convert2PPResult(m_roborState, stateEnum::STOP);
                as_.setPreempted(result);
                break;
            }
            VectorXd v = m_path.row(ii);
            m_roborState = vectorXd2RobState(v);
            ppFeedback feedback = convert2PPFeedback(m_roborState, stateEnum::MOVING);
            as_.publishFeedback(feedback);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    // 发布执行完成的信息
    ppResult result = convert2PPResult(m_roborState, stateEnum::FINISHED);
    as_.setSucceeded(result);
    return;
  }


  // 发布一个全局路径
  void PurePursuitServer::pubGlobalPath(const std::vector<OutputData>& outputDataList)
  {
    nav_msgs::Path pathShow;
    pathShow.header.stamp = ros::Time::now();
    pathShow.header.frame_id = "map";
    pathShow.poses.clear();

    for (int i = 0; i < outputDataList.size(); i++)
    {
        const OutputData& output = outputDataList[i];
        Eigen::MatrixXd m_path = output.path;
        for(int ii = 0; ii < m_path.rows(); ii++)
        {
            Eigen::VectorXd ro = m_path.row(ii);
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose.position.x = ro(0);
            pose_stamped.pose.position.y = ro(1);
            pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(ro(3));
            pathShow.poses.push_back(pose_stamped);
        }
    }
    global_path_pub_.publish(pathShow);

  }