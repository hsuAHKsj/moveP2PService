  
#include <moveP2PServer.h>
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
    nh_.getParam("car_linear_velocity_", car_linear_velocity_);
    nh_.getParam("car_angular_velocity_", car_angular_velocity_);
    nh_.getParam("preview_distance_", preview_dis_);
    nh_.getParam("is_use_pid_param_", is_use_pid_param_);
    nh_.getParam("Velocity_KP", pidParam.Velocity_KP);//PID参数加载
    nh_.getParam("Velocity_KI", pidParam.Velocity_KI);
    nh_.getParam("Velocity_KD", pidParam.Velocity_KD);
    nh_.getParam("Position_KP", pidParam.Position_KP);
    nh_.getParam("Position_KI", pidParam.Position_KI);
    nh_.getParam("Position_KD", pidParam.Position_KD);
    nh_.getParam("rounding_radius", blendingRaduis);


    defaultHead = true;
    as_.registerGoalCallback(boost::bind(&PurePursuitServer::goalCallback, this));
    as_.registerPreemptCallback(boost::bind(&PurePursuitServer::preemptCallback, this));
    global_path_pub_ = nh_.advertise<nav_msgs::Path>("/global_path", 1);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

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

    // Execute the pure pursuit logic
    executePurePursuit(goal->start_xy, goal->end_xy);
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
  void PurePursuitServer::executePurePursuit(const boost::array<double, 2>& start_xy, const boost::array<double, 2>& end_xy)
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

    robotStateList robotStateList = outputDataList2VectorList(outputDataList);

    // 输出最近的一个id 
    int nearest_id = 0;
    
    // 找到并定义出起始zuobiao
    const double rate = 30.0;
    ros::Rate loop_rate(rate);

    // 执行，并且根据不同返回返回值返回状态
    while (ros::ok())
    {  
      if (as_.isPreemptRequested()){
          ROS_ERROR("exit the looping for preempted.");
          // 发布执行完成的信息
          ppResult result = convert2PPResult(m_roborState, stateEnum::PREEMPTED);
          as_.setPreempted(result);
          break;
      }

      if(is_current_pose_sub_)
      {
        run(robotStateList, cur_roborState);
        switch(car_state_)//主状态机
        {
          case stateEnum::PURE_PURSUIT:{
            ppFeedback feedback = convert2PPFeedback(m_roborState, stateEnum::PURE_PURSUIT);
            as_.publishFeedback(feedback);
            std::cout << "小车状态：纯跟踪跟线" << std::endl;
            break;
          }
          case stateEnum::CORRECT_THE_LINE:{ 
            ppFeedback feedback = convert2PPFeedback(m_roborState, stateEnum::CORRECT_THE_LINE);
            as_.publishFeedback(feedback);
            std::cout << "小车状态：纠正线路" << std::endl;
            break;
          }
          case stateEnum::FINISHED:
            std::cout << "小车状态：正常停车" << std::endl;
            break; 
          default:
            break;
        }
        is_current_pose_sub_ = false;
      }
      else
      {
        std::cout << "no odom info recevied. " << std::endl;
      }


      static int stop_count = 0;
      if (car_state_ == stateEnum::FINISHED)
      {
          stop_count ++;
          if(stop_count > 10) break;
      }else{
        stop_count = 0;
      }
      ros::spinOnce();
      loop_rate.sleep();
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

  