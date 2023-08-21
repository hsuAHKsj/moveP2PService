   
  #include <moveP2PServer.h>

  int PurePursuitServer::getIndex(const robotStateList rs_list, const robotState& rs)
  {
    // 方案:通过计算当前坐标和目标轨迹距离，找到一个最小距离的索引号
    int index;
    std::vector<double> best_points;
    for (int i = 0; i < rs_list.size(); i++) 
    {
      robotState rs_check = rs_list.at(i);
      // 遍历所有路径点和当前位置的距离，保存到数组中
      float lad = sqrt(pow(rs.x - rs_check.x, 2) + pow(rs.y - rs_check.y, 2));
      best_points.push_back(lad);
    }

    index = distance(best_points.begin(), min_element(best_points.begin(), best_points.end()));
    std::cout << "index_1 " << index << std::endl;

    int temp_index;
    for (int i = index; i < rs_list.size(); i++) 
    {
      robotState rs_check = rs_list.at(i);
      // 遍历路径点和预瞄点的距离，从最小横向位置的索引开始
      float dis = sqrt(pow(rs_list.at(index).x - rs_check.x, 2) + pow(rs_list.at(index).y - rs_check.y, 2));
      // 判断跟预瞄点的距离
      if (dis < preview_dis_) 
      {
        temp_index = i;
      } 
      else 
      {
        break;
      }
    }
    index = temp_index;
    std::cout << "index_2 " << index << std::endl;
    if(rs_list.at(index).headState == 1)
    {
        defaultHead = true;
    }else{
        defaultHead = true;
    }
    

    return index;
  }
  

  inline double PurePursuitServer::thetaLimit(double yaw)//角度限制 0~180度，-180度~0
  {
    double theta = yaw;
    if(theta > M_PI)
    {
      theta = theta - 2 * M_PI;
    }
    else if(theta < -M_PI)
    {
      theta = theta + 2 * M_PI;
    }
    return theta;
  } 


  void PurePursuitServer::run(const robotStateList rs_list, const robotState& rs)
  {
    cout << "Into run" << endl;
    int index = getIndex(rs_list, rs);
    std::cout << "index " << index << std::endl;
    robotState rs_index = rs_list.at(index);
    m_roborState = rs_list.at(index);
    
    robotState rs_end = rs_list.back();

    double alpha = thetaLimit(atan2(rs_index.y - cur_roborState.y, rs_index.x - cur_roborState.x) - cur_roborState.theta);
    std::cout << "alpha " << alpha << std::endl;//0~180度，-180度~0

    // 当前点和目标点的距离Id
    double dl = sqrt(pow(rs_index.y - cur_roborState.y, 2) + pow(rs_index.x - cur_roborState.x, 2));

    double curvature_k = 2 * sin(alpha) / dl;//跟踪曲率 k = 2 * sin(a) / Ld
    std::cout << "curvature_k " << curvature_k << std::endl;

    double dis_pos = sqrt(pow(rs_end.y - cur_roborState.y, 2) + pow(rs_end.x - cur_roborState.x, 2));//距离终点的距离
    std::cout << "dis_pos " << dis_pos << std::endl;

    if((alpha >= 10.0 * M_PI / 180.0 || alpha <= -10.0 * M_PI / 180.0) && dl <= 0.5)//纠正线路
    {
      car_state_ = stateEnum::CORRECT_THE_LINE;//小车状态：纠正线路

      geometry_msgs::Twist vel_msg;//发布速度
      vel_msg.linear.x = 0.0;
      if(curvature_k >= 0.0)//区分往左边还是往右边掉头
      {
        vel_msg.angular.z = car_angular_velocity_;
      }
      else
      {
        vel_msg.angular.z = -car_angular_velocity_;
      }
      cmd_vel_pub_.publish(vel_msg); 
    }
    else
    {
      double theta = car_linear_velocity_ * curvature_k;
      std::cout << "theta " << theta << std::endl;

      // 发布小车运动指令
      if(dl > 0.12 || index != (rs_list.size()-1)) 
      {
        car_state_ = stateEnum::PURE_PURSUIT;//小车状态：纯跟踪跟线

        geometry_msgs::Twist vel_msg;//发布速度
        if(is_use_pid_param_)//使用PID调速
        {
            //增加串级PID控制线速度
            double motor = Position_PID(-dis_pos, 0.0);//小车当前位置到目标位置的距离，距离逐渐增加
            double sign;
            if(defaultHead == true) sign = 1;
            else sign = -1;

            //Todo:
            double cur_v = 0;
            vel_msg.linear.x = sign * Incremental_PI(cur_v, motor);
            std::cout << "小车线速度 " << vel_msg.linear.x << std::endl;
            vel_msg.angular.z = theta;
            cmd_vel_pub_.publish(vel_msg);

        }
        else//不使用PID调速
        {
            vel_msg.linear.x = car_linear_velocity_;
            vel_msg.angular.z = theta;
            cmd_vel_pub_.publish(vel_msg);
        }
      } 
      else 
      {
        car_state_ = stateEnum::FINISHED;//小车状态：正常停车

        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        cmd_vel_pub_.publish(vel_msg);
      }
    }
  }
  
    // 函数功能：增量PI控制器
  // 入口参数：当前速度，目标速度
  // 返回  值：控制的速度
  // 根据增量式离散PID公式 
  // Speed+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
  // e(k)代表本次偏差 
  // e(k-1)代表上一次的偏差  以此类推 
  // speed代表增量输出
  // 在我们的速度控制闭环系统里面，只使用PI控制
  // Speed+=Kp[e（k）-e(k-1)]+Ki*e(k)
  double PurePursuitServer::Incremental_PI(double Velocity, double Target)
  {   
    static double Bias, Speed, Last_bias, Integral_bias;
    Bias = Target - Velocity;//计算偏差
    Integral_bias += Bias;
    Integral_bias = Xianfu(Integral_bias, 1.0);//速度最大限制在0.8
    Speed = pidParam.Velocity_KP * Bias + pidParam.Velocity_KI * Integral_bias + pidParam.Velocity_KD *(Bias - Last_bias);//增量式PI控制器
    Last_bias = Bias;//保存上一次偏差 

    if(Speed > 1.0)
      Speed = 1.0;
    if(Speed < -1.0)
      Speed = -1.0;

    return Speed;//增量输出
  }

  double PurePursuitServer::Xianfu(double value, double Amplitude)//限制最大速度幅度
  {
    double temp;
    if(value > Amplitude)
      temp = Amplitude;
    else if(value < -Amplitude)
      temp = -Amplitude;
    else
      temp = value;

    return temp;
  }

  // 函数功能：位置式PID控制器
  // 入口参数：当前位置，目标位置
  // 返回  值：控制的速度
  // 根据位置式离散PID公式 
  // Speed=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
  // e(k)代表本次偏差 
  // e(k-1)代表上一次的偏差  
  // ∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
  // Speed代表输出
  double PurePursuitServer::Position_PID(double Position, double target)
  {   
    static float Bias, Speed, Integral_bias, Last_Bias;
    Bias = target - Position;//计算偏差
    Integral_bias += Bias;//求出偏差的积分
    Integral_bias = Xianfu(Integral_bias, 1.0);
    Speed = pidParam.Position_KP * Bias + pidParam.Position_KI * Integral_bias + pidParam.Position_KD * (Bias - Last_Bias);//位置式PID控制器
    Last_Bias = Bias;//保存上一次偏差 
    
    if(Speed > 1.0)
      Speed = 1.0;
    if(Speed < -1.0)
      Speed = -1.0;

    return Speed;//增量输出
  }