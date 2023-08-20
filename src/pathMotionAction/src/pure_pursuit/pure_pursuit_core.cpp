#include "agv_robot/pure_pursuit.h"

namespace PurePursuitNS
{
  PurePursuit::PurePursuit() : is_global_path_sub_(false), is_current_pose_sub_(false) 
  {
    nh_.getParam("car_linear_velocity_", car_linear_velocity_);
    nh_.getParam("car_angular_velocity_", car_angular_velocity_);
    nh_.getParam("preview_distance_", preview_distance_);
    nh_.getParam("is_use_pid_param_", is_use_pid_param_);
    nh_.getParam("is_use_detour_", is_use_detour_);
    nh_.getParam("Velocity_KP", Velocity_KP);//PID参数加载
    nh_.getParam("Velocity_KI", Velocity_KI);
    nh_.getParam("Velocity_KD", Velocity_KD);
    nh_.getParam("Position_KP", Position_KP);
    nh_.getParam("Position_KI", Position_KI);
    nh_.getParam("Position_KD", Position_KD);

    initial();//初始化

    global_path_sub_ = nh_.subscribe("/global_path", 1, &PurePursuit::globalPathCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 10, &PurePursuit::odomCallback, this);
    head_direction_sub_ = nh_.subscribe("/is_Head_Revert", 1, &PurePursuit::headCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  }

  void PurePursuit::initial()
  {
    k_ = 0.1;
    preview_dis_ = k_ * car_linear_velocity_ + preview_distance_;
    point_num_ = 0;  //保存路径点的个数
    target_index_ = point_num_ - 1;
    defaultHead = true;

    //雷达避障碍参数
    x_pillar_ = 0.0;//机器人距离物体在X轴上的距离
    y_pillar_ = 0.0;//机器人距离物体在Y轴上的距离
    alpha_pillar_ = 0.0;//机器人于物体之间的角度
    smallest_distance_ = 10.0;//定义了一个接收激光雷达扫描的距离

    //雷达绕障参数
    is_record_pose_ = true;
    is_distance_flag_ = true;
    is_back_finish_ = false;
    is_turn_finish_ = false;
    left_obs_num_ = 0;
    right_obs_num_ = 0;
  }

  //计算发送给模型车的转角
  void PurePursuit::odomCallback(const nav_msgs::Odometry &msg) 
  {
      ros::Time current_timestamp = msg.header.stamp;
      current_pose_x_ = msg.pose.pose.position.x;
      current_pose_y_ = msg.pose.pose.position.y;
      if(defaultHead == true){
          current_pose_yaw_ = M_PI + tf::getYaw(msg.pose.pose.orientation);
      }
      else{
          current_pose_yaw_ = tf::getYaw(msg.pose.pose.orientation);
      }
      
      // 检查是否是第一个时间点，如果是，则仅保存当前时间戳和位置
      if (previous_timestamp.isZero()) {
          previous_timestamp = current_timestamp;
          previous_pose_x = current_pose_x_;
          previous_pose_y = current_pose_y_;
          previous_pose_yaw = current_pose_yaw_;
          return;
      }

      // 计算时间间隔
      double dt = (current_timestamp - previous_timestamp).toSec();

      // 计算位置差异
      double delta_x = current_pose_x_ - previous_pose_x;
      double delta_y = current_pose_y_ - previous_pose_y;

      // 计算角速度
      double delta_yaw = current_pose_yaw_ - previous_pose_yaw;
      double angular_velocity = delta_yaw / dt;

      // 计算线速度
      double linear_velocity = sqrt(delta_x * delta_x + delta_y * delta_y) / dt;

      // 更新先前的时间戳和位置
      previous_timestamp = current_timestamp;
      previous_pose_x = current_pose_x_;
      previous_pose_y = current_pose_y_;
      previous_pose_yaw = current_pose_yaw_;
    
      // current_velocity_.;

      current_velocity_.angular.z = angular_velocity; // 设置角速度
      current_velocity_.linear.x = linear_velocity; // 设置线速度

      is_current_pose_sub_ = true;
  }

  void PurePursuit::globalPathCallback(const nav_msgs::Path &msg) 
  {
    point_num_ = msg.poses.size();
    //std::cout << "point_num_ " << point_num_ << std::endl;

    r_x_.clear();
    r_y_.clear();
    for (int i = 0; i < point_num_; i++) 
    {
      r_x_.push_back(msg.poses[i].pose.position.x);
      r_y_.push_back(msg.poses[i].pose.position.y);
    }

    is_global_path_sub_ = true;
  }

  void PurePursuit::scanCallback(const sensor_msgs::LaserScanConstPtr &msg)
  {
    //使用arr_size接收激光雷达扫描一次的激光点数（(最大角度-最小角度)/单位角度 = 激光点的个数）
    int arr_size = floor((msg->angle_max - msg->angle_min) / msg->angle_increment);
    for(int i = 0; i < arr_size; i++) 
    {
      //cout << " " << i << " " << msg->ranges[i] << endl;
      if(msg->ranges[i] < 0.15) //滤波
      {
        continue;
      }
      if(msg->ranges[i] < smallest_distance_) 
      {
        smallest_distance_ = msg->ranges[i];
        //并计算出角弧度
        alpha_pillar_ = (msg->angle_min + i * msg->angle_increment);
      }
    }
    //cout << "smallest_distance_: " << smallest_distance_ << endl;
    //通过得到的直线距离点，再通过三角函数公式，算出物体在以机器人为原点的X轴和Y轴的偏移量
    x_pillar_ = -smallest_distance_ * cos(alpha_pillar_);
    y_pillar_ = smallest_distance_ * sin(alpha_pillar_);
    std::cout << "x_pillar_ " << x_pillar_ << " y_pillar_ " << y_pillar_ << std::endl;

    if(smallest_distance_ <= 0.5)//雷达避障的距离
    {
      car_state_ = SCAN_STOP;//小车状态：雷达停车

      car_linear_velocity_ = 0.0; //小车停止
    } 
    else 
    {
      car_linear_velocity_ = 0.5; //小车前进
    }
    smallest_distance_ = 10.0;
  }

  int PurePursuit::getIndex(double pose_x, double pose_y, double pose_yaw)
  {
    // 方案:通过计算当前坐标和目标轨迹距离，找到一个最小距离的索引号
    int index;
    std::vector<float> best_points;
    for (int i = 0; i < point_num_; i++) 
    {
      float path_x = r_x_[i];
      float path_y = r_y_[i];
      // 遍历所有路径点和当前位置的距离，保存到数组中
      float lad = sqrt(pow(path_x - pose_x, 2) + pow(path_y - pose_y, 2));

      best_points.push_back(lad);
    }
    // 找到数组中最小横向距离
    //std::cout << "数组中最小横向距离 " << *min_element(best_points.begin(), best_points.end()) << std::endl;
    //std::cout << "数组中最小横向距离的索引位置 " << distance(best_points.begin(), min_element(best_points.begin(), best_points.end())) << std::endl;

    index = distance(best_points.begin(), min_element(best_points.begin(), best_points.end()));

    int temp_index;
    for (int i = index; i < point_num_; i++) 
    {
      // 遍历路径点和预瞄点的距离，从最小横向位置的索引开始
      float dis = sqrt(pow(r_y_[index] - r_y_[i], 2) + pow(r_x_[index] - r_x_[i], 2));
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
    //std::cout << "index " << index << std::endl;

    return index;
  }

  void PurePursuit::headCallback(const std_msgs::Int32ConstPtr &msg)
  {
      int isRvertHead = msg->data;
      if(isRvertHead == 1)
      {
          defaultHead = false;
      }
      else if(isRvertHead == 0)
      {
          defaultHead = true;
      }
  };


  inline double PurePursuit::thetaLimit(double yaw)//角度限制 0~180度，-180度~0
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

  double PurePursuit::Xianfu(double value, double Amplitude)//限制最大速度幅度
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
  double PurePursuit::Incremental_PI(double Velocity, double Target)
  {   
    static double Bias, Speed, Last_bias, Integral_bias;
    Bias = Target - Velocity;//计算偏差
    Integral_bias += Bias;
    Integral_bias = Xianfu(Integral_bias, 1.0);//速度最大限制在0.8
    Speed = Velocity_KP * Bias + Velocity_KI * Integral_bias + Velocity_KD *(Bias - Last_bias);//增量式PI控制器
    Last_bias = Bias;//保存上一次偏差 

    if(Speed > 1.0)
      Speed = 1.0;
    if(Speed < -1.0)
      Speed = -1.0;

    return Speed;//增量输出
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
  double PurePursuit::Position_PID(double Position, double target)
  {   
    static float Bias, Speed, Integral_bias, Last_Bias;
    Bias = target - Position;//计算偏差
    Integral_bias += Bias;//求出偏差的积分
    Integral_bias = Xianfu(Integral_bias, 1.0);
    Speed = Position_KP * Bias + Position_KI * Integral_bias + Position_KD * (Bias - Last_Bias);//位置式PID控制器
    Last_Bias = Bias;//保存上一次偏差 
    
    if(Speed > 1.0)
      Speed = 1.0;
    if(Speed < -1.0)
      Speed = -1.0;

    return Speed;//增量输出
  }


  void PurePursuit::run()
  {
    int index = getIndex(current_pose_x_, current_pose_y_, current_pose_yaw_);
    //std::cout << "index " << index << std::endl;

    double alpha = thetaLimit(atan2(r_y_[index] - current_pose_y_, r_x_[index] - current_pose_x_) - current_pose_yaw_);
    //std::cout << "alpha " << alpha << std::endl;//0~180度，-180度~0

    // 当前点和目标点的距离Id
    double dl = sqrt(pow(r_y_[index] - current_pose_y_, 2) + pow(r_x_[index] - current_pose_x_, 2));
    std::cout << "dl " << dl << std::endl;

    double curvature_k = 2 * sin(alpha) / dl;//跟踪曲率 k = 2 * sin(a) / Ld
    //std::cout << "curvature_k " << curvature_k << std::endl;

    double dis_pos = sqrt(pow(r_y_.back() - current_pose_y_, 2) + pow(r_x_.back() - current_pose_x_, 2));//距离终点的距离
    //std::cout << "dis_pos " << dis_pos << std::endl;

    double dl = sqrt(pow(r_y_[index] - current_pose_y_, 2) + pow(r_x_[index] - current_pose_x_, 2));//路径范围，给判断是否纠正线路用
    //std::cout << "path_range " << path_range << std::endl;

    if((alpha >= 10.0 * M_PI / 180.0 || alpha <= -10.0 * M_PI / 180.0) && dl <= 0.5)//纠正线路
    {
      car_state_ = CORRECT_THE_LINE;//小车状态：纠正线路

      geometry_msgs::Twist vel_msg;//发布速度
      vel_msg.linear.x = 0.0;
      if(curvature_k >= 0.0)//区分往左边还是往右边掉头
      {
        vel_msg.angular.z = current_velocity_.angular.z * 0.6 + car_angular_velocity_ * 0.4;
      }
      else
      {
        vel_msg.angular.z = current_velocity_.angular.z * 0.6 -car_angular_velocity_* 0.4;
      }
      cmd_vel_pub_.publish(vel_msg); 
    }
    else
    {
      double theta = car_linear_velocity_ * curvature_k;
      //std::cout << "theta " << theta << std::endl;

      // 发布小车运动指令
      if(dl > 0.12 || index != (point_num_-1)) 
      {
        car_state_ = PURE_PURSUIT;//小车状态：纯跟踪跟线

        geometry_msgs::Twist vel_msg;//发布速度
        if(is_use_pid_param_)//使用PID调速
        {
            //增加串级PID控制线速度
            double motor = Position_PID(-dis_pos, 0.0);//小车当前位置到目标位置的距离，距离逐渐增加
            double sign;
            if(defaultHead == true) sign = -1;
            else if(defaultHead == false) sign = 1;

            vel_msg.linear.x = sign * Incremental_PI(current_velocity_.linear.x, motor);
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
        car_state_ = CAR_STOP;//小车状态：正常停车

        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        cmd_vel_pub_.publish(vel_msg);

        is_global_path_sub_ = false;
      }
    }
  }

  void PurePursuit::MainLoop()
  { 
    ros::Rate loop_rate(20);

    while (ros::ok())
    {  
      if(is_global_path_sub_ && is_current_pose_sub_)
      {
        run();
        switch(car_state_)//主状态机
        {
          case PURE_PURSUIT:
            std::cout << "小车状态：纯跟踪跟线" << std::endl;
            break;
          case CORRECT_THE_LINE:
            std::cout << "小车状态：纠正线路" << std::endl;
            break;
          case CAR_STOP:
            std::cout << "小车状态：正常停车" << std::endl;
            break; 
          case SCAN_STOP:
            std::cout << "小车状态：雷达避障停车" << std::endl;
            break; 
          case CIRCUMVENT:
            std::cout << "小车状态：绕障" << std::endl;
            break; 
          default:
            break;
        }
        is_current_pose_sub_ = false;
      } 

      static int stop_count = 0;
      if (car_state_ == CAR_STOP)
      {
          stop_count ++;
          if(stop_count > 10) break;
      }else{
        stop_count = 0;
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
  }

}

