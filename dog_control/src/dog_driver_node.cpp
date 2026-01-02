#include "dog_driver.h" 
#include <pthread.h>
#include <sched.h>

DogDriverNode::DogDriverNode() : Node("dog_driver_node") 
{
    initParam();
    initSDK();  
    initROS2Node();   
}

void DogDriverNode::initParam()
{
    keep_running_ = true;
    last_cmd_time_ = std::chrono::steady_clock::now();
    current_mode_ = RobotMode::PASSIVE; // 默认状态
    CMD_TIMEOUT_MS_ = std::chrono::milliseconds(100);
}

void DogDriverNode::initROS2Node()
{
    // 订阅速度指令 (cmd_vel)
    sub_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&DogDriverNode::cmdVelCallback, this, _1));

    // 订阅特殊动作指令 (cmd_action)
    sub_action_ = this->create_subscription<std_msgs::msg::String>(
        "cmd_action", 10, std::bind(&DogDriverNode::actionCallback, this, _1));
     
    // 发布话题 "odom"
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    pub_mode_ = this->create_publisher<std_msgs::msg::UInt32>("robot_mode", 10);
    
    //发布当前已经走轨迹
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("run_path", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    car_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("car_marker", 10);

    control_thread_ = std::thread(&DogDriverNode::udpControlLoop, this);
    // timer_writer_robot_state_ = this->create_wall_timer(
    //     std::chrono::milliseconds(20), std::bind(&DogDriverNode::getRobotStateCallback, this));
}

void DogDriverNode::initSDK()
{
    this->declare_parameter<std::string>("client_ip", "192.168.234.10");
    this->declare_parameter<int>("client_port", 43988);
    this->declare_parameter<std::string>("dog_ip", "192.168.234.1");
    std::string client_ip = this->get_parameter("client_ip").as_string();
    int client_port = this->get_parameter("client_port").as_int();
    std::string dog_ip = this->get_parameter("dog_ip").as_string();

    // 初始化SDK
    RCLCPP_INFO(this->get_logger(), "Initializing Robot Connection: Client %s:%d -> Dog %s", 
    client_ip.c_str(), client_port, dog_ip.c_str());
    highlevel_ = std::make_shared<HighLevel>();  
    try 
    {
        highlevel_->initRobot(client_ip, client_port, dog_ip);
        RCLCPP_INFO(this->get_logger(), "Robot Connected Successfully.");
    } catch (const std::exception &e) 
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot: %s", e.what());
    }
}

// 处理 cmd_vel 回调
void DogDriverNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) 
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_cmd_.vx = static_cast<float>(msg->linear.x);
    current_cmd_.vy = static_cast<float>(msg->linear.y);
    current_cmd_.w = static_cast<float>(msg->angular.z);
    last_cmd_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "cmdVelCallback: vx=%.2f, vy=%.2f, yaw=%.2f", current_cmd_.vx, current_cmd_.vy, current_cmd_.w);
}

void DogDriverNode::udpControlLoop()
{
    // // 设置当前线程为实时优先级
    // struct sched_param param;
    // param.sched_priority = 80; // 建议 80，留点余地
    // if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0)
    // {
    //     RCLCPP_WARN(this->get_logger(), "Failed to set real-time priority. Did you run with sudo?");
    // }
    
    while(keep_running_ && rclcpp::ok())
    {
        auto start_time = std::chrono::steady_clock::now();
        uint32_t raw_mode = highlevel_->getCurrentCtrlmode();
        {
            // std::lock_guard<std::mutex> lock(data_mode_mutex_);
            current_robot_mode_ = raw_mode; 
        }

        // 【新增】只有 模式下，才启用速度看门狗
        if (raw_mode == 18 || raw_mode == 1 || raw_mode == 21) 
        {
            float vx, vy, w;
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                // 看门狗检查
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_cmd_time_);
                // 检查超时
                if (elapsed > CMD_TIMEOUT_MS_) 
                {
                    vx = 0.0f;
                    vy = 0.0f; 
                    w = 0.0f;
                } 
                else 
                {
                    vx = current_cmd_.vx; vy = current_cmd_.vy; w = current_cmd_.w;
                }
            }
            RCLCPP_INFO(this->get_logger(), "udpControlLoop: vx=%.2f, vy=%.2f, yaw=%.2f", vx, vy, w);
            // 发送速度指令
            highlevel_->move(vx, vy, w);
        }

        getRobotStateCallback();
        RCLCPP_INFO(this->get_logger(), "udpControlLoop: mode=%d", raw_mode);
        std::this_thread::sleep_until(start_time + std::chrono::milliseconds(20));
    }

}

// 处理特殊动作回调
void DogDriverNode::actionCallback(const std_msgs::msg::String::SharedPtr msg) 
{
    std::string cmd = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received Action: %s", cmd.c_str());
    if (cmd == "passive") 
    {
        highlevel_->passive();
        // current_mode_ = RobotMode::PASSIVE; 
    }
    else if (cmd == "lie_down") 
    {
        highlevel_->lieDown();
        // current_mode_ = RobotMode::LIE_DOWN;; 
    }
    else if (cmd == "stand_up")
    {
        highlevel_->standUp();
        // current_mode_ = RobotMode::STAND;
    }
    else if (cmd == "jump") 
    {
        highlevel_->jump();
        // current_mode_ = RobotMode::JUMP;
    }
    else if (cmd == "front_jump") 
    {
        highlevel_->frontJump();
        // current_mode_ = RobotMode::FRONT_JUMP;
    }
    else if (cmd == "backflip") 
    {
        highlevel_->backflip();
        // current_mode_ = RobotMode::BACK_FLIP;
    }
    else if (cmd == "shake_hand") 
    {
        highlevel_->shakeHand();
        // current_mode_ = RobotMode::SHAKE_HAND;
    }
    else if (cmd == "two_leg_stand") 
    {
        highlevel_->twoLegStand(0.0, 0.0); // 默认参数
        // current_mode_ = RobotMode::TWO_LEG_STAND;
    }
    else if (cmd == "cancel_two_leg") 
    {
        highlevel_->cancelTwoLegStand();
        // current_mode_ = RobotMode::CANCEL_TWO_LEG;
    }
    else if (cmd == "attitude_demo") 
    {
        // 演示一下姿态控制
        highlevel_->attitudeControl(0.1, 0.0, 0.0, 0.0); 
        // current_mode_ = RobotMode::ATTITUDE_DEMO;
    }
    else 
    {
        // current_mode_ = RobotMode::PASSIVE; 
        RCLCPP_WARN(this->get_logger(), "Unknown action command: %s", cmd.c_str());
    }
}

// 周期性获取机器人信息
void DogDriverNode::getRobotStateCallback()
{
    // while(true)
    {
        uint32_t ret = highlevel_->getBatteryPower();
        static float value = 0.1;
        value += 0.1;
        // 2. 从 SDK 获取数据
        // 假设 getPosition 返回 {x, y, z}
        auto pos = highlevel_->getPosition(); 
        // 假设 getRPY 返回 {roll, pitch, yaw}
        auto rpy = highlevel_->getRPY();
        // 假设 getBodyVelocity 返回 {vx, vy, vz}
        auto vel = highlevel_->getBodyVelocity();
        // 假设 getBodyGyro 返回 {wx, wy, wz}
        auto gyro = highlevel_->getBodyGyro();

        // --- 更新本地调试用的结构体 ---
        robot_state_.x = pos[0];// + value;
        // std::cout << "模拟" << std::endl;
        robot_state_.y = pos[1];// + 0.05 * value;
        robot_state_.theta = rpy[2];// + 0.01 * value; // Yaw
        robot_state_.v = vel[0];// + 0.1 * value;     // Linear X
        robot_state_.w = gyro[2];// + 0.05 * value;    // Angular Z

        // 打印调试信息 (如果你不想刷屏太快，可以加个计数器限制打印频率)
        std::cout << "State: x=" << robot_state_.x << ", y=" << robot_state_.y 
                << ", th=" << robot_state_.theta << std::endl;
        std::cout << "v =" << robot_state_.v << " robot_state_.w =" << robot_state_.w << std::endl;
        rclcpp::Time stamp_now = this->now(); // 获取统一的时间戳
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = stamp_now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // 1. 填充位置 (XYZ)
        odom_msg.pose.pose.position.x = robot_state_.x;
        odom_msg.pose.pose.position.y = robot_state_.y;
        odom_msg.pose.pose.position.z = 0.0; // 如果是平面移动，Z设为0

        // 2. 【核心】将 Theta (Yaw) 转换为四元数
        tf2::Quaternion q;
        // setRPY 参数顺序: (Roll, Pitch, Yaw)
        // 这里的 theta 就是 Yaw。因为是平面导航，Roll 和 Pitch 通常设为 0
        q.setRPY(0.0, 0.0, robot_state_.theta); 

        // 3. 将 tf2::Quaternion 转换为 msg 格式并赋值
        odom_msg.pose.pose.orientation = tf2::toMsg(q);

        // 4. 填充速度
        odom_msg.twist.twist.linear.x = robot_state_.v;
        odom_msg.twist.twist.angular.z = robot_state_.w;

        // 发布
        pub_odom_->publish(odom_msg);

        // std::cout << " pub_odom_->publish(odom_msg) OK" << std::endl;


        // ===========================
        // === 处理轨迹 Path ===
        // ===========================
        #if 1
        // 1. 获取当前位置
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = odom_msg.header; // 使用相同的时间戳和坐标系
        pose_stamped.pose = odom_msg.pose.pose; // 直接拷贝里程计的位姿

        // 2. (可选) 简单的过滤器：只有当移动距离超过一定阈值才记录
        // 防止原地不动时数组无限膨胀
        bool should_record = true;
        if (!path_msg_.poses.empty()) 
        {
            auto last_pose = path_msg_.poses.back().pose.position;
            double dist = std::sqrt(std::pow(last_pose.x - robot_state_.x, 2) + 
                                        std::pow(last_pose.y - robot_state_.y, 2));
            if (dist < 0.05) { // 如果移动小于 5cm，就不记录
                    should_record = false;
            }
            std::cout << "path_msg_ " << "should_record =" << should_record  << std::endl;
        }

        std::cout <<"should_record =" << should_record  << std::endl;

        // 3. 添加进路径并发布
        if (should_record) {
            path_msg_.header.stamp = stamp_now; // 更新路径的时间戳
            path_msg_.header.frame_id = "odom"; //必须显式设置容器的坐标系
            path_msg_.poses.push_back(pose_stamped);
            pub_path_->publish(path_msg_);
        }
        #endif
        auto mode_msg = std_msgs::msg::UInt32();
        {
            // std::lock_guard<std::mutex> lock(data_mode_mutex_);
            mode_msg.data = current_robot_mode_;
        }
        pub_mode_->publish(mode_msg);


        publish_robot_marker(stamp_now);
    }
}

void DogDriverNode::publish_robot_marker(const rclcpp::Time& stamp)
{

    // 1. TF: map -> base_link
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "dog_base_link";
    t.transform.translation.x = robot_state_.x;
    t.transform.translation.y = robot_state_.y;
    t.transform.translation.z = 0.0;
        
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_state_.theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);

    // 2. Car Marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "dog_base_link"; // 相对于车身
    marker.header.stamp = stamp;
    marker.ns = "sim_car";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.4; marker.scale.y = 0.2; marker.scale.z = 0.1;
    marker.color.a = 0.8; marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
        
    // Marker在 base_link 下是原点
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1.0;

    car_marker_pub_->publish(marker);
}

int main(int argc, char **argv) {
    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<DogDriverNode>());
    // rclcpp::shutdown();
    // return 0;
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 4);
    auto node = std::make_shared<DogDriverNode>();
    exec.add_node(node);
    exec.spin();
}