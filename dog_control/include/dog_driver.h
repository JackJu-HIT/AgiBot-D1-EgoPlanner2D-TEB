/**
 * @file      dog_driver.h
 * @brief     D1 SDK Driver for ROS2 
 * @author    juchunyu@qq.com
 * @date      2025-12-20  9:00:06
 * @copyright Copyright (c) 2025, Institute of Robotics Planning and Control (irpc). All rights reserved.
 */

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include "tf2_ros/transform_broadcaster.h"
#include <nav_msgs/msg/path.hpp> 
#include <std_msgs/msg/u_int32.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "zsl-1/highlevel.h"
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <chrono>


using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace mc_sdk::zsl_1;

class DogDriverNode : public rclcpp::Node
{
    struct vehicleStates
    {
        float x;
        float y;
        float theta;
        float v;
        float w;
    };

    struct cmdInfo
    {
       float vx;
       float vy;
       float w;
    };

    enum class RobotMode 
    {
        PASSIVE,    // 失能/软急停
        LIE_DOWN,   // 趴下
        STAND,      // 站立 
        JUMP,
        FRONT_JUMP,
        BACK_FLIP,
        SHAKE_HAND,
        TWO_LEG_STAND,
        CANCEL_TWO_LEG,
        ATTITUDE_DEMO
    };

    enum class SdkMode : uint32_t {
        DAMPING     = 0,  // 阻尼/失能
        STANDING    = 1,  // 站立/打招呼
        LIE_DOWN_PASSIVE = 10, // 趴下(电机自由?)
        WALKING     = 18, // 移动状态
        ACTION      = 21, // 动作状态 (姿态/跳跃等)
        LIE_DOWN_ACTIVE  = 51  // 趴下状态
    };
        
    
public:
    DogDriverNode();
    ~DogDriverNode() 
    {
        keep_running_ = false;
        if(control_thread_.joinable()) control_thread_.join();
    }

private:

    void initSDK();
    void initParam();
    void initROS2Node();
    // 处理 cmd_vel 回调
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    // 处理特殊动作回调
    void actionCallback(const std_msgs::msg::String::SharedPtr msg);

    // 周期性获取机器人状态信息
    void getRobotStateCallback();

    void udpControlLoop();

    void publish_robot_marker(const rclcpp::Time& stamp);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_action_;
    rclcpp::TimerBase::SharedPtr timer_read_robot_state_;
    rclcpp::TimerBase::SharedPtr timer_writer_robot_state_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_mode_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr car_marker_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    

    nav_msgs::msg::Path path_msg_;
    std::shared_ptr<HighLevel> highlevel_;

    vehicleStates  robot_state_;
    std::thread control_thread_;
    std::chrono::steady_clock::time_point last_cmd_time_;
    cmdInfo current_cmd_;
    std::chrono::milliseconds CMD_TIMEOUT_MS_;

    std::mutex data_mutex_;
    std::mutex data_mode_mutex_;

    RobotMode current_mode_;
    uint32_t current_robot_mode_; 
    bool keep_running_;

};
