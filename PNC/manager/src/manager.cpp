
#include "manager.h"
#include <tf2/utils.h>
#include "geometry_msgs/msg/quaternion.hpp"  


manager_Interface::manager_Interface() 
    : Node("ego_planner_interactive_node"),
      has_valid_global_path_(false),
      has_obstacles_(false),
      should_plan_(false),
      needs_replan_(false)
{
    // 1. 创建发布者（可视化用）
    global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("visual_global_path", 10);
    ego_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("visual_ego__trajectory", 10);
    teb_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("visual_teb__trajectory", 10);
    obs_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("visual_obstacles", 10);
    obs_local_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("visual_local_obstacles", 10);
    a_star_path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectories", 10);
    pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose",
        10,
        std::bind(&manager_Interface::goal_pose_callback, this, std::placeholders::_1)
    );

    // 路径添加：使用Publish Point工具逐个添加
    rviz_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point",
        10,
        std::bind(&manager_Interface::rviz_point_callback, this, std::placeholders::_1)
    );

    // 障碍物输入方式3：使用2D Pose Estimate工具添加
    pose_estimate_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose",
        10,
        std::bind(&manager_Interface::pose_estimate_callback, this, std::placeholders::_1)
    );


    currPose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom",
            10,
            std::bind(&manager_Interface::pose_callback, this, std::placeholders::_1)
    );


    // 4. 初始化 Planner基础配置
    init_ego_planner_base();

    // 5. 定时器：20Hz触发规划与发布
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&manager_Interface::make_plan_and_control, this)
    );

    RCLCPP_INFO(this->get_logger(), "Interactive Ego Planner Node Ready!");
    RCLCPP_INFO(this->get_logger(), "=== 使用方法 ===");
    RCLCPP_INFO(this->get_logger(), "1. 添加全局路径：使用2D Nav Goal工具设置终点");
    RCLCPP_INFO(this->get_logger(), "2. 添加障碍物（三种方法任选）：");
    RCLCPP_INFO(this->get_logger(), "   - 方法A：使用Publish Point工具点击地图");
    RCLCPP_INFO(this->get_logger(), "   - 方法B：使用2D Pose Estimate工具点击Grid任意位置（推荐）");
    RCLCPP_INFO(this->get_logger(), "   - 方法C：发布PointCloud2到 /rviz_input_obstacles");
    RCLCPP_INFO(this->get_logger(), "3. 触发规划：ros2 topic pub /trigger_plan std_msgs/Bool \"{data: true}\"");
    RCLCPP_INFO(this->get_logger(), "4. 停止规划：ros2 topic pub /trigger_plan std_msgs/Bool \"{data: false}\"");
    RCLCPP_INFO(this->get_logger(), "5. 支持多次规划：可以反复发送true/false控制规划");
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "=== RViz2设置步骤 ===");
    RCLCPP_INFO(this->get_logger(), "1. 添加Grid显示");
    RCLCPP_INFO(this->get_logger(), "2. 添加2D Nav Goal工具（话题：/goal_pose）");
    RCLCPP_INFO(this->get_logger(), "3. 添加2D Pose Estimate工具（使用默认话题：/initialpose）");
    RCLCPP_INFO(this->get_logger(), "4. 添加Publish Point工具（使用默认话题：/clicked_point）");
    RCLCPP_INFO(this->get_logger(), "5. 添加PointCloud2显示（话题：/visual_obstacles）");
    RCLCPP_INFO(this->get_logger(), "6. 添加Path显示（话题：/visual_global_path和/visual_local_trajectory）");
}

void manager_Interface::pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{

    std::lock_guard<std::mutex> lock(data_mutex_);
    // 1. 更新机器人初始位姿（原有逻辑）
    cur_pose_.x = msg->pose.pose.position.x;         

    cur_pose_.y = msg->pose.pose.position.y;
    cur_pose_.z = 0;
    // 计算偏航角（使用tf2或手动计算，确保正确）
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    cur_pose_.theta = tf2::getYaw(q);
    cur_pose_.v     = msg->twist.twist.linear.x;
    cur_pose_.w     = msg->twist.twist.angular.z;
    std::cout << "cur_pose_.x = " << cur_pose_.x << " cur_pose_.y =" << cur_pose_.y << " cur_pose_.theta =" << cur_pose_.theta
    << " v =" << cur_pose_.v  << " cur_pose_.w =" << cur_pose_.w << std::endl;
}

// 初始化Ego Planner基础参数
void manager_Interface::init_ego_planner_base()
{
    teb_planner_ = std::make_shared<irpc::planning::TebPlannerInterface>();
    teb_planner_->initialize();

    ego_planner_ = std::make_shared<ego_planner::PlannerInterface>();
    ego_planner_->initParam(max_vel_, max_acc_, max_jerk_);
    ego_planner_->initEsdfMap(
        map_x_size_, map_y_size_, map_z_size_,
        map_resolution_, map_origin_, map_inflate_value_
    );
}

// 统一添加障碍物函数
void manager_Interface::add_obstacle_at_position(double x, double y)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    ObstacleInfo obs;
    obs.x = x;
    obs.y = y;
    obs.z = 0.0;
    
    obstacles_.push_back(obs);
    has_obstacles_ = true;
    
    // 如果有障碍物更新且正在规划中，则标记需要重新规划
    if (should_plan_) {
        needs_replan_ = true;
        RCLCPP_INFO(this->get_logger(), "障碍物更新，已标记需要重新规划");
    }
    
    RCLCPP_INFO(this->get_logger(), "添加障碍物: (%.2f, %.2f), 总障碍物数量: %zu", 
                x, y, obstacles_.size());
}

// 2D Pose Estimate回调函数
void manager_Interface::pose_estimate_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    #if 0
    std::lock_guard<std::mutex> lock(data_mutex_);
   // 1. 更新机器人初始位姿（原有逻辑）
    cur_pose_.x = msg->pose.pose.position.x;
    cur_pose_.y = msg->pose.pose.position.y;
    cur_pose_.z = 0;
    // 计算偏航角（使用tf2或手动计算，确保正确）
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    cur_pose_.z = tf2::getYaw(q);
    std::cout << "pose_estimate_callback " << std::endl;
    // 2. 触发规划逻辑（新增）
    if (has_valid_global_path_) {  // 确保已有全局路径
        needs_replan_ = true;  // 标记需要重新规划
        if (should_plan_) {
            RCLCPP_INFO(this->get_logger(), "初始位置更新，触发重新规划！");
        } else {
            RCLCPP_WARN(this->get_logger(), "初始位置已更新，但规划未启动！请先发送 /trigger_plan true 启动规划");
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "初始位置已更新，但无有效全局路径，无法规划！");
    }
    #endif
}

// 处理2D Nav Goal - 生成从起点到目标的直线路
void manager_Interface::goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    add_obstacle_at_position(msg->pose.position.x, msg->pose.position.y);
}

// 生成直线路径
void manager_Interface::generate_straight_path(const geometry_msgs::msg::PoseStamped& start, 
                                                           const geometry_msgs::msg::PoseStamped& goal)
{
    global_plan_traj_.clear();

    // 计算路径点数量（每0.1米一个点）
    double dx = goal.pose.position.x - start.pose.position.x;
    double dy = goal.pose.position.y - start.pose.position.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    int num_points = std::max(2, static_cast<int>(distance / 0.1));

    // 生成直线路径点
    for (int i = 0; i < num_points; ++i) {
        double ratio = static_cast<double>(i) / (num_points - 1);
        PathPoint point;
        point.x = start.pose.position.x + ratio * dx;
        point.y = start.pose.position.y + ratio * dy;
        point.z = 0.0;
        global_plan_traj_.push_back(point);
    }
}

// // 处理Publish Point点击 - 添加障碍物
void manager_Interface::rviz_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    //std::lock_guard<std::mutex> lock(data_mutex_);
    // global_plan_traj_.clear();
    PathPoint point;
    point.x = msg->point.x;
    point.y = msg->point.y;
    point.z = 0; // 使用z作为theta角度
    global_plan_traj_.push_back(point);
    std::cout << "从RViz接收到全局路径点: (" << point.x << ", " << point.y << ")" << std::endl;
    has_valid_global_path_ = true;
    
    // 如果有路径更新且正在规划中，则标记需要重新规划
    // if (should_plan_) 
    {
        needs_replan_ = true;
        RCLCPP_INFO(this->get_logger(), "路径更新，已标记需要重新规划");
    }
}

// 原有的全局路径回调
void manager_Interface::rviz_global_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (msg->poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "从RViz接收到空的全局路径!");
        has_valid_global_path_ = false;
        return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    global_plan_traj_.clear();
    for (const auto& pose_stamped : msg->poses)
    {
        PathPoint path_point;
        path_point.x = pose_stamped.pose.position.x;
        path_point.y = pose_stamped.pose.position.y;
        path_point.z = pose_stamped.pose.position.z;
        global_plan_traj_.push_back(path_point);
    }

    has_valid_global_path_ = true;
    
    // 如果有路径更新且正在规划中，则标记需要重新规划
    if (should_plan_) {
        needs_replan_ = true;
        RCLCPP_INFO(this->get_logger(), "路径更新，已标记需要重新规划");
    }
    
    RCLCPP_INFO(this->get_logger(), "从RViz接收到全局路径 (大小: %zu)", global_plan_traj_.size());
}

void manager_Interface::updateRobotState()
{
    std::lock_guard<std::mutex> lock(data_mutex_); 
    local_pose_.x = cur_pose_.x;
    local_pose_.y = cur_pose_.y;
    local_pose_.z = 0;
    local_pose_.theta = cur_pose_.theta;
    ego_planner_->resetMap();
    ego_planner_->setGridMap(local_pose_);
    ego_planner_->setObstacles(obstacles_); 
    ego_planner_->setCurrentVehiclePos(local_pose_);

    irpc::planning::vehicleStateInfo curPose;
    curPose.x = cur_pose_.x;
    curPose.y = cur_pose_.y;
    curPose.theta =  cur_pose_.theta;
    curPose.v = cur_pose_.v;
    curPose.w = cur_pose_.w;
    teb_planner_->setVehicleState(curPose);

}

bool manager_Interface::collisionDetection(std::vector<PathPoint>& planned_traj)
{
    for(int i = 0; i < planned_traj.size();i++)
    {
        bool status = ego_planner_->getInflateOccupancy(planned_traj[i]);
        if(status)
        {
            std::cout << "[collisionDetection] ok=gtrue" << std::endl; 
            return true;
        }
    }

    return false; 

}

void manager_Interface::useGlobalPath()
{
    std::vector<PathPoint> global_plan_traj_temp;

    discretize_trajectory(global_plan_traj_, global_plan_traj_temp, 0.1);
             
    float mindist = 100000000;
    int minddex = 0;
    for(int i = 0; i < global_plan_traj_temp.size();i++)
    {
        double dist =  distance(global_plan_traj_temp[i], local_pose_); 
        if(dist < mindist)
        {
            mindist = dist;
            minddex = i;
        } 
    }
    std::vector<PathPoint> global_plan_traj_after;
    global_plan_traj_after.push_back(local_pose_);
    for(int i = minddex; i < global_plan_traj_temp.size();i++)
    {
        global_plan_traj_after.push_back(global_plan_traj_temp[i]);
    }
    discretize_trajectory(global_plan_traj_after, global_plan_traj_temp, 0.1);

    
    globalPlan_.clear();
    for(int i = 0 ; i < global_plan_traj_temp.size() - 1;i++)
    {
        irpc::planning::trajPointInfo pt;
        pt.x = global_plan_traj_temp[i].x;
        pt.y = global_plan_traj_temp[i].y;
          
        pt.theta = atan2(global_plan_traj_temp[i+1].y - global_plan_traj_temp[i].y ,  global_plan_traj_temp[i+1].x -  global_plan_traj_temp[i].x);
        globalPlan_.push_back(pt);
    }
    
    if(globalPlan_.size() < 2)
    {
        std::cout << "[useGlobalPath()]global plan size less 2，maybe reached" << std::endl;
        return;
    } 
    teb_planner_->setReferencePath(globalPlan_);
}

void manager_Interface::requestEgoPlanner()
{
    if (!global_plan_traj_.empty())
    {
        std::vector<PathPoint> global_plan_traj_temp;

        discretize_trajectory(global_plan_traj_, global_plan_traj_temp, 0.4);
             
        float mindist = 100000000;
        int minddex = 0;
        for(int i = 0; i < global_plan_traj_temp.size();i++)
        {
            double dist =  distance(global_plan_traj_temp[i], local_pose_); 
            if(dist < mindist)
            {
                mindist = dist;
                 minddex = i;
            } 
        }

        std::vector<PathPoint> global_plan_traj_after;
        global_plan_traj_after.push_back(local_pose_);
        for(int i = minddex; i < global_plan_traj_temp.size();i++)
        {
            global_plan_traj_after.push_back(global_plan_traj_temp[i]);
        }
        discretize_trajectory(global_plan_traj_after, global_plan_traj_temp, 0.4);
        
        if(global_plan_traj_temp.size() < 4)
        {
            std::cout  << "global_plan_traj_temp less 3 point  return false;" << std::endl;
            return;
        }
        int length = 50;
        // if(global_plan_traj_temp.size() < length) 
        length = global_plan_traj_temp.size();
        global_plan_traj_res_.clear();
        std::cout << "publish point start:" << std::endl;
        for(int i = 0; i < length;i++)
        {
            global_plan_traj_res_.push_back(global_plan_traj_temp[i]);
            std::cout << " x =" << global_plan_traj_temp[i].x << " y=" << global_plan_traj_temp[i].y << " theta =" 
            << global_plan_traj_temp[i].theta << std::endl;
        }
        
        std::cout << "publish point end:" << std::endl;
        ego_planner_->setPathPoint(global_plan_traj_res_);
        ego_planner_->makePlan();
        ego_planner_->getAStarPath(a_star_pathes_);
        ego_planned_traj_.clear();
        std::cout << "global_plan_traj_res_.size()  =" << global_plan_traj_res_.size()  << std::endl;
        ego_planner_->getLocalPlanTrajResults(ego_planned_traj_);
    }
    else 
    {
        std::cout << "global_plan_traj_.empty!!!!" << std::endl;
    }

}

void manager_Interface::pruneGlobalPath()
{

    if(ego_planned_traj_.empty())
    {
        std::cout << "ego_planned_traj_.empty !!! " << std::endl;
        return;
    }
    std::cout << " ego_planned_traj_.size =" << ego_planned_traj_.size() << std::endl;
    std::vector<PathPoint> global_plan_traj_temp;
    for(int i =0 ; i < ego_planned_traj_.size();i++)
    {
        std::cout << " ego_planned_traj_ = (x , y)" << ego_planned_traj_[i].x << "," << ego_planned_traj_[i].y << std::endl;
    }

    discretize_trajectory(ego_planned_traj_, global_plan_traj_temp, 0.1);

    std::cout << " global_plan_traj_temp.size()  " << global_plan_traj_temp.size() << std::endl;
             
    float mindist = 100000000;
    int minddex = 0;
    for(int i = 0; i < global_plan_traj_temp.size();i++)
    {
        std::cout << " global_plan_traj_temp[i] x=" << global_plan_traj_temp[i].x << " , y" << 
        global_plan_traj_temp[i].y << std::endl; 
        double dist =  distance(global_plan_traj_temp[i], cur_pose_); 
        if(dist < mindist)
        {
            mindist = dist;
            minddex = i;
        } 
    }
    std::cout << "[pruneGlobalPath] cur_pose_ x =" << cur_pose_.x << " cur_pose_.y =" << cur_pose_.y << std::endl;
    globalPlan_.clear();
    std::cout << "pt start" << std::endl;
    for(int i = minddex ; i < global_plan_traj_temp.size() - 1;i++)
    {
        irpc::planning::trajPointInfo pt;
        pt.x = global_plan_traj_temp[i].x;
        pt.y = global_plan_traj_temp[i].y;
        std::cout << " pt.x =" << pt.x << " ,pt.y =" << pt.y << std::endl;
          
        pt.theta = atan2(global_plan_traj_temp[i+1].y - global_plan_traj_temp[i].y ,  global_plan_traj_temp[i+1].x -  global_plan_traj_temp[i].x);
        globalPlan_.push_back(pt);
    }

    if(globalPlan_.size() < 2)
    {
        std::cout << "global plan size less 2，maybe reached" << std::endl;
        return;
    } 
    teb_planner_->setReferencePath(globalPlan_);
}



// 核心逻辑：检查数据更新→触发规划→发布结果
void manager_Interface::make_plan_and_control()
{
    if (!has_valid_global_path_ || global_plan_traj_.empty() || global_plan_traj_.size() < 2) 
    {
        printf("请在rviz2中使用publish point 发布路径！\n");
        publish_global_path(); // 即使空的也发布，保持RViz状态
        return;
    }

    updateRobotState();


    // 如果有全局路径且应该规划，并且需要重新规划，则进行规划
    if (collisionDetection(ego_planned_traj_) || needs_replan_) 
    {
        needs_replan_ = false;
        std::cout << "requestEgoPlanner" << std::endl;
        requestEgoPlanner();
    }

    std::cout << "global_plan_traj_res_.size()  =" << global_plan_traj_res_.size()  << std::endl;

    pruneGlobalPath();

    if(globalPlan_.size() < 2) 
    {
        std::cout<< "use glaobla path !!!" << std::endl;
        return;
        useGlobalPath();
    }

    teb_planned_traj_.clear();
        
    std::vector<Eigen::Vector3f> traj;
    teb_planner_->plan(cmd_,traj);
        
    for(int i = 0; i < traj.size();i++)
    {
        Eigen::Vector3f pathP = traj[i];
        PathPoint point;
        point.x = pathP[0];
        point.y = pathP[1];
        point.z = 0;
        teb_planned_traj_.push_back(point);
    }

    publishCmd();
        
    if (has_obstacles_) 
    {
        RCLCPP_INFO(this->get_logger(), "规划完成! 包含障碍物避让. 路径点: %zu, 障碍物: %zu", 
                       global_plan_traj_.size(), obstacles_.size());
    } 
    else 
    {
        RCLCPP_INFO(this->get_logger(), "规划完成! 无障碍物. 路径点: %zu", 
                       global_plan_traj_.size());
    }
        
    // 发布所有可视化数据（无论是否更新，保持实时显示）
    publish_global_path();
    publish_ego_planned_trajectory();
    publish_teb_planned_trajectory();
    publish_obstacles();
    publish_local_obstacles();
    publish_a_star_path();
}

void manager_Interface::publishCmd()
{
    // 1. 应用斜率限制
    float smoothed_v = limit_slope(cmd_.vx, last_v_, max_v_step_);
    float smoothed_w = limit_slope(cmd_.w, last_w_, max_w_step_);
    // 2. 更新状态，供下一帧参考
    last_v_ = smoothed_v;
    last_w_ = smoothed_w;

    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = smoothed_v;
    msg.linear.y = 0;
    msg.angular.z = smoothed_w;
    pub_cmd_->publish(msg);
    std::cout << "publishCmd : v =" << smoothed_v << " w =" << smoothed_w << std::endl;
}

float manager_Interface::limit_slope(float target, float current, float step) 
{
    float diff = target - current;

    if (diff > step) {
        // 目标值比当前值大太多 (加速/正向增大)，向上限制
        return current + step;
    } else if (diff < -step) {
        // 目标值比当前值小太多 (减速/反向增大)，向下限制
        return current - step;
    }

    // 变化量在 [-0.02, 0.02] 之间，直接取目标值
    return target;
}

void manager_Interface::publish_a_star_path()
{
    // 如果没有路径数据，直接返回
    if (a_star_pathes_.empty())
    {
    //   RCLCPP_WARN(this->get_logger(), "a_star_pathes_ is empty, no trajectories to publish.");
      return;
    }

    // 创建一个MarkerArray消息
    visualization_msgs::msg::MarkerArray marker_array_msg;

    // 遍历 a_star_pathes_ 中的每一条路径
    for (size_t i = 0; i < a_star_pathes_.size(); ++i)
    {
      const std::vector<Eigen::Vector2d>& current_path = a_star_pathes_[i];

      // 1. 创建一个Marker对象
      visualization_msgs::msg::Marker marker;

      // 2. 设置Marker的基本信息
      marker.header.frame_id = "odom"; // 非常重要！指定轨迹所在的坐标系
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "a_star_paths"; // 命名空间，用于分组管理Marker
      marker.id = i;              // 每条路径必须有唯一的ID
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP; // 类型为线串
      marker.action = visualization_msgs::msg::Marker::ADD;      // 动作是添加

      // 3. 设置线的视觉属性
      marker.scale.x = 0.1; // 线宽为0.1米

      // 设置颜色（RGBA格式），每条路径可以设置不同的颜色
      marker.color.r = 1.0 - (i * 0.3); // 红色分量
      marker.color.g = 0.2;             // 绿色分量
      marker.color.b = 0.2 + (i * 0.3); // 蓝色分量
      marker.color.a = 1.0;             // 透明度（1.0为完全不透明）

      // 4. 填充路径点
      for (const auto& eigen_point : current_path)
      {
        geometry_msgs::msg::Point ros_point;
        ros_point.x = eigen_point.x();
        ros_point.y = eigen_point.y();
        ros_point.z = 0.0; // 2D轨迹，z轴设为0

        marker.points.push_back(ros_point);
      }

      // 5. 将当前路径的Marker添加到MarkerArray中
      marker_array_msg.markers.push_back(marker);
    }

    // 6. 发布MarkerArray消息
    a_star_path_pub_->publish(marker_array_msg);
    // RCLCPP_INFO(this->get_logger(), "a_star_path_pub_ Published %zu trajectories.", a_star_pathes_.size());

}

// 发布可视化全局路径
void manager_Interface::publish_global_path()
{
    // if (global_plan_traj_.empty()) return;

    nav_msgs::msg::Path visual_path;
    visual_path.header.stamp = this->now();
    visual_path.header.frame_id = "odom";

    for (const auto& path_point : global_plan_traj_)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = visual_path.header;
        pose.pose.position.x = path_point.x;
        pose.pose.position.y = path_point.y;
        pose.pose.position.z = path_point.z;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        visual_path.poses.push_back(pose);
    }

    global_path_pub_->publish(visual_path);
}

// 发布Ego Planner规划后的局部轨迹
void manager_Interface::publish_ego_planned_trajectory()
{
    nav_msgs::msg::Path visual_traj;
    visual_traj.header.stamp = this->now();
    visual_traj.header.frame_id = "odom";

    for (size_t i = 0; i < ego_planned_traj_.size(); ++i)
    {
        // std::cout << "[publish_planned_trajectory] x = " << planned_traj[i].x << " y =" << planned_traj[i].y << std::endl;
        geometry_msgs::msg::PoseStamped pose;
        pose.header = visual_traj.header;
        pose.pose.position.x = ego_planned_traj_[i].x;
        pose.pose.position.y = ego_planned_traj_[i].y;
        pose.pose.position.z = ego_planned_traj_[i].z;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        visual_traj.poses.push_back(pose);
    }

    ego_traj_pub_->publish(visual_traj);
}

void manager_Interface::publish_teb_planned_trajectory()
{
    nav_msgs::msg::Path visual_traj;
    visual_traj.header.stamp = this->now();
    visual_traj.header.frame_id = "odom";

    for (size_t i = 0; i < teb_planned_traj_.size(); ++i)
    {
        // std::cout << "[publish_planned_trajectory] x = " << planned_traj[i].x << " y =" << planned_traj[i].y << std::endl;
        geometry_msgs::msg::PoseStamped pose;
        pose.header = visual_traj.header;
        pose.pose.position.x = teb_planned_traj_[i].x;
        pose.pose.position.y = teb_planned_traj_[i].y;
        pose.pose.position.z = teb_planned_traj_[i].z;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        visual_traj.poses.push_back(pose);
    }

    teb_traj_pub_->publish(visual_traj);
}

// 发布可视化障碍物
void manager_Interface::publish_obstacles()
{
    // if (obstacles_.empty()) return;

    sensor_msgs::msg::PointCloud2 visual_obs;
    visual_obs.header.stamp = this->now();
    visual_obs.header.frame_id = "odom";
    visual_obs.width = obstacles_.size();
    visual_obs.height = 1;
    visual_obs.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(visual_obs);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    sensor_msgs::PointCloud2Iterator<float> iter_x(visual_obs, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(visual_obs, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(visual_obs, "z");

    for (const auto& obs : obstacles_)
    {
        *iter_x = static_cast<float>(obs.x);
        *iter_y = static_cast<float>(obs.y);
        *iter_z = 0.0f;
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    obs_pub_->publish(visual_obs);
}

void manager_Interface::publish_local_obstacles()
{
    std::vector<ObstacleInfo> obstacles;
    ego_planner_->getObstacles(obstacles);

    if (obstacles.empty()) return;

    sensor_msgs::msg::PointCloud2 visual_obs;
    visual_obs.header.stamp = this->now();
    visual_obs.header.frame_id = "odom";

    // 正确设置点云大小
    visual_obs.height = 1;
    visual_obs.width = obstacles.size();
    visual_obs.is_dense = true;

    // 为点云分配字段
    sensor_msgs::PointCloud2Modifier modifier(visual_obs);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(obstacles.size());   // 关键！分配空间

    // 创建迭代器
    sensor_msgs::PointCloud2Iterator<float> iter_x(visual_obs, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(visual_obs, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(visual_obs, "z");

    for (const auto& obs : obstacles)
    {
        *iter_x = static_cast<float>(obs.x);
        *iter_y = static_cast<float>(obs.y);
        *iter_z = 0.0f;

        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    obs_local_pub_->publish(visual_obs);
}

// 计算两点之间的欧氏距离（单位：米）
double manager_Interface::distance(const PathPoint& p1, const PathPoint& p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx*dx + dy*dy);
}

/**
 * 将轨迹离散为均匀间隔的点（间隔10cm）
 * @param original_trajectory 原始轨迹（由多个顶点组成的折线）
 * @param discrete_trajectory 输出的离散轨迹
 * @param interval 间隔距离（单位：米，默认0.1米即10cm）
 */
void manager_Interface::discretize_trajectory(const std::vector<PathPoint>& original_trajectory,
                                              std::vector<PathPoint>& discrete_trajectory,
                                              double interval) {
    if (original_trajectory.size() < 2) {
        if(!original_trajectory.empty()) {
             discrete_trajectory.push_back(original_trajectory[0]);
        }
        return;
    }

    discrete_trajectory.clear();
    // 1. 加入起点
    discrete_trajectory.push_back(original_trajectory[0]);

    double accumulated_dist = 0.0;
    double target_dist = interval; // 下一个目标采样点的距离

    // 2. 遍历原始轨迹
    for (size_t i = 0; i < original_trajectory.size() - 1; ++i) {
        const PathPoint& start = original_trajectory[i];
        const PathPoint& end = original_trajectory[i+1];
        double seg_length = distance(start, end);

        // 如果线段太短，直接跳过，累积误差极小
        if (seg_length < 1e-6) continue;

        // 当前线段覆盖了多少距离
        double current_seg_end_dist = accumulated_dist + seg_length;

        // 在当前线段上寻找所有符合步长的点
        while (target_dist <= current_seg_end_dist) {
            // 计算插值比例
            // distance_into_segment 是目标点在当前线段上的局部距离
            double distance_into_segment = target_dist - accumulated_dist;
            double ratio = distance_into_segment / seg_length;

            PathPoint p;
            p.x = start.x + ratio * (end.x - start.x);
            p.y = start.y + ratio * (end.y - start.y);
            // Z轴或其他属性也可以插值
            p.z = 0; 

            discrete_trajectory.push_back(p);

            // 准备找下一个点
            target_dist += interval;
        }

        // 更新累积距离
        accumulated_dist += seg_length;
    }

    // 3. 强制加入终点（防止因最后一段不足 interval 而丢失终点）
    // 只有当最后一个采样点距离终点较远时才加，避免重复
    if (distance(discrete_trajectory.back(), original_trajectory.back()) > 1e-3) {
        discrete_trajectory.push_back(original_trajectory.back());
    }
}

// 主函数：启动节点
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<manager_Interface>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}