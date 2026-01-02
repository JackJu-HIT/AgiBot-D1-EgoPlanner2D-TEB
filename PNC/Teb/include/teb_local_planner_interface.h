/**
 * @file      teb_local_planner_interface.h
 * @brief     TEB Interface
 * @author    juchunyu@qq.com
 * @date      2025-12-27  8:00:01
 * @copyright Copyright (c) 2025, Institute of Robotics Planning and Control (irpc). All rights reserved.
 */
#pragma once

#include <iostream>
#include "teb_config.h"
#include "pose_se2.h"
#include "robot_footprint_model.h"
#include "obstacles.h"
#include "optimal_planner.h"
#include <boost/smart_ptr.hpp>

#include <vector>
#include <memory>
#include <string>
#include <Eigen/Dense>

namespace irpc {
namespace planning {


struct trajPointInfo
{
    double x;
    double y;
    double theta;  
};

struct cmdInfo
{
    float vx;
    float vy;
    float w;
};

struct vehicleStateInfo
{
   float x;
   float y;
   float theta;
   float v;
   float w;
};

struct obstacleInfo
{
    float x;
    float y;
};

class TebPlannerInterface 
{
    public:

        TebPlannerInterface();

        ~TebPlannerInterface();
    
        void initialize();

        void setObstacleInfo(std::vector<obstacleInfo>& obs);

        void setVehicleState(vehicleStateInfo& robotPose);

        void setReferencePath(std::vector<trajPointInfo>& traj);

        bool plan(cmdInfo& cmd,std::vector<Eigen::Vector3f>& traj);
    
    private:

        void limitVelocity(cmdInfo& vel, float max_v, float max_w);

    private:

        teb_local_planner::TebConfig* config_;
        teb_local_planner::RobotFootprintModelPtr robot_model_;
        teb_local_planner::ObstContainer obstacles_;
        boost::shared_ptr<teb_local_planner::TebOptimalPlanner> planner_;
        teb_local_planner::ViaPointContainer viaPoints_;

        vehicleStateInfo robotPose_;
        vehicleStateInfo goalPose_;
        int look_ahead_poses_;
       
};

} // namespace planning
} // namespace irpc
#pragma once