#include "teb_local_planner_interface.h"
#include "teb_config.h"
#include <algorithm> 


namespace irpc
{

    namespace planning 
    {

        TebPlannerInterface::TebPlannerInterface()
        {

        }

        TebPlannerInterface::~TebPlannerInterface()
        {
            delete config_;
        }

        void TebPlannerInterface::initialize()
        {
            look_ahead_poses_ = 3;
            robot_model_ = boost::make_shared<teb_local_planner::CircularRobotFootprint>(0.6);
            config_ =  new teb_local_planner::TebConfig();

            obstacles_.reserve(500);
            viaPoints_.reserve(100);
            planner_ = boost::make_shared<teb_local_planner::TebOptimalPlanner>(config_, &obstacles_, robot_model_,&viaPoints_);
        }

        void TebPlannerInterface::setVehicleState(vehicleStateInfo& robotPose)
        {
            robotPose_ = robotPose;
        }

        void TebPlannerInterface::setObstacleInfo(std::vector<obstacleInfo>& obs)
        {
            obstacles_.clear();
            for(int i = 0;i < obs.size();i++)
            {
                Eigen::Vector2d obsTemp;
                obsTemp.coeffRef(0) = obs[i].x;
                obsTemp.coeffRef(1) = obs[i].y;
                double dist = pow((obsTemp.coeffRef(0) - robotPose_.x),2) + pow((obsTemp.coeffRef(1) - robotPose_.y),2);
                dist = sqrt(dist);
                // if(dist < 3.0)
                {
                    obstacles_.push_back(teb_local_planner::ObstaclePtr(new teb_local_planner::PointObstacle(obsTemp)));
                }
            }
            printf("TebPlannerInterface::setObstacleInfo:%d\n",obstacles_.size());
        }

        void TebPlannerInterface::setReferencePath(std::vector<trajPointInfo>& traj)
        {
            viaPoints_.clear();
            double dist = 0;
            int length = 15;
            if(length > traj.size())
            {
                length = traj.size();
            }
            for(int i = 0;i < length;i++)
            {
                viaPoints_.emplace_back(traj[i].x,traj[i].y);
                printf("%f  %f\n",traj[i].x,traj[i].y);
            }
            goalPose_.x     = traj[length - 1].x;
            goalPose_.y     = traj[length - 1].y;
            goalPose_.theta = traj[length - 1].theta;
        }

        void TebPlannerInterface::limitVelocity(cmdInfo& vel, float max_v, float max_w) 
        {
            // std::clamp(值, 下限, 上限)
            vel.vx = std::clamp(vel.vx, -max_v, max_v);
            vel.w = std::clamp(vel.w, -max_w, max_w);
        }

        bool TebPlannerInterface::plan(cmdInfo& cmd,std::vector<Eigen::Vector3f>& traj)
        {            
            teb_local_planner::PoseSE2 start(robotPose_.x,robotPose_.y,robotPose_.theta);
            teb_local_planner::PoseSE2 goal(goalPose_.x,goalPose_.y,goalPose_.theta);

            Twist velStart;
            velStart.linear.x() = robotPose_.v;
            velStart.linear.y() = 0.0;
            velStart.angular.z() = robotPose_.w;

            bool plannerStatus = false;
            plannerStatus = planner_->plan(start,goal,&velStart,true);

            if(plannerStatus)
            {
                printf("teb planner success ! \n");
                planner_->getVelocityCommand(cmd.vx,cmd.vy,cmd.w,look_ahead_poses_);
                planner_->getFullTrajectory(traj);

                float max_w = config_->robot.max_vel_theta;
                float max_v = config_->robot.max_vel_x;

                std::cout << "max_v =" << max_v << " max_w =" << max_w << std::endl;
 
                limitVelocity(cmd,max_v,max_w);
                
                // if(cmd.vx < 0)
                // {
                //     printf("backward! \n");
                //     cmd.vx = 0;
                // }
                printf("TEB success! cmd.vx = %f,cmd.w = %f \n",cmd.vx,cmd.w);
                return true;
            }
            else 
            {
                planner_->clearPlanner();
                printf("TEB Planner Failed, Resetting planner...");
                cmd.vx = 0.0000;
                cmd.w  = 0.0000;
                printf("TEB Failed! cmd.vx = %f,cmd.w = %f \n",cmd.vx,cmd.w);
                return false;
            }

        }
    }

}
