#ifndef _PLANNERCORE_H
#define _PLANNERCORE_H
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#define POT_HIGH 1.0e10        // unassigned cell potential
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <dynamic_reconfigure/server.h>
#include <global_planner/GlobalPlannerConfig.h>
#include <global_planner/hybrid_astar/planner.h>
#include <global_planner/world_model.h>

#include <base_local_planner/odometry_helper_ros.h>

namespace global_planner {

const float dx[] = { 0.7068582,   0.705224,   0.705224};
const float dy[] = { 0,        -0.0415893,  0.0415893};
const float dr[] = { 0,         0.1178097,   -0.1178097};


/**
 * @class PlannerCore
 * @brief Provides a ROS wrapper for the global_planner planner which runs a fast, interpolated navigation function on a costmap.
 */

class GlobalPlanner : public nav_core::BaseGlobalPlanner {
    public:
        /**
         * @brief  Default constructor for the PlannerCore object
         */
        GlobalPlanner();

        /**
         * @brief  Constructor for the PlannerCore object
         * @param  name The name of this planner
         * @param  costmap A pointer to the costmap to use
         * @param  frame_id Frame of the costmap
         */
        GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros, std::string frame_id);

        /**
         * @brief  Default deconstructor for the PlannerCore object
         */
        ~GlobalPlanner(){}

        /**
         * @brief  Initialization function for the PlannerCore object
         * @param  name The name of this planner
         * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
         */
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros, std::string frame_id);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);

        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
                      std::vector<geometry_msgs::PoseStamped>& plan);

        double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);


    protected:

        costmap_2d::Costmap2D* costmap_;
        std::string frame_id_;
        ros::Publisher plan_pub_;
        bool initialized_, allow_unknown_;

    private:

        void mapToWorld(double mx, double my, double& wx, double& wy);
        bool worldToMap(double wx, double wy, double& mx, double& my);
        void clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my);

        // double planner_window_x_, planner_window_y_, 
        double default_tolerance_;
        std::string tf_prefix_;
        boost::mutex mutex_;

        HybridAStar::Planner *yt_planner_ ;

        unsigned int start_x_, start_y_, end_x_, end_y_;

        float convert_offset_;

        std::vector<geometry_msgs::Point> footprint_spec_;

        WorldModel* world_model_; ///< @brief The world model that the controller will use

};

} //end namespace global_planner

#endif
