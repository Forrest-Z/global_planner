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
#include <global_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <global_planner/hybrid_astar/planner.h>

#include <base_local_planner/odometry_helper_ros.h>

#include <global_planner/costmap_model.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

namespace global_planner {


GlobalPlanner::GlobalPlanner() :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros, std::string frame_id) :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
    initialize(name, costmap_ros, frame_id);
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros, costmap_ros->getGlobalFrameID());
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros, std::string frame_id) 
{
    if (!initialized_)
    {

        ros::NodeHandle private_nh("~/" + name);


        frame_id_ = frame_id;

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);


        private_nh.param("allow_unknown", allow_unknown_, true);
        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        private_nh.param("default_tolerance", default_tolerance_, 0.1);


        double costmap_pub_freq;
        private_nh.param("planner_costmap_publish_frequency", costmap_pub_freq, 0.0);

        //get the tf prefix
        ros::NodeHandle prefix_nh;
        tf_prefix_ = tf::getPrefixParam(prefix_nh);
        

        yt_planner_ = new HybridAStar::Planner(costmap_, footprint_spec_);


        costmap_ = costmap_ros->getCostmap();
        
        footprint_spec_ = costmap_ros->getRobotFootprint();

        world_model_ = new CostmapModel(*costmap_);
        
        initialized_ = true;
    } 
    else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
}

void GlobalPlanner::clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }
    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

void GlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

bool GlobalPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}


bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan) {
        return makePlan(start, goal, default_tolerance_, plan);
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) {
    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }


    //hot start or cold start
    if(plan.size() != 0)
    {
        std::cout << "YT: hot start planner" << std::endl;
        plan.clear();
    }
    else
    {
        std::cout << "YT: cold start planner" << std::endl;
    }

    std::string global_frame = frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
        return false;
    }

    if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;


    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        ROS_WARN(
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }

        worldToMap(wx, wy, start_x, start_y);

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }

        worldToMap(wx, wy, goal_x, goal_y);

    //clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);

    clearRobotCell(start_pose, start_x_i, start_y_i);

////////////////////////////////////////////////////////////////

    nav_msgs::OccupancyGrid::Ptr temp_map(new nav_msgs::OccupancyGrid);

    geometry_msgs::PoseWithCovarianceStamped::Ptr temp_start(new geometry_msgs::PoseWithCovarianceStamped);

    geometry_msgs::PoseStamped::Ptr temp_goal(new geometry_msgs::PoseStamped);

    temp_map->info.resolution = costmap_->getResolution();
    temp_map->info.width = costmap_->getSizeInCellsX();
    temp_map->info.height = costmap_->getSizeInCellsY();
    temp_map->info.origin.position.x = costmap_->getOriginX();
    temp_map->info.origin.position.y = costmap_->getOriginY();
    temp_map->info.origin.position.z = 0;
    temp_map->info.origin.orientation.x = 0;
    temp_map->info.origin.orientation.y = 0;
    temp_map->info.origin.orientation.z = 0;
    temp_map->info.origin.orientation.w = 1;

        temp_map->data.resize(costmap_->getSizeInCellsX()*costmap_->getSizeInCellsY());

        memcpy(temp_map->data.data(), costmap_->getCharMap(), costmap_->getSizeInCellsX()*costmap_->getSizeInCellsY()*sizeof(char));

        yt_planner_->plan(temp_map, start, goal);

       //YT toggle the result path and add start and goal
    if(yt_planner_->smoothedPath.path.poses.size() != 0)
    {
    plan.resize(yt_planner_->smoothedPath.path.poses.size() + 2);

    plan.at(0).pose.position.x = start.pose.position.x;
    plan.at(0).pose.position.y = start.pose.position.y;
    plan.at(0).pose.position.z = start.pose.position.z;
    plan.at(0).pose.orientation.x = start.pose.orientation.x;
    plan.at(0).pose.orientation.y = start.pose.orientation.y;
    plan.at(0).pose.orientation.z = start.pose.orientation.z;
    plan.at(0).pose.orientation.w = start.pose.orientation.w;
    plan.at(0).header.frame_id = frame_id_;

    plan.at(yt_planner_->smoothedPath.path.poses.size()+1).pose.position.x = goal.pose.position.x;
    plan.at(yt_planner_->smoothedPath.path.poses.size()+1).pose.position.y = goal.pose.position.y;
    plan.at(yt_planner_->smoothedPath.path.poses.size()+1).pose.position.z = goal.pose.position.z;
    plan.at(yt_planner_->smoothedPath.path.poses.size()+1).pose.orientation.w = goal.pose.orientation.w;
    plan.at(yt_planner_->smoothedPath.path.poses.size()+1).pose.orientation.x = goal.pose.orientation.x;
    plan.at(yt_planner_->smoothedPath.path.poses.size()+1).pose.orientation.y = goal.pose.orientation.y;
    plan.at(yt_planner_->smoothedPath.path.poses.size()+1).pose.orientation.z = goal.pose.orientation.z;
    plan.at(yt_planner_->smoothedPath.path.poses.size()+1).header.frame_id = frame_id_;


    for(unsigned int i = 0;i <yt_planner_->smoothedPath.path.poses.size();i++)
    {
        plan.at(yt_planner_->smoothedPath.path.poses.size()-i).pose.position.x = yt_planner_->smoothedPath.path.poses.at(i).pose.position.x;
        plan.at(yt_planner_->smoothedPath.path.poses.size()-i).pose.position.y = yt_planner_->smoothedPath.path.poses.at(i).pose.position.y;
        plan.at(yt_planner_->smoothedPath.path.poses.size()-i).pose.position.z = yt_planner_->smoothedPath.path.poses.at(i).pose.position.z;
        plan.at(yt_planner_->smoothedPath.path.poses.size()-i).pose.orientation.x = yt_planner_->smoothedPath.path.poses.at(i).pose.orientation.x;
        plan.at(yt_planner_->smoothedPath.path.poses.size()-i).pose.orientation.y = yt_planner_->smoothedPath.path.poses.at(i).pose.orientation.y;
        plan.at(yt_planner_->smoothedPath.path.poses.size()-i).pose.orientation.z = yt_planner_->smoothedPath.path.poses.at(i).pose.orientation.z;
        plan.at(yt_planner_->smoothedPath.path.poses.size()-i).pose.orientation.w = yt_planner_->smoothedPath.path.poses.at(i).pose.orientation.w;
        plan.at(yt_planner_->smoothedPath.path.poses.size()-i).header.frame_id = frame_id_;
    }
}
    //////////////////////////////////////////////////////////////////////

    if(!plan.empty())
    {
        return true;
    }
    else
    {
        std::cout << "YT: plan is empty" << std::endl;
        return false;
    }
}


double GlobalPlanner::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{
  return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
}

} //end namespace global_planner
