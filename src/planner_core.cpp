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
#include <planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <planner.h>

#include <base_local_planner/odometry_helper_ros.h>

#include <costmap_model.h>

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
        ROS_WARN("YT: start initializing global_planner");
        ros::NodeHandle private_nh("~/" + name);

        frame_id_ = frame_id;

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        mid_result_pub_ = private_nh.advertise<geometry_msgs::PoseArray>("mid_result", 1);

        private_nh.param("allow_unknown", allow_unknown_, true);//YT 将地图上没有的空间都视为自由空间
        private_nh.param("default_tolerance", default_tolerance_, 0.1);
        private_nh.param("cell_divider", cell_divider_, 1);
        private_nh.param("using_voronoi", using_voronoi_, true);
        private_nh.param("lazy_replanning", lazy_replanning_, false);//YT 如果启动lazy_replanning那么只有当障碍物被挡住时才会启动重新规划路径，可以应对相同代价路径之间的抖动
        //get the tf prefix
        ros::NodeHandle prefix_nh;
        tf_prefix_ = tf::getPrefixParam(prefix_nh);
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros->getCostmap();
        
        footprint_spec_ = costmap_ros->getRobotFootprint();

        yt_planner_ = new global_planner::Planner(costmap_, footprint_spec_, cell_divider_, using_voronoi_, lazy_replanning_);

        initialized_ = true;
        ROS_WARN("YT: global_planner has been initialized");
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
    ROS_WARN("YT: start making plan by global_planner");
    plan.clear();

    std::string global_frame = frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                    tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
        return false;
    }

    if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                    tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
        return false;
    }


    // unsigned int start_cell_x, start_cell_y, goal_cell_x, goal_cell_y;

    // if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_cell_x, start_cell_y)) {
    //     ROS_WARN(
    //             "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
    //     return false;
    // }

    // if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_cell_x, goal_cell_y)) {
    //     ROS_WARN_THROTTLE(1.0,
    //             "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
    //     return false;
    // }

    // bool clear_start_pose = true;
    // if(clear_start_pose)
    // {
    //     std::cout << "YT: clear the starting cell within the costmap because we know it can't be an obstacle" << std::endl;
    //     tf::Stamped<tf::Pose> start_pose;
    //     tf::poseStampedMsgToTF(start, start_pose);
    //     clearRobotCell(start_pose, start_cell_x, start_cell_y);
    // }

        yt_planner_->plan(start, goal, plan);

    publishMidResult(yt_planner_->mid_result);

    for(unsigned int i = 0; i < plan.size(); i++)
    {
        plan.at(i).header.frame_id = global_frame;
    }

    if(!plan.empty())
    {
        publishPlan(plan);
    }

    return !plan.empty();
}

void GlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if (!path.empty()) {
        gui_path.header.frame_id = path[0].header.frame_id;
        gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

void GlobalPlanner::publishMidResult(geometry_msgs::PoseArray& mid_result){
    if(!initialized_){
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    // geometry_msgs::PoseArray mid_temp;
    // mid_temp.poses.resize(mid_result.size());
    
    // if(!mid_result.empty()){
    //     mid_temp.header.frame_id = frame_id_;
    //     mid_temp.header.stamp = ros::Time::now();
    // }

    // for (unsigned int i = 0; i < mid_result.size(); i++){
    //     mid_temp.poses.at(i).position.x = mid_result.at(i).getX();
    //     mid_temp.poses.at(i).position.y = mid_result.at(i).getY();
    //     mid_temp.poses.at(i).orientation.w = 1;
    // }
    // mid_result_pub_.publish(mid_temp);

    mid_result.header.frame_id = frame_id_;
    mid_result.header.stamp = ros::Time::now();
    mid_result_pub_.publish(mid_result);
}



} //end namespace global_planner
