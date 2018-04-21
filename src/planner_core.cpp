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

#include <global_planner/dijkstra.h>
#include <global_planner/astar.h>
#include <global_planner/grid_path.h>
#include <global_planner/gradient_path.h>
#include <global_planner/quadratic_calculator.h>

#include <global_planner/hybrid_astar/planner.h>

#include <base_local_planner/odometry_helper_ros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

namespace global_planner {

void GlobalPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

GlobalPlanner::GlobalPlanner() :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
//    ROS_ERROR("YT: call default constructor");
}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
    //initialize the planner
  //      ROS_ERROR("YT: call normal constructor");
    initialize(name, costmap, frame_id);
}

GlobalPlanner::~GlobalPlanner() {
    if (p_calc_)
        delete p_calc_;
    if (planner_)
        delete planner_;
    if (path_maker_)
        delete path_maker_;
    if (dsrv_)
        delete dsrv_;
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
//        ROS_ERROR("YT: call initialize of GlobalPlanner");
    if (!initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        costmap_ = costmap;
        frame_id_ = frame_id;

        unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();

        private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);
        if(!old_navfn_behavior_)
        {
            convert_offset_ = 0.5;}
        else
            convert_offset_ = 0.0;

        bool use_quadratic;
        private_nh.param("use_quadratic", use_quadratic, true);
        if (use_quadratic)
        {    //ROS_ERROR("YT: use_quadratic, set size");
            p_calc_ = new QuadraticCalculator(cx, cy);}
        else
            p_calc_ = new PotentialCalculator(cx, cy);

        bool use_dijkstra;
        private_nh.param("use_dijkstra", use_dijkstra, true);
        if (use_dijkstra)
        {
//            ROS_ERROR("YT: use_dijkstra");
            DijkstraExpansion* de = new DijkstraExpansion(p_calc_, cx, cy);
            if(!old_navfn_behavior_)
                de->setPreciseStart(true);
            planner_ = de;
        }
        else
            planner_ = new AStarExpansion(p_calc_, cx, cy);

        bool use_grid_path;
        private_nh.param("use_grid_path", use_grid_path, false);
        if (use_grid_path)
          {  //ROS_ERROR("YT: use_grid_path");
            path_maker_ = new GridPath(p_calc_);}
        else
            path_maker_ = new GradientPath(p_calc_);
            
        orientation_filter_ = new OrientationFilter();

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
//        plan_pub_orientation_ = private_nh.advertise<geometry_msgs::Twist>("HAstarpathposeastwist", 1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

        private_nh.param("allow_unknown", allow_unknown_, true);
        planner_->setHasUnknown(allow_unknown_);
        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        private_nh.param("default_tolerance", default_tolerance_, 0.1);
        private_nh.param("publish_scale", publish_scale_, 100);

        double costmap_pub_freq;
        private_nh.param("planner_costmap_publish_frequency", costmap_pub_freq, 0.0);


        //get the tf prefix
        ros::NodeHandle prefix_nh;
        tf_prefix_ = tf::getPrefixParam(prefix_nh);

        make_plan_srv_ = private_nh.advertiseService("make_plan", &GlobalPlanner::makePlanService, this);

        dsrv_ = new dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>::CallbackType cb = boost::bind(
                &GlobalPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        initialized_ = true;
    } else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");

}

void GlobalPlanner::reconfigureCB(global_planner::GlobalPlannerConfig& config, uint32_t level) {
    planner_->setLethalCost(config.lethal_cost);
    path_maker_->setLethalCost(config.lethal_cost);
    planner_->setNeutralCost(config.neutral_cost);
    planner_->setFactor(config.cost_factor);
    publish_potential_ = config.publish_potential;
    orientation_filter_->setMode(config.orientation_mode);
}

void GlobalPlanner::clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool GlobalPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    //ROS_ERROR("YT: someone calls makePlanService");
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
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
    ///YT if we want to avoid the obstacle locally, then a plan of last time must be passed through "plan"
    ///
    bool last_is_feasible = false;
    if(!last_is_feasible)
        return makePlan(start, goal, default_tolerance_, plan);
    else
        ///YT try to generate new path based on last path
        return false;
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) {
    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;
    ROS_INFO("YT: frame_id=%s", frame_id_.c_str());

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
    if(old_navfn_behavior_){
        start_x = start_x_i;
        start_y = start_y_i;
    }else{
        worldToMap(wx, wy, start_x, start_y);
    }

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;
    //goal_theta = goal.pose.orientation.w;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    if(old_navfn_behavior_){
        goal_x = goal_x_i;
        goal_y = goal_y_i;
    }else{
        worldToMap(wx, wy, goal_x, goal_y);
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);

    clearRobotCell(start_pose, start_x_i, start_y_i);

    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();//3D A*

    //make sure to resize the underlying array that Navfn uses
    p_calc_->setSize(nx, ny);
    planner_->setSize(nx, ny);
    path_maker_->setSize(nx, ny);
    potential_array_ = new float[nx * ny];//YT cannot change this, only change the value of each cell based on the searching method

    //outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

//    bool found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, start_theta, goal_x, goal_y, goal_theta,
//                                                    nx * ny * 2, potential_array_);

//    if(!old_navfn_behavior_)
//        planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, goal_theta_i, 2);
//    if(publish_potential_)
//        publishPotential(potential_array_);

//    if (found_legal) {
//        //extract the plan
//        if (getPlanFromPotential(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, goal, plan)) {
////        if (getHybridAstarPlan(costmap_->getCharMap(), start_x, start_y, start_theta, goal_x, goal_y, goal_theta, goal, plan)) {
//            //make sure the goal we push on has the same timestamp as the rest of the plan
//            ROS_ERROR("YT: found plan from potential");
//            geometry_msgs::PoseStamped goal_copy = goal;
//            goal_copy.header.stamp = ros::Time::now();
//            plan.push_back(goal_copy);
//        } else {
//            ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
//        }
//    }else{
//        ROS_ERROR("Failed to get a plan.");
//    }

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

        temp_start->pose.pose.position.x = start.pose.position.x;
        temp_start->pose.pose.position.y = start.pose.position.y;
        temp_start->pose.pose.position.z = start.pose.position.z;
        temp_start->pose.pose.orientation.x = start.pose.orientation.x;
        temp_start->pose.pose.orientation.y = start.pose.orientation.y;
        temp_start->pose.pose.orientation.z = start.pose.orientation.z;
        temp_start->pose.pose.orientation.w = start.pose.orientation.w;

        temp_goal->pose.position.x = goal.pose.position.x;
        temp_goal->pose.position.y = goal.pose.position.y;
        temp_goal->pose.position.z = goal.pose.position.z;
        temp_goal->pose.orientation.w = goal.pose.orientation.w;
        temp_goal->pose.orientation.x = goal.pose.orientation.x;
        temp_goal->pose.orientation.y = goal.pose.orientation.y;
        temp_goal->pose.orientation.z = goal.pose.orientation.z;

geometry_msgs::PoseWithCovarianceStamped::ConstPtr temp_start_c = temp_start;
geometry_msgs::PoseStamped::ConstPtr temp_goal_c = temp_goal;
//    HybridAStar::Planner yt_planner(temp_map_ptr, temp_start_ptr, temp_goal_ptr);
      this->yt_planner_.setMapfromParam(temp_map);
      this->yt_planner_.setStartfromParam(temp_start_c);
      this->yt_planner_.setGoalfromParam(temp_goal_c);

    //YT toggle the result path and add start and goal
if(yt_planner_.smoothedPath.getPath().poses.size() != 0)

{
    plan.resize(yt_planner_.smoothedPath.getPath().poses.size() + 2);

    plan.at(0).pose.position.x = temp_start->pose.pose.position.x;
    plan.at(0).pose.position.y = temp_start->pose.pose.position.y;
    plan.at(0).pose.position.z = temp_start->pose.pose.position.z;
    plan.at(0).pose.orientation.x = temp_start->pose.pose.orientation.x;
    plan.at(0).pose.orientation.y = temp_start->pose.pose.orientation.y;
    plan.at(0).pose.orientation.z = temp_start->pose.pose.orientation.z;
    plan.at(0).pose.orientation.w = temp_start->pose.pose.orientation.w;
    plan.at(0).header.frame_id = frame_id_;

    plan.at(yt_planner_.smoothedPath.getPath().poses.size()+1).pose.position.x = temp_goal->pose.position.x;
    plan.at(yt_planner_.smoothedPath.getPath().poses.size()+1).pose.position.y = temp_goal->pose.position.y;
    plan.at(yt_planner_.smoothedPath.getPath().poses.size()+1).pose.position.z = temp_goal->pose.position.z;
    plan.at(yt_planner_.smoothedPath.getPath().poses.size()+1).pose.orientation.w = temp_goal->pose.orientation.w;
    plan.at(yt_planner_.smoothedPath.getPath().poses.size()+1).pose.orientation.x = temp_goal->pose.orientation.x;
    plan.at(yt_planner_.smoothedPath.getPath().poses.size()+1).pose.orientation.y = temp_goal->pose.orientation.y;
    plan.at(yt_planner_.smoothedPath.getPath().poses.size()+1).pose.orientation.z = temp_goal->pose.orientation.z;
    plan.at(yt_planner_.smoothedPath.getPath().poses.size()+1).header.frame_id = frame_id_;


    for(unsigned int i = 0;i <yt_planner_.smoothedPath.getPath().poses.size();i++)
    {
        plan.at(yt_planner_.smoothedPath.getPath().poses.size()-i).pose.position.x = yt_planner_.smoothedPath.getPath().poses.at(i).pose.position.x;
        plan.at(yt_planner_.smoothedPath.getPath().poses.size()-i).pose.position.y = yt_planner_.smoothedPath.getPath().poses.at(i).pose.position.y;
        plan.at(yt_planner_.smoothedPath.getPath().poses.size()-i).pose.position.z = yt_planner_.smoothedPath.getPath().poses.at(i).pose.position.z;
        plan.at(yt_planner_.smoothedPath.getPath().poses.size()-i).pose.orientation.x = yt_planner_.smoothedPath.getPath().poses.at(i).pose.orientation.x;
        plan.at(yt_planner_.smoothedPath.getPath().poses.size()-i).pose.orientation.y = yt_planner_.smoothedPath.getPath().poses.at(i).pose.orientation.y;
        plan.at(yt_planner_.smoothedPath.getPath().poses.size()-i).pose.orientation.z = yt_planner_.smoothedPath.getPath().poses.at(i).pose.orientation.z;
        plan.at(yt_planner_.smoothedPath.getPath().poses.size()-i).pose.orientation.w = yt_planner_.smoothedPath.getPath().poses.at(i).pose.orientation.w;
        plan.at(yt_planner_.smoothedPath.getPath().poses.size()-i).header.frame_id = frame_id_;
    }
}

//    //YT delete some of the start waypoint
//    if(plan.size() > 5)
//    {
//        std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
//        for(unsigned int i = 0;i<4;i++)
//        {
//            it = plan.erase(it);
//        }
//    }

    //////////////////////////////////////////////////////////////////////
    // add orientations if needed
//    orientation_filter_->processPath(start, plan);
    if(!plan.empty())
    {
        ROS_ERROR("YT: publish global_path, 0 is start:");
        //publish the plan for visualization purposes
           for(unsigned int i = 0; i< plan.size();i++)
           {
//               std::cout << "YT: plan["
//                         << i
//                         << "] : "
//                         << plan.at(i).pose.position.x
//                         << ", "
//                         << plan.at(i).pose.position.y
//                         << ", ["
//                         << plan.at(i).pose.orientation.x
//                         << ", "
//                         << plan.at(i).pose.orientation.y
//                         << ", "
//                         << plan.at(i).pose.orientation.z
//                         << ", "
//                         << plan.at(i).pose.orientation.w
//                         << "] "
//                         << tf::getYaw(plan.at(i).pose.orientation)
//                         << std::endl;
           }
        publishPlan(plan);
    }
    delete potential_array_;
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
    //YT publish another array with orientation
//    geometry_msgs::Twist orientation;
//    for(unsigned int i=0;i<path.size();i++)
//    {
//        orientation.linear.x =cos(tf::getYaw(gui_path.poses.at(i).pose.orientation));
//        orientation.linear.y =sin(tf::getYaw(gui_path.poses.at(i).pose.orientation));
//        plan_pub_orientation_.publish(orientation);
//    }
}

bool GlobalPlanner::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                      const geometry_msgs::PoseStamped& goal,
                                       std::vector<geometry_msgs::PoseStamped>& plan) {
    ROS_ERROR("Enter GlobalPlanner::getPlanFromPotential");
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::string global_frame = frame_id_;

    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float> > path;

    if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
        ROS_ERROR("NO PATH!");
        return false;
    }

    ros::Time plan_time = ros::Time::now();
    ROS_ERROR("YT: found path in getpath");
    for (int i = path.size() -1; i>=0; i--) {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;
        mapToWorld(point.first, point.second, world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }
    if(old_navfn_behavior_){
            plan.push_back(goal);
    }
    return !plan.empty();
}

void GlobalPlanner::publishPotential(float* potential)
{
    ROS_ERROR("Enter GlobalPlanner::publishPotential");
    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
    double resolution = costmap_->getResolution();
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++) {
        float potential = potential_array_[i];
        if (potential < POT_HIGH) {
            if (potential > max) {
                max = potential;
            }
        }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++) {
        if (potential_array_[i] >= POT_HIGH) {
            grid.data[i] = -1;
        } else
            grid.data[i] = potential_array_[i] * publish_scale_ / max;
    }
    potential_pub_.publish(grid);
}

} //end namespace global_planner

