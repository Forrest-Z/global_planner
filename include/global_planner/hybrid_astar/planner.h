#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <costmap_2d/costmap_2d.h>

#include "global_planner/hybrid_astar/constants.h"
#include "global_planner/hybrid_astar/helper.h"
#include "global_planner/hybrid_astar/collisiondetection.h"
#include "global_planner/hybrid_astar/dynamicvoronoi.h"
#include "global_planner/hybrid_astar/algorithm.h"
#include "global_planner/hybrid_astar/pose2d.h"
#include "global_planner/hybrid_astar/path.h"
#include "global_planner/hybrid_astar/lookup.h"

namespace HybridAStar {
/*!
   \brief A class that creates the interface for the hybrid A* algorithm.

    It inherits from `ros::nav_core::BaseGlobalPlanner` so that it can easily be used with the ROS navigation stack
   \todo make it actually inherit from nav_core::BaseGlobalPlanner
*/
class Planner {
 public:
  /// The default constructor
  Planner();

  Planner(costmap_2d::Costmap2D* costmap, std::vector<geometry_msgs::Point> footprint_spec);

  /*!
     \brief Sets the map e.g. through a callback from a subscriber listening to map updates.
     \param map the map or occupancy grid
  */
  void setMapfromParam(costmap_2d::Costmap2D& cost_map);
  void setMapfromTopic(const nav_msgs::OccupancyGrid::Ptr map);
  /*!
     \brief setStart
     \param start the start pose
  */
  void setStartfromParam(const geometry_msgs::PoseStamped start);
  void setStartfromTopic(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  /*!
     \brief setGoal
     \param goal the goal pose
  */
  void setGoalfromParam(const geometry_msgs::PoseStamped end);
  void setGoalfromTopic(const geometry_msgs::PoseStamped::ConstPtr& msg);
  /*!
     \brief The central function entry point making the necessary preparations to start the planning.
  */
  void plan(std::vector<geometry_msgs::PoseStamped>& result_path);

  void plan(const nav_msgs::OccupancyGrid::Ptr temp_map, 
            const geometry_msgs::PoseStamped temp_start, 
            const geometry_msgs::PoseStamped temp_goal,
            std::vector<geometry_msgs::PoseStamped>& result_path);

  void tracePath(const Pose2D* node, int i, std::vector<Pose2D>& path);

  /// The path smoothed and ready for the controller
  Path smoothedPath = Path(true);
 private:
  /// The path produced by the hybrid A* algorithm
  Path path;

  /// The collission detection for testing specific configurations
  CollisionDetection configurationSpace;
  /// The voronoi diagram
  DynamicVoronoi voronoiDiagram;
  /// A pointer to the grid the planner runs on
  nav_msgs::OccupancyGrid::Ptr grid;
  /// The start pose set through RViz
  geometry_msgs::PoseWithCovarianceStamped start;
  /// The goal pose set through RViz
  geometry_msgs::PoseStamped goal;
  /// Flags for allowing the planner to plan
  bool validStart = false;
  /// Flags for allowing the planner to plan
  bool validGoal = false;
  /// A lookup table for configurations of the vehicle and their spatial occupancy enumeration
  Constants::config collisionLookup[Constants::headings * Constants::positions];
  /// A lookup of analytical solutions (Dubin's paths)
  // float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];

  costmap_2d::Costmap2D* costmap_;
  std::vector<geometry_msgs::Point> footprint_spec_;
};
}
#endif // PLANNER_H
