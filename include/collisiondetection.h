#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d.h>

#include "constants.h"
#include "lookup.h"
#include "node2d.h"
#include "pose2d.h"
#include "world_model.h"

namespace HybridAStar {

namespace {
void getConfiguration(const global_planner::Node2D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  // avoid 2D collision checking
  t = 99;
}

void getConfiguration(const global_planner::Pose2D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  t = node->getT();
}
}
/*!
   \brief The CollisionDetection class determines whether a given configuration q of the robot will result in a collision with the environment.

   It is supposed to return a boolean value that returns true for collisions and false in the case of a safe node.
*/

class CollisionDetection {
  public:
  //YT 这个原来的栅格地图的匹配肯定不能用，所以先删掉，实现两个部分：
  //YT 简单的碰撞检测就是在configuration space 对机器人几何中心所在网格进行查表
  //YT 复杂的碰撞检测先读取制作，机器人footprint，在costmap_2d框架下进行碰撞检测

  CollisionDetection(costmap_2d::Costmap2D* costmap );


  bool isTraversable(const global_planner::Node2D* node)
  {
    float x;
    float y;
    float t;
    getConfiguration(node, x, y, t);
    
    return !grid->data[node->getIdx()];
  }

  bool isTraversable(const global_planner::Pose2D* pose)
  {
    float x;
    float y;
    float t;
    getConfiguration(pose, x, y, t);

    return configurationTest(x, y, t);
  }

  /*!
     \brief Calculates the cost of the robot taking a specific configuration q int the World W
     \param x the x position
     \param y the y position
     \param t the theta angle
     \return the cost of the configuration q of W(q)
     \todo needs to be implemented correctly
  */
  float configurationCost(float x, float y, float t) {return 0;}

  /*!
     \brief Tests whether the configuration q of the robot is in C_free
     \param x the x position
     \param y the y position
     \param t the theta angle
     \return true if it is in C_free, else false
     \YT 检测是否碰撞障碍物的关键函数
  */
  bool configurationTest(float x, float y, float t);

  /*!
     \brief updates the grid with the world map
  */
  void updateGrid(nav_msgs::OccupancyGrid::Ptr map) {grid = map;}

 private:
  /// The occupancy grid
  nav_msgs::OccupancyGrid::Ptr grid;
  /// The collision lookup table
  global_planner::Constants::config collisionLookup[global_planner::Constants::headings * global_planner::Constants::positions];

  costmap_2d::Costmap2D* costmap_;

  global_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use

  std::vector<geometry_msgs::Point> footprint_spec_;

};
}
#endif // COLLISIONDETECTION_H