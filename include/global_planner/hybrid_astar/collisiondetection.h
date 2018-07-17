#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d.h>

#include "global_planner/hybrid_astar/constants.h"
#include "global_planner/hybrid_astar/lookup.h"
#include "global_planner/hybrid_astar/node2d.h"
#include "global_planner/hybrid_astar/pose2d.h"


namespace HybridAStar {
namespace {
void getConfiguration(const Node2D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  // avoid 2D collision checking
  t = 99;
}

void getConfiguration(const Pose2D* node, float& x, float& y, float& t) {
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


  CollisionDetection(costmap_2d::Costmap2D* costmap );

  /*!
     \brief evaluates whether the configuration is safe
     \return true if it is traversable, else false
  */
  template<typename T> bool isTraversable(const T* node) {
    /* Depending on the used collision checking mechanism this needs to be adjusted
       standard: collision checking using the spatial occupancy enumeration
       other: collision checking using the 2d costmap and the navigation stack
    */
    float cost = 0;
    float x;
    float y;
    float t;
    // assign values to the configuration
    getConfiguration(node, x, y, t);

    // 2D collision test
    if (t == 99) {
      return !grid->data[node->getIdx()];
    }

    cost = configurationTest(x, y, t) ? 0 : 1;

    return cost <= 0;
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
  */
  bool configurationTest(float x, float y, float t);

  /*!
     \brief updates the grid with the world map
  */
  void updateGrid(nav_msgs::OccupancyGrid::Ptr map) {grid = map;}

 private:
  /// Constructor
  CollisionDetection();
  /// The occupancy grid
  nav_msgs::OccupancyGrid::Ptr grid;
  /// The collision lookup table
  Constants::config collisionLookup[Constants::headings * Constants::positions];

  costmap_2d::Costmap2D* costmap_;
};
}
#endif // COLLISIONDETECTION_H