#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>

typedef ompl::base::SE2StateSpace::StateType State;

#include "pose2d.h"
#include "node2d.h"
#include "collisiondetection.h"

// namespace HybridAStar {
namespace Algorithm {

/*!
 * \brief A class that encompasses the functions central to the search.
 */
class Algorithm {
 public:
  /// The deault constructor
  Algorithm() {}

  // HYBRID A* ALGORITHM
  /*!
     \brief The heart of the planner, the main algorithm starting the search for a collision free and drivable path.

     \param start the start pose
     \param goal the goal pose
     \param nodes3D the array of 3D nodes representing the configuration space C in R^3
     \param nodes2D the array of 2D nodes representing the configuration space C in R^2
     \param width the width of the grid in number of cells
     \param height the height of the grid in number of cells
     \param configurationSpace the lookup of configurations and their spatial occupancy enumeration
     \param dubinsLookup the lookup of analytical solutions (Dubin's paths)
     \return the pointer to the node satisfying the goal condition
  */
  // virtual bool plan(HybridAStar::Pose2D& start, 
  //                   const HybridAStar::Pose2D& goal, 
  //                   HybridAStar::Pose2D* nodes3D, 
  //                   HybridAStar::Node2D* nodes2D, 
  //                   int width, 
  //                   int height, 
  //                   HybridAStar::CollisionDetection& configurationSpace);
  // virtual bool updateH(HybridAStar::Pose2D& start, const HybridAStar::Pose2D& goal, HybridAStar::Node2D* nodes2D, int width, int height, HybridAStar::CollisionDetection& configurationSpace);
  static HybridAStar::Pose2D* hybridAStar(HybridAStar::Pose2D& start,
                             const HybridAStar::Pose2D& goal,
                             HybridAStar::Pose2D* nodes3D,
                             HybridAStar::Node2D* nodes2D,
                             int width,
                             int height,
                             HybridAStar::CollisionDetection& configurationSpace
                             );
  float aStar(HybridAStar::Node2D& start, HybridAStar::Node2D& goal, HybridAStar::Node2D* nodes2D, int width, int height, HybridAStar::CollisionDetection& configurationSpace);
};
}//namespace
#endif // ALGORITHM_H
