#include "algorithm/HAStar.h"


#include "algorithm/algorithm.h"
#include <Eigen/Dense>
#include <boost/heap/binomial_heap.hpp>

using namespace Algorithm;

void tracePath(const global_planner::Pose2D* node, int i, std::vector<global_planner::Pose2D>& path);


void updateH(global_planner::Pose2D& start, const global_planner::Pose2D& goal);
bool isOnGrid(const global_planner::Pose2D pose, const int width, const int height);

//###################################################
//                                    NODE COMPARISON
//###################################################
/*!
   \brief A structure to sort nodes in a heap structure
*/
struct CompareNodes {
  /// Sorting 3D nodes by increasing C value - the total estimated cost
  bool operator()( global_planner::Pose2D* lhs,  global_planner::Pose2D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
};


bool isOnGrid(const global_planner::Pose2D pose, const int width, const int height){
  return pose.getX() >= 0 && pose.getX() < width && pose.getY() >= 0 && pose.getY() < height && (int)(pose.getT() / global_planner::Constants::deltaHeadingRad) >= 0 && (int)(pose.getT() / global_planner::Constants::deltaHeadingRad) < global_planner::Constants::headings;
}

//###################################################
//                                        3D A*
//###################################################
// global_planner::Pose2D* Algorithm::HAStar::plan(global_planner::Pose2D& start,
//                                const global_planner::Pose2D& goal,
//                                global_planner::Pose2D* nodes3D,
//                                global_planner::Node2D* nodes2D,
//                                int width,
//                                int height,
//                                HybridAStar::CollisionDetection& configurationSpace
//                                ) {

//////////////////////////////////////////

bool Algorithm::HAStar::plan(global_planner::Pose2D& start,
                               const global_planner::Pose2D& goal,
                               global_planner::Pose2D* nodes3D,
                               global_planner::Node2D* nodes2D,
                               int width,
                               int height,
                               CollisionDetection* configurationSpace,
                               std::vector<global_planner::Pose2D>& plan) {

//////////////////////////////////////////
  ROS_ERROR("YT: HAStar algorithm is planning, HAStar.cpp line 58");
  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;
  // Number of possible directions, 3 for forward driving and an additional 3 for reversing
  // int dir = Constants::reverse ? 6 : 3;
  int dir = 3;
  // Number of iterations the algorithm has run for stopping based on Constants::iterations
  int iterations = 0;

  // OPEN LIST AS BOOST IMPLEMENTATION
  typedef boost::heap::binomial_heap<global_planner::Pose2D*,
          boost::heap::compare<CompareNodes>
          > priorityQueue;
  priorityQueue O;

  // update h value
  updateH(start, goal);
  // mark start as open
  start.open();

  // push on priority queue aka open list
  O.push(&start);


  iPred = start.setIdx(width, height);

  nodes3D[iPred] = start;

  // NODE POINTER
  global_planner::Pose2D* nPred;
  global_planner::Pose2D* nSucc;
  // ROS_ERROR("YT: PROBE, HAStar.cpp, line 90");
  // continue until O empty
  while (!O.empty()) {

    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(width, height);
    
    
    // std::cout<<"YT: iPred is " << iPred <<std::endl;
    
    iterations++;

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes3D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      ROS_WARN("YT: nodes3D[iPred].isClosed, HAStar.cpp, line 109");
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes3D[iPred].isOpen()) {
      // add node to closed list
      nodes3D[iPred].close();
      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST
      if (*nPred == goal || iterations > global_planner::Constants::iterations) {
        std::cout<<"nPred == goal"<<(*nPred == goal)<< "or iterations > constants::iterations"<<(iterations>global_planner::Constants::iterations)<<std::endl;
        // DEBUG
	      std::cout<<"x= "<<nPred->getX()<<", y= "<<nPred->getY()<<std::endl;
        return nPred;
      }

      // ____________________
      // CONTINUE WITH SEARCH
      else {
        //  ROS_ERROR("YT: PROBE, HAStar.cpp, line 135");
        // ______________________________
        // SEARCH WITH FORWARD SIMULATION
        for (int i = 0; i < dir; i++) {//YT search with different directions
          // create possible successor
          nSucc = nPred->createSuccessor(i);

          //YT 保存中间结果
          mid_result.push_back(*nSucc);
          // set index of the successor
          iSucc = nSucc->setIdx(width, height);
          
          // std::cout<< "YT: iSucc = " << iSucc<< std::endl;
          // ensure successor is on grid and traversable
          if (isOnGrid(*nSucc, width, height) && configurationSpace->isTraversable(nSucc)) {

            // ensure successor is not on closed list or it has the same index as the predecessor
            if (!nodes3D[iSucc].isClosed() || iPred == iSucc) {

              // calculate new G value
              nSucc->updateG();
              newG = nSucc->getG();

              // if successor not on open list or found a shorter way to the cell
              if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc) {

                // calculate H value
                updateH(*nSucc, goal);

                // if the successor is in the same cell but the C value is larger
                if (iPred == iSucc && nSucc->getC() > nPred->getC()) {
                    //std::cout << "YT nPred, nSucc in the same cell"  << std::endl;
                  delete nSucc;
                  continue;
                }
                // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                else if (iPred == iSucc && nSucc->getC() <= nPred->getC()) {
                  nSucc->setPred(nPred->getPred());
                }

                if (nSucc->getPred() == nSucc) {
                  std::cout << "looping";
                }

                // put successor on open list
                nSucc->open();
                nodes3D[iSucc] = *nSucc;
                O.push(&nodes3D[iSucc]);
                delete nSucc;
              } else { delete nSucc; }
            } else { delete nSucc; }
          } else { ROS_WARN("YT: the nSucc is not on grid, or it is not traversable");delete nSucc; }
        }
      }
    }
  }

  if (O.empty()) {
std::cout<<"openlist is empty"<<std::endl;
    return false;
  }
  return false;
}

//###################################################
//                                         COST TO GO
//###################################################
void updateH(global_planner::Pose2D& start, const global_planner::Pose2D& goal) {

  // float reedsSheppCost = 0;
  // float twoDoffset = 0;

  // // if twoD heuristic is activated determine shortest path
  // // unconstrained with obstacles
  // if ( !nodes2D[(int)start.getY() * width + (int)start.getX()].isDiscovered()) {

  //   // create a 2d start node
  //   Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
  //   // create a 2d goal node
  //   Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);

  //   // run 2d astar and return the cost of the cheapest path for that node
  //   nodes2D[(int)start.getY() * width + (int)start.getX()].setG(aStar(goal2d, start2d, nodes2D, width, height, configurationSpace));

  // }

  //   // offset for same node in cell
  //   twoDoffset = sqrt(((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) * ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) +
  //                     ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())) * ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())));
  //   twoDCost = nodes2D[(int)start.getY() * width + (int)start.getX()].getG() - twoDoffset;

  // // return the maximum of the heuristics, making the heuristic admissable
  // start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));
  double H;
  H = hypot(start.getX() - goal.getX(),   start.getY() - goal.getY()  );
  start.setH(H);
}


void tracePath(const global_planner::Pose2D* node, int i, std::vector<global_planner::Pose2D>& path)
{
    if (node == nullptr) {
      // std::cout << "YT: maybe no path to trace" << std::endl;
    return;
  }
  i++;

  path.push_back(*node);
  tracePath(node->getPred(), i, path);
}
