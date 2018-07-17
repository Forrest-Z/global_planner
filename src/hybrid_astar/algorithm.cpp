#include "global_planner/hybrid_astar/algorithm.h"
#include <Eigen/Dense>
#include <boost/heap/binomial_heap.hpp>

using namespace HybridAStar;


void updateH(Pose2D& start, const Pose2D& goal, Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace);
Pose2D* dubinsShot(Pose2D& start, const Pose2D& goal, CollisionDetection& configurationSpace);

//###################################################
//                                    NODE COMPARISON
//###################################################
/*!
   \brief A structure to sort nodes in a heap structure
*/
struct CompareNodes {
  /// Sorting 3D nodes by increasing C value - the total estimated cost
  bool operator()(const Pose2D* lhs, const Pose2D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
  /// Sorting 2D nodes by increasing C value - the total estimated cost
  bool operator()(const Node2D* lhs, const Node2D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
};

//###################################################
//                                        3D A*
//###################################################
Pose2D* Algorithm::hybridAStar(Pose2D& start,
                               const Pose2D& goal,
                               Pose2D* nodes3D,
                               Node2D* nodes2D,
                               int width,
                               int height,
                               CollisionDetection& configurationSpace
                               ) {

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;
  // Number of possible directions, 3 for forward driving and an additional 3 for reversing
  // int dir = Constants::reverse ? 6 : 3;
  int dir = 6;
  // Number of iterations the algorithm has run for stopping based on Constants::iterations
  int iterations = 0;

  // OPEN LIST AS BOOST IMPLEMENTATION
  typedef boost::heap::binomial_heap<Pose2D*,
          boost::heap::compare<CompareNodes>
          > priorityQueue;
  priorityQueue O;

  // update h value
  updateH(start, goal, nodes2D, width, height, configurationSpace);
  // mark start as open
  start.open();

  // push on priority queue aka open list
  O.push(&start);


  iPred = start.setIdx(width, height);

  nodes3D[iPred] = start;

  // NODE POINTER
  Pose2D* nPred;
  Pose2D* nSucc;

  // float max = 0.f;

  // continue until O empty
  while (!O.empty()) {

    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(width, height);
    //std::cout<<"YT: iPred is " << iPred <<std::endl;
    iterations++;



    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes3D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
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
      if (*nPred == goal || iterations > Constants::iterations) {

        std::cout<<"nPred == goal"<<(*nPred == goal)<< "or iterations > constants::iterations"<<(iterations>Constants::iterations)<<std::endl;
        // DEBUG
	      std::cout<<"x= "<<nPred->getX()<<", y= "<<nPred->getY()<<std::endl;
        return nPred;
      }

      // ____________________
      // CONTINUE WITH SEARCH
      else {

        // ______________________________
        // SEARCH WITH FORWARD SIMULATION
        for (int i = 0; i < dir; i++) {//YT search with different directions
          // create possible successor
          nSucc = nPred->createSuccessor(i);


          // set index of the successor
          iSucc = nSucc->setIdx(width, height);
          //std::cout<< "YT: iSucc = " << iSucc<< std::endl;

          // ensure successor is on grid and traversable
          if (nSucc->isOnGrid(width, height) && configurationSpace.isTraversable(nSucc)) {

            // ensure successor is not on closed list or it has the same index as the predecessor
            if (!nodes3D[iSucc].isClosed() || iPred == iSucc) {

              // calculate new G value
              nSucc->updateG();
              newG = nSucc->getG();

              // if successor not on open list or found a shorter way to the cell
              if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc) {

                // calculate H value
                updateH(*nSucc, goal, nodes2D, width, height, configurationSpace);

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
          } else { delete nSucc; }
        }
      }
    }
  }

  if (O.empty()) {
std::cout<<"openlist is empty"<<std::endl;
    return nullptr;
  }
  return nullptr;
}

//###################################################
//                                        2D A*
//###################################################
float aStar(Node2D& start,
            Node2D& goal,
            Node2D* nodes2D,
            int width,
            int height,
            CollisionDetection& configurationSpace
            ) {

  // PREDECESSOR AND SUCCESSOR INDEX
  //std::cout<<"only begin astar"<<std::endl;
  int iPred, iSucc;
  float newG;

  // reset the open and closed list
  for (int i = 0; i < width * height; ++i) {
    nodes2D[i].reset();
  }

  // VISUALIZATION DELAY
  ros::Duration d(0.001);

  boost::heap::binomial_heap<Node2D*,
        boost::heap::compare<CompareNodes>> O;

  // update h value
  start.updateH(goal);

  // mark start as open
  start.open();
  // push on priority queue
  O.push(&start);
  iPred = start.setIdx(width);

//std::cout << "YT: iPred in 2D, x = " << start.getX() << ", y = " << start.getY() << ", index = " << iPred << std::endl;
  nodes2D[iPred] = start;

  // NODE POINTER
  Node2D* nPred;
  Node2D* nSucc;

  // continue until O empty
  while (!O.empty()) {
    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(width);

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes2D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes2D[iPred].isOpen()) {
      // add node to closed list
      nodes2D[iPred].close();
      nodes2D[iPred].discover();


      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST
      if (*nPred == goal) {
        return nPred->getG();
      }
      // ____________________
      // CONTINUE WITH SEARCH
      else {
        // _______________________________
        // CREATE POSSIBLE SUCCESSOR NODES
        for (int i = 0; i < Node2D::dir; i++) {
          // create possible successor
          nSucc = nPred->createSuccessor(i);

          // set index of the successor
          iSucc = nSucc->setIdx(width);

          // ensure successor is on grid ROW MAJOR
          // ensure successor is not blocked by obstacle
          // ensure successor is not on closed list
          if (nSucc->isOnGrid(width, height) &&  configurationSpace.isTraversable(nSucc) && !nodes2D[iSucc].isClosed()) {
            // calculate new G value
            nSucc->updateG();
            newG = nSucc->getG();

            // if successor not on open list or g value lower than before put it on open list
            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG()) {
              // calculate the H value
              nSucc->updateH(goal);
              // put successor on open list
              nSucc->open();
              nodes2D[iSucc] = *nSucc;
              O.push(&nodes2D[iSucc]);
              delete nSucc;
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  // return large number to guide search away
  return 1000;
}

//###################################################
//                                         COST TO GO
//###################################################
void updateH(Pose2D& start, const Pose2D& goal, Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace) {
  float dubinsCost = 0;
  float reedsSheppCost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;

  // if dubins heuristic is activated calculate the shortest path
  // constrained without obstacles
  // if (Constants::dubins) {
  if(0){
    ompl::base::DubinsStateSpace dubinsPath(Constants::r);
    State* dbStart = (State*)dubinsPath.allocState();
    State* dbEnd = (State*)dubinsPath.allocState();
    dbStart->setXY(start.getX(), start.getY());
    dbStart->setYaw(start.getT());
    dbEnd->setXY(goal.getX(), goal.getY());
    dbEnd->setYaw(goal.getT());
    dubinsCost = dubinsPath.distance(dbStart, dbEnd);

  }

  // if reversing is active use a
  if (/*Constants::reverse && !Constants::dubins*/0) {
        ros::Time t0 = ros::Time::now();
    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    State* rsStart = (State*)reedsSheppPath.allocState();
    State* rsEnd = (State*)reedsSheppPath.allocState();
    rsStart->setXY(start.getX(), start.getY());
    rsStart->setYaw(start.getT());
    rsEnd->setXY(goal.getX(), goal.getY());
    rsEnd->setYaw(goal.getT());
    reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
        ros::Time t1 = ros::Time::now();
        ros::Duration d(t1 - t0);

        //std::cout << "calculated Reed-Sheep Heuristic in ms: " << d * 1000 << ", cost is " << reedsSheppCost << std::endl;
  }

  //std::cout << "YT: Constants::twoD = " << Constants::twoD << std::endl;
  // if twoD heuristic is activated determine shortest path
  // unconstrained with obstacles
  if (Constants::twoD && !nodes2D[(int)start.getY() * width + (int)start.getX()].isDiscovered()) {

//        ros::Time t0 = ros::Time::now();
    // create a 2d start node
    Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
    // create a 2d goal node
    Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);

    // run 2d astar and return the cost of the cheapest path for that node
    nodes2D[(int)start.getY() * width + (int)start.getX()].setG(aStar(goal2d, start2d, nodes2D, width, height, configurationSpace));

//        ros::Time t1 = ros::Time::now();
//        ros::Duration d(t1 - t0);
//        std::cout << "calculated 2D Heuristic in ms: " << d * 1000 << std::endl;
  }



  if (Constants::twoD) {
    // offset for same node in cell
    twoDoffset = sqrt(((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) * ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) +
                      ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())) * ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())));
    twoDCost = nodes2D[(int)start.getY() * width + (int)start.getX()].getG() - twoDoffset;

  }

  // return the maximum of the heuristics, making the heuristic admissable
  start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));
}

//###################################################
//                                        DUBINS SHOT
//###################################################
Pose2D* dubinsShot(Pose2D& start, const Pose2D& goal, CollisionDetection& configurationSpace) {
  // // start
  // double q0[] = { start.getX(), start.getY(), start.getT() };
  // // goal
  // double q1[] = { goal.getX(), goal.getY(), goal.getT() };
  // // initialize the path
  // DubinsPath path;
  // // calculate the path
  // dubins_init(q0, q1, Constants::r, &path);

  // int i = 0;
  // float x = 0.f;
  // float length = dubins_path_length(&path);

  // Pose2D* dubinsNodes = new Pose2D [(int)(length / Constants::dubinsStepSize) + 1];

  // while (x <  length) {
  //   double q[3];
  //   dubins_path_sample(&path, x, q);
  //   dubinsNodes[i].setX(q[0]);
  //   dubinsNodes[i].setY(q[1]);
  //   dubinsNodes[i].setT(Helper::normalizeHeadingRad(q[2]));

  //   // collision check
  //   if (configurationSpace.isTraversable(&dubinsNodes[i])) {

  //     // set the predecessor to the previous step
  //     if (i > 0) {
  //       dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
  //     } else {
  //       dubinsNodes[i].setPred(&start);
  //     }

  //     if (&dubinsNodes[i] == dubinsNodes[i].getPred()) {
  //       std::cout << "looping shot";
  //     }

  //     x += Constants::dubinsStepSize;
  //     i++;
  //   } else {
  //   //        std::cout << "Dubins shot collided, discarding the path" << "\n";
  //     // delete all nodes
  //     delete [] dubinsNodes;
  //     return nullptr;
  //   }
  // }

  //   //std::cout << "Dubins shot connected, returning the path" << "\n";
  // return &dubinsNodes[i - 1];
  return nullptr;
}
