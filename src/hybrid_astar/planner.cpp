#include "global_planner/hybrid_astar/planner.h"
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################

Planner::Planner() {}

Planner::Planner(costmap_2d::Costmap2D* costmap, std::vector<geometry_msgs::Point> footprint_spec):
  costmap_(costmap),footprint_spec_(footprint_spec) 
{}


//###################################################
//                                                MAP
//###################################################
void Planner::setMapfromParam(costmap_2d::Costmap2D& cost_map) {

  // grid = map;

  // //update the configuration space with the current map
  // configurationSpace.updateGrid(map);

  // int height = map->info.height;
  // int width = map->info.width;

  // bool** binMap;
  // binMap = new bool*[width];

  // for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

  // for (int x = 0; x < width; ++x) {
  //   for (int y = 0; y < height; ++y) {
  //     binMap[x][y] = map->data.at((int)(y * width + x)) ? true : false;
  //   }
  // }

  // voronoiDiagram.initializeMap(width, height, binMap);
  // voronoiDiagram.update();
  // voronoiDiagram.visualize();
}


//###################################################
//                                                MAP
//###################################################
void Planner::setMapfromTopic(const nav_msgs::OccupancyGrid::Ptr map) {
    std::cout << "I am seeing the map..." << std::endl;

  grid = map;

  //update the configuration space with the current map
  configurationSpace.updateGrid(map);

  int height = map->info.height;
  int width = map->info.width;

  bool** binMap;
  binMap = new bool*[width];

  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      binMap[x][y] = map->data.at((int)(y * width + x)) ? true : false;
    }
  }

  voronoiDiagram.initializeMap(width, height, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize();
}

//###################################################
//                                   INITIALIZE START
//###################################################
void Planner::setStartfromParam(const geometry_msgs::PoseStamped initial) {
    //YT must convert x,y to the coordinate of binMap before start the algorithm
  float x = (initial.pose.position.x - grid->info.origin.position.x) / grid->info.resolution;
  float y = (initial.pose.position.y - grid->info.origin.position.y) / grid->info.resolution;
  float t = tf::getYaw(initial.pose.orientation);

  if ( grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0 ) {
    validStart = true;
    start.pose.pose.position.x = initial.pose.position.x;
    start.pose.pose.position.y = initial.pose.position.y;
    start.pose.pose.position.z = initial.pose.position.z;
    start.pose.pose.orientation.w = initial.pose.orientation.w;
    start.pose.pose.orientation.x = initial.pose.orientation.x;
    start.pose.pose.orientation.y = initial.pose.orientation.y;
    start.pose.pose.orientation.z = initial.pose.orientation.z;
  }
  else {
    std::cout << "YT: start is off grid: x = " << x << ", y = " << y << ", t = " << Helper::toDeg(t) << std::endl;
  }
}
void Planner::setStartfromTopic(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  float x = (msg->pose.pose.position.x - grid->info.origin.position.x) / grid->info.resolution;
  float y = (msg->pose.pose.position.y - grid->info.origin.position.y) / grid->info.resolution;
  float t = tf::getYaw(msg->pose.pose.orientation);

  if ( grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0 ) 
  {
    validStart = true;
    start = *msg;
  }
  else 
  {
    std::cout << "YT: start is off grid: x = " << x << ", y = " << y << ", t = " << Helper::toDeg(t) << std::endl;
  }
}
//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::setGoalfromParam(const geometry_msgs::PoseStamped end) {
  // retrieving goal position
  float x = (end.pose.position.x - grid->info.origin.position.x)  / grid->info.resolution;
  float y = (end.pose.position.y - grid->info.origin.position.y)  / grid->info.resolution;
  float t = tf::getYaw(end.pose.orientation);

  if ( grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0 ) {
    validGoal = true;
    goal.pose.position.x = end.pose.position.x;
    goal.pose.position.y = end.pose.position.y;
    goal.pose.position.z = end.pose.position.z;
    goal.pose.orientation.w = end.pose.orientation.w;
    goal.pose.orientation.x = end.pose.orientation.x;
    goal.pose.orientation.y = end.pose.orientation.y;
    goal.pose.orientation.z = end.pose.orientation.z;
  } 
  else 
  {
    std::cout << "YT: goal is off grid: x = " << x << ", y = " << y << ", t = " << Helper::toDeg(t) << std::endl;
  }
}

void Planner::setGoalfromTopic(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  float x = (msg->pose.position.x - grid->info.origin.position.x) / grid->info.resolution;
  float y = (msg->pose.position.y - grid->info.origin.position.y) / grid->info.resolution;
  float t = tf::getYaw(msg->pose.orientation);

  if( grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0)
  {
    validGoal = true;
    goal = *msg;
  }
  else
  {
    std::cout << "YT: goal is off grid: x = " << x << ", y = " << y << ", t = " << Helper::toDeg(t) << std::endl;
  }
}


void Planner::plan(const nav_msgs::OccupancyGrid::Ptr temp_map, 
                   const geometry_msgs::PoseStamped temp_start, 
                   const geometry_msgs::PoseStamped temp_goal,
                   std::vector<geometry_msgs::PoseStamped>& result_path)
{

  setMapfromTopic(temp_map);
  setStartfromParam(temp_start);
  setGoalfromParam(temp_goal);
  plan(result_path);
}

//###################################################
//                                      PLAN THE PATH
//###################################################
void Planner::plan(std::vector<geometry_msgs::PoseStamped>& result_path_temp) {
  // if a start as well as goal are defined go ahead and plan
  if (!(validStart && validGoal)) {
    return;
  }
    // ___________________________
    // LISTS ALLOWCATED ROW MAJOR ORDER
    int width = grid->info.width;
    int height = grid->info.height;
    int depth = Constants::headings;
    int length = width * height * depth;
    // define list pointers and initialize lists
    Pose2D* nodes3D = new Pose2D[length]();
    Node2D* nodes2D = new Node2D[width * height]();

    // ________________________
    // retrieving goal position
    float x = (goal.pose.position.x - grid->info.origin.position.x) / grid->info.resolution;
    float y = (goal.pose.position.y - grid->info.origin.position.y) / grid->info.resolution;
    float t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);

    const Pose2D nGoal(x, y, t, 0, 0, nullptr);

    // _________________________
    // retrieving start position
    x = (start.pose.pose.position.x - grid->info.origin.position.x) / grid->info.resolution;
    y = (start.pose.pose.position.y - grid->info.origin.position.y) / grid->info.resolution;
    t = tf::getYaw(start.pose.pose.orientation);

    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);

    Pose2D nStart(x, y, t, 0, 0, nullptr);

    // CLEAR THE PATH
    path.path.poses.clear();
    smoothedPath.path.poses.clear();

    // FIND THE PATH
    Pose2D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace);

    // TRACE THE PATH
    std::vector<Pose2D> result_path;
    result_path.clear();
    tracePath(nSolution, 0, result_path);


    // CREATE THE UPDATED PATH
    // smoothedPath.updatePath(result_path);

    smoothedPath.path.header.stamp = ros::Time::now();
   
    for (unsigned int i = 0; i < result_path.size(); ++i) {
      geometry_msgs::PoseStamped vertex;
      vertex.pose.position.x = result_path[i].getX() * Constants::cellSize;
      vertex.pose.position.y = result_path[i].getY() * Constants::cellSize;
      vertex.pose.position.z = 0;

      vertex.pose.orientation = tf::createQuaternionMsgFromYaw(result_path[i].getT());
      smoothedPath.path.poses.push_back(vertex);
    } 



    //YT transform back, if nopath then size() = 0
    for(unsigned int i = 0;i< smoothedPath.path.poses.size();i++)
    {
        smoothedPath.setPath(i, smoothedPath.path.poses.at(i).pose.position.x*grid->info.resolution+ grid->info.origin.position.x,
                             smoothedPath.path.poses.at(i).pose.position.y*grid->info.resolution + grid->info.origin.position.y);
    }

    delete [] nodes3D;
    delete [] nodes2D;

}

void Planner::tracePath(const Pose2D* node, int i, std::vector<Pose2D>& path)
{
    if (node == nullptr) {
      std::cout << "YT: maybe no path to trace" << std::endl;
    return;
  }
  i++;
  path.push_back(*node);
  tracePath(node->getPred(), i, path);
}