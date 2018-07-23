#include "planner.h"
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "algorithm/HAStar.h"
#include "algorithm/AStar.h"
#include "tf/transform_datatypes.h"

using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################

global_planner::Planner::Planner(costmap_2d::Costmap2D* costmap, 
                                 std::vector<geometry_msgs::Point> footprint_spec, 
                                 unsigned int cell_divider, 
                                 bool using_voronoi, 
                                 bool lazy_replanning_):
  costmap_(costmap),footprint_spec_(footprint_spec)
{


using_voronoi_ = using_voronoi;
  cell_divider_ = cell_divider;
  ROS_WARN("YT: start creating planner");
  path_.header.frame_id = "path";
  setupCollisionDetection(costmap);

//////////////////////////YT 建立binMap/////////////////////////////////
  //YT 初始化地图数据
  costmap_width_x_ = costmap->getSizeInCellsX();//YT x是列像素数
  costmap_height_y_ = costmap->getSizeInCellsY();//YT y是行像素数
  costmap_resolution_ = costmap->getResolution();
  origin_position_x_ = costmap->getOriginX();
  origin_position_y_ = costmap->getOriginY();
  origin_position_z_ = 0;
  origin_orientation_x_ = 0;
  origin_orientation_y_ = 0;
  origin_orientation_z_ = 0;
  origin_orientation_w_ = 1;

  //YT 建立gridmap数据
  gridmap_width_x_ = costmap_width_x_ * cell_divider_;
  gridmap_height_y_ = costmap_height_y_ * cell_divider_;
  gridmap_resolution_ = costmap_resolution_ / cell_divider_;//YT 单位：m

  //YT 建立binmap，但其实没什么用，后续删掉
  unsigned int height = gridmap_height_y_;
  unsigned int width = gridmap_width_x_;

  binMap_ = new bool*[width];

  for (unsigned int x = 0; x < width; x++) 
  { binMap_[x] = new bool[height]; }
 
  
}

//###################################################
//                                                MAP
//###################################################
void global_planner::Planner::setupCollisionDetection(costmap_2d::Costmap2D* costmap) {

    //YT width是x，height是y

}





//###################################################
//                                   INITIALIZE START
//###################################################
void global_planner::Planner::setStartfromParam(const geometry_msgs::PoseStamped initial) {
    //YT must convert x,y to the coordinate of binMap before start the algorithm
  float x = (initial.pose.position.x - costmap_->getOriginX()) / costmap_->getResolution();
  float y = (initial.pose.position.y - costmap_->getOriginY()) / costmap_->getResolution();
  float t = tf::getYaw(initial.pose.orientation);

  if ( costmap_->getSizeInCellsY() >= y && y >= 0 && costmap_->getSizeInCellsX() >= x && x >= 0 ) {
    start.pose.position.x = initial.pose.position.x;
    start.pose.position.y = initial.pose.position.y;
    start.pose.position.z = initial.pose.position.z;
    start.pose.orientation.w = initial.pose.orientation.w;
    start.pose.orientation.x = initial.pose.orientation.x;
    start.pose.orientation.y = initial.pose.orientation.y;
    start.pose.orientation.z = initial.pose.orientation.z;
  }
  else {
    std::cout << "YT: start is off grid: x = " << x << ", y = " << y << ", t = " << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void global_planner::Planner::setGoalfromParam(const geometry_msgs::PoseStamped end) {
  // retrieving goal position
  float x = (end.pose.position.x - costmap_->getOriginX())  / costmap_->getResolution();
  float y = (end.pose.position.y - costmap_->getOriginY())  / costmap_->getResolution();
  float t = tf::getYaw(end.pose.orientation);

  if ( costmap_->getSizeInCellsY() >= y && y >= 0 && costmap_->getSizeInCellsX() >= x && x >= 0 ) {
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

void global_planner::Planner::plan(const geometry_msgs::PoseStamped temp_start, 
                   const geometry_msgs::PoseStamped temp_goal,
                   std::vector<geometry_msgs::PoseStamped>& result_path)
{
ROS_ERROR("YT: make plan by planner");
  setStartfromParam(temp_start);
  setGoalfromParam(temp_goal);
  plan(result_path);
}


//###################################################
//                                      PLAN THE PATH
//###################################################
void global_planner::Planner::plan(std::vector<geometry_msgs::PoseStamped>& plan) {




  // if(using_voronoi_)
  // {
  //   voronoiDiagram = new DynamicVoronoi();
  //   voronoiDiagram->initializeMap(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(), binMap_);

  //   voronoiDiagram->update();
  //   ROS_WARN("YT: start saving voronoi graph");
  //   voronoiDiagram->visualize();
  //   ROS_WARN("YT: voronoi_graph has been saved");
  // }


  //YT 更新
 for (unsigned int x = 0; x < gridmap_width_x_; ++x) {
    for (unsigned int y = 0; y < gridmap_height_y_; ++y) {
       binMap_[x][y] = (*(costmap_->getCharMap() + (unsigned int)(y/cell_divider_) * costmap_->getSizeInCellsX() + (unsigned int)(x/cell_divider_)) == 0) ? true : false;
    }
  }


  visualizeBinMap("/home/yangtong/binMap.ppm");


  configurationSpace = new CollisionDetection(costmap_, cell_divider_, footprint_spec_, origin_position_x_, origin_position_y_, gridmap_resolution_);



    // ___________________________
    // LISTS ALLOWCATED ROW MAJOR ORDER
    int width = gridmap_width_x_;
    int height = gridmap_height_y_;

    int depth = Constants::headings;
    
    int length = width * height * depth;
    // define list pointers and initialize lists
    Pose2D* nodes3D = new Pose2D[length]();
    Node2D* nodes2D = new Node2D[width * height]();

    // ________________________
    // retrieving goal position
    float x = (goal.pose.position.x - origin_position_x_) / gridmap_resolution_;
    float y = (goal.pose.position.y - origin_position_y_) / gridmap_resolution_;
    float t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);

    const Pose2D nGoal(x, y, t, 0, 0, nullptr);

    // _________________________
    // retrieving start position
    x = (start.pose.position.x - origin_position_x_) / gridmap_resolution_;
    y = (start.pose.position.y - origin_position_y_) / gridmap_resolution_;
    t = tf::getYaw(start.pose.orientation);

    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);

    Pose2D nStart(x, y, t, 0, 0, nullptr);
 
    // ROS_ERROR("YT: planner.cpp line 206");

    // CLEAR THE PATH
    path_.poses.clear();

    yt_alg_ = new Algorithm::AStar();
    std::vector<Pose2D> result_path;
    result_path.clear();

    //YT nStart和nGoal都是以gridmap原点为原点、gridmap分辨率为单位1的坐标点，生成的结果也是基于gridmap的
    //YT 只将CollisionDetection的指针传进去，实际上CollisionDetection是建立在global_planner下的，而不是在algorithm类中
    yt_alg_->plan(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace, result_path);
    //YT 保存中间结果

    mid_result.poses.resize(yt_alg_->mid_result.size());

    for(unsigned int i = 0;i< yt_alg_->mid_result.size();i++)
    {
        mid_result.poses.at(i).position.x = yt_alg_->mid_result.at(i).getX() * gridmap_resolution_ + origin_position_x_;
        mid_result.poses.at(i).position.y = yt_alg_->mid_result.at(i).getY() * gridmap_resolution_ + origin_position_y_;
        tf::Quaternion q = tf::createQuaternionFromYaw(yt_alg_->mid_result.at(i).getT());
        tf::quaternionTFToMsg(q, mid_result.poses.at(i).orientation);
        // mid_result.poses.at(i).orientation.w = 1;
    }






    ROS_ERROR("YT: yt_alg_ has finished, planner.cpp line 195");
    if(result_path.size() == 0)
    {
      std::cout << "YT: no result_path" << std::endl;
      
      return;
    }

    // CREATE THE UPDATED PATH

    path_.header.stamp = ros::Time::now();
   
    for (unsigned int i = 0; i < result_path.size(); ++i) {
      geometry_msgs::PoseStamped vertex;
      vertex.pose.position.x = result_path[i].getX();
      vertex.pose.position.y = result_path[i].getY();
      vertex.pose.position.z = 0;
      vertex.pose.orientation = tf::createQuaternionMsgFromYaw(result_path[i].getT());
      path_.poses.push_back(vertex);
    } 

    //YT 恢复回世界坐标
    for(unsigned int i = 0;i< path_.poses.size();i++)
    {
        path_.poses.at(i).pose.position.x = path_.poses.at(i).pose.position.x * gridmap_resolution_ + origin_position_x_;
        path_.poses.at(i).pose.position.y = path_.poses.at(i).pose.position.y * gridmap_resolution_ + origin_position_y_;
    }

///////////////////////////////////////////////////////////////////////////


       //YT 将路径反转，再添加起终点
    if(path_.poses.size() != 0)
    {
      plan.resize(path_.poses.size() + 2);

      plan.at(0).pose.position.x = start.pose.position.x;
      plan.at(0).pose.position.y = start.pose.position.y;
      plan.at(0).pose.position.z = start.pose.position.z;
      plan.at(0).pose.orientation.x = start.pose.orientation.x;
      plan.at(0).pose.orientation.y = start.pose.orientation.y;
      plan.at(0).pose.orientation.z = start.pose.orientation.z;
      plan.at(0).pose.orientation.w = start.pose.orientation.w;

      plan.at(path_.poses.size()+1).pose.position.x = goal.pose.position.x;
      plan.at(path_.poses.size()+1).pose.position.y = goal.pose.position.y;
      plan.at(path_.poses.size()+1).pose.position.z = goal.pose.position.z;
      plan.at(path_.poses.size()+1).pose.orientation.w = goal.pose.orientation.w;
      plan.at(path_.poses.size()+1).pose.orientation.x = goal.pose.orientation.x;
      plan.at(path_.poses.size()+1).pose.orientation.y = goal.pose.orientation.y;
      plan.at(path_.poses.size()+1).pose.orientation.z = goal.pose.orientation.z;


      for(unsigned int i = 0;i <path_.poses.size();i++)
      {
        plan.at(path_.poses.size()-i).pose.position.x = path_.poses.at(i).pose.position.x;
        plan.at(path_.poses.size()-i).pose.position.y = path_.poses.at(i).pose.position.y;
        plan.at(path_.poses.size()-i).pose.position.z = path_.poses.at(i).pose.position.z;
        plan.at(path_.poses.size()-i).pose.orientation.x = path_.poses.at(i).pose.orientation.x;
        plan.at(path_.poses.size()-i).pose.orientation.y = path_.poses.at(i).pose.orientation.y;
        plan.at(path_.poses.size()-i).pose.orientation.z = path_.poses.at(i).pose.orientation.z;
        plan.at(path_.poses.size()-i).pose.orientation.w = path_.poses.at(i).pose.orientation.w;
      }
    }

//////////////////////////////////////////////////////////////////////////

    delete [] nodes3D;
    delete [] nodes2D;

}

void global_planner::Planner::tracePath(const Pose2D* node, int i, std::vector<Pose2D>& path)
{
    if (node == nullptr) {
      // std::cout << "YT: maybe no path to trace" << std::endl;
    return;
  }
  i++;
  path.push_back(*node);
  tracePath(node->getPred(), i, path);
}

void global_planner::Planner::visualizeBinMap(const char* filename)
{
  FILE* F = fopen(filename, "w");
  if (!F) {
    std::cerr << "could not open 'binMap.ppm' for writing!\n";
    return;
  }
  fprintf(F, "P6\n");
  fprintf(F, "%d %d 255\n", costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());//YT 列像素数，行像素数，中间有空格

  for(int y = costmap_->getSizeInCellsY()-1; y >=0; y--){      
    for(int x = 0; x < costmap_->getSizeInCellsX(); x++){	
      unsigned char c = 0;
      if (binMap_[x][y] == 0) {
        fputc( 255, F );
        fputc( 0, F );
        fputc( 0, F );
      } else {
        fputc( 0, F );
        fputc( 0, F );
        fputc( 0, F );
      }
    }
  }
  fclose(F);
}