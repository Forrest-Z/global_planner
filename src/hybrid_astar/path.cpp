#include "global_planner/hybrid_astar/path.h"

using namespace HybridAStar;


//###################################################
//                                         CLEAR PATH
//###################################################

void Path::clear() {
  Node3D node;
  path.poses.clear();
  pathNodes.markers.clear();
  pathVehicles.markers.clear();
  addNode(node, 0);
  addVehicle(node, 1);
  publishPath();
  publishPathNodes();
  publishPathVehicles();
}

////###################################################
////                                         TRACE PATH
////###################################################
//// __________
//// TRACE PATH
//void Path::tracePath(const Node3D* node, int i) {
//  if (i == 0) {
//    path.header.stamp = ros::Time::now();
//  }

//  if (node == nullptr) { return; }

//  addSegment(node);
//  addNode(node, i);
//  i++;
//  addVehicle(node, i);
//  i++;

//  tracePath(node->getPred(), i);
//}

//###################################################
//                                         TRACE PATH
//###################################################
// __________
// TRACE PATH
void Path::updatePath(std::vector<Node3D> nodePath) {
  path.header.stamp = ros::Time::now();
  int k = 0;

  for (unsigned int i = 0; i < nodePath.size(); ++i) {
    addSegment(nodePath[i]);
    addNode(nodePath[i], k);
    k++;
    addVehicle(nodePath[i], k);
    k++;
  }

  return;
}
// ___________
// ADD SEGMENT
void Path::addSegment(const Node3D& node) {
  geometry_msgs::PoseStamped vertex;
  vertex.pose.position.x = node.getX() * Constants::cellSize;
  vertex.pose.position.y = node.getY() * Constants::cellSize;
  vertex.pose.position.z = 0;

//  vertex.pose.orientation.x = 0;
//  vertex.pose.orientation.y = 0;
//  vertex.pose.orientation.z = 0;
//  vertex.pose.orientation.w = 0;

//  tf::Quaternion q;
//  q.setEuler(node.getT(), 0, 0);
//  vertex.pose.orientation.x = q.x();
//  vertex.pose.orientation.y = q.y();
//  vertex.pose.orientation.z = q.z();
//  vertex.pose.orientation.w = q.w();
vertex.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
  path.poses.push_back(vertex);
}

// ________
// ADD NODE
void Path::addNode(const Node3D& node, int i) {
  visualization_msgs::Marker pathNode;

  // delete all previous markers
  if (i == 0) {
    pathNode.action = 3;
  }

  pathNode.header.frame_id = "path";
  pathNode.header.stamp = ros::Time(0);
  pathNode.id = i;
  pathNode.type = visualization_msgs::Marker::SPHERE;
  pathNode.scale.x = 0.1;
  pathNode.scale.y = 0.1;
  pathNode.scale.z = 0.1;
  pathNode.color.a = 1.0;

  if (smoothed) {
    pathNode.color.r = Constants::pink.red;
    pathNode.color.g = Constants::pink.green;
    pathNode.color.b = Constants::pink.blue;
  } else {
    pathNode.color.r = Constants::purple.red;
    pathNode.color.g = Constants::purple.green;
    pathNode.color.b = Constants::purple.blue;
  }

  pathNode.pose.position.x = node.getX() * Constants::cellSize;
  pathNode.pose.position.y = node.getY() * Constants::cellSize;
  pathNodes.markers.push_back(pathNode);
}

void Path::addVehicle(const Node3D& node, int i) {
  visualization_msgs::Marker pathVehicle;

  // delete all previous markersg
  if (i == 1) {
    pathVehicle.action = 3;
  }

  pathVehicle.header.frame_id = "path";
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.id = i;
  pathVehicle.type = visualization_msgs::Marker::CUBE;
  pathVehicle.scale.x = Constants::length - Constants::bloating * 2;
  pathVehicle.scale.y = Constants::width - Constants::bloating * 2;
  pathVehicle.scale.z = 1;
  pathVehicle.color.a = 0.1;

  if (smoothed) {
    pathVehicle.color.r = Constants::orange.red;
    pathVehicle.color.g = Constants::orange.green;
    pathVehicle.color.b = Constants::orange.blue;
  } else {
    pathVehicle.color.r = Constants::teal.red;
    pathVehicle.color.g = Constants::teal.green;
    pathVehicle.color.b = Constants::teal.blue;
  }

  pathVehicle.pose.position.x = node.getX() * Constants::cellSize;
  pathVehicle.pose.position.y = node.getY() * Constants::cellSize;
  pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
  pathVehicles.markers.push_back(pathVehicle);
}

void Path::PrintPath()
{
    std::cout << "YT: Get the size of resultpath: " << this->path.poses.size() << std::endl;
    for(int i = this->path.poses.size() - 1;i >= 0;i--)
    {
        std::cout << "YT: resultpath["
                  << i
                  << "] = "
                  << this->path.poses.at(i).pose.position.x
                  << ", "
                  << this->path.poses.at(i).pose.position.y
                  << ", ["
                  << this->path.poses.at(i).pose.orientation.x
                  << ", "
                  << this->path.poses.at(i).pose.orientation.y
                  << ", "
                  << this->path.poses.at(i).pose.orientation.z
                  << ", "
                  << this->path.poses.at(i).pose.orientation.w
                  << "] "
                  << tf::getYaw(this->path.poses.at(i).pose.orientation)
                  << std::endl;
    }
}


void Path::setPath(int index, float x, float y)
{
    this->path.poses.at(index).pose.position.x = x;
    this->path.poses.at(index).pose.position.y = y;
}
