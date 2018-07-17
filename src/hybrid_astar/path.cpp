#include "global_planner/hybrid_astar/path.h"

using namespace HybridAStar;

//###################################################
//                                         TRACE PATH
//###################################################
// __________
// TRACE PATH
void Path::updatePath(std::vector<Pose2D> nodePath) {
  path.header.stamp = ros::Time::now();

  for (unsigned int i = 0; i < nodePath.size(); ++i) {
    geometry_msgs::PoseStamped vertex;
    vertex.pose.position.x = nodePath[i].getX() * Constants::cellSize;
    vertex.pose.position.y = nodePath[i].getY() * Constants::cellSize;
    vertex.pose.position.z = 0;

    vertex.pose.orientation = tf::createQuaternionMsgFromYaw(nodePath[i].getT());
    path.poses.push_back(vertex);
  }
}




// void Path::PrintPath()
// {
//     std::cout << "YT: Get the size of resultpath: " << this->path.poses.size() << std::endl;
//     for(int i = this->path.poses.size() - 1;i >= 0;i--)
//     {
//         std::cout << "YT: resultpath["
//                   << i
//                   << "] = "
//                   << this->path.poses.at(i).pose.position.x
//                   << ", "
//                   << this->path.poses.at(i).pose.position.y
//                   << ", ["
//                   << this->path.poses.at(i).pose.orientation.x
//                   << ", "
//                   << this->path.poses.at(i).pose.orientation.y
//                   << ", "
//                   << this->path.poses.at(i).pose.orientation.z
//                   << ", "
//                   << this->path.poses.at(i).pose.orientation.w
//                   << "] "
//                   << tf::getYaw(this->path.poses.at(i).pose.orientation)
//                   << std::endl;
//     }
// }


void Path::setPath(int index, float x, float y)
{
    this->path.poses.at(index).pose.position.x = x;
    this->path.poses.at(index).pose.position.y = y;
}
