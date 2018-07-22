#include "collisiondetection.h"
#include <costmap_2d/costmap_2d.h>
#include "costmap_model.h"


// bool CollisionDetection::configurationTest(float x, float y, float t) {
//   int X = (int)x;
//   int Y = (int)y;
//   int iX = (int)((x - (long)x) * global_planner::Constants::positionResolution);
//   iX = iX > 0 ? iX : 0;
//   int iY = (int)((y - (long)y) * global_planner::Constants::positionResolution);
//   iY = iY > 0 ? iY : 0;
//   int iT = (int)(t / global_planner::Constants::deltaHeadingRad);
//   int idx = iY * global_planner::Constants::positionResolution * global_planner::Constants::headings + iX * global_planner::Constants::headings + iT;
//   int cX;
//   int cY;

//   for (int i = 0; i < collisionLookup[idx].length; ++i) {
//     cX = (X + collisionLookup[idx].pos[i].x);
//     cY = (Y + collisionLookup[idx].pos[i].y);

//     // make sure the configuration coordinates are actually on the grid
//     //YT grid中对应的网格必须都是false,只要有一个true就说明这个configuration会碰撞障碍物
//     if (cX >= 0 && (unsigned int)cX < costmap_->getSizeInCellsX() && cY >= 0 && (unsigned int)cY < costmap_->getSizeInCellsY()) {
//       if (grid->data[cY * costmap_->getSizeInCellsX() + cX]) {
//         return false;
//       }
//     }
//   }

//   return true;
// }

double CollisionDetection::footprintCost(double x_i, double y_i, double theta_i)
{
  //YT 先临时定义两个变量
  double inscribed_radius_ = 0.1;
  double circumscribed_radius_ = 0.5;
  return world_model_->footprintCost(x_i, y_i, theta_i, footprint_spec_, inscribed_radius_, circumscribed_radius_);
}