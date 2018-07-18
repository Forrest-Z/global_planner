#include "collisiondetection.h"
#include <costmap_2d/costmap_2d.h>
#include "costmap_model.h"

using namespace HybridAStar;

CollisionDetection::CollisionDetection(costmap_2d::Costmap2D* costmap)
  {
    costmap_ = costmap;
    this->grid = nullptr;
    Lookup::collisionLookup(collisionLookup);

    world_model_ = new global_planner::CostmapModel(*costmap);

  }

bool CollisionDetection::configurationTest(float x, float y, float t) {
  int X = (int)x;
  int Y = (int)y;
  int iX = (int)((x - (long)x) * Constants::positionResolution);
  iX = iX > 0 ? iX : 0;
  int iY = (int)((y - (long)y) * Constants::positionResolution);
  iY = iY > 0 ? iY : 0;
  int iT = (int)(t / Constants::deltaHeadingRad);
  int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
  int cX;
  int cY;

  for (int i = 0; i < collisionLookup[idx].length; ++i) {
    cX = (X + collisionLookup[idx].pos[i].x);
    cY = (Y + collisionLookup[idx].pos[i].y);

    // make sure the configuration coordinates are actually on the grid
    //YT grid中对应的网格必须都是false,只要有一个true就说明这个configuration会碰撞障碍物
    if (cX >= 0 && (unsigned int)cX < costmap_->getSizeInCellsX() && cY >= 0 && (unsigned int)cY < costmap_->getSizeInCellsY()) {
      if (grid->data[cY * costmap_->getSizeInCellsX() + cX]) {
        return false;
      }
    }
  }

  return true;
}
