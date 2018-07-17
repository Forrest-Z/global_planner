#include "global_planner/hybrid_astar/collisiondetection.h"
#include <costmap_2d/costmap_2d.h>


using namespace HybridAStar;

CollisionDetection::CollisionDetection() {
  this->grid = nullptr;
  Lookup::collisionLookup(collisionLookup);
}
CollisionDetection::CollisionDetection(costmap_2d::Costmap2D* costmap)
  {
    costmap_ = costmap;
    CollisionDetection();
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
    if (cX >= 0 && (unsigned int)cX < costmap_->getSizeInCellsX() && cY >= 0 && (unsigned int)cY < costmap_->getSizeInCellsY()) {
      if (grid->data[cY * costmap_->getSizeInCellsX() + cX]) {
        return false;
      }
    }
  }

  return true;
}
