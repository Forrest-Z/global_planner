#include "collisiondetection.h"
#include <costmap_2d/costmap_2d.h>
#include "costmap_model.h"


double CollisionDetection::footprintCost(double x_i, double y_i, double theta_i)
{
  //YT 先临时定义两个变量
  double inscribed_radius_ = 0.1;
  double circumscribed_radius_ = 0.5;
  return world_model_->footprintCost(x_i, y_i, theta_i, footprint_spec_, inscribed_radius_, circumscribed_radius_);
}