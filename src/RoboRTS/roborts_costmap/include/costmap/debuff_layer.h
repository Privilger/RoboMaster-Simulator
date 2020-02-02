#ifndef ROBORTS_COSTMAP_DEBUFF_LAYER_H
#define ROBORTS_COSTMAP_DEBUFF_LAYER_H

#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <vector>
#include "io/io.h"
#include "map_common.h"
#include "costmap_layer.h"

namespace roborts_costmap {

class DebuffLayer : public CostmapLayer {

 public:
  DebuffLayer() {}
  virtual ~DebuffLayer() {}
  virtual void OnInitialize();
  virtual void Activate();
  virtual void Deactivate();
  virtual void Reset();
  virtual void UpdateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void UpdateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);

 private:
  void receiveDebuffSignal(const std_msgs::String& new_debuff);
  std::string global_frame_;
  std::string map_frame_;
  std::string map_topic_;
  unsigned int debuff_layer_x_, debuff_layer_y_, width_, height_;
  ros::Subscriber map_sub_;
  std::vector<int> debuff_;
  struct DebuffZone{
      float x;    // debuff zone 左下角 x
      float y;    // debuff zone 左下角 y
  }debuff_zones[7];
    float debuff_width_ = 0.54;
    float debuff_height_ = 0.48;
  float resolution_;
  float debuff_inflation_;
  int offset_x, offset_y;
    std::string ns_;
};


} // namespace roborts_costmap
#endif
