/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#include "costmap_layer.h"

namespace roborts_costmap {

void CostmapLayer::Touch(double x, double y, double *min_x, double *min_y, double *max_x, double *max_y) {
  *min_x = std::min(x, *min_x);
  *min_y = std::min(y, *min_y);
  *max_x = std::max(x, *max_x);
  *max_y = std::max(y, *max_y);
}

void CostmapLayer::MatchSize() {
  Costmap2D *master = layered_costmap_->GetCostMap();
  ResizeMap(master->GetSizeXCell(), master->GetSizeYCell(), master->GetResolution(),
            master->GetOriginX(), master->GetOriginY());
}

void CostmapLayer::AddExtraBounds(double mx0, double my0, double mx1, double my1) {
  extra_min_x_ = std::min(mx0, extra_min_x_);
  extra_max_x_ = std::max(mx1, extra_max_x_);
  extra_min_y_ = std::min(my0, extra_min_y_);
  extra_max_y_ = std::max(my1, extra_max_y_);
  has_extra_bounds_ = true;
}

void CostmapLayer::UseExtraBounds(double *min_x, double *min_y, double *max_x, double *max_y) {
  if (!has_extra_bounds_)
    return;

  *min_x = std::min(extra_min_x_, *min_x);
  *min_y = std::min(extra_min_y_, *min_y);
  *max_x = std::max(extra_max_x_, *max_x);
  *max_y = std::max(extra_max_y_, *max_y);
  extra_min_x_ = 1e6;
  extra_min_y_ = 1e6;
  extra_max_x_ = -1e6;
  extra_max_y_ = -1e6;
  has_extra_bounds_ = false;
}

void CostmapLayer::UpdateOverwriteByMax(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) {
  if (!is_enabled_)
    return;

  unsigned char *master_array = master_grid.GetCharMap();
  unsigned int span = master_grid.GetSizeXCell();

  for (int j = min_j; j < max_j; j++) {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++) {
      if (costmap_[it] == NO_INFORMATION) {
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];
      if (old_cost == NO_INFORMATION || old_cost < costmap_[it])
        master_array[it] = costmap_[it];
      it++;
    }
  }
}

void CostmapLayer::UpdateOverwriteByAll(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) {
  if (!is_current_)
    return;
  unsigned char *master = master_grid.GetCharMap();
  unsigned int span = master_grid.GetSizeXCell();

  for (int j = min_j; j < max_j; j++) {
    unsigned int it = span * j + min_i;
    for (int i = min_i; i < max_i; i++) {
      master[it] = costmap_[it];
      it++;
    }
  }
}

void CostmapLayer::UpdateOverwriteByValid(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) {
  if (!is_enabled_)
    return;
  unsigned char *master = master_grid.GetCharMap();
  unsigned int span = master_grid.GetSizeXCell();

  for (int j = min_j; j < max_j; j++) {
    unsigned int it = span * j + min_i;
    for (int i = min_i; i < max_i; i++) {
      if (costmap_[it] != NO_INFORMATION)
        master[it] = costmap_[it];
      it++;
    }
  }
}

void CostmapLayer::UpdateOverwriteByAdd(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) {
  if (!is_enabled_)
    return;
  unsigned char *master_array = master_grid.GetCharMap();
  unsigned int span = master_grid.GetSizeXCell();

  for (int j = min_j; j < max_j; j++) {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++) {
      if (costmap_[it] == NO_INFORMATION) {
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];
      if (old_cost == NO_INFORMATION)
        master_array[it] = costmap_[it];
      else {
        int sum = old_cost + costmap_[it];
        if (sum >= INSCRIBED_INFLATED_OBSTACLE)
          master_array[it] = INSCRIBED_INFLATED_OBSTACLE - 1;
        else
          master_array[it] = sum;
      }
      it++;
    }
  }
}

} //namespace roborts_costmap