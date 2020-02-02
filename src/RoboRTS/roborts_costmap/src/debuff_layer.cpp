#include "debuff_layer_setting.pb.h"
#include "debuff_layer.h"

namespace roborts_costmap {

void DebuffLayer::OnInitialize() {
    ros::NodeHandle nh;
    ParaDebuffLayer para_debuff_layer;

    std::string debuff_map = ros::package::getPath("roborts_costmap") + \
      "/config/debuff_layer_config.prototxt";
    roborts_common::ReadProtoFromTextFile(debuff_map.c_str(), &para_debuff_layer);
    global_frame_ = layered_costmap_-> GetGlobalFrameID();
    map_topic_ = para_debuff_layer.topic_name();
    map_sub_ = nh.subscribe(map_topic_.c_str(), 1, &DebuffLayer::receiveDebuffSignal, this);

    debuff_inflation_ = para_debuff_layer.debuff_inflation();
    Costmap2D* master = layered_costmap_->GetCostMap();
    resolution_ = master->GetResolution();
    offset_x = -master->GetOriginX() * 50;   // * 50 == / 0.02
    offset_y = -master->GetOriginY() * 50;

//    ROS_ERROR("offset_x: %d", offset_x);
//    ROS_ERROR("offset_y: %d", offset_y);

    debuff_zones[1].x = 0.23,   debuff_zones[1].y = 3.12;
    debuff_zones[2].x = 1.63,   debuff_zones[2].y = 1.695;
    debuff_zones[3].x = 3.73,   debuff_zones[3].y = 4.35;
    debuff_zones[4].x = 7.33,   debuff_zones[4].y = 1.5;
    debuff_zones[5].x = 5.93,   debuff_zones[5].y = 2.925;
    debuff_zones[6].x = 3.83,   debuff_zones[6].y = 0.27;

/*
 * 新地图
    debuff_zones[1].x = 0.23,   debuff_zones[1].y = 2.55;
    debuff_zones[2].x = 1.63,   debuff_zones[2].y = 1.41;
    debuff_zones[3].x = 3.77,   debuff_zones[3].y = 4.275;
    debuff_zones[4].x = 7.31,   debuff_zones[4].y = 1.45;
    debuff_zones[5].x = 5.91,   debuff_zones[5].y = 2.59;
    debuff_zones[6].x = 3.77,   debuff_zones[6].y = 0.205;
*/
}

void DebuffLayer::receiveDebuffSignal(const std_msgs::String& new_buff){
    /*
     * new_buff 内容： 指明那几个debuff zone被激活, 中间用空格隔开
     * Eg. 被激活的是 1 2 4 6 区, 则new_buff的内容是：
     * index: 0 1 2 3 4 5 6
     *       "0 1 1 0 1 0 1"
    */
    debuff_.clear();
    for(int i=1; i<=6; ++i){
        if(new_buff.data[2*i] == '1'){
            debuff_.push_back(i);
        }
    }
}

void DebuffLayer::Activate() {
  OnInitialize();
}

void DebuffLayer::Deactivate() {
    // delete cost_map_;
    // shut down the map topic message subscriber
  map_sub_.shutdown();
}

void DebuffLayer::Reset() {
    OnInitialize();
}

void DebuffLayer::UpdateBounds(double robot_x,
                               double robot_y,
                               double robot_yaw,
                               double *min_x,
                               double *min_y,
                               double *max_x,
                               double *max_y) {
    return;
}

void DebuffLayer::UpdateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {

    for (auto k : debuff_) {
        int start_x = (int)( (debuff_zones[k].x + debuff_inflation_) / resolution_ ) + offset_x;
        int end_x   = (int)( (debuff_zones[k].x + debuff_width_  - debuff_inflation_ ) / resolution_ ) + offset_x;
        int start_y = (int)( (debuff_zones[k].y + debuff_inflation_) / resolution_ ) + offset_y;
        int end_y   = (int)( (debuff_zones[k].y + debuff_height_ - debuff_inflation_ ) / resolution_ ) + offset_y;

        for (auto i = start_x; i < end_x; ++i) {
            for (auto j = start_y; j < end_y; ++j) {
                master_grid.SetCost(i, j, LETHAL_OBSTACLE);
            }
        }
    }
}

} //namespace roborts_costmap

