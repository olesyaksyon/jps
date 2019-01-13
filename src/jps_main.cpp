//
// Created by olesya on 07.01.19.
//

#include "jps_main.h"


jps_main::jps_main(){
    subMap = n.subscribe("/map", 1, &jps_main::set_map, this);
    subGoal = n.subscribe("/move_base_simple/goal", 1, &jps_main::set_goal, this);
    subStart = n.subscribe("/initialpose", 1, &jps_main::set_start, this);
}



void jps_main::process() {
    algo_.clear();
    algo_.init(start, goal, grid);
    std::shared_ptr<node> last = algo_.jps();

    if (last != nullptr) {

    }
}

void jps_main::set_map(const nav_msgs::OccupancyGrid::Ptr map){

    ///проверять не старая ли карта
    grid = map;
    process();
}

//TODO: проверка на валидную цель
void jps_main::set_start(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start) {
    this->start = *start;
    process();
}


//TODO: проверка на валидную цель
void jps_main::set_goal(const geometry_msgs::PoseStamped::ConstPtr& goal) {
    ROS_INFO("goal taken");
    this->goal = *goal;
}