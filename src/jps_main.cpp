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
    algo_.jps();
}

void jps_main::set_map(const nav_msgs::OccupancyGrid::Ptr map){
    ROS_INFO("map taken");
    grid = map;
}

//TODO: проверка на валидную цель
void jps_main::set_start(const geometry_msgs::PoseStamped::ConstPtr& start) {
    ROS_INFO("start taken");
    this->start = *start;
}

//TODO: проверка на валидную цель
void jps_main::set_goal(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& goal) {
    ROS_INFO("goal taken");
    this->goal = *goal;
}