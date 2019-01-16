//
// Created by olesya on 07.01.19.
//

#include "jps_main.h"


jps_main::jps_main(){

    subMap = n.subscribe("/map", 1, &jps_main::set_map, this);
    subGoal = n.subscribe("/move_base_simple/goal", 1, &jps_main::set_goal, this);
    subStart = n.subscribe("/initialpose", 1, &jps_main::set_start, this);

    pubPath = n.advertise<nav_msgs::Path>("/path", 1);
    pubPathNodes = n.advertise<visualization_msgs::MarkerArray>("/pathNodes", 1);
    pubPathVehicles = n.advertise<visualization_msgs::MarkerArray>("/pathVehicle", 1);

    path.header.frame_id = "path";
}

void jps_main::get_path(shared_ptr_node last){
    auto curr_ = last;
    while (curr_!= NULL) {

        curr_ = curr_->get_prev();
    }
}

void jps_main::process() {
    algo_.clear();
    algo_.init(start, goal, grid);
    std::shared_ptr<node> last = algo_.jps();
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