//
// Created by olesya on 07.01.19.
//

#include <tf/transform_datatypes.h>
#include "algo.h"
#include <ros/console.h>


algo::algo(){
    grid = NULL;
}

float algo::distance_to_goal(std::shared_ptr<node> n1){
    auto goal_ = goal.lock();
    return sqrt(n1->x - goal_->x)
}

void algo::prune_neibours(std::shared_ptr<node> curr_){

}

void algo::init(geometry_msgs::PoseWithCovarianceStamped start_, geometry_msgs::PoseStamped goal_, nav_msgs::OccupancyGrid::Ptr grid){
    float t_st = tf::getYaw(start_.pose.pose.orientation);
    float t_g = tf::getYaw(goal_.pose.orientation);
    start = std::make_shared<node>(start_.pose.pose.position.x, start_.pose.pose.position.y, t_st, -1, 1);
    goal = std::make_shared<node>(goal_.pose.position.x, goal_.pose.position.y, t_g, -1, 1);
    this->grid = grid;
}

void algo::jump(std::shared_ptr<node>& curr_n){
}



void algo::jps(){
    if (start.expired() || goal.expired() || !grid) {
        return;
    }

    auto curr_ = start.lock();

    curr_->conctruct_neighbours();

    //choose_direction();
    //вычисляем доминантного нейбора
    //прыгаем
    //пока не будем на расстоянии р до цели

    //выбираем по эвристике направление

    //jump




    }


void algo::clear(){
    path.clear();
    start.reset();
    goal.reset();
    grid = NULL;
}