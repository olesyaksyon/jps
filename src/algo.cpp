//
// Created by olesya on 07.01.19.
//

#include <tf/transform_datatypes.h>
#include "algo.h"
#include <ros/console.h>


algo::algo(){
    start = NULL;
    goal = NULL;
    grid = NULL;
}

void algo::init(geometry_msgs::PoseStamped start_, geometry_msgs::PoseWithCovarianceStamped goal_, nav_msgs::OccupancyGrid::Ptr grid){
    float t_st = tf::getYaw(start_.pose.orientation);
    float t_g = tf::getYaw(goal_.pose.pose.orientation);
    start.reset(new node(start_.pose.position.x, start_.pose.position.y, t_st, NULL, 0));
    goal.reset(new node(goal_.pose.pose.position.x, goal_.pose.pose.position.y, t_g, NULL, 0));
    this->grid = grid;
}



void algo::jps(){
    if (!start || !goal || !grid) {
        ROS_ERROR("the class was not inited! check inited start, goal, grid");
        return;
    }



}

void algo::clear(){
    path.clear();
    start.release();
    goal.release();
    grid = NULL;
}