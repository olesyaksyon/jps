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

void prune_neibours(node* node){

}

void algo::init(geometry_msgs::PoseWithCovarianceStamped start_, geometry_msgs::PoseStamped goal_, nav_msgs::OccupancyGrid::Ptr grid){
    ROS_ERROR("init..");
    float t_st = tf::getYaw(start_.pose.pose.orientation);
    float t_g = tf::getYaw(goal_.pose.orientation);
    start.reset(new node(start_.pose.pose.position.x, start_.pose.pose.position.y, t_st, NULL, 0));
    goal.reset(new node(goal_.pose.position.x, goal_.pose.position.y, t_g, NULL, 0));
    this->grid = grid;
}



void algo::jps(){
    if (!start || !goal || !grid) {
        return;
    }



    }


void algo::clear(){
    path.clear();
    start.release();
    goal.release();
    grid = NULL;
}