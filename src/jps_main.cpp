//
// Created by olesya on 07.01.19.
//

#include <config.h>
#include "jps_main.h"


jps_main::jps_main(){
    subMap = n.subscribe("/map", 1, &jps_main::set_map, this);
    subGoal = n.subscribe("/move_base_simple/goal", 1, &jps_main::set_goal, this);
    subStart = n.subscribe("/initialpose", 1, &jps_main::set_start, this);

    pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);
    pubPath = n.advertise<nav_msgs::Path>("/path", 1);
    pubPathNodes = n.advertise<visualization_msgs::MarkerArray>("/pathNodes", 1);
    pubPathVehicles = n.advertise<visualization_msgs::MarkerArray>("/pathVehicle", 1);

    path.header.frame_id = "path";

    start = NULL;
    goal = NULL;

}

void jps_main::addVehicle(shared_ptr_node node, int i) {
    visualization_msgs::Marker pathVehicle;

    if (i == 1) {
        pathVehicle.action = 3;
    }

    pathVehicle.header.frame_id = "path";
    pathVehicle.header.stamp = ros::Time(0);
    pathVehicle.id = i;
    pathVehicle.type = visualization_msgs::Marker::CUBE;
    pathVehicle.scale.x = config::length;
    pathVehicle.scale.y = config::width;
    pathVehicle.scale.z = 1;
    pathVehicle.color.a = 0.1;


    pathVehicle.color.r = 255;
    pathVehicle.color.g = 0;
    pathVehicle.color.b = 0;


    pathVehicle.pose.position.x = node->get_x();
    pathVehicle.pose.position.y = node->get_y();
    //pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
    pathVehicles.markers.push_back(pathVehicle);
}

void jps_main::addSegment(shared_ptr_node node) {
    geometry_msgs::PoseStamped vertex;
    vertex.pose.position.x = node->get_x();
    vertex.pose.position.y = node->get_y();
    vertex.pose.position.z = 0;
    vertex.pose.orientation.x = 0;
    vertex.pose.orientation.y = 0;
    vertex.pose.orientation.z = 0;
    vertex.pose.orientation.w = 0;
    path.poses.push_back(vertex);
}


void jps_main::process() {
    algo_.clear();
    path.poses.clear();

    if (start && goal)
        algo_.init(start, goal, grid);
    std::shared_ptr<node> last = algo_.jps();

    while (last != nullptr) {
        addSegment(last);
        last = last->get_prev();
    }

    pubPath.publish(path);
    //pubPathVehicles.publish(pathVehicles);
}

void jps_main::set_map(const nav_msgs::OccupancyGrid::Ptr map){

    //TODO: проверять не старая ли карта
    grid = map;
    process();
}


void jps_main::set_start(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start) {
    this->start = start;

    geometry_msgs::PoseStamped start_;
    start_.pose.position = start->pose.pose.position;
    start_.pose.orientation = start->pose.pose.orientation;
    start_.header.frame_id = "map";
    start_.header.stamp = ros::Time::now();

    pubStart.publish(start_);

    process();
}



void jps_main::set_goal(const geometry_msgs::PoseStamped::ConstPtr& goal) {
    ROS_INFO("goal taken");
    this->goal = goal;

    process();
}