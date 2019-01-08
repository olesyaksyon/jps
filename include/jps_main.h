//
// Created by olesya on 07.01.19.
//

#ifndef JPS_JPS_MAIN_H
#define JPS_JPS_MAIN_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <algo.h>

class jps_main {

    ///==============All needed ros vars==========================
    /// The node handle
    ros::NodeHandle n;
    /// A publisher publishing the start position for RViz
    ros::Publisher pubStart;
    /// A subscriber for receiving map updates
    ros::Subscriber subMap;
    /// A subscriber for receiving goal updates
    ros::Subscriber subGoal;
    /// A subscriber for receiving start updates
    ros::Subscriber subStart;
    /// A listener that awaits transforms
    tf::TransformListener listener;
    /// A transform for moving start positions
    tf::StampedTransform transform;
    /// A pointer to the grid the planner runs on
    nav_msgs::OccupancyGrid::Ptr grid;
    /// The start pose set through RViz
    geometry_msgs::PoseStamped start;
    /// The goal pose set through RViz
    geometry_msgs::PoseWithCovarianceStamped goal;


    ros::Publisher pubPath;
    /// Publisher for the nodes on the path
    ros::Publisher pubPathNodes;
    /// Publisher for the vehicle along the path
    ros::Publisher pubPathVehicles;
    /// Path data structure for visualization
    nav_msgs::Path path;
    /// Nodes data structure for visualization
    visualization_msgs::MarkerArray pathNodes;
    /// Vehicle data structure for visualization
    visualization_msgs::MarkerArray pathVehicles;


    ///===========================================

    ///==========local vars========================
    algo algo_;
    //path handler

public:
    jps_main();
    void write_messages();
    void process();

    void set_map(const nav_msgs::OccupancyGrid::Ptr map);
    void set_start(const geometry_msgs::PoseStamped::ConstPtr& start);
    void set_goal(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& goal);





};


#endif //JPS_JPS_MAIN_H
