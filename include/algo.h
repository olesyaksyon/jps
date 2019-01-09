//
// Created by olesya on 07.01.19.
//

#ifndef JPS_ALGO_H
#define JPS_ALGO_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <node.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>

class algo {
    std::unique_ptr<node> start;
    std::unique_ptr<node> goal;
    nav_msgs::OccupancyGrid::Ptr grid;

    std::vector<std::unique_ptr<node>> path;

public:
    algo();
    void init(geometry_msgs::PoseWithCovarianceStamped start, geometry_msgs::PoseStamped goal, nav_msgs::OccupancyGrid::Ptr grid);
    void prune_neibours(node* node);
    void jps();
    void jump(node* curr_n, node* neibour);
    float countH(node* curr_n, node* neibour);
    void clear();

};


#endif //JPS_ALGO_H
