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
    std::weak_ptr<node> start;
    std::weak_ptr<node> goal;
    nav_msgs::OccupancyGrid::Ptr grid;

    std::vector<std::unique_ptr<node>> path;

public:
    algo();
    void init(geometry_msgs::PoseWithCovarianceStamped start, geometry_msgs::PoseStamped goal, nav_msgs::OccupancyGrid::Ptr grid);
    void prune_neibours(std::shared_ptr<node> curr_);
    void jps();
    bool is_dominant(std::shared_ptr<node> curr, std::shared_ptr<node> next);
    std::shared_ptr<node> jump(std::shared_ptr<node> curr_n);
    float countH(node* curr_n, node* neibour);
    void clear();

};


#endif //JPS_ALGO_H
