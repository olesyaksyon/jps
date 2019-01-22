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
#include <queue>
#include <robot.h>

typedef std::weak_ptr<node> weak_ptr_node;
typedef std::shared_ptr<node> shared_ptr_node;

struct node_cmp {
    bool operator()(const shared_ptr_node& a, const shared_ptr_node& b) {
        return (b->get_f() < a->get_f());
    }
};

///основной алгоритм
class algo {
    shared_ptr_node start;
    shared_ptr_node goal;
    nav_msgs::OccupancyGrid::Ptr grid;
    int possible_dirs;
    robot robot_;


public:
    algo();
    void init(geometry_msgs::PoseWithCovarianceStamped::ConstPtr start, geometry_msgs::PoseStamped::ConstPtr goal, nav_msgs::OccupancyGrid::Ptr grid);
    float distance_to_goal(shared_ptr_node curr_);

    void prune_neibours(shared_ptr_node curr_, std::vector<shared_ptr_node> &neighbours);

    shared_ptr_node jps();
    void jump(shared_ptr_node& curr_n);
    void identify_successors(shared_ptr_node curr_, std::priority_queue<node, std::vector<shared_ptr_node>, node_cmp> & open_);
    void clear();

};


#endif //JPS_ALGO_H
