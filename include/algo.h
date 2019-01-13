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

typedef std::weak_ptr<node> weak_ptr_node;
typedef std::shared_ptr<node> shared_ptr_node;

struct node_cmp {
    bool operator()(const shared_ptr_node& a, const shared_ptr_node& b) {
        return (b->get_f() < a->get_f());
    }
};

class algo {
    shared_ptr_node start;
    shared_ptr_node goal;
    nav_msgs::OccupancyGrid::Ptr grid;

    std::list<std::weak_ptr<node>> path;

public:
    algo();
    void init(geometry_msgs::PoseWithCovarianceStamped start, geometry_msgs::PoseStamped goal, nav_msgs::OccupancyGrid::Ptr grid);
    float distance_to_goal(shared_ptr_node curr_);
    float distance_to_start(shared_ptr_node curr_);

    void prune_neibours(shared_ptr_node curr_, std::priority_queue<node, std::vector<weak_ptr_node>, node_cmp> &);
    float get_goal_distance(shared_ptr_node curr_);
    void simple_a_star(std::priority_queue<node, std::vector<shared_ptr_node>, node_cmp>& open_, shared_ptr_node curr_);


    bool is_on_grid(shared_ptr_node curr_);
    bool is_traversible(shared_ptr_node curr_);

    std::shared_ptr<node> choose_direction(shared_ptr_node curr_);
    shared_ptr_node jps();
    bool is_dominant(shared_ptr_node curr, shared_ptr_node next);
    void jump(shared_ptr_node& curr_n);
    void clear();

};


#endif //JPS_ALGO_H
