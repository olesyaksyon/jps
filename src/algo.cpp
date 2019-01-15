//
// Created by olesya on 07.01.19.
//

#include <tf/transform_datatypes.h>
#include "algo.h"
#include <ros/console.h>
#include <config.h>
#include <unordered_map>

algo::algo(){
    grid = NULL;
    possible_dirs = 8;
}

bool algo::is_on_grid(shared_ptr_node curr_) {
    float curr_x = curr_->get_x();
    float curr_y = curr_->get_y();
    return (curr_x > 0 && curr_y > 0 && curr_x < grid->info.width && curr_y < grid->info.height);
}


bool algo::is_traversible(shared_ptr_node curr_) {
    std::shared_ptr<node> exper_node = curr_;

    if (is_on_grid(curr_))
        return false;

    int x = curr_->get_x();
    int y = curr_->get_y();

    if (grid->data[x + y * grid->info.height] > 0)
        return false;

    return true;
}


float algo::distance_to_goal(shared_ptr_node curr_){
    return sqrt((curr_->get_x() - goal.lock()->get_x()) * (curr_->get_x() - goal.lock()->get_x())  + (curr_->get_y() - goal.lock()->get_y()) * (curr_->get_y() - goal.lock()->get_y()));
}

float distance_betw_nodes(shared_ptr_node node1, shared_ptr_node node2){
    return sqrt((node1->get_x() - node2->get_x()) * (node1->get_x() - node2->get_x()) + (node1->get_y() - node2->get_y()) * (node1->get_y() - node2->get_y()));
}


void algo::prune_neibours(shared_ptr_node curr_, std::vector<shared_ptr_node> &neighbours) {
    auto prev = curr_->get_prev();

    int dir1 = prev->get_dir() - 1;

    if (dir1 < 0)
        dir1 = possible_dirs - 1;

    int dir2 = prev->get_dir() + 1;

    if (dir2 >= possible_dirs)
        dir2 = 0;

    auto poss_curr1 = prev->construct_neigbour_dir(dir1);
    auto poss_curr2 = prev->construct_neigbour_dir(dir2);

    for (int i = 0; i < possible_dirs; i ++) {
        auto neighbour = curr_->construct_neigbour_dir(i);

        float len_with_x = curr_->dist_to_prev + neighbour->dist_to_prev;
        float len_without_x1 = poss_curr1->dist_to_prev + distance_betw_nodes(poss_curr1, neighbour);
        float len_withous_x2 = poss_curr2->dist_to_prev + distance_betw_nodes(poss_curr2, neighbour);

        if (len_without_x1 < len_with_x || len_withous_x2 < len_with_x)
            continue;
        else
            neighbours.push_back(neighbour);
    }
}

void algo::init(geometry_msgs::PoseWithCovarianceStamped start_, geometry_msgs::PoseStamped goal_, nav_msgs::OccupancyGrid::Ptr grid){
    start = std::make_shared<node>(start_.pose.pose.position.x, start_.pose.pose.position.y, -1, nullptr);
    goal = std::make_shared<node>(goal_.pose.position.x, goal_.pose.position.y, -1, nullptr);
    this->grid = grid;
}

void algo::jump(shared_ptr_node& curr_n){
    curr_n = curr_n->construct_neigbour_dir(curr_n->get_dir());
    if (distance_to_goal(curr_n) < 0.001)
        return;

    if ( !is_traversible(curr_n))
        curr_n = nullptr;

    auto neihbour = curr_n->construct_neigbour_dir(curr_n->get_dir());

    if (!is_traversible(neihbour))
        return;

}

void algo::identify_successors(shared_ptr_node curr_, std::priority_queue<node, std::vector<shared_ptr_node>, node_cmp> & open_){
    if (curr_->is_start()) {
        for (int i = 0; i < possible_dirs; i ++) {
            auto neighbour = curr_->construct_neigbour_dir(i);
            neighbour->open_n();
            neighbour->set_h(distance_to_goal(neighbour));
            open_.push(neighbour);
        }
    }

    else {
        std::vector <shared_ptr_node> neighbours;
        prune_neibours(curr_, neighbours);

        for (auto n = neighbours.begin(); n != neighbours.end(); n ++ ) {
            shared_ptr_node neighbour = *n;
            jump(neighbour);

            if (neighbour) {
                neighbour->open_n();
                neighbour->set_h(distance_to_goal(neighbour));
                open_.push(neighbour);
            }
        }
    }
}


shared_ptr_node algo::jps(){
    if (start.expired() || goal.expired() || !grid) {
        return nullptr;
    }

    auto curr_ = start.lock();
    std::priority_queue<node, std::vector<shared_ptr_node>, node_cmp> open_;
    std::unordered_map<int, shared_ptr_node> inited_node;
    open_.push(curr_);

    while (!open_.empty()) {
        shared_ptr_node curr_ = open_.top();

        curr_->set_idx(curr_->get_x() + curr_->get_y() * (int)grid->info.width);
        inited_node.emplace(curr_->get_idx(), curr_);

        if (distance_to_goal(curr_) < 0.01)
            return curr_;

        open_.pop();
        curr_->close_n();
        identify_successors(curr_, open_);
    }
    return nullptr;
}


void algo::clear(){
    //path.clear();
    start.reset();
    goal.reset();
    grid = NULL;
}