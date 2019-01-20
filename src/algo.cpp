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
    possible_dirs = config::possible_dirs;
    start = NULL;
    goal = NULL;
}

bool algo::is_on_grid(shared_ptr_node curr_) {
    float curr_x = curr_->get_x();
    float curr_y = curr_->get_y();
    return (curr_x >= 0 && curr_y >= 0 && curr_x < grid->info.width && curr_y < grid->info.height);
}


bool algo::is_traversible(shared_ptr_node curr_) {
    std::shared_ptr<node> exper_node = curr_;

    if (!is_on_grid(curr_))
        return false;

    int x = curr_->get_x();
    int y = curr_->get_y();

    int offset = 3;

    if (grid->data[x + y * grid->info.width])
        return false;

    //это hot-fix, я потом перепишу этот код нормально
    for (int i = 1; i < offset; i ++) {
        if (grid->data[x + offset + (y + offset) * grid->info.width] || grid->data[x + (y + offset) * grid->info.width] || grid->data[x + offset + y * grid->info.width])
            return false;

        if (x > 3 && y > 3) {
            if (grid->data[x - offset + (y - offset) * grid->info.width] || grid->data[x + (y - offset) * grid->info.width] || grid->data[x - offset + y * grid->info.width])
                return false;
        }

    }

    return true;
}


float algo::distance_to_goal(shared_ptr_node curr_){
    return sqrt((curr_->get_x() - goal->get_x()) * (curr_->get_x() - goal->get_x())  + (curr_->get_y() - goal->get_y()) * (curr_->get_y() - goal->get_y()));
}

float distance_betw_nodes(shared_ptr_node node1, shared_ptr_node node2){
    return sqrt((node1->get_x() - node2->get_x()) * (node1->get_x() - node2->get_x()) + (node1->get_y() - node2->get_y()) * (node1->get_y() - node2->get_y()));
}

void algo::prune_neibours(shared_ptr_node curr_, std::vector<shared_ptr_node> &neighbours) {
    auto prev = curr_->get_prev();
    std::vector<int> dirs;

    int dir = curr_->get_dir() - 1;

    if (dir < 0)
        dir = possible_dirs - 1;

    dirs.push_back(dir);

    dir = curr_->get_dir() + 1;

    if (dir >= possible_dirs)
        dir = 0;

    dirs.push_back(dir);


    for (int i = 0; i < possible_dirs; i ++) {
        auto neighbour = curr_->construct_neigbour_dir(i);

        if (!is_traversible(neighbour))
            continue;

        float len_with_x = curr_->dist_to_prev + neighbour->dist_to_prev;

        for (auto dir_it = dirs.begin(); dir_it != dirs.end(); dir_it ++) {
            auto poss_curr = prev->construct_neigbour_dir(*dir_it);
            float len_without_x = poss_curr->dist_to_prev + distance_betw_nodes(poss_curr, neighbour);

            if (len_without_x < len_with_x)
                continue;
            else
                neighbours.push_back(neighbour);
        }
    }
}

void algo::init(geometry_msgs::PoseWithCovarianceStamped start, geometry_msgs::PoseStamped goal, nav_msgs::OccupancyGrid::Ptr grid){
    this->grid = grid;

    int x = static_cast<int>(start.pose.pose.position.x);
    int y = static_cast<int>(start.pose.pose.position.y);

    this->start = std::make_shared<node>(x, y, -1, nullptr);

    x = static_cast<int>(goal.pose.position.x);
    y = static_cast<int>(goal.pose.position.y);

    this->goal = std::make_shared<node>(x, y, -1, nullptr);
}

void algo::jump(shared_ptr_node& curr_n){
    curr_n = curr_n->construct_neigbour_dir(curr_n->get_dir());
    curr_n->set_h(distance_to_goal(curr_n));

    if ( !is_traversible(curr_n)) {
        curr_n = nullptr;
        return;
    }

    if (distance_to_goal(curr_n) < 0.001)
        return;

    auto neihbour = curr_n->construct_neigbour_dir(curr_n->get_dir());
    if (!is_traversible(neihbour) || curr_n->get_prev()->get_h() < curr_n->get_h()) {
        neihbour = NULL;
        return;
    }

    if (!(curr_n->get_dir()%2)) {
        int dir1 = curr_n->get_dir() - 1;
        if (dir1 < 0)
            dir1 = 7;

        int dir2 = curr_n->get_dir() + 1;

        auto neighbour1 = curr_n->construct_neigbour_dir(dir1);
        auto neighbour2 = curr_n->construct_neigbour_dir(dir2);

        jump(neighbour1);
        jump(neighbour2);

        if (neighbour1 && neighbour2)
            return;
    }

    jump(curr_n);
}

void algo::identify_successors(shared_ptr_node curr_, std::priority_queue<node, std::vector<shared_ptr_node>, node_cmp> & open_){
    if (!curr_->get_prev()) {
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
    if (!start || !goal || !grid) {
        return nullptr;
    }

    if (!is_traversible(start) || !is_traversible(goal)) {
        ROS_INFO("start or goal is in obstacle or out map");
        return nullptr;
    }

    auto curr_ = start;
    std::priority_queue<node, std::vector<shared_ptr_node>, node_cmp> open_;
    std::unordered_map<int, shared_ptr_node> inited_node;
    open_.push(curr_);

    while (!open_.empty()) {
        shared_ptr_node curr_ = open_.top();
        open_.pop();

        if (!is_traversible(curr_))
            continue;

        int idx = curr_->get_x() + curr_->get_y() * grid->info.width;
        curr_->set_idx(idx);

        if (inited_node.find(idx) == inited_node.end())
            inited_node.emplace(curr_->get_idx(), curr_);
        else
            continue;

        if (distance_to_goal(curr_) < 0.01)
            return curr_;

        curr_->close_n();
        identify_successors(curr_, open_);
    }
    return nullptr;
}


void algo::clear(){
    start.reset();
    goal.reset();
    grid = NULL;
}