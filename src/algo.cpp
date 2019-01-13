//
// Created by olesya on 07.01.19.
//

#include <tf/transform_datatypes.h>
#include "algo.h"
#include <ros/console.h>
#include <config.h>


algo::algo(){
    grid = NULL;
}

bool algo::is_on_grid(shared_ptr_node curr_) {
    float curr_x = curr_->get_x();
    float curr_y = curr_->get_y();
    //TODO: add t
    return (curr_x > 0 && curr_y > 0 && curr_x < grid->info.width && curr_y < grid->info.height);
}


bool algo::is_traversible(shared_ptr_node curr_) {
    std::shared_ptr<node> exper_node = curr_;

    for (int i = 0; i < config::min_dist_to_obstacle; i ++) {
        exper_node = exper_node->construct_neigbour_dir(1);
    }

    int x = int(exper_node->get_x());
    int y = int(exper_node->get_y());

    if (x < 0 || y < 0 || x > grid->info.width || y > grid->info.height)
        return false;

    if (grid->data[x + y * grid->info.height] > 0)
        return false;

    return true;
}


float algo::distance_to_goal(shared_ptr_node curr_){

    return sqrt((curr_->get_x() - goal->get_x()) * (curr_->get_x() - goal->get_x())  + (curr_->get_y() - goal->get_y()) * (curr_->get_y() - goal->get_y()));
}



float algo::distance_to_start(shared_ptr_node curr_) {
    return sqrt((curr_->get_x() - start->get_x()) * (curr_->get_x() - start->get_x())  + (curr_->get_y() - start->get_y()) * (curr_->get_y() - start->get_y()));
}

void algo::prune_neibours(shared_ptr_node curr_, std::priority_queue<node, std::vector<weak_ptr_node>, node_cmp> &) {

}

void algo::init(geometry_msgs::PoseWithCovarianceStamped start_, geometry_msgs::PoseStamped goal_, nav_msgs::OccupancyGrid::Ptr grid){
    float t_st = tf::getYaw(start_.pose.pose.orientation);
    float t_g = tf::getYaw(goal_.pose.orientation);
    start = std::make_shared<node>(start_.pose.pose.position.x, start_.pose.pose.position.y, t_st, -1, nullptr);
    goal = std::make_shared<node>(goal_.pose.position.x, goal_.pose.position.y, t_g, -1, nullptr);
    this->grid = grid;
}

void algo::jump(shared_ptr_node& curr_n){
    if (distance_to_goal(curr_n) < 0.001)
        return;

    if (!is_on_grid(curr_n) || !is_traversible(curr_n))
        curr_n = nullptr;

    auto neihbour = curr_n->construct_neigbour_dir(curr_n->get_dir());
    neihbour->set_h(distance_to_goal(neihbour));

    if (neihbour->get_f() - curr_n->get_f() < 0.8) {
        jump(neihbour);
    }

}

void algo::simple_a_star(std::priority_queue<node, std::vector<shared_ptr_node>, node_cmp>& open_, shared_ptr_node curr_) {
    for (int i = 0; i < 6; i ++) {
        if (!is_on_grid(curr_) || (!is_traversible(curr_)))
            continue;

        auto neighbour = curr_->construct_neigbour_dir(i);
        neighbour->set_h(distance_to_goal(neighbour));
        open_.push(neighbour);
    }
}



shared_ptr_node algo::jps(){
    if (!start || !goal || !grid) {
        return nullptr;
    }

    auto curr_ = start;
    std::priority_queue<node, std::vector<shared_ptr_node>, node_cmp> open_;
    open_.push(start);

    while (!open_.empty()) {
        shared_ptr_node curr_ = open_.top();

        if (distance_to_goal(curr_) < 0.001)
            return curr_;

        if (!curr_->is_start()) {
            jump(curr_);
        }

        else {
            simple_a_star(open_, curr_);
        }

        open_.pop();
        curr_->close_n();
    }
    return nullptr;
}


void algo::clear(){
    path.clear();
    start.reset();
    goal.reset();
    grid = NULL;
}