//
// Created by olesya on 08.01.19.
//

#include <cmath>
#include "node.h"
#include "config.h"


const int node::dx[] = { 1, 1,  1,  0, -1, -1, -1, 1};
const int node::dy[] = { 1, 0, -1, -1, -1,  0,  1, 0};



//тут же сразу считаем g

node::node(int x, int y, int dir, std::shared_ptr<node> prev):
x(x),
y(y),
dir(dir)
{
    open = false;
    close = false;

    if (prev) {
        dist_to_prev = sqrt((prev->get_x() - x) * (prev->get_x() - x) + (prev->get_y() - y) * (prev->get_y() - y));

        if (!dir%2) {
            g = prev->get_g() + dist_to_prev * config::penalty_rotate;
        }

        else
            g = prev->get_g() + dist_to_prev;
        this->prev = prev;
    }

    else {
        dist_to_prev = 0;
        g = 0;
    }

}

void node::set_idx(int idx){
    this->idx = idx;
}


node::node(const node &other) :
x(other.x),
y(other.y),
g(other.g),
h(other.h)
{
}

int node::get_x() {
    return x;
}

int node::get_y() {
    return y;
}

int node::get_idx() {
    return idx;
}

float node::get_h(){
    return h;
}

std::shared_ptr<node> node::construct_neigbour_dir(int dir) {
    int new_x = x + dx[dir];
    int new_y = y + dy[dir];
    //float new_t;


    /*if (dir < 3) {
        new_x = x + dx[dir] * cos(t) - dy[dir] * sin(t);
        new_y = y + dx[dir] * sin(t) + dy[dir] * cos(t);
        new_t = normalizeHeadingRad(t + dt[dir]);
    }

    else {
        new_x = x - dx[dir - 3] * cos(t) - dy[dir - 3] * sin(t);
        new_y = y - dx[dir - 3] * sin(t) + dy[dir - 3] * cos(t);
        new_t = normalizeHeadingRad(t - dt[dir - 3]);
    }*/

    std::shared_ptr<node> ex = shared_from_this();
    std::shared_ptr<node> neigbour(new node(new_x, new_y, dir, ex));
    return neigbour;
}

std::shared_ptr<node> node::get_prev(){
    return prev;
}

void node::set_h(float h) {
    this->h += h;
}

int node::get_dir(){
    return dir;
}

float node::get_f() const {
    return h + g;
}

void node::close_n() {
    close = true;
    open = false;
}

void node::open_n() {
    open = true;
    close = false;
}

bool node::is_closed() {
    return close;
}

bool node::is_open() {
    return open;
}

bool node::is_start() {
    return (!prev && dir < 0);
}

float node::get_g() {
    return g;
}