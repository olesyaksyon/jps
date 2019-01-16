//
// Created by olesya on 08.01.19.
//

#include <cmath>
#include "node.h"
#include "config.h"

static inline float normalizeHeadingRad(float t) {
    if (t < 0) {
        t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
        return 2.f * M_PI + t;
    }

    return t - 2.f * M_PI * (int)(t / (2.f * M_PI));
}

const int node::dx[] = { 1, 1,  1,  0, -1, -1, -1};
const int node::dy[] = { 1, 0, -1, -1, -1,  0,  1};



//тут же сразу считаем g

node::node(int x, int y, int dir, std::shared_ptr<node> prev):
x(x),
y(y),
dir(dir)
{
    open = false;
    close = false;

    //idx = x + y * width;

    /*if (prev) {
        g = prev->g;
        h = prev->h;
    }

    else {
        g = 0;
        h = 0;
    }


    if (dir > 0) {

        if (dir > 3)
            this->g += config::penalty_backwards;

        if (dir == 3 || dir == 5 || dir == 0 || dir == 2)
            this->g += config::penalty_rotate;
    }*/

    if (prev) {
        dist_to_prev = sqrt((prev->get_x() - x) * (prev->get_x() - x) + (prev->get_y() - y) * (prev->get_y() - y));
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


    std::shared_ptr<node> neigbour(new node(new_x, new_y, dir, shared_from_this()));
    return neigbour;
}

std::shared_ptr<node> node::get_prev(){
    return prev.lock();
}

void node::set_h(float h) {
    this->h += h;
}

int node::get_dir(){
    this->dir;
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
    return (prev.expired() && dir < 0);
}

float node::get_g() {
    return g;
}