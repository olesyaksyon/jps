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


const float node::dy[] = { -0.0415893,  0,          0.0415893};
const float node::dx[] = { 0.705224,    0.7068582,   0.705224};
const float node::dt[] = { 0.35,        0,           -0.35};


//тут же сразу считаем g
//TODO: пиздец с указателями
node::node(float x, float y, float t, int dir, std::shared_ptr<node> prev):
x(x),
y(y),
t(t),
dir(dir)
{
    open = false;
    close = false;

    if (prev) {
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
    }

    this->prev = prev;
}

node::node(const node &other) :
x(other.x),
y(other.y),
t(other.t),
g(other.g),
h(other.h)
{
}

float node::get_x() {
    return x;
}

float node::get_y() {
    return y;
}

std::shared_ptr<node> node::construct_neigbour_dir(int dir) {
    float new_x;
    float new_y;
    float new_t;


    if (dir < 3) {
        new_x = x + dx[dir] * cos(t) - dy[dir] * sin(t);
        new_y = y + dx[dir] * sin(t) + dy[dir] * cos(t);
        new_t = normalizeHeadingRad(t + dt[dir]);
    }

    else {
        new_x = x - dx[dir - 3] * cos(t) - dy[dir - 3] * sin(t);
        new_y = y - dx[dir - 3] * sin(t) + dy[dir - 3] * cos(t);
        new_t = normalizeHeadingRad(t - dt[dir - 3]);
    }
    std::shared_ptr<node> neigbour(new node(new_x, new_y, new_t, dir, shared_from_this()));
    return neigbour;
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

void node::open_() {
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