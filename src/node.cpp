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


const float node::dy[] = { -0.16578,        0,      0.16578};
const float node::dx[] = { 1.40067,     1.41372,    1.40067};
const float node::dt[] = { 0.2356194,         0,   -0.2356194};



node::node(float x, float y, float t, int dir, std::shared_ptr<node> prev):
x(x),
y(y),
t(t),
dir(dir)
{https://ru.wikipedia.org/wiki/A*
    open = false;
    close = false;

    if (prev) {
        dist_to_prev = sqrt((prev->get_x() - x) * (prev->get_x() - x) + (prev->get_y() - y) * (prev->get_y() - y));
        g = dist_to_prev;

        if (!dir%2) {
            g *= config::penalty_rotate;
        }

        if (dir > 2){
            g *= config::penalty_backwards;
        }

        if (prev->dir != dir && (dir > 2 || prev->dir > 2)) {
            g *= config::penaltyCOD;
        }

        g += prev->g;
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

int node::get_idx() {
    return idx;
}

float node::get_h(){
    return h;
}

float node::get_t(){
    return t;
}

std::shared_ptr<node> node::construct_neigbour_dir(int dir) {
    float new_x; //= x + dx[dir];
    float new_y; //= y + dy[dir];
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

    std::shared_ptr<node> ex = shared_from_this();
    std::shared_ptr<node> neigbour(new node(new_x, new_y, new_t, dir, ex));
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