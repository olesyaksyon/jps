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
const float node::dt[] = { 0.1178097,   0,          -0.1178097};


//тут же сразу считаем g
node::node(float x, float y, float t, int dir, float g):
x(x),
y(y),
t(t),
g(g),
dir(dir)
{
///у первой ноды -1
    if (dir > 0) {

        if (dir > 3)
            this->g += config::penalty_backwards;

        if (dir == 3 || dir == 5 || dir == 0 || dir == 2)
            this->g += config::penalty_rotate;
    }

    conctruct_neighbours();
}

void node::construct_neigbour_dir(int dir) {
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

    std::shared_ptr<node> neigbour(new node(new_x, new_y, new_t, dir, g));
    neighbours.emplace(neigbour->get_g(), std::weak_ptr<node>(neigbour));
}

void node::conctruct_neighbours(){
    for (int i = 0; i < 6; i ++) {
        construct_neigbour_dir(i);
    }
}

float node::get_g() {
    return g;
}