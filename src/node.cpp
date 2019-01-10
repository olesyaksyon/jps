//
// Created by olesya on 08.01.19.
//

#include "node.h"
#include "config.h"

//тут же сразу считаем g
node::node(float x, float y, float t, std::weak_ptr<node> prev, int dir):
x(x),
y(y),
t(t),
dir(dir)
{
    std::shared_ptr<node> prev_ = prev.lock();
    if (prev_) {

        g = prev_->g;

        if (dir == prev_->dir)
            g += config::penalty_forward;

        if (dir != prev_->dir && dir < 3) {
            dir < prev_->dir ? prev_->dir = dir - 1 : prev_->dir = dir + 1;
            g += config::penalty_rotate;
        }

        if (dir > 3) {
            g += config::penalty_backwards;
        }

        if (dir > 3 && prev_->dir > 3 && prev_->dir != dir) {
            g += config::penalty_rotate;

            dir < prev_->dir ? prev_->dir = dir - 1 : prev_->dir = dir + 1;
        }
    }



    else
        g = 1;

    this->prev = prev_;
}

void node::conctruct_neighbours(){
    for (int i = 0; i < 6; i ++) {

    }
}

float node::get_g() {
    return g;
}