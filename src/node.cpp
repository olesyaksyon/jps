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

    if (!prev.expired()) {
        std::shared_ptr<node> prev_ = prev.lock();
        g = prev_->g;

        if (dir > 3) {
            g += config::penalty_backwards;

            if (dir == 3 || dir == 5)
                g += config::penalty_rotate;
        }


        else if (prev_->dir != dir)
            g += config::penalty_rotate;
    }


    else
        g = 1;


    conctruct_neighbours();

}

void node::conctruct_neighbours(){
    for (int i = 0; i < 6; i ++) {
        ///map - ключ g

    }
}

float node::get_g() {
    return g;
}