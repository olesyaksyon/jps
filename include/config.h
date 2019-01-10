//
// Created by olesya on 07.01.19.
//

#ifndef JPS_CONFIG_H
#define JPS_CONFIG_H

namespace config {
    ///пусть расстояние между колёсами будет 3, тогда на поворот закладываем 4
    int min_dist_to_obstacle = 4;

    float penalty_rotate = 1.05;
    float penalty_backwards = 2.0;
    float penalty_forward = 0.2;
}


#endif //JPS_CONFIG_H
