//
// Created by olesya on 07.01.19.
//

#ifndef JPS_CONFIG_H
#define JPS_CONFIG_H

namespace config {

    static int R = 6;
    static int width = 1.75;
    static int length = 2.75;
    static int possible_dirs = 8;

    static float penalty_rotate = 1.05;
    static float penalty_backwards = 2.0;
    static float penalty_forward = 0.2;

    //static constexpr color teal = {102.f / 255.f, 217.f / 255.f, 239.f / 255.f};
}


#endif //JPS_CONFIG_H
