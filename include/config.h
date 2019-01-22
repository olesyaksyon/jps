//
// Created by olesya on 07.01.19.
//

#ifndef JPS_CONFIG_H
#define JPS_CONFIG_H

#include <cmath>

namespace config {

    static int R = 3;
    static int width = 1.75;
    static int length = 2.75;
    static int possible_dirs = 6;

    static float penalty_rotate = 1.05;
    static float penalty_backwards = 2.0;
    static float penaltyCOD = 2.0;

    static int headings = 36;
    static float deltaHeadingDeg = 360 / (float)headings;
    static float deltaHeadingRad = 2 * M_PI / (float)headings;
    static const float deltaHeadingNegRad = 2 * M_PI - deltaHeadingRad;
}


#endif //JPS_CONFIG_H
