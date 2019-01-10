//
// Created by olesya on 10.01.19.
//

#ifndef JPS_MAP_HANDLER_H
#define JPS_MAP_HANDLER_H

#include <nav_msgs/OccupancyGrid.h>

struct sector {
    float tl_x;
    float tl_y;

    int h;
    int w;

    bool is_point_in(float x, float y) {

    }
};

class map_handler {

    int width;
    int height;

    map_handler(nav_msgs::OccupancyGrid::Ptr grid);

};


#endif //JPS_MAP_HANDLER_H
