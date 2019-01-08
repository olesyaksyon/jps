//
// Created by olesya on 07.01.19.
//

#ifndef JPS_CONFIG_H
#define JPS_CONFIG_H


class config {
    int robot_w;//for obstacles
    int robot_h;
    //TODO: metres to coords
    int min_distance_to_obstacle; //in coords


public:
    config(int robot_w, int robot_h);




};


#endif //JPS_CONFIG_H
