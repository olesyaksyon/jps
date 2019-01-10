//
// Created by olesya on 08.01.19.
//

#ifndef JPS_NODE_H
#define JPS_NODE_H

#include <memory>
#include <vector>

///dirs
///3         0
///4  point  1
///5         2

class node {
    float x;
    float y;
    float t;

    //?
    float g;
    float h;

    int dir;

    std::weak_ptr<node> prev;

    ///array

public:
    std::vector<std::weak_ptr<node>> neighbours;

    node(float x, float y, float t, std::weak_ptr<node> prev, int dir);
    void conctruct_neighbours();
    float get_g();




};


#endif //JPS_NODE_H
