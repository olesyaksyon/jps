//
// Created by olesya on 08.01.19.
//

#ifndef JPS_NODE_H
#define JPS_NODE_H

///здесь заготовки для dir
///dir путь будут глобальными переменными

class node {
    float x;
    float y;
    float t;

    //?
    float g;
    float h;

    node * prev;
    ///array

public:
    node(float x, float y, float t, node * prev, int dir) {}
    node* getNeibour(int dir);



};


#endif //JPS_NODE_H
