//
// Created by olesya on 08.01.19.
//

#ifndef JPS_NODE_H
#define JPS_NODE_H

#include <memory>
#include <vector>
#include <map>
#include <functional>


///dirs
///3         0
///4  point  1
///5         2

class node : public std::enable_shared_from_this<node>{
    float x;
    float y;
    float t;

    float g;
    float h;


    std::shared_ptr<node> prev;

    int dir;///направление относительно последней ноды

    static const float dx[];
    static const float dy[];
    static const float dt[];

    bool close;
    bool open;

public:


    node(float x, float y, float t, int dir, std::shared_ptr<node> prev);
    node(const node &other);

    std::shared_ptr<node> construct_neigbour_dir(int dir);

    float get_g();
    void set_h(float h);
    float get_f() const;
    float get_x();
    float get_y();
    int get_dir();

    void close_n();
    void open_();

    bool is_closed();
    bool is_open();
    bool is_start();
};


#endif //JPS_NODE_H
