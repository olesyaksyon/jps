//
// Created by olesya on 08.01.19.
//

#ifndef JPS_NODE_H
#define JPS_NODE_H

#include <memory>
#include <vector>
#include <map>
#include <functional>

///класс, описывающие положение
class node : public std::enable_shared_from_this<node>{
    float x;
    float y;
    float t;

    float g;
    float h;

    int idx;

    std::shared_ptr<node> prev;

    int dir;///направление относительно последней ноды
    static const float dx[];
    static const float dy[];
    static const float dt[];

    bool close;
    bool open;

public:
    float dist_to_prev;

    node(float x, float y, float t, int dir, std::shared_ptr<node> prev);
    node(const node &other);

    std::shared_ptr<node> construct_neigbour_dir(int dir);

    float get_g();
    float get_h();
    void set_h(float h);
    float get_f() const;
    float get_x();
    float get_y();
    float get_t();
    int get_dir();
    int get_idx();
    std::shared_ptr<node> get_prev();

    void close_n();
    void open_n();
    void set_idx(int idx);

    bool is_closed();
    bool is_open();
    bool is_start();
};


#endif //JPS_NODE_H
