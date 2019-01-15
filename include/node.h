//
// Created by olesya on 08.01.19.
//

#ifndef JPS_NODE_H
#define JPS_NODE_H

#include <memory>
#include <vector>
#include <map>
#include <functional>

class node : public std::enable_shared_from_this<node>{
    int x;
    int y;

    float g;
    float h;



    int idx;

    std::weak_ptr<node> prev;

    int dir;///направление относительно последней ноды
    static const int dx[];
    static const int dy[];
    //static const float dt[];

    bool close;
    bool open;

public:
    float dist_to_prev;

    node(int x, int y, int dir, std::shared_ptr<node> prev);
    node(const node &other);

    std::shared_ptr<node> construct_neigbour_dir(int dir);

    float get_g();
    void set_h(float h);
    float get_f() const;
    int get_x();
    int get_y();
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
