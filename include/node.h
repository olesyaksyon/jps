//
// Created by olesya on 08.01.19.
//

#ifndef JPS_NODE_H
#define JPS_NODE_H

#include <memory>
#include <vector>
#include <map>
#include <functional>

class node;

class own_double_less : public std::binary_function<float,std::weak_ptr<node>,bool>
{
public:
    own_double_less( float arg_ = 1e-7 ) : epsilon(arg_) {}
    bool operator()( const float &left, const float &right  ) const
    {
        return (abs(left - right) > epsilon) && (left < right);
    }
    double epsilon;
};

///dirs
///3         0
///4  point  1
///5         2

class node {
    float x;
    float y;
    float t;

    float g;
    float h;

    int dir;///направление относительно последней ноды

    static const float dx[];
    static const float dy[];
    static const float dt[];


    ///array

public:
    std::map<float, std::weak_ptr<node>, own_double_less> neighbours;

    node(float x, float y, float t, int dir, float g);
    void conctruct_neighbours();
    void construct_neigbour_dir(int dir);
    float get_g();
};


#endif //JPS_NODE_H
