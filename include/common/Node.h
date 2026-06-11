#ifndef NODE_H
#define NODE_H

#include <limits>

struct Node
{
    double s;
    double l;

    double x;
    double y;

    double cost;
    bool is_obstacle;
    //Node* parent;

    int parent_i;
    int parent_j;

    Node(double s_, double l_, double x_, double y_) : s(s_), l(l_), x(x_), y(y_), 
                                                        cost(std::numeric_limits<double>::infinity()),
                                                         is_obstacle(false), parent_i(-1), parent_j(-1) {}
};

#endif