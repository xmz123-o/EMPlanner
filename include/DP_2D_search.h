#ifndef DP_2D_SEARCH_H
#define DP_2D_SEARCH_H

#include<iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>

struct Node
{
    double layer;
    double col;
    double cost;
    bool is_obstacle;
    Node* parent;

    Node(double layer_, double col_) : layer(layer_), col(col_), cost(std::numeric_limits<double>::infinity()), is_obstacle(false), parent(nullptr) {}
};




#endif // DP_2D_SEARCH_H