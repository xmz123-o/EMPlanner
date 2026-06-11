#ifndef OBSTACLE_FRENET_H
#define OBSTACLE_FRENET_H

struct ObstacleFrenet
{
    double s;
    double l;
    double radius;

    ObstacleFrenet(double s_, double l_, double r_) : s(s_), l(l_), radius(r_) {}
};

#endif