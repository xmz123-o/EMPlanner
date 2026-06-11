#ifndef OBSTACLE_H
#define OBSTACLE_H

struct Obstacle
{
    double x;
    double y;
    double radius;

    Obstacle(double x_, double y_, double r_) : x(x_), y(y_), radius(r_) {}
};

#endif