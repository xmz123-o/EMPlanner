#ifndef COORDITRANS_H
#define COORDITRANS_H

#include <vector>
#include <cmath>
#include <limits>
#include "common/ReferencePoint.h"
#include "common/Obstacle.h"
#include "common/ObstacleFrenet.h"

struct FrenetPoint
{
    double s;
    double l;
};

class CoordinateTransform
{
public:

    static FrenetPoint CartesianToFrenet( double x, double y, const std::vector<ReferencePoint>& reference_line);

    static void FrenetToCartesian(double s, double l, const std::vector<ReferencePoint>& reference_line,
                                 double& x, double& y);
    
    static ObstacleFrenet ObstacleToFrenet(const Obstacle& obstacle, const std::vector<ReferencePoint>& reference_line);
};

#endif