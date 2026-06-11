#ifndef PATHBOUNDARY_H
#define PATHBOUNDARY_H

#include <vector>
#include<cmath>
#include "common/ReferencePoint.h"
#include "common/Obstacle.h"
#include "common/ObstacleFrenet.h"

struct PathBoundaryPoint
{
    double s;
    double l_min;
    double l_max;

    PathBoundaryPoint(double s_,double l_min_,double l_max_) : s(s_),l_min(l_min_),l_max(l_max_) {}
};

class PathBoundaryGenerator
{
public:
    std::vector<PathBoundaryPoint> Generate(const std::vector<ReferencePoint>& reference_line,
                                            const std::vector<ObstacleFrenet>& obstacles_frenet);
};

#endif // PATHBOUNDARY_H