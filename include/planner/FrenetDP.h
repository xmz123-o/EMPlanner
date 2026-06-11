#ifndef FRENETDP_H
#define FRENETDP_H
#include<iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include "common/Node.h"
#include "common/ReferencePoint.h"
#include "common/Obstacle.h"
#include "planner/PathBoundary.h"

class FrenetDP
{
public:
    static bool ISCollision(const Node& node, const Obstacle& obs);

    double DisToObstacle(const Node& node,const Obstacle& obs);

    std::vector<Node> DP_Search(const std::vector<ReferencePoint>& reference_line, 
        const std::vector<PathBoundaryPoint>& boundary, const std::vector<Obstacle>& obstacles,double start_l);
private:
    double w_smoothness = 15.0;
    double w_reference = 5.0;
    double w_collision = 10.0;
    double w_curvature = 5.0;
};

#endif // FRENETDP_H