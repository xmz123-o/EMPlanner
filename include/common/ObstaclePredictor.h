#pragma once

#include <vector>
#include "common/DynamicObstacle.h"
#include "common/ReferencePoint.h"
#include "common/CoordiTrans.h"

struct PredictedObstaclePoint
{
    double t;

    double x;
    double y;

    double s;
    double l;
};

class ObstaclePredictor
{
public:

    std::vector<PredictedObstaclePoint> Predict(const DynamicObstacle& obs, const std::vector<ReferencePoint>& ref_line,
                                                double total_time, double dt);
};