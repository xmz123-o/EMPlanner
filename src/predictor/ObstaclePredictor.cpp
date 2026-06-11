#include "common/ObstaclePredictor.h"
#include <iostream>

std::vector<PredictedObstaclePoint> ObstaclePredictor::Predict(const DynamicObstacle& obs,
                    const std::vector<ReferencePoint>& ref_line,double total_time,double dt)
{
    std::vector<PredictedObstaclePoint> result;

    for(double t=0;t<=total_time;t+=dt)
    {
        PredictedObstaclePoint pt;

        pt.t = t;

        pt.x = obs.x + obs.vx * t;

        pt.y = obs.y + obs.vy * t;

        auto frenet = CoordinateTransform::CartesianToFrenet(pt.x,pt.y,ref_line);

        pt.s = frenet.s;
        pt.l = frenet.l;

        std::cout
        << "t=" << pt.t
        << " s=" << pt.s
        << " l=" << pt.l
        << std::endl;

        result.push_back(pt);
    }

    return result;
}