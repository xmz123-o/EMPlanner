#ifndef DPSTCOST_H
#define DPSTCOST_H

#include "planner/STGraph.h"
#include "planner/SpeedDP.h"

class DpStCost
{
public:

    DpStCost(const std::vector<STBoundary>& boundaries);

    bool IsInObstacle(double s, double t) const;

    bool IsCollision(double s0, double t0, double s1, double t1) const;

    double GetObstacleCost(double s,double t) const;

    double GetSpeedCost( double v,double ref_speed) const;

    double GetAccelCost(const SpeedNode& parent,const SpeedNode& current,const SpeedNode& next) const;

    double GetJerkCost(const SpeedNode& grand_parent,const SpeedNode& parent,const SpeedNode& current,const SpeedNode& next) const;

private:

    std::vector<STBoundary> boundaries_;
};

#endif