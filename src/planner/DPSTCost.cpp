#include "planner/DPSTCost.h"
#include <cmath>
#include <iostream>

DpStCost::DpStCost(const std::vector<STBoundary>& boundaries) : boundaries_(boundaries) {}

bool DpStCost::IsInObstacle(double s, double t) const
{
    for(const auto& boundary : boundaries_)
    {
        for(size_t i = 0; i < boundary.lower_points.size(); ++i)
        {
            if(std::fabs(boundary.lower_points[i].t - t) > 1e-6)
                continue;

            double lower = boundary.lower_points[i].s;
            double upper = boundary.upper_points[i].s;

            if(s >= lower && s <= upper)
            {
                return true;
            }
        }
    }
    return false;
}

bool DpStCost::IsCollision(double s0, double t0, double s1, double t1) const
{
    int sample_num = 10;

    for(int k=0;k<=sample_num;k++)
    {
        double r = static_cast<double>(k) / sample_num;

        double s = s0 + r * (s1-s0);

        double t = t0 + r * (t1-t0);

        if(IsInObstacle(s,t))
        {
            return true;
        }
    }

    return false;
}

double DpStCost::GetObstacleCost( double s,double t) const
{
    double total_cost = 0.0;

    constexpr double kCollisionCost = 1e8;

    constexpr double kSafeDistance = 15.0;

    constexpr double kObstacleWeight = 500.0;

    for(const auto& boundary : boundaries_)
    {
        //----------------------------------
        // 找到对应时刻
        //----------------------------------
        for(size_t i = 0; i < boundary.lower_points.size(); ++i)
        {
            if(std::fabs( boundary.lower_points[i].t - t) > 0.1)
            {
                continue;
            }

            double lower = boundary.lower_points[i].s;

            double upper = boundary.upper_points[i].s;

            //----------------------------------
            // collision
            //----------------------------------
            if(s >= lower && s <= upper)
            {
                return kCollisionCost;
            }

            //----------------------------------
            // distance to obstacle
            //----------------------------------
            double distance;

            if(s < lower)
            {
                distance = lower - s;
            }
            else
            {
                distance = s - upper;
            }

            //----------------------------------
            // obstacle potential field
            //----------------------------------
            if(distance < kSafeDistance)
            {
                //  std::cout
                //     << "obs influence "
                //     << "t=" << t
                //     << " s=" << s
                //     << " dist=" << distance
                //     << std::endl;
                double ratio = (kSafeDistance - distance) / kSafeDistance;

                total_cost += kObstacleWeight * ratio * ratio;
            }

        }
    }

    return total_cost;
}

double DpStCost::GetSpeedCost(double v, double ref_speed) const
{
    double ration = (ref_speed - v) / ref_speed;
    return 500.0 * ration * ration;
}

double DpStCost::GetAccelCost(const SpeedNode& parent,const SpeedNode& current,const SpeedNode& next) const
{
    double dt = current.t - parent.t;
    double acc = (next.s - 2 * current.s + parent.s) / (dt * dt);
    double max_acc = 4.0;
    double ration = acc / max_acc;
    return 30.0 * ration * ration;
}

double  DpStCost::GetJerkCost(const SpeedNode& grand_parent,const SpeedNode& parent,
                              const SpeedNode& current,const SpeedNode& next) const
{
    double dt = current.t - parent.t;
    double jerk = (next.s - 3 * current.s + 3 * parent.s - grand_parent.s) / (dt * dt * dt);
    double max_jerk = 8.0;
    double ration = jerk / max_jerk;
    return 50.0 * ration * ration;
}