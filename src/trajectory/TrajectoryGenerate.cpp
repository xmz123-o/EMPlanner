#include "trajectory/TrajectoryGenerate.h"
#include <cmath>
#include <iostream>


double TrajectoryGenerate::InterpolateL(double query_s, const std::vector<PathQPPoint>& path)
{
    if(path.empty()) {
        return 0.0;
    }

    if(query_s <= path.front().s) {
        return path.front().l;
    }

    if(query_s >= path.back().s) {
        return path.back().l;
    }
    
    for(size_t i = 0; i < path.size() - 1; ++i) {
        if(query_s >= path[i].s && query_s <= path[i + 1].s) {
            double ration = (query_s - path[i].s) / (path[i + 1].s - path[i].s);
            return path[i].l + ration * (path[i + 1].l - path[i].l);
        }
    }

    return path.back().l;
}

double TrajectoryGenerate::InterpolateDL(double s, const std::vector<PathQPPoint>& path)
{
    if(s <= path.front().s)
        return path.front().dl;

    if(s >= path.back().s)
        return path.back().dl;

    for(size_t i=0;i<path.size()-1;i++)
    {
        const auto& p0 = path[i];
        const auto& p1 = path[i+1];

        if(s >= p0.s && s <= p1.s)
        {
            double ratio = (s - p0.s) / (p1.s - p0.s);

            return p0.dl + ratio * (p1.dl - p0.dl);
        }
    }

    return path.back().dl;
}

double TrajectoryGenerate::InterpolateDDL(double s, const std::vector<PathQPPoint>& path)
{
    if(s <= path.front().s)
        return path.front().ddl;

    if(s >= path.back().s)
        return path.back().ddl;

    for(size_t i=0;i<path.size()-1;i++)
    {
        const auto& p0 = path[i];
        const auto& p1 = path[i+1];

        if(s >= p0.s && s <= p1.s)
        {
            double ratio = (s - p0.s) / (p1.s - p0.s);

            return p0.ddl + ratio * (p1.ddl - p0.ddl);
        }
    }

    return path.back().ddl;
}

ReferencePoint TrajectoryGenerate::InterpolateReferencePoint(double query_s, const std::vector<ReferencePoint>& refline)
{   if(refline.empty())
    return {};

    if(query_s <= refline.front().s)
    {
        return refline.front();
    }

    if(query_s >= refline.back().s)
    {
        return refline.back();
    }

    for(size_t i = 0; i < refline.size()-1; ++i) {
        if(query_s >= refline[i].s && query_s <= refline[i + 1].s) {
            double ration = (query_s - refline[i].s) / (refline[i + 1].s - refline[i].s);
            ReferencePoint ref_pt;
            ref_pt.s = refline[i].s + ration * (refline[i + 1].s - refline[i].s);
            ref_pt.x = refline[i].x + ration * (refline[i + 1].x - refline[i].x);
            ref_pt.y = refline[i].y + ration * (refline[i + 1].y - refline[i].y);
            ref_pt.theta = refline[i].theta + ration * (refline[i + 1].theta - refline[i].theta);
            return ref_pt;
        }
    }

    return refline.back();
}



std::vector<TrajectoryPoint> TrajectoryGenerate::TrajGenerate(const std::vector<PathQPPoint>& path,
                                              const std::vector<SpeedQPPoint>& speed,
                                              const std::vector<ReferencePoint>& reference_line)
{
    std::vector<TrajectoryPoint> trajectory;

    double start_s = path.front().s;

    for(const auto& sp : speed)
    {
        double global_s = start_s + sp.s;

        double l   = InterpolateL(global_s,path);

        double dl  = InterpolateDL(global_s,path);

        double ddl = InterpolateDDL(global_s,path);

        auto ref_pt = InterpolateReferencePoint( global_s, reference_line);

        double x = ref_pt.x - l * std::sin(ref_pt.theta);
        double y = ref_pt.y + l * std::cos(ref_pt.theta);

        TrajectoryPoint traj_pt;
        traj_pt.t = sp.t;
        traj_pt.s = global_s;
        traj_pt.l = l;
        traj_pt.dl = dl;
        traj_pt.ddl = ddl;
        traj_pt.x = x;
        traj_pt.y = y;
        traj_pt.v = sp.v;
        traj_pt.a = sp.a;

        trajectory.push_back(traj_pt);
    }

    return trajectory;
}