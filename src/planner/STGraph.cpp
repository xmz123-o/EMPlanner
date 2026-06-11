#include "planner/STGraph.h"
#include <fstream>
#include <iostream>
#include <cmath>

void STGraph::SetReferenceLine(const std::vector<ReferencePoint>& ref_line) {
    ref_line_ = ref_line;
}

void STGraph::SetTimeConfig(double total_time, double dt) {
    total_time_ = total_time;
    dt_ = dt;
}

STBoundary STGraph::GenerateFromPrediction(const std::vector<PredictedObstaclePoint>& prediction,
                                           const DynamicObstacle& obs,int obs_id)
{
    STBoundary boundary;

    boundary.id = obs_id;

    for(const auto& pt : prediction)
    {
        STPoint lower;
        STPoint upper;

        lower.t = pt.t;
        upper.t = pt.t;

        lower.s = pt.s - obs.length * 0.5;

        upper.s = pt.s + obs.length * 0.5;

        boundary.lower_points.push_back(lower);
        boundary.upper_points.push_back(upper);
    }

    for(const auto& pt : prediction)
    {
        std::cout
            << "t="
            << pt.t
            << " s="
            << pt.s
            << std::endl;
    }

    return boundary;
}

STBoundary STGraph::GenerateSTBoundary(const DynamicObstacle& obs,int obs_id) 
{
    auto prediction = predictor_.Predict(obs,ref_line_,total_time_,dt_);

    return GenerateFromPrediction(prediction,obs,obs_id);
}


void STGraph::SaveSTObstacles(const std::vector<STBoundary>& boundaries, const std::string& filename)
{
    std::ofstream file(filename);

    file << "obs_id,t,s_lower,s_upper\n";

    for(const auto& boundary : boundaries)
    {
        for(size_t i = 0; i < boundary.lower_points.size(); ++i)
        {
            file
                << boundary.id << ","
                << boundary.lower_points[i].t << ","
                << boundary.lower_points[i].s << ","
                << boundary.upper_points[i].s
                << "\n";
        }
    }

    file.close();
}

