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
                                            const DynamicObstacle& obs, int obs_id)
{
    STBoundary boundary;

    boundary.id = obs_id;

    constexpr double lane_half_width = 3.0;
    constexpr double vehicle_half_width = 1.0;
    constexpr double lateral_buffer = 0.5;

    const double l_threshold =
        lane_half_width +
        vehicle_half_width +
        lateral_buffer;

    for(const auto& pt : prediction)
    {
        STPoint lower;
        STPoint upper;

        lower.t = pt.t;
        upper.t = pt.t;

        //----------------------------------
        // 不占据车道
        //----------------------------------
        if(std::fabs(pt.l) > l_threshold)
        {
            lower.s = -10000.0;
            upper.s = -10000.0;
        }
        else
        {
            lower.s = pt.s - obs.length * 0.5;
            upper.s = pt.s + obs.length * 0.5;
        }

        boundary.lower_points.push_back(lower);
        boundary.upper_points.push_back(upper);
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

