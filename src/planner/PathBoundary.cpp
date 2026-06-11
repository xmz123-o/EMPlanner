#include "planner/PathBoundary.h"

std::vector<PathBoundaryPoint> PathBoundaryGenerator::Generate(
    const std::vector<ReferencePoint>& reference_line,
    const std::vector<ObstacleFrenet>& obstacles_frenet)
{
    std::vector<PathBoundaryPoint> path_boundary;
    const double default_l_min = -3.0;
    const double default_l_max = 3.0;
    const double safe_gap = 0.2; // 最小可行宽度

    for(size_t i = 0; i < reference_line.size(); ++i)
    {
        double s = reference_line[i].s;
        double l_min = default_l_min;
        double l_max = default_l_max;

        for(const auto& obs : obstacles_frenet)
        {
            double obs_s = obs.s; 
            double obs_l = obs.l; 

            if(std::abs(obs_s - s) < 5.0) // Check if obstacle is within 5m 
            {
                if(obs_l > 0.0) // right
                {
                    l_max = std::min(l_max, obs_l - obs.radius); 
                }
                else // left
                {
                    l_min = std::max(l_min, obs_l + obs.radius); 
                }
            }
        }

        // 保证最小宽度
        if(l_min > l_max - safe_gap)
        {
            l_min = l_max - safe_gap;
        }

        path_boundary.emplace_back(s, l_min, l_max);

        // 调试打印
        // std::cout << i << " s=" << s << " l_min=" << l_min << " l_max=" << l_max << std::endl;
    }

    return path_boundary;
}