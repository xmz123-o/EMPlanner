#include "common/CoordiTrans.h"
#include <cmath>
#include <limits>
#include <algorithm>


FrenetPoint CoordinateTransform::CartesianToFrenet(double x, double y,const std::vector<ReferencePoint>& ref_line)
{
    FrenetPoint frenet_pt;

    if(ref_line.size() < 2)
    {
        frenet_pt.s = 0.0;
        frenet_pt.l = 0.0;
        return frenet_pt;
    }

    double min_dist = std::numeric_limits<double>::max();

    int best_idx = 0;

    double best_ratio = 0.0;
    double best_proj_x = 0.0;
    double best_proj_y = 0.0;

    for(size_t i = 0; i < ref_line.size() - 1; ++i)
    {
        double x1 = ref_line[i].x;
        double y1 = ref_line[i].y;

        double x2 = ref_line[i + 1].x;
        double y2 = ref_line[i + 1].y;

        double vx = x2 - x1;
        double vy = y2 - y1;

        double wx = x - x1;
        double wy = y - y1;

        double seg_len2 = vx * vx + vy * vy;

        if(seg_len2 < 1e-6)
        {
            continue;
        }

        double ratio = (wx * vx + wy * vy) / seg_len2;

        ratio = std::max(0.0, std::min(1.0, ratio));

        double proj_x = x1 + ratio * vx;
        double proj_y = y1 + ratio * vy;

        double dist = std::hypot(x - proj_x, y - proj_y);

        if(dist < min_dist)
        {
            min_dist = dist;

            best_idx = static_cast<int>(i);

            best_ratio = ratio;

            best_proj_x = proj_x;
            best_proj_y = proj_y;
        }
    }

    double s = ref_line[best_idx].s + best_ratio * (ref_line[best_idx + 1].s - ref_line[best_idx].s);

    double theta = ref_line[best_idx].theta + best_ratio * (ref_line[best_idx + 1].theta - ref_line[best_idx].theta);


    double dx = x - best_proj_x;
    double dy = y - best_proj_y;

    double l = -dx * std::sin(theta) + dy * std::cos(theta);

    frenet_pt.s = s;
    frenet_pt.l = l;

    return frenet_pt;
}

void CoordinateTransform::FrenetToCartesian(double s, double l, const std::vector<ReferencePoint>& ref_line,
                                            double& x, double& y)
{
    double min_s_diff = std::numeric_limits<double>::max();

    int nearest_index = 0;

    // find nearest s
    for(size_t i = 0; i < ref_line.size(); ++i)
    {
        double ds = std::abs(ref_line[i].s - s);

        if(ds < min_s_diff)
        {
            min_s_diff = ds;
            nearest_index = i;
        }
    }

    const auto& ref_pt = ref_line[nearest_index];

    x = ref_pt.x - l * std::sin(ref_pt.theta);
    y = ref_pt.y + l * std::cos(ref_pt.theta);
}

ObstacleFrenet CoordinateTransform::ObstacleToFrenet(const Obstacle& obstacle, const std::vector<ReferencePoint>& reference_line)
{
    FrenetPoint frenet_pt = CartesianToFrenet(obstacle.x, obstacle.y, reference_line);
    return ObstacleFrenet(frenet_pt.s, frenet_pt.l, obstacle.radius);
}