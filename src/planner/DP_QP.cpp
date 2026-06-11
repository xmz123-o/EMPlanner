#include "planner/FrenetQP.h"
#include "planner/FrenetDP.h"
#include "referenceLine/ReLi_Provider.h"
#include "common/CoordiTrans.h"

#include <fstream>
#include <iostream>
#include <cmath>

int main()
{
    // =========================
    // Reference Line
    // =========================
    ReferenceLineProvider ref_line_provider;
    auto reference_line = ref_line_provider.GenerateSineCurve();

    // =========================
    // Obstacles
    // =========================
    std::vector<Obstacle> obstacles =
    {
        {50.0,  0.4, 1.0},
        {60.0,  1.5, 1.0},
        {90.0, -0.3, 1.0},
        //{120.0, 1.6, 1.0},
        {140.0,-1.0, 1.0}
    };

    std::vector<ObstacleFrenet> obstacles_frenet;

    for(const auto& obs : obstacles)
    {
        obstacles_frenet.push_back(
            CoordinateTransform::ObstacleToFrenet(
                obs,
                reference_line));
    }

    // =========================
    // Path Boundary
    // =========================
    PathBoundaryGenerator boundary_gen;

    auto path_boundary =
        boundary_gen.Generate(
            reference_line,
            obstacles_frenet);

    if(path_boundary.empty())
    {
        std::cout
            << "[DP_QP] Boundary generation failed!"
            << std::endl;
        return 0;
    }

    // =========================
    // DP Search
    // =========================
    double init_l = 0.0;

    FrenetDP dp;

    auto dp_path =
        dp.DP_Search(
            reference_line,
            path_boundary,
            obstacles,
            init_l);

    if(dp_path.empty())
    {
        std::cout
            << "[DP_QP] DP failed!"
            << std::endl;
        return 0;
    }

    // =========================
    // Build QP Reference
    // =========================
    std::vector<double> l_ref;

    for(const auto& node : dp_path)
    {
        l_ref.push_back(node.l);
    }

    // =========================
    // QP Optimize
    // =========================
    FrenetState init_state;

    init_state.l = 0.0;
    init_state.dl = 0.0;
    init_state.ddl = 0.0;

    FrenetQP qp;

    auto qp_path =
        qp.Solve(
            l_ref,
            path_boundary,
            init_state);

    if(qp_path.empty())
    {
        std::cout
            << "[DP_QP] QP failed!"
            << std::endl;
        return 0;
    }

    // =========================
    // Print Result
    // =========================
    std::cout
        << "\n===== QP Result ====="
        << std::endl;

    for(size_t i = 0; i < qp_path.size(); ++i)
    {
        std::cout
            << "s="
            << qp_path[i].s
            << "  l="
            << qp_path[i].l
            << "  dl="
            << qp_path[i].dl
            << "  ddl="
            << qp_path[i].ddl
            << std::endl;
    }

    // =========================
    // DP Path
    // =========================
    std::ofstream dp_file(
        "../output/dp_path.csv");

    dp_file << "x,y,s,l\n";

    for(const auto& node : dp_path)
    {
        dp_file
            << node.x << ","
            << node.y << ","
            << node.s << ","
            << node.l
            << "\n";
    }

    dp_file.close();

    // =========================
    // QP Path
    // =========================
    std::ofstream qp_file(
        "../output/qp_path.csv");

    qp_file
        << "x,y,s,l,dl,ddl\n";

    size_t N =
        std::min(
            qp_path.size(),
            reference_line.size());

    for(size_t i = 0; i < N; ++i)
    {
        double ref_x =
            reference_line[i].x;

        double ref_y =
            reference_line[i].y;

        double theta =
            reference_line[i].theta;

        double l =
            qp_path[i].l;

        double x =
            ref_x - l * std::sin(theta);

        double y =
            ref_y + l * std::cos(theta);

        qp_file
            << x << ","
            << y << ","
            << qp_path[i].s << ","
            << qp_path[i].l << ","
            << qp_path[i].dl << ","
            << qp_path[i].ddl
            << "\n";
    }

    qp_file.close();

    // =========================
    // Obstacles
    // =========================
    std::ofstream obs_file(
        "../output/obstacles.csv");

    obs_file
        << "x,y,s,l\n";

    for(size_t i = 0;
        i < obstacles.size();
        ++i)
    {
        obs_file
            << obstacles[i].x << ","
            << obstacles[i].y << ","
            << obstacles_frenet[i].s << ","
            << obstacles_frenet[i].l
            << "\n";
    }

    obs_file.close();

    // =========================
    // Boundary
    // =========================
    std::ofstream boundary_file(
        "../output/boundary.csv");

    boundary_file
        << "s,l_min,l_max\n";

    for(const auto& pt : path_boundary)
    {
        boundary_file
            << pt.s << ","
            << pt.l_min << ","
            << pt.l_max
            << "\n";
    }

    boundary_file.close();

    // =========================
    // Reference Line
    // =========================
    std::ofstream ref_file(
        "../output/reference_line.csv");

    ref_file
        << "x,y,s\n";

    for(const auto& pt : reference_line)
    {
        ref_file
            << pt.x << ","
            << pt.y << ","
            << pt.s
            << "\n";
    }

    ref_file.close();

    std::cout
        << "\nAll files generated."
        << std::endl;

    return 0;
}