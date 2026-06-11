#include "trajectory/TrajectoryGenerate.h"

#include "planner/FrenetDP.h"
#include "planner/FrenetQP.h"

#include "planner/SpeedDP.h"
#include "planner/SpeedQP.h"

#include "planner/STGraph.h"

#include "referenceLine/ReLi_Provider.h"

#include "common/CoordiTrans.h"
#include "common/ObstaclePredictor.h"

#include <fstream>
#include <iostream>

int main()
{
    /************************************************
     * Reference Line
     ************************************************/
    ReferenceLineProvider ref_provider;

    auto reference_line =
        ref_provider.GenerateSineCurve();

    std::ofstream ref_file("../output/ref_line.csv");

    ref_file << "s,x,y\n";

    for(const auto& pt : reference_line)
    {
        ref_file
            << pt.s << ","
            << pt.x << ","
            << pt.y << "\n";
    }

    ref_file.close();

    /************************************************
     * Static Obstacles
     ************************************************/
    std::vector<Obstacle> obstacles =
    {
        {60.0,1.5,1.0},
        {90.0,-0.5,1.0},
        {140.0,-1.0,1.0}
    };

    std::ofstream obs_file("../output/obstacles.csv");

    obs_file << "x,y,r\n";

    for(const auto& obs : obstacles)
    {
        obs_file
            << obs.x << ","
            << obs.y << ","
            << obs.radius
            << "\n";
    }

    obs_file.close();

    /************************************************
     * Frenet Obstacle
     ************************************************/
    std::vector<ObstacleFrenet> obs_frenet;

    for(const auto& obs : obstacles)
    {
        obs_frenet.push_back(
            CoordinateTransform::ObstacleToFrenet(
                obs,
                reference_line));
    }

    /************************************************
     * Path Boundary
     ************************************************/
    PathBoundaryGenerator boundary_gen;

    auto path_boundary =
        boundary_gen.Generate(
            reference_line,
            obs_frenet);

    if(path_boundary.empty())
    {
        std::cout<<"Boundary failed\n";
        return 0;
    }

    /************************************************
     * Path DP
     ************************************************/
    FrenetDP dp;

    double start_l = 0.0;

    auto dp_path =
        dp.DP_Search(
            reference_line,
            path_boundary,
            obstacles,
            start_l);

    std::ofstream dp_file("../output/path_dp.csv");

    dp_file << "s,l,x,y\n";

    for(const auto& pt : dp_path)
    {
        dp_file
            << pt.s << ","
            << pt.l << ","
            << pt.x << ","
            << pt.y << "\n";
    }

    dp_file.close();

    if(dp_path.empty())
    {
        std::cout<<"DP failed\n";
        return 0;
    }

    /************************************************
     * Path QP
     ************************************************/
    std::vector<double> l_ref;

    for(const auto& node : dp_path)
    {
        l_ref.push_back(node.l);
    }

    FrenetState start_state;

    start_state.l = 0.0;
    start_state.dl = 0.0;
    start_state.ddl = 0.0;

    FrenetQP qp;

    auto path_qp_result =
        qp.Solve(
            l_ref,
            path_boundary,
            start_state);

    if(path_qp_result.empty())
    {
        std::cout<<"QP failed\n";
        return 0;
    }

    for(size_t i=0;i<path_qp_result.size();i++)
    {
        path_qp_result[i].s =
            reference_line[i].s;
    }

    std::ofstream qp_file("../output/path_qp.csv");

    qp_file << "s,l,x,y\n";

    TrajectoryGenerate traj_gen;

    for(const auto& pt : path_qp_result)
    {
        auto ref =
            traj_gen.InterpolateReferencePoint(
                pt.s,
                reference_line);

        double x =
            ref.x -
            pt.l * std::sin(ref.theta);

        double y =
            ref.y +
            pt.l * std::cos(ref.theta);

        qp_file
            << pt.s << ","
            << pt.l << ","
            << x << ","
            << y << "\n";
    }

    qp_file.close();

    /************************************************
     * Dynamic Obstacle
     ************************************************/
    DynamicObstacle dyn_obs;

    dyn_obs.x = 20.0;
    dyn_obs.y = 1.5;

    dyn_obs.vx = 1.0;
    dyn_obs.vy = 0.0;

    dyn_obs.length = 4.0;
    dyn_obs.width = 2.0;

    ObstaclePredictor predictor;

    double total_time = 10.0;
    double dt = 0.2;

    auto prediction =
        predictor.Predict(
            dyn_obs,
            reference_line,
            total_time,
            dt);

    STGraph st_graph;

    std::vector<STBoundary> st_boundaries;

    st_boundaries.push_back(
        st_graph.GenerateFromPrediction(
            prediction,
            dyn_obs,
            0));

    st_graph.SaveSTObstacles(
        st_boundaries,
        "../output/st_obstacles.csv");

    /************************************************
     * Speed DP
     ************************************************/
    SpeedDP speed_dp;

    double init_v = 10.0;

    auto speed_dp_path =
        speed_dp.SpeedDP_Search(
            st_boundaries,
            init_v);

    std::ofstream speed_dp_file(
        "../output/speed_dp.csv");

    speed_dp_file << "t,s\n";

    for(const auto& pt : speed_dp_path)
    {
        speed_dp_file
            << pt.t << ","
            << pt.s << "\n";
    }

    speed_dp_file.close();

    /************************************************
     * Speed QP
     ************************************************/
    SpeedQP speed_qp;

    auto speed_qp_s =
        speed_qp.QP_Optimize(
            speed_dp_path);

    std::ofstream speed_qp_file(
        "../output/speed_qp.csv");

    speed_qp_file << "t,s\n";

    for(size_t i=0;i<speed_qp_s.size();i++)
    {
        speed_qp_file
            << i*dt << ","
            << speed_qp_s[i]
            << "\n";
    }

    speed_qp_file.close();

    /************************************************
     * Build SpeedQPPoint
     ************************************************/
    std::vector<SpeedQPPoint> speed_qp_result;

    for(size_t i=0;i<speed_qp_s.size();i++)
    {
        SpeedQPPoint pt;

        pt.t = i * dt;

        pt.s = speed_qp_s[i];

        pt.v = 0.0;
        pt.a = 0.0;

        if(i < speed_qp_s.size()-1)
        {
            pt.v =
                (speed_qp_s[i+1]
                - speed_qp_s[i])
                / dt;
        }

        if(i>0 && i<speed_qp_s.size()-1)
        {
            pt.a =
                (speed_qp_s[i+1]
                -2.0*speed_qp_s[i]
                +speed_qp_s[i-1])
                /(dt*dt);
        }

        speed_qp_result.push_back(pt);
    }

    /************************************************
     * Trajectory
     ************************************************/
    auto trajectory =
        traj_gen.TrajGenerate(
            path_qp_result,
            speed_qp_result,
            reference_line);

    std::ofstream traj_file(
        "../output/trajectory.csv");

    traj_file
        << "t,s,l,x,y,v,a\n";

    for(const auto& pt : trajectory)
    {
        traj_file
            << pt.t << ","
            << pt.s << ","
            << pt.l << ","
            << pt.x << ","
            << pt.y << ","
            << pt.v << ","
            << pt.a << "\n";
    }

    traj_file.close();

    std::cout
        << "All test finished."
        << std::endl;

    return 0;
}