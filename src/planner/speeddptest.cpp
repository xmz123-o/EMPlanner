#include "planner/STGraph.h"
#include "planner/SpeedDP.h"
#include "planner/SpeedQP.h"

#include <fstream>
#include <iostream>
#include <vector>

int main()
{
    //----------------------------------
    // Reference Line
    //----------------------------------

    std::vector<ReferencePoint> ref_line;

    for(int i=0;i<=200;i++)
    {
        ReferencePoint pt;

        pt.x = i;
        pt.y = 0.0;

        pt.s = i;
        pt.theta = 0.0;

        ref_line.push_back(pt);
    }

    //----------------------------------
    // Dynamic Obstacles
    //----------------------------------

    DynamicObstacle obs0;
    obs0.x = 20.0;
    obs0.y = 0.0;
    obs0.vx = 1.5;
    obs0.vy = 0.0;
    obs0.length = 4.0;
    obs0.width = 2.0;

    DynamicObstacle obs1;
    obs1.x = 45.0;
    obs1.y = 0.0;
    obs1.vx = 1.0;
    obs1.vy = 0.0;
    obs1.length = 5.0;
    obs1.width = 2.0;

    DynamicObstacle obs2;
    obs2.x = 65.0;
    obs2.y = 0.0;
    obs2.vx = -2.0;
    obs2.vy = 0.0;
    obs2.length = 2.0;
    obs2.width = 2.0;

    //----------------------------------
    // ST Graph
    //----------------------------------

    STGraph st_graph;

    st_graph.SetReferenceLine(ref_line);

    st_graph.SetTimeConfig(
        10.0,
        0.2);

    std::vector<STBoundary> boundaries;

    boundaries.push_back(
        st_graph.GenerateSTBoundary(
            obs0,
            0));

    boundaries.push_back(
        st_graph.GenerateSTBoundary(
            obs1,
            1));

    boundaries.push_back(
        st_graph.GenerateSTBoundary(
            obs2,
            2));

    //----------------------------------
    // Save ST
    //----------------------------------

    st_graph.SaveSTObstacles(
        boundaries,
        "../output/st_boundary.csv");

    //----------------------------------
    // Speed DP
    //----------------------------------

    SpeedDP speed_dp;

    double init_v = 5.0;

    auto dp_path =
        speed_dp.SpeedDP_Search(
            boundaries,
            init_v);

    //----------------------------------
    // Save DP
    //----------------------------------

    std::ofstream dp_file(
        "../output/speed_dp_path.csv");

    dp_file
    << "t,s,cost\n";

    for(const auto& p : dp_path)
    {
        dp_file
            << p.t << ","
            << p.s << ","
            << p.cost
            << "\n";
    }

    dp_file.close();

    //----------------------------------
    // Speed QP
    //----------------------------------

    SpeedQP speed_qp;

    auto qp_path =
        speed_qp.QP_Optimize(
            dp_path);

    //----------------------------------
    // Save QP
    //----------------------------------

    std::ofstream qp_file(
        "../output/speed_qp_path.csv");

    qp_file
    << "t,s,v,a\n";

    for(size_t i=0;i<qp_path.size();i++)
    {
        double v = 0.0;
        double a = 0.0;

        if(i>=1)
        {
            v =
                (qp_path[i]
                -qp_path[i-1])
                /0.2;
        }

        if(i>=2)
        {
            a =
                (qp_path[i]
                -2.0*qp_path[i-1]
                +qp_path[i-2])
                /(0.2*0.2);
        }

        qp_file
            << i*0.2 << ","
            << qp_path[i] << ","
            << v << ","
            << a
            << "\n";
    }

    qp_file.close();

    //----------------------------------
    // Print speed
    //----------------------------------

    std::cout
    << "\n===== SPEED ====="
    << std::endl;

    for(size_t i=1;
        i<std::min((size_t)20,qp_path.size());
        i++)
    {
        double v =
            (qp_path[i]
            -qp_path[i-1])
            /0.2;

        std::cout
            << "t="
            << i*0.2
            << " v="
            << v
            << std::endl;
    }

    return 0;
}