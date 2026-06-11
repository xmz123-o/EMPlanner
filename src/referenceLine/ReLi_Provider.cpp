#include "referenceLine/ReLi_Provider.h"
#include <fstream>
#include <sstream>
#include <cmath>

std::vector<ReferencePoint>
ReferenceLineProvider::GenerateSineCurve()
{
    std::vector<ReferencePoint> ref_line;

    double accumulated_s = 0.0;

    for(int i = 0; i < 100; ++i)
    {
        double x = i * 2.0;
        double y = 2.0 * std::sin(0.03 * x);

        ReferencePoint pt;
        pt.x = x;
        pt.y = y;

        if(i > 0)
        {
            auto& prev = ref_line.back();

            double dx = x - prev.x;
            double dy = y - prev.y;

            double ds = std::hypot(dx, dy);

            accumulated_s += ds;

            pt.theta = std::atan2(dy, dx);
            pt.s = accumulated_s;
        }
        else
        {
            pt.theta = 0.0;
            pt.s = 0.0;
        }

        pt.kappa = 0.0;
        ref_line.push_back(pt);
    }

    return ref_line;
}

std::vector<ReferencePoint> ReferenceLineProvider::LoadFromCSV(const std::string& file_path)
{
    std::vector<ReferencePoint> ref_line;

    std::ifstream file(file_path);
    std::string line;
    std::getline(file, line);

    double accumulated_s = 0.0;

    while(std::getline(file, line))
    {
        std::stringstream ss(line);

        std::string x_str;
        std::string y_str;

        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');

        ReferencePoint pt;

        pt.x = std::stod(x_str);
        pt.y = std::stod(y_str);

        if(!ref_line.empty())
        {
            auto& prev = ref_line.back();

            double dx = pt.x - prev.x;
            double dy = pt.y - prev.y;

            double ds = std::hypot(dx, dy);

            accumulated_s += ds;

            pt.theta = std::atan2(dy, dx);

            pt.s = accumulated_s;
        }
        else
        {
            pt.theta = 0.0;
            pt.s = 0.0;
        }

        pt.kappa = 0.0;

        ref_line.push_back(pt);
    }

    return ref_line;
}