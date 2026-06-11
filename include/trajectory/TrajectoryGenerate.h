#ifndef TRAJECTORYGENERATE_H
#define TRAJECTORYGENERATE_H
#include "common/PathQPPoint.h"
#include "common/SpeedQPPoint.h"
#include "common/TrajectoryPoint.h"
#include "common/ReferencePoint.h"
#include <vector>

class TrajectoryGenerate
{
public:
    std::vector<TrajectoryPoint> TrajGenerate(const std::vector<PathQPPoint>& path,
                                              const std::vector<SpeedQPPoint>& speed,
                                              const std::vector<ReferencePoint>& reference_line);
    
    double InterpolateL(double query_s, const std::vector<PathQPPoint>& path);

    double InterpolateDL(double s, const std::vector<PathQPPoint>& path);

    double InterpolateDDL(double s, const std::vector<PathQPPoint>& path);

    ReferencePoint InterpolateReferencePoint( double query_s,const std::vector<ReferencePoint>& refline);
};

#endif