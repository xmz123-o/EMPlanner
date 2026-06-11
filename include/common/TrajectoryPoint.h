#ifndef     TRAJECTORYPOINT_H
#define     TRAJECTORYPOINT_H

struct TrajectoryPoint
{
    double t = 0.0;

    double s = 0.0;
    double l = 0.0;

    double x = 0.0;
    double y = 0.0;

    double theta = 0.0;

    double v = 0.0;
    double a = 0.0;

    double dl = 0.0;
    double ddl = 0.0;
};

#endif