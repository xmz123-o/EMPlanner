#ifndef EGOSTATE_H
#define EGOSTATE_H

struct EgoState
{
    double x;
    double y;

    double s;
    double l;

    double dl;
    double ddl;

    double v;
    double a;
};

#endif