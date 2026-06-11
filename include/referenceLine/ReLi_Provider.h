#ifndef RELI_PROVIDER_H
#define RELI_PROVIDER_H

#include <vector>
#include <string>
#include "common/ReferencePoint.h"

class ReferenceLineProvider
{
public:

    // static std::vector<ReferencePoint>
    // GenerateStraightLine();

    static std::vector<ReferencePoint>
    GenerateSineCurve();

    static std::vector<ReferencePoint>
    LoadFromCSV(
        const std::string& file_path);
};

#endif