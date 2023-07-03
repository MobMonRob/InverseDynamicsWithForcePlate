#pragma once

#include <cstdint>

namespace Acquisition
{
    class ForcePlateData;
}

class Acquisition::ForcePlateData
{
public:
    ForcePlateData();
    ForcePlateData(double fX, double fY, double fZ, double mX, double mY, double mZ);
    ForcePlateData(ForcePlateData &&) = default;
    // ForcePlateData(const ForcePlateData &) = default;

    double fX;
    double fY;
    double fZ;
    double mX;
    double mY;
    double mZ;
};
