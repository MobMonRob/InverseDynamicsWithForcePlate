#pragma once

#include <cstdint>

namespace Acquisition
{
    class ForcePlateData;
}

class Acquisition::ForcePlateData
{
public:
    ForcePlateData() = default;
    ForcePlateData(double fX, double fY, double fZ, double mX, double mY, double mZ);
    ForcePlateData(ForcePlateData &&) = default;
    ForcePlateData(const ForcePlateData &) = delete;

    double fX;
    double fY;
    double fZ;
    double mX;
    double mY;
    double mZ;
};
