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
    ForcePlateData(uint64_t frameNumber, uint64_t subsampleNumber, double fX, double fY, double fZ, double mX, double mY, double mZ);
    ForcePlateData(Acquisition::ForcePlateData &&) = default;
    ForcePlateData(const Acquisition::ForcePlateData &) = default;

    const uint64_t frameNumber;
    const uint64_t subsampleNumber;
    const double fX;
    const double fY;
    const double fZ;
    const double mX;
    const double mY;
    const double mZ;
};
