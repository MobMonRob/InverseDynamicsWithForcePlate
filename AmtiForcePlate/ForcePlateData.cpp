#include "ForcePlateData.hpp"

using namespace Acquisition;

ForcePlateData::ForcePlateData()
    : frameNumber(0), subsampleNumber(0), fX(0), fY(0), fZ(0), mX(0), mY(0), mZ(0)
{
}

ForcePlateData::ForcePlateData(uint64_t frameNumber, uint64_t subsampleNumber, double fX, double fY, double fZ, double mX, double mY, double mZ)
    : frameNumber(frameNumber), subsampleNumber(subsampleNumber), fX(fX), fY(fY), fZ(fZ), mX(mX), mY(mY), mZ(mZ)
{
}
