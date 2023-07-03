#include "ForcePlateData.hpp"

using namespace Acquisition;

ForcePlateData::ForcePlateData()
    : fX(0), fY(0), fZ(0), mX(0), mY(0), mZ(0)
{
}

ForcePlateData::ForcePlateData(double fX, double fY, double fZ, double mX, double mY, double mZ)
    : fX(fX), fY(fY), fZ(fZ), mX(mX), mY(mY), mZ(mZ)
{
}
