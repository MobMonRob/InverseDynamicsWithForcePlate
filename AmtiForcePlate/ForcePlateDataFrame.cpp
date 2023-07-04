#include "ForcePlateDataFrame.hpp"

using namespace Acquisition;

ForcePlateDataFrame::ForcePlateDataFrame(long frameNumber, std::vector<ForcePlateData> &&forcePlateDataVector)
    : frameNumber(frameNumber), forcePlateDataVector(std::move(forcePlateDataVector))
{
}
