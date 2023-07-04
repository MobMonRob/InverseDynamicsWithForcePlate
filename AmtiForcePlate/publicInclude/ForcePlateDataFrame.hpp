#pragma once

#include <cstdint>
#include <vector>

#include "ForcePlateData.hpp"

namespace Acquisition
{
    class ForcePlateDataFrame;
}

class Acquisition::ForcePlateDataFrame
{
public:
    ForcePlateDataFrame(long frameNumber, std::vector<ForcePlateData> &&forcePlateDataVector);
    ForcePlateDataFrame(ForcePlateDataFrame &&) = default;
    ForcePlateDataFrame(const ForcePlateDataFrame &) = delete;

    long frameNumber;
    std::vector<ForcePlateData> forcePlateDataVector;
};
