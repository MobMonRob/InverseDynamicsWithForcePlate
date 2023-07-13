#pragma once

#include <string>

namespace Acquisition
{
    class MarkerGlobalTranslationData;
}

class Acquisition::MarkerGlobalTranslationData
{
public:
    MarkerGlobalTranslationData() = default;
    MarkerGlobalTranslationData(MarkerGlobalTranslationData &&) = default;
    MarkerGlobalTranslationData(const MarkerGlobalTranslationData &) = delete;
    MarkerGlobalTranslationData(bool occluded, double x, double y, double z);

    bool occluded;
    double x;
    double y;
    double z;
};
