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
    MarkerGlobalTranslationData(double x, double y, double z, bool occluded, const std::string& markerName);

    double x;
    double y;
    double z;
    bool occluded;
    const std::string& markerName;
};
