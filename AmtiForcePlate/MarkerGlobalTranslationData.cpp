#include "MarkerGlobalTranslationData.hpp"

using namespace Acquisition;

MarkerGlobalTranslationData::MarkerGlobalTranslationData(double x, double y, double z, bool occluded, const std::string& markerName)
    : x(x), y(y), z(z), occluded(occluded), markerName(markerName)
{

}
