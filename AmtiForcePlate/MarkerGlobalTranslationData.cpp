#include "MarkerGlobalTranslationData.hpp"

using namespace Acquisition;

MarkerGlobalTranslationData::MarkerGlobalTranslationData(bool occluded, double x, double y, double z)
    : occluded(occluded), x(x), y(y), z(z)
{
}
