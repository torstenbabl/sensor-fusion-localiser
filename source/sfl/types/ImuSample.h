#ifndef __SFL_TYPES_IMUSAMPLE_H__
#define __SFL_TYPES_IMUSAMPLE_H__

#include "sfl/types/MetricUnits.h"

namespace sfl::types {
  struct ImuSample {
    MetersPerSecondSquared acceleration[3];
    RadiansPerSecond angularVelocity[3];
  };
}

#endif // __SFL_TYPES_IMUSAMPLE_H__