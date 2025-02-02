#ifndef __SFL_TYPES_GPSREADING_H__
#define __SFL_TYPES_GPSREADING_H__

#include "sfl/types/MetricUnits.h"

namespace sfl::types {
  struct GpsReading {
    Degrees longitude;
    Degrees latitude;
    Meters heightAboveSpheriod;
  };
}

#endif // __SFL_TYPES_GPSREADING_H__