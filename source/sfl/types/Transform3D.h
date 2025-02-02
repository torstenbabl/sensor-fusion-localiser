#ifndef __SFL_TYPES_TRANSFORM3D_H__
#define __SFL_TYPES_TRANSFORM3D_H__

#include "sfl/types/MetricUnits.h"
#include "sfl/types/Quarternion.h"

namespace sfl::types {
  struct Transform3D {
    Meters translation[3];
    Quarternion rotation;
  };
}

#endif // __SFL_TYPES_TRANSFORM3D_H__