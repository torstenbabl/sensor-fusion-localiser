#ifndef __SFL_TYPES_ROSTIMEDDATA_H__
#define __SFL_TYPES_ROSTIMEDDATA_H__

#include "sfl/types/RosTime.h"

namespace sfl::types {
  template<typename DataType>
  struct RosTimedData {
    RosTime time;
    DataType data;
  };
}

#endif // __SFL_TYPES_ROSTIMEDDATA_H__