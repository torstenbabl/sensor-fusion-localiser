#ifndef __SFL_TYPES_ROSTIME_H__
#define __SFL_TYPES_ROSTIME_H__

namespace sfl::types {
  struct RosTime {
    unsigned long seconds;
    unsigned long nanoseconds;
  };
}

#endif // __SFL_TYPES_ROSTIME_H__