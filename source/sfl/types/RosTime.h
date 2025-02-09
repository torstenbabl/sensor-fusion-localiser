#ifndef __SFL_TYPES_ROSTIME_H__
#define __SFL_TYPES_ROSTIME_H__

namespace sfl::types {
  struct RosTime {
    unsigned long seconds;
    unsigned long nanoseconds;

    bool operator<(RosTime const& other)
    {
      if (seconds < other.seconds)
      {
        return true;
      }

      if (seconds > other.seconds)
      {
        return false;
      }

      return nanoseconds < other.nanoseconds;
    }

    bool operator==(RosTime const& other)
    {
      return (seconds == other.seconds) && (nanoseconds == other.nanoseconds);
    }

    bool operator<=(RosTime const& other)
    {
      return operator<(other) || operator==(other);
    }

    bool operator>(RosTime const& other)
    {
      return !operator<=(other);
    }

    bool operator>=(RosTime const& other)
    {
      return !operator<(other);
    }
  };
  /// @todo Add time comparison method
}

#endif // __SFL_TYPES_ROSTIME_H__