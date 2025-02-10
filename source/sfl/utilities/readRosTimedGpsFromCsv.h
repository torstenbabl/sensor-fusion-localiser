#ifndef __SFL_UTILITIES_READROSTIMEDGPSFROMCSV_H__
#define __SFL_UTILITIES_READROSTIMEDGPSFROMCSV_H__

#include <vector>
#include <filesystem>

#include "sfl/types/RosTimedData.h"
#include "sfl/types/GpsReading.h"

namespace sfl::utilities {
typedef types::RosTimedData<types::GpsReading> RosTimedGps;

/// @brief Read GPS reading and place into vector
/// @returns Vector of GPS readings including ROS timestamps
std::vector<RosTimedGps> readRosTimedGpsFromCsv(
  std::filesystem::path const csvFilePath,  ///< [in] path of CSV file
  bool const ignoreFirstLine = true         ///< [in] Ignore first line if it contains field names
);

}

#endif // __SFL_UTILITIES_READROSTIMEDGPSFROMCSV_H__