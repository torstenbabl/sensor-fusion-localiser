#ifndef __SFL_UTILITIES_READVOTRANSFORMSFROMCSV_H__
#define __SFL_UTILITIES_READVOTRANSFORMSFROMCSV_H__

#include <vector>
#include <filesystem>

#include "sfl/types/RosTimedData.h"
#include "sfl/types/Transform3D.h"

namespace sfl::utilities {
typedef types::RosTimedData<types::Transform3D> RosTimedT3D;

/// @brief Read Visual Odometry transforms from a CSV file and place into vector
/// @returns Vector of Visual Odometry samples including ROS timestamps
std::vector<RosTimedT3D> readVoTransformFromCsv(
  std::filesystem::path const csvFilePath,  ///< [in] Path of CSV file
  bool const ignoreFirstLine = true         ///< [in] Ignore first line if it contains field names
);

}

#endif // __SFL_UTILITIES_READVOTRANSFORMSFROMCSV_H__