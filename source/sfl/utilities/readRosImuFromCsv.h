#ifndef __READROSIMUFROMCSV_H__
#define __READROSIMUFROMCSV_H__

#include <vector>
#include <filesystem>

#include "sfl/types/RosTimedData.h"
#include "sfl/types/ImuSample.h"

namespace sfl::utilities {
typedef types::RosTimedData<types::ImuSample> RosTimedImu;

/// @brief Read IMU samples and place into vector
/// @returns Vector of IMU samples including ROS timestamps
std::vector<RosTimedImu> readRosImuFromCsv(
  std::filesystem::path const csvFilePath,  ///< [in] path of CSV file
  bool const ignoreFirstLine = true         ///< [in] Ignore first line if it contains field names
);

}

#endif // __READROSIMUFROMCSV_H__