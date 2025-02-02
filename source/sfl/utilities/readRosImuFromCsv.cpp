#include "sfl/utilities/readRosImuFromCsv.h"
#include <fstream>
#include <sstream>
#include <string>

namespace fs = std::filesystem;

namespace sfl::utilities {
std::vector<RosTimedImu> readRosImuFromCsv(
  std::filesystem::path const csvFilePath,
  bool const ignoreFirstLine
)
{
  std::vector<RosTimedImu> outputVector;

  // Check if given path exists and is a file.
  if(!fs::exists(csvFilePath) || !fs::is_regular_file(csvFilePath))
  {
    return outputVector;
  }

  std::ifstream file(csvFilePath);

  if (!file.is_open()) {
    return  outputVector;
  }

  std::string line;

  // Pop first line if required
  if (ignoreFirstLine)
  {
    std::getline(file, line);
  }

  while (std::getline(file, line))
  {
    RosTimedImu imuItem;
    std::stringstream ss(line);

    std::string value;
    std::getline(ss, value, ',');
    imuItem.time.seconds = std::stoul(value);
    std::getline(ss, value, ',');
    imuItem.time.nanoseconds = std::stoul(value);

    for (uint i = 0; i < 3; i++)
    {
      std::getline(ss, value, ',');
      imuItem.data.acceleration[i] = std::stod(value);
    }

    for (uint i = 0; i < 3; i++)
    {
      std::getline(ss, value, ',');
      imuItem.data.angularVelocity[i] = std::stod(value);
    }

    outputVector.push_back(imuItem);
  }

  file.close();

  return outputVector;
}

}