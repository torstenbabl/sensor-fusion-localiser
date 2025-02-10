#include "sfl/utilities/readRosTimedGpsFromCsv.h"
#include <fstream>
#include <sstream>
#include <string>

namespace fs = std::filesystem;

namespace sfl::utilities {
std::vector<RosTimedGps> readRosTimedGpsFromCsv(
  std::filesystem::path const csvFilePath,
  bool const ignoreFirstLine
)
{
  std::vector<RosTimedGps> outputVector;

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

  // sec,nanosec,latitude,longitude,altitude,position_covariance,position_covariance_type

  while (std::getline(file, line))
  {
    RosTimedGps gpsReading;
    std::stringstream ss(line);

    std::string value;
    std::getline(ss, value, ',');
    gpsReading.time.seconds = std::stoul(value);
    std::getline(ss, value, ',');
    gpsReading.time.nanoseconds = std::stoul(value);

    std::getline(ss, value, ',');
    gpsReading.data.latitude = stod(value);
    std::getline(ss, value, ',');
    gpsReading.data.longitude = stod(value);
    std::getline(ss, value, ',');
    gpsReading.data.heightAboveSpheriod = stod(value);

    outputVector.push_back(gpsReading);
  }

  file.close();

  return outputVector;
}

}