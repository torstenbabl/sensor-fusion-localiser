#include "sfl/utilities/readVoTransformsFromCsv.h"
#include <fstream>
#include <sstream>
#include <string>

namespace fs = std::filesystem;

namespace sfl::utilities {
std::vector<RosTimedT3D> readVoTransformFromCsv(
  std::filesystem::path const csvFilePath,
  bool const ignoreFirstLine
)
{
  std::vector<RosTimedT3D> outputVector;

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

  //sec,nanosec,t_x,t_y,t_z,r_x,r_y,r_z,r_w

  while (std::getline(file, line))
  {
    RosTimedT3D transform;
    std::stringstream ss(line);

    std::string value;
    std::getline(ss, value, ',');
    transform.time.seconds = std::stoul(value);
    std::getline(ss, value, ',');
    transform.time.nanoseconds = std::stoul(value);

    for (uint i = 0; i < 3; i++)
    {
      std::getline(ss, value, ',');
      transform.data.translation[i] = std::stod(value);
    }

    std::getline(ss, value, ',');
    transform.data.rotation.i = std::stod(value);
    std::getline(ss, value, ',');
    transform.data.rotation.j = std::stod(value);
    std::getline(ss, value, ',');
    transform.data.rotation.k = std::stod(value);
    std::getline(ss, value, ',');
    transform.data.rotation.w = std::stod(value);

    outputVector.push_back(transform);
  }

  file.close();

  return outputVector;
}

}