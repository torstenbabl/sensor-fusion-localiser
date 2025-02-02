/// @brief Simple program to test the build system
/// @author Torsten Babl

#include <iostream>
#include "sfl/utilities/readRosImuFromCsv.h"

using sfl::utilities::readRosImuFromCsv;
using sfl::utilities::RosTimedImu;

int main()
{
  
  std::vector<RosTimedImu> imuSamples = readRosImuFromCsv("/home/torsten/Documents/masters/citrus_data_set/04_13D_Jackal/imu0.csv");

  std::cout << "Number of imu samples: " << imuSamples.size() << std::endl;

  std::cout << "[11]av_z: " << imuSamples.at(11).data.angularVelocity[2] << std::endl;

  return 0;
}