/// @brief Simple program to test the build system
/// @author Torsten Babl

#include <iostream>
#include "sfl/utilities/readVoTransformsFromCsv.h"

using sfl::utilities::readVoTransformFromCsv;
using sfl::utilities::RosTimedT3D;

int main()
{
  
  std::vector<RosTimedT3D> imuSamples = readVoTransformFromCsv("/home/torsten/Documents/masters/citrus_data_set/04_13D_Jackal/vo.csv");

  std::cout << "Number of VO transforms: " << imuSamples.size() << std::endl;

  std::cout << "[1]k: " << imuSamples.at(1).data.rotation.k << std::endl;

  return 0;
}