/// @brief Simple program to test the build system
/// @author Torsten Babl

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <optional>
#include <ostream>
#include <vector>
#include <string>
#include <filesystem>
#include <regex>
#include <algorithm>
#include <cstring>

#include "sfl/types/Quarternion.h"
#include "sfl/utilities/PathsFromRegex.h"
#include "sfl/lookups/CitrusDatasetZedCalibration.h"
#include "sfl/utilities/RotationMatrixToQuarternion.h"

#include "viso2/matrix.h"
#include "viso2/viso_stereo.h"

#include <gflags/gflags.h>

using std::filesystem::path;
using std::string;
using std::regex;
using std::vector;

using sfl::utilities::pathsFromRegex;
using sfl::utilities::rotationMatrixToQuaternion;
using sfl::lookups::CitrusDatasetZedCalibration;
using sfl::types::Quarternion;

using viso2::VisualOdometryStereo;

// Define command-line flags
DEFINE_string(pngdir, "/home/torsten/Documents/masters/citrus_data_set/04_13D_Jackal/png", "Full file path to the bag with images");

CitrusDatasetZedCalibration ZED_CALIBRATION;

std::optional<std::pair<uint, uint>> getTimeFromPngFilename(std::string filename);

//-----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
  // Parse command-line flags
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Enumerate right and left png channels
  // PngFileReaders for left and right channels
  // Init with directory and regex to match on all files
  // Provides a give next png
  path pngDir(FLAGS_pngdir);

  // Rectified images (according to dataset authors)
  vector<path> leftImages = pathsFromRegex(pngDir, regex(".*left.*png"));
  vector<path> rightImages = pathsFromRegex(pngDir, regex(".*right.*png"));

  std::sort(leftImages.begin(), leftImages.end());
  std::sort(rightImages.begin(), rightImages.end());

  std::cout << "Left images: " << leftImages.size() << " Right images: " << rightImages.size() << std::endl;

  for (uint i = 0; i < std::min(leftImages.size(), rightImages.size()); i++)
  {
    std::cout << leftImages.at(i).filename() << rightImages.at(i).filename() << std::endl;
  }

  // VO 
  VisualOdometryStereo::parameters params;
  params.calib.cu = 648;
  params.calib.cv = 357;
  params.calib.f = 529;
  params.base = 0.1198;
  VisualOdometryStereo viso(params);

  std::cout << "sec,nanosec,t_x,t_y,t_z,r_x,r_y,r_z,r_w" << std::endl;

  for (uint i = 0; i < std::min(leftImages.size(), rightImages.size() ); i++)
  {
    cv::Mat leftImage = cv::imread(leftImages.at(i), cv::IMREAD_COLOR);
    cv::Mat rightImage = cv::imread(rightImages.at(i), cv::IMREAD_COLOR);

    cv::Mat leftGrey, rightGrey;
    cv::cvtColor(leftImage, leftGrey, cv::COLOR_BGR2GRAY);
    cv::cvtColor(rightImage, rightGrey, cv::COLOR_BGR2GRAY);

    // Memory management: Memory is copied over and not needed after function returns. Can use statically allocated buffer to pass images

    // Create two static buffers. Check that read images will fit into the buffer
    uint const WIDTH = 1280;
    size_t const BUFFER_SIZE = 720 * WIDTH;
    uint8_t leftBuffer[BUFFER_SIZE], rightBuffer[BUFFER_SIZE];
    int DIMS[] = {WIDTH, 720, WIDTH};

    // populate buffers from cv mats
    for (uint row = 0; row < leftGrey.rows; row++)
    {
      memcpy(&leftBuffer[row * WIDTH], leftGrey.ptr(row), WIDTH);
    }
    for (uint row = 0; row < rightGrey.rows; row++)
    {
      memcpy(&rightBuffer[row * WIDTH], rightGrey.ptr(row), WIDTH);
    }

    viso.process(leftBuffer, rightBuffer, DIMS);
    viso2::Matrix pose = viso.getMotion();

    //std::cout << pose << std::endl;
    double flatPoseMatrix[16];
    pose.getData(flatPoseMatrix);
    double flatRotationMatrix[9] = {
      flatPoseMatrix[0], flatPoseMatrix[1], flatPoseMatrix[2], 
      flatPoseMatrix[4], flatPoseMatrix[5], flatPoseMatrix[6], 
      flatPoseMatrix[8], flatPoseMatrix[9], flatPoseMatrix[10], 
    };

    Quarternion voQuarternion = rotationMatrixToQuaternion(flatRotationMatrix);

    std::optional<std::pair<uint, uint>> timePairOption = getTimeFromPngFilename(leftImages.at(i).filename());
    if (timePairOption.has_value())
    {
      std::cout << timePairOption->first << "," << + timePairOption->second << ",";
    } else {
      std::cout << "0,";
    }

    std::cout << flatPoseMatrix[3] << "," << flatPoseMatrix[7] << "," << flatPoseMatrix[11] << ",";
    std::cout << voQuarternion.i << "," << voQuarternion.j << ","<< voQuarternion.k << ","<< voQuarternion.w << std::endl;

  }

  return 0;
}

//-----------------------------------------------------------------------------
std::optional<std::pair<uint, uint>> getTimeFromPngFilename(std::string filename)
{
  std::pair<uint, uint> timePair;
  // Regular expression to match the format "text_<int>_<int>.png"
  std::regex pattern(R"((.*)_(\d+)_(\d+)\.png)");

  // Perform regex matching and extraction
  std::smatch matches;
  if (std::regex_match(filename, matches, pattern) && matches.size() == 4) {
    // Convert the matched substrings to integers
    timePair.first = std::stoi(matches[2].str());
    timePair.second = std::stoi(matches[3].str());
    
    return timePair;
  }
  std::cout << "mathes: " << matches.size() << std::endl;
  return std::nullopt;
}
