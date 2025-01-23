/// @brief Simple program to test the build system
/// @author Torsten Babl

#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <regex>

#include "sfl/utilities/PathsFromRegex.h"

using std::filesystem::path;
using std::string;
using std::regex;
using std::vector;
using sfl::utilities::pathsFromRegex;

//-----------------------------------------------------------------------------
int main()
{
  // Enumerate right and left png channels
  // PngFileReaders for left and right channels
  // Init with directory and regex to match on all files
  // Provides a give next png
  path pngDir("/home/torsten/Documents/masters/citrus_data_set/04_13D_Jackal/png");

  vector<path> leftImages = pathsFromRegex(pngDir, regex(".*left.*"));
  vector<path> rightImages = pathsFromRegex(pngDir, regex(".*right.*"));

  // for (path const& imagePath : leftImages)
  // {
  //   std::cout << imagePath << std::endl;
  // }



  // Rectification

  // VO 

  // Print transform to stdout

  return 0;
}
