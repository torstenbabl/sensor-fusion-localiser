#ifndef _UTILITIES_PATHSFROMREGEX_H_
#define _UTILITIES_PATHSFROMREGEX_H_

#include <filesystem>
#include <regex>
#include <vector>

namespace sfl::utilities {
/// @brief Return a vector of paths that match the regular expression provided
std::vector<std::filesystem::path> pathsFromRegex(
  std::filesystem::path directory, ///< [in] Directory in which to look for files
  std::regex pattern   ///< [in] Regulat expression to match files against
);
}

#endif // _UTILITIES_PATHSFROMREGEX_H_