#include "PathsFromRegex.h"

namespace sfl::utilities {
//-----------------------------------------------------------------------------
std::vector<std::filesystem::path> pathsFromRegex(
  std::filesystem::path directory,
  std::regex pattern
)
{
  std::vector<std::filesystem::path> paths;

  using std::filesystem::directory_iterator;
  using std::filesystem::directory_entry;
  for (directory_entry const& directoryEntry : directory_iterator{directory})
  {
    if (std::regex_match(directoryEntry.path().filename().c_str(), pattern))
    {
      paths.push_back(directoryEntry.path());
    }
  }

  return paths;
}
}