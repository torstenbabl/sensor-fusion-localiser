add_library(sfl STATIC )
target_include_directories(sfl PUBLIC ../)

add_subdirectory(utilities)
add_subdirectory(types)
add_subdirectory(lookups)