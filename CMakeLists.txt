cmake_minimum_required(VERSION 3.5)

project(sensor_fusion_localiser VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)

# Find Boost program options
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(Boost COMPONENTS serialization REQUIRED)
#target_link_libraries(target PRIVATE Boost::program_options)
#target_link_libraries(target PRIVATE Boost::serialization)

# Find tbb
find_package(TBB REQUIRED)
# target_link_libraries(target PRIVATE TBB::tbb)

# Find Eigen
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
# target_link_libraries (target PRIVATE Eigen3::Eigen)


find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
#target_link_libraries (target PRIVATE gtsam)

find_package(gflags REQUIRED)
#target_link_libraries (target PRIVATE gflags)

add_subdirectory(source/sfl)
add_subdirectory(tests)
add_subdirectory(third_party_sources)
add_subdirectory(applications)
