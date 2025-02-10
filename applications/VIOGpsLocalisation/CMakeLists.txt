add_executable(VIOGpsLocalisation main.cpp)
target_link_libraries(VIOGpsLocalisation PRIVATE sfl)
target_link_libraries (VIOGpsLocalisation PRIVATE gflags)

target_link_libraries(VIOGpsLocalisation PRIVATE Boost::program_options)
target_link_libraries(VIOGpsLocalisation PRIVATE Boost::serialization)
target_link_libraries(VIOGpsLocalisation PRIVATE TBB::tbb)
target_link_libraries (VIOGpsLocalisation PRIVATE Eigen3::Eigen)
target_link_libraries (VIOGpsLocalisation PRIVATE gtsam)