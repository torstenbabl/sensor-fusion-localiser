add_executable(SingleThreadedVO main.cpp)
target_link_libraries(SingleThreadedVO PRIVATE sfl)
target_link_libraries(SingleThreadedVO PRIVATE viso2)
target_link_libraries (SingleThreadedVO PRIVATE gflags)

#target_include_directories(SingleThreadedVO PRIVATE ${OpenCV_INCLUDE_DIRS})
#target_link_libraries(SingleThreadedVO PRIVATE ${OpenCV_LIBS})