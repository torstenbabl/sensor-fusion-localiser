# Add viso2 as a static library
add_library(viso2 STATIC 
${CMAKE_CURRENT_SOURCE_DIR}/viso2/src/filter.cpp 
${CMAKE_CURRENT_SOURCE_DIR}/viso2/src/matcher.cpp 
${CMAKE_CURRENT_SOURCE_DIR}/viso2/src/matrix.cpp 
${CMAKE_CURRENT_SOURCE_DIR}/viso2/src/reconstruction.cpp 
${CMAKE_CURRENT_SOURCE_DIR}/viso2/src/triangle.cpp 
${CMAKE_CURRENT_SOURCE_DIR}/viso2/src/viso_mono.cpp 
${CMAKE_CURRENT_SOURCE_DIR}/viso2/src/viso_stereo.cpp 
${CMAKE_CURRENT_SOURCE_DIR}/viso2/src/viso.cpp)

target_include_directories(viso2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/viso2/include/viso2)
target_include_directories(viso2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/viso2/include)

target_include_directories(viso2 INTERFACE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(viso2 PRIVATE ${OpenCV_LIBS})

target_compile_options(viso2 INTERFACE -w)

