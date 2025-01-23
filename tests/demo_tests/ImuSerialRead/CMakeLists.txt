add_executable(ImuSerialRead main.cpp)

# Locate libserial 
#find_package(PkgConfig REQUIRED) 
#pkg_check_modules(LIBSERIALPORT REQUIRED libserialport) 

# Link libraries 
#target_include_directories(ImuSerialRead PRIVATE ${LIBSERIALPORT_INCLUDE_DIRS}) 
#target_link_libraries(ImuSerialRead PRIVATE ${LIBSERIALPORT_LIBRARIES}) 