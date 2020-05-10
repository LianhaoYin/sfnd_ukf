cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")



#set(PCL_LIBRARY_DIRS "/usr/local/Cellar/pcl/1.9.1_4/lib")
set(PCL_LIBRARY_DIRS "/usr/local/Cellar/pcl/1.9.1_8/lib")

#-- PCL_LIBRARY_DIRS: /usr/local/lib


# set sysroot
#SET(DCMAKE_OSX_SYSROOT "/Library/Developer/CommandLineTools/SDKs/MacOSX10.15.sdk")


project(test_sfnd_kalman)

find_package(PCL 1.2 REQUIRED COMPONENT common io VISUALIZATION filters segmentation)

message(STATUS "PCL Version: " ${PCL_VERSION})
message(STATUS "PCL Libraries: " ${PCL_LIBRARIES})
message(STATUS "PCL_LIBRARY_DIRS: " ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_COMPONENTS: " ${PCL_COMPONENTS})
message(STATUS "PCL_DIR " ${PCL_DIR})
message(STATUS "PCL_SAMPLE_CONSESUS_LIBRARIES: " ${PCL_SAMPLE_CONSESUS_LIBRARIES})



include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
#link_directories("/usr/local/Cellar/pcl/1.9.1_4/lib")
link_directories("/usr/local/Cellar/pcl/1.9.1_8/lib")

add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (ukf_highway src/main.cpp src/ukf.cpp src/tools.cpp src/render/render.cpp)
target_link_libraries (ukf_highway  ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_filters_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES})







