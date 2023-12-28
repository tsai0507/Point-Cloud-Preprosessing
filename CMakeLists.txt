cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(yoda)

find_package(PCL 1.10 REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add_executable(pathplan pathplan.cpp)
add_executable(pcl_stl_sample pcl_stl_sample.cpp)
# add_executable(test_pcl test_pcl.cpp)
add_executable(3D_vision 3D_vision.cpp)
add_executable(try_bounding try_bounding.cpp)

# target_link_libraries(pathplan ${PCL_LIBRARIES} )
target_link_libraries(pcl_stl_sample  ${PCL_LIBRARIES} )
# target_link_libraries(test_pcl  ${PCL_LIBRARIES} )
target_link_libraries(3D_vision  ${PCL_LIBRARIES} )
target_link_libraries(try_bounding  ${PCL_LIBRARIES} )