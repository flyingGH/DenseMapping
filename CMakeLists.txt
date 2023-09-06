cmake_minimum_required(VERSION 3.20)
project("dense_mapping" CXX)
set(CMAKE_BUILD_TYPE "Release")

set(Sophus_LIBS "Sophus::Sophus")
set(Sophus_INCLUDE_DIRS "/usr/local/include/sophus")

include_directories("include")
file(GLOB SOURCE_FILES "${CMAKE_SOURCE_DIR}/src/*.cpp")

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED COMPONENTS "Sophus")
find_package(PCL 1.13 REQUIRED COMPONENTS "common" "io" "visualization")
find_package(OpenCV REQUIRED)

add_executable(
        ${PROJECT_NAME}
        ${SOURCE_FILES}
)
target_include_directories(
        ${PROJECT_NAME}
        PRIVATE ${EIGEN3_INCLUDE_DIRS}
        PRIVATE ${OpenCV_INCLUDE_DIRS}
        PRIVATE ${Sophus_INCLUDE_DIRS}
        PRIVATE ${PCL_INCLUDE_DIRS}
)
target_link_libraries(
        ${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${Sophus_LIBS}
        ${PCL_LIBRARIES}
)

#add_executable("read_map" "src/read_map.cpp")
#target_link_libraries(
#        "read_map"
#        ${PCL_LIBRARIES}
#)


#
#set(CMAKE_BUILD_TYPE "Release")
#
#set(PROJECT_BINARY_DIR "${PROJECT_SOURCE_DIR}/build")
#set(POLAR_SEARCH_LIBRARY "polar_search")
#set(BLOCK_MATCH_LIBRARY "block_match")
#set(DEPTH_FILTER_LIBRARY "depth_filter")
#set(READ_DATASET_LIBRARY "read_dataset")
#set(TRIANGULATION_LIBRARY "triangulation")
#set(DEPEND_LIBRARY "depend_library")
#set(Sophus_LIBS "Sophus::Sophus")
#set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib")
#set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")
#set(Sophus_INCLUDE_DIRS "/usr/local/include/sophus")
#set(
#        LINK_LIBS
#        ${POLAR_SEARCH_LIBRARY}
#        ${BLOCK_MATCH_LIBRARY}
#        ${DEPTH_FILTER_LIBRARY}
#        ${TRIANGULATION_LIBRARY}
#        ${READ_DATASET_LIBRARY}
#        ${DEPEND_LIBRARY}
#)
#
#include_directories("include")
#
#find_package(Eigen3 REQUIRED)
#find_package(Sophus REQUIRED COMPONENTS "Sophus")
#find_package(PCL 1.13 REQUIRED COMPONENTS "common" "io" "visualization")
#find_package(OpenCV REQUIRED)
#
#add_library(
#        ${DEPEND_LIBRARY} INTERFACE
#)
#target_compile_features(
#        ${DEPEND_LIBRARY} INTERFACE "cxx_std_14"
#)
#target_include_directories(
#        ${DEPEND_LIBRARY}
#        INTERFACE ${EIGEN3_INCLUDE_DIRS}
#        INTERFACE ${OpenCV_INCLUDE_DIRS}
#        INTERFACE ${Sophus_INCLUDE_DIRS}
#        INTERFACE ${PCL_INCLUDE_DIRS}
#)
#target_link_libraries(
#        ${DEPEND_LIBRARY}
#        INTERFACE ${OpenCV_LIBS}
#        INTERFACE ${Sophus_LIBS}
#        INTERFACE ${PCL_LIBRARIES}
#)
#
#add_library(
#        ${POLAR_SEARCH_LIBRARY}
#        src/polar_search.cpp
#)
#target_link_libraries(
#        ${POLAR_SEARCH_LIBRARY}
#        ${DEPEND_LIBRARY}
#)
#
#add_library(
#        ${BLOCK_MATCH_LIBRARY}
#        src/block_match.cpp
#)
#target_link_libraries(
#        ${BLOCK_MATCH_LIBRARY}
#        ${DEPEND_LIBRARY}
#)
#
#add_library(
#        ${DEPTH_FILTER_LIBRARY}
#        src/depth_filter.cpp
#)
#target_link_libraries(
#        ${DEPTH_FILTER_LIBRARY}
#        ${DEPEND_LIBRARY}
#)
#
#add_library(
#        ${READ_DATASET_LIBRARY}
#        src/read_dataset_files.cpp
#)
#target_link_libraries(
#        ${READ_DATASET_LIBRARY}
#        ${DEPEND_LIBRARY}
#)
#add_library(
#        ${TRIANGULATION_LIBRARY}
#        src/triangulation.cpp
#)
#target_link_libraries(
#        ${TRIANGULATION_LIBRARY}
#        ${DEPEND_LIBRARY}
#)
#
#add_executable(
#        ${PROJECT_NAME}
#        src/main.cpp
#)
#target_link_libraries(
#        ${PROJECT_NAME}
#        ${LINK_LIBS}
#)
#
#message(STATUS "EIGEN3_INCLUDE_DIRS -- ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "OpenCV_INCLUDE_DIRS -- ${OpenCV_INCLUDE_DIRS}")
#message(STATUS "Sophus_INCLUDE_DIRS -- ${Sophus_INCLUDE_DIRS}")
#message(STATUS "PCL_INCLUDE_DIRS -- ${PCL_INCLUDE_DIRS}")