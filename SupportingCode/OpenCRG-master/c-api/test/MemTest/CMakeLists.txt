cmake_minimum_required(VERSION 3.19)

project(crgMemTest LANGUAGES C)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/OpenCRGBuildDemoExecutable.cmake)
opencrg_build_demo_executable (${PROJECT_NAME})