# libasic third-party dependencies configuration
cmake_minimum_required(VERSION 3.14)

# Ensure Eigen's fetched source path is visible globally to the IDE's language server
if(eigen_SOURCE_DIR)
    include_directories(SYSTEM ${eigen_SOURCE_DIR})
endif()

message(STATUS "libasic: Third-party include paths exported successfully.")