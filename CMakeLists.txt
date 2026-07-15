cmake_minimum_required(VERSION 3.20)
project(libasic CXX)

# Force C++20 Standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Crucial for VSCode/Clangd to understand your includes
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

# Fetch Eigen3
include(FetchContent)
FetchContent_Declare(
    Eigen3
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
)
# Disable Eigen's internal tests to speed up our compilation
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(Eigen3)

# Define the libasic static library
add_library(libasic STATIC
    src/core/circuit.cpp
    src/solver/mna_matrix.cpp
    src/solver/newton_raphson.cpp
)

# Expose the src directory so '#include "core/..."' works
target_include_directories(libasic PUBLIC src)
target_link_libraries(libasic PUBLIC Eigen3::Eigen)

# Define the local test bench
add_executable(asic_sim src/main.cpp)
target_link_libraries(asic_sim PRIVATE libasic)