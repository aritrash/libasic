cmake_minimum_required(VERSION 3.14)
project(libasic VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Load Dependencies First
add_subdirectory(third_party)

# 2. Build the Core Library
add_library(asic STATIC
    src/circuit.cpp
    src/memristor.cpp
    src/mna_matrix.cpp
    src/newton_raphson.cpp
    src/transient_solver.cpp
)

# 3. Define Include Directories
target_include_directories(asic PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 4. Link Dependencies to your Library
target_link_libraries(asic PUBLIC Eigen3::Eigen)

# 5. Build the Example Executable
add_executable(binary_suite examples/binary_suite.cpp)
target_link_libraries(binary_suite PRIVATE asic)

enable_testing()

add_executable(asic_tests tests/test_cmos.cpp)
target_link_libraries(asic_tests PRIVATE asic gtest_main)

# Register it with CTest
include(GoogleTest)
gtest_discover_tests(asic_tests)

# ==========================================
# SYSTEM DEPLOYMENT & INSTALL TARGETS
# ==========================================
include(GNUInstallDirs)

# 1. Install the compiled library and tell CMake to track it in an "Export Set"
install(TARGETS asic
    EXPORT libasicTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# 2. Install the public headers so other projects can #include them
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# 3. Generate the CMake configuration files so find_package() works
install(EXPORT libasicTargets
    FILE libasicTargets.cmake
    NAMESPACE libasic::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libasic
)

# 4. Create a basic Config file
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/libasicConfig.cmake
    "include(\"\${CMAKE_CURRENT_LIST_DIR}/libasicTargets.cmake\")\n"
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libasicConfig.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libasic
)