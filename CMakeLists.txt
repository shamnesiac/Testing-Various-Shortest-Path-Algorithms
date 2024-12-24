cmake_minimum_required(VERSION 3.21)

# Project name and language
project(GraphAlgorithms LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Boost using vcpkg
find_package(Boost REQUIRED COMPONENTS heap)

# Add the executable
add_executable(GraphAlgorithms main.cpp)

# Include directories for Boost
target_include_directories(GraphAlgorithms PRIVATE ${Boost_INCLUDE_DIRS})

# Link Boost libraries
target_link_libraries(GraphAlgorithms PRIVATE ${Boost_LIBRARIES})

# Use vcpkg toolchain if available
if(DEFINED ENV{VCPKG_ROOT})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "Vcpkg toolchain file")
endif()
