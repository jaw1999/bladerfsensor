cmake_minimum_required(VERSION 3.10)
project(bladerf_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBBLADERF REQUIRED libbladeRF)
pkg_check_modules(FFTW3 REQUIRED fftw3f)
find_package(Threads REQUIRED)
find_library(LIQUID_LIB liquid REQUIRED)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${LIBBLADERF_INCLUDE_DIRS}
    ${FFTW3_INCLUDE_DIRS}
)

# Source files
set(SOURCES
    src/main.cpp
    src/web_server.cpp
    src/cfar_detector.cpp
    src/os_cfar_detector.cpp
    src/array_calibration.cpp
    src/signal_processing.cpp
    src/df_processing.cpp
    src/recording.cpp
    src/config_validation.cpp
    src/scanner.cpp
)

# Optional: Add mongoose support
option(USE_MONGOOSE "Enable mongoose web server" ON)
if(USE_MONGOOSE)
    # Check if mongoose files exist
    if(EXISTS "${PROJECT_SOURCE_DIR}/src/mongoose.c")
        list(APPEND SOURCES src/mongoose.c)
        add_definitions(-DUSE_MONGOOSE)
        message(STATUS "Mongoose web server enabled")
    else()
        message(WARNING "USE_MONGOOSE is ON but mongoose.c not found. Download from https://github.com/cesanta/mongoose")
        message(WARNING "Web server will run in stub mode. See server/MONGOOSE_SETUP.md for instructions.")
    endif()
endif()

# Create executable
add_executable(bladerf_server ${SOURCES})

# Link libraries
target_link_libraries(bladerf_server
    ${LIBBLADERF_LIBRARIES}
    ${FFTW3_LIBRARIES}
    ${LIQUID_LIB}
    Threads::Threads
    m
)

# Compiler flags
target_compile_options(bladerf_server PRIVATE -Wall -Wextra -O3)
