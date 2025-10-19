cmake_minimum_required(VERSION 3.22)
project(safe_mrc_device)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Testing support
include(CTest)

include(GNUInstallDirs)

# Set local library and include paths
set(LOCAL_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(LOCAL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Prefer system libserial, fallback to local copy if not found
find_library(SERIAL_LIB_SYSTEM NAMES serial libserial)
find_library(SERIAL_LIB_LOCAL NAMES serial libserial PATHS ${LOCAL_LIB_DIR} NO_DEFAULT_PATH)

set(SERIAL_LIB "")
if(SERIAL_LIB_SYSTEM)
  set(SERIAL_LIB ${SERIAL_LIB_SYSTEM})
  set(SERIAL_LIB_SOURCE "system")
elseif(SERIAL_LIB_LOCAL)
  set(SERIAL_LIB ${SERIAL_LIB_LOCAL})
  set(SERIAL_LIB_SOURCE "local")
else()
  message(FATAL_ERROR "Could not find libserial. Install system package (e.g. libserial-dev) or place libserial.so in ${LOCAL_LIB_DIR}")
endif()

# Threads
find_package(Threads REQUIRED)

# Include directories - prioritize local includes
include_directories(${LOCAL_INCLUDE_DIR})
include_directories(include)

# Source files
set(SOURCES
    src/safe_mrc/safe_mrc.cpp
    src/safe_mrc/safe_mrc_device.cpp
    src/safe_mrc/safe_mrc_device_collection.cpp
    src/safe_mrc/safeguarder.cpp
    src/safe_mrc/safe_mrc_component.cpp
    src/rs485bus/rs485_serial.cpp
    src/rs485bus/rs485_device_collection.cpp
)

# Create library
add_library(${PROJECT_NAME} ${SOURCES})

# Link libraries - use discovered serial library and threads
target_link_libraries(${PROJECT_NAME}
    ${SERIAL_LIB}
    Threads::Threads
)

add_executable(demo examples/demo.cpp)
# Link demo against our library
target_link_libraries(demo ${PROJECT_NAME})

# 1000Hz pressure test executable
add_executable(pressure_test_1000Hz test/pressure_test_1000Hz.cpp)
target_link_libraries(pressure_test_1000Hz ${PROJECT_NAME})

# Set RPATH only if using local library so it can be found at runtime
if(SERIAL_LIB_SOURCE STREQUAL "local")
set_target_properties(${PROJECT_NAME} PROPERTIES
    INSTALL_RPATH "${LOCAL_LIB_DIR}"
    BUILD_WITH_INSTALL_RPATH TRUE
)
endif()

# Install rules
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install headers
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# Install the local serial library only if it exists and we used the local one
if(SERIAL_LIB_SOURCE STREQUAL "local")
  install(FILES ${SERIAL_LIB}
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
endif()

# Export targets
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# Print configuration info
message(STATUS "Using ${SERIAL_LIB_SOURCE} serial library: ${SERIAL_LIB}")
message(STATUS "Using local include directory: ${LOCAL_INCLUDE_DIR}")
