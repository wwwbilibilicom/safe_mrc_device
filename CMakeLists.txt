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

# Find Threads package (still needed from system)
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

# Link libraries - use local serial library
target_link_libraries(${PROJECT_NAME} 
    ${LOCAL_LIB_DIR}/libserial.so
    Threads::Threads
)

add_executable(demo.cpp examples/demo.cpp)
target_link_libraries(demo.cpp ${PROJECT_NAME})

# Set RPATH to find the local library at runtime
set_target_properties(${PROJECT_NAME} PROPERTIES
    INSTALL_RPATH "${LOCAL_LIB_DIR}"
    BUILD_WITH_INSTALL_RPATH TRUE
)

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

# Install the local serial library
install(FILES ${LOCAL_LIB_DIR}/libserial.so
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Export targets
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# Print configuration info
message(STATUS "Using local serial library: ${LOCAL_LIB_DIR}/libserial.so")
message(STATUS "Using local include directory: ${LOCAL_INCLUDE_DIR}")