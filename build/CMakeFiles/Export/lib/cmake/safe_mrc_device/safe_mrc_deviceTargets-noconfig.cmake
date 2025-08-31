#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "safe_mrc_device::safe_mrc_device" for configuration ""
set_property(TARGET safe_mrc_device::safe_mrc_device APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(safe_mrc_device::safe_mrc_device PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libsafe_mrc_device.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS safe_mrc_device::safe_mrc_device )
list(APPEND _IMPORT_CHECK_FILES_FOR_safe_mrc_device::safe_mrc_device "${_IMPORT_PREFIX}/lib/libsafe_mrc_device.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
