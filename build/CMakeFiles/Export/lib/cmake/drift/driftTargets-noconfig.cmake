#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "drift::drift" for configuration ""
set_property(TARGET drift::drift APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(drift::drift PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "/usr/local/lib/libdrift.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS drift::drift )
list(APPEND _IMPORT_CHECK_FILES_FOR_drift::drift "/usr/local/lib/libdrift.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
