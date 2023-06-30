#----------------------------------------------------------------
# Generated CMake target import file for configuration "release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pyrealsense2::pyrsutils" for configuration "release"
set_property(TARGET pyrealsense2::pyrsutils APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pyrealsense2::pyrsutils PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/OFF/pyrsutils.cpython-36m-aarch64-linux-gnu.so.2.54.1"
  IMPORTED_SONAME_RELEASE "pyrsutils.cpython-36m-aarch64-linux-gnu.so.2.54"
  )

list(APPEND _IMPORT_CHECK_TARGETS pyrealsense2::pyrsutils )
list(APPEND _IMPORT_CHECK_FILES_FOR_pyrealsense2::pyrsutils "${_IMPORT_PREFIX}/OFF/pyrsutils.cpython-36m-aarch64-linux-gnu.so.2.54.1" )

# Import target "pyrealsense2::pybackend2" for configuration "release"
set_property(TARGET pyrealsense2::pybackend2 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pyrealsense2::pybackend2 PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/OFF/pybackend2.cpython-36m-aarch64-linux-gnu.so.2.54.1"
  IMPORTED_SONAME_RELEASE "pybackend2.cpython-36m-aarch64-linux-gnu.so.2"
  )

list(APPEND _IMPORT_CHECK_TARGETS pyrealsense2::pybackend2 )
list(APPEND _IMPORT_CHECK_FILES_FOR_pyrealsense2::pybackend2 "${_IMPORT_PREFIX}/OFF/pybackend2.cpython-36m-aarch64-linux-gnu.so.2.54.1" )

# Import target "pyrealsense2::pyrealsense2" for configuration "release"
set_property(TARGET pyrealsense2::pyrealsense2 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pyrealsense2::pyrealsense2 PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/OFF/pyrealsense2.cpython-36m-aarch64-linux-gnu.so.2.54.1"
  IMPORTED_SONAME_RELEASE "pyrealsense2.cpython-36m-aarch64-linux-gnu.so.2.54"
  )

list(APPEND _IMPORT_CHECK_TARGETS pyrealsense2::pyrealsense2 )
list(APPEND _IMPORT_CHECK_FILES_FOR_pyrealsense2::pyrealsense2 "${_IMPORT_PREFIX}/OFF/pyrealsense2.cpython-36m-aarch64-linux-gnu.so.2.54.1" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
