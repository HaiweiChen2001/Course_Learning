# Generated by CMake

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.8)
   message(FATAL_ERROR "CMake >= 2.8.3 required")
endif()
if(CMAKE_VERSION VERSION_LESS "2.8.3")
   message(FATAL_ERROR "CMake >= 2.8.3 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.8.3...3.29)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_cmake_targets_defined "")
set(_cmake_targets_not_defined "")
set(_cmake_expected_targets "")
foreach(_cmake_expected_target IN ITEMS qdldl::qdldlstatic qdldl::qdldl)
  list(APPEND _cmake_expected_targets "${_cmake_expected_target}")
  if(TARGET "${_cmake_expected_target}")
    list(APPEND _cmake_targets_defined "${_cmake_expected_target}")
  else()
    list(APPEND _cmake_targets_not_defined "${_cmake_expected_target}")
  endif()
endforeach()
unset(_cmake_expected_target)
if(_cmake_targets_defined STREQUAL _cmake_expected_targets)
  unset(_cmake_targets_defined)
  unset(_cmake_targets_not_defined)
  unset(_cmake_expected_targets)
  unset(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT _cmake_targets_defined STREQUAL "")
  string(REPLACE ";" ", " _cmake_targets_defined_text "${_cmake_targets_defined}")
  string(REPLACE ";" ", " _cmake_targets_not_defined_text "${_cmake_targets_not_defined}")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_cmake_targets_defined_text}\nTargets not yet defined: ${_cmake_targets_not_defined_text}\n")
endif()
unset(_cmake_targets_defined)
unset(_cmake_targets_not_defined)
unset(_cmake_expected_targets)


# Create imported target qdldl::qdldlstatic
add_library(qdldl::qdldlstatic STATIC IMPORTED)

set_target_properties(qdldl::qdldlstatic PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/harvey/Downloads/mpc/osqp/lin_sys/direct/qdldl/qdldl_sources/include"
)

# Create imported target qdldl::qdldl
add_library(qdldl::qdldl SHARED IMPORTED)

set_target_properties(qdldl::qdldl PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/harvey/Downloads/mpc/osqp/lin_sys/direct/qdldl/qdldl_sources/include"
)

# Import target "qdldl::qdldlstatic" for configuration ""
set_property(TARGET qdldl::qdldlstatic APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(qdldl::qdldlstatic PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "C"
  IMPORTED_LOCATION_NOCONFIG "/home/harvey/Downloads/mpc/osqp/build/lin_sys/direct/qdldl/qdldl_sources/out/libqdldl.a"
  )

# Import target "qdldl::qdldl" for configuration ""
set_property(TARGET qdldl::qdldl APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(qdldl::qdldl PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "/home/harvey/Downloads/mpc/osqp/build/lin_sys/direct/qdldl/qdldl_sources/out/libqdldl.so"
  IMPORTED_SONAME_NOCONFIG "libqdldl.so"
  )

# This file does not depend on other imported targets which have
# been exported from the same project but in a separate export set.

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)