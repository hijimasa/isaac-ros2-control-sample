# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_xacro_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED xacro_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(xacro_FOUND FALSE)
  elseif(NOT xacro_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(xacro_FOUND FALSE)
  endif()
  return()
endif()
set(_xacro_CONFIG_INCLUDED TRUE)

# output package information
if(NOT xacro_FIND_QUIETLY)
  message(STATUS "Found xacro: 2.0.9 (${xacro_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'xacro' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${xacro_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(xacro_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "xacro-extras.cmake")
foreach(_extra ${_extras})
  include("${xacro_DIR}/${_extra}")
endforeach()
