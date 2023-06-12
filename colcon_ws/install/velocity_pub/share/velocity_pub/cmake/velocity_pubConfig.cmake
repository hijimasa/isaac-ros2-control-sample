# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_velocity_pub_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED velocity_pub_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(velocity_pub_FOUND FALSE)
  elseif(NOT velocity_pub_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(velocity_pub_FOUND FALSE)
  endif()
  return()
endif()
set(_velocity_pub_CONFIG_INCLUDED TRUE)

# output package information
if(NOT velocity_pub_FIND_QUIETLY)
  message(STATUS "Found velocity_pub: 0.0.0 (${velocity_pub_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'velocity_pub' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${velocity_pub_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(velocity_pub_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${velocity_pub_DIR}/${_extra}")
endforeach()
