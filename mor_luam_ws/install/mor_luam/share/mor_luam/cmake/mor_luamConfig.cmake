# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mor_luam_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mor_luam_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mor_luam_FOUND FALSE)
  elseif(NOT mor_luam_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mor_luam_FOUND FALSE)
  endif()
  return()
endif()
set(_mor_luam_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mor_luam_FIND_QUIETLY)
  message(STATUS "Found mor_luam: 2.1.5 (${mor_luam_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mor_luam' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT mor_luam_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mor_luam_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${mor_luam_DIR}/${_extra}")
endforeach()
