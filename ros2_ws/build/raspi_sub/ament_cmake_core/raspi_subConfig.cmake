# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_raspi_sub_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED raspi_sub_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(raspi_sub_FOUND FALSE)
  elseif(NOT raspi_sub_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(raspi_sub_FOUND FALSE)
  endif()
  return()
endif()
set(_raspi_sub_CONFIG_INCLUDED TRUE)

# output package information
if(NOT raspi_sub_FIND_QUIETLY)
  message(STATUS "Found raspi_sub: 0.0.0 (${raspi_sub_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'raspi_sub' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${raspi_sub_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(raspi_sub_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${raspi_sub_DIR}/${_extra}")
endforeach()
