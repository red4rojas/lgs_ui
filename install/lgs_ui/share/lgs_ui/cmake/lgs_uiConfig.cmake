# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_lgs_ui_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED lgs_ui_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(lgs_ui_FOUND FALSE)
  elseif(NOT lgs_ui_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(lgs_ui_FOUND FALSE)
  endif()
  return()
endif()
set(_lgs_ui_CONFIG_INCLUDED TRUE)

# output package information
if(NOT lgs_ui_FIND_QUIETLY)
  message(STATUS "Found lgs_ui: 0.0.0 (${lgs_ui_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'lgs_ui' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${lgs_ui_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(lgs_ui_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${lgs_ui_DIR}/${_extra}")
endforeach()
