# libratConfig.cmake
# Version: VERSION_PLACEHOLDER

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

set(librat_VERSION "VERSION_PLACEHOLDER")
set(librat_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include")

# Find the shared library
find_library(LIBRAT_LIBRARY NAMES rat HINTS "${PACKAGE_PREFIX_DIR}/lib" NO_DEFAULT_PATH)

if(LIBRAT_LIBRARY AND NOT TARGET minot::rat)
  add_library(minot::rat SHARED IMPORTED)
  set_target_properties(minot::rat PROPERTIES
    IMPORTED_LOCATION "${LIBRAT_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${librat_INCLUDE_DIRS}"
  )
endif()

# Find the static library
find_library(LIBRAT_STATIC_LIBRARY NAMES rat HINTS "${PACKAGE_PREFIX_DIR}/lib" NO_DEFAULT_PATH)
if(LIBRAT_STATIC_LIBRARY AND LIBRAT_STATIC_LIBRARY MATCHES "\.a$")
  if(NOT TARGET minot::rat_static)
    add_library(minot::rat_static STATIC IMPORTED)
    set_target_properties(minot::rat_static PROPERTIES
      IMPORTED_LOCATION "${LIBRAT_STATIC_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${librat_INCLUDE_DIRS}"
    )
  endif()
endif()

set(librat_LIBRARIES minot::rat)
