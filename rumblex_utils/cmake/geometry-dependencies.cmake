find_package(magic_enum CONFIG REQUIRED)

# Prefer an installed library; otherwise fetch a reproducible, header-only build.
find_package(mp-units 2.5 CONFIG QUIET)
if(NOT mp-units_FOUND)
  include(FetchContent)
  set(MP_UNITS_BUILD_INSTALL OFF CACHE BOOL "" FORCE)
  set(MP_UNITS_API_CONTRACTS NONE CACHE STRING "" FORCE)
  FetchContent_Declare(mp-units
    URL https://github.com/mpusz/mp-units/archive/refs/tags/v2.5.0.tar.gz
    URL_HASH SHA256=a6bd48bee699f11f0ed5b04b8c5006d15f76e6d898e058db3880554d2e47a400
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR src
    SYSTEM
  )
  FetchContent_MakeAvailable(mp-units)
  # Keep upstream header install rules out of the ROS package installation.
  set_property(DIRECTORY "${mp-units_SOURCE_DIR}/src" PROPERTY EXCLUDE_FROM_ALL TRUE)
endif()


# The pinned fallback is a local CMake target. Expose it through an imported
# dependency so consumers can recreate it without exporting upstream targets.
if(NOT TARGET rumblex_mp_units)
  add_library(rumblex_mp_units INTERFACE IMPORTED)
  set_target_properties(rumblex_mp_units PROPERTIES INTERFACE_LINK_LIBRARIES mp-units::mp-units)
endif()
