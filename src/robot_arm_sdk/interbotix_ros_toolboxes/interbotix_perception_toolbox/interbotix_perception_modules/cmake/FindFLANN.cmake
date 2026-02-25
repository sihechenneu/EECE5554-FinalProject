# Wrapper to set CMP0144 before PCL's FindFLANN (silences dev warning about FLANN_ROOT).
# PCL's FindFLANN is in the same Modules dir as this would be found from when PCL adds it.
# We are only used when we are prepended to CMAKE_MODULE_PATH before find_package(pcl_conversions).
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

# Include PCL's FindFLANN, supporting multi-arch (x86_64, aarch64, etc.)
# Prefer the multi-arch libdir via CMAKE_LIBRARY_ARCHITECTURE, but fall back to common paths.
set(_PCL_FIND_FLANN_CANDIDATES
  "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/pcl/Modules/FindFLANN.cmake"
  "/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules/FindFLANN.cmake"
  "/usr/lib/aarch64-linux-gnu/cmake/pcl/Modules/FindFLANN.cmake"
  "/usr/lib/arm-linux-gnueabihf/cmake/pcl/Modules/FindFLANN.cmake"
  "/usr/lib/cmake/pcl/Modules/FindFLANN.cmake"
)

set(_PCL_FIND_FLANN_FOUND FALSE)
foreach(_pcl_flann_path IN LISTS _PCL_FIND_FLANN_CANDIDATES)
  if(EXISTS "${_pcl_flann_path}")
    include("${_pcl_flann_path}")
    set(_PCL_FIND_FLANN_FOUND TRUE)
    break()
  endif()
endforeach()

if(NOT _PCL_FIND_FLANN_FOUND)
  message(FATAL_ERROR
    "Could not find PCL's FindFLANN.cmake. "
    "Tried: ${_PCL_FIND_FLANN_CANDIDATES}. "
    "Install libpcl-dev or adjust this path for your platform.")
endif()

