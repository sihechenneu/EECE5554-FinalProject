# Wrapper to set CMP0144 before PCL's FindFLANN (silences dev warning about FLANN_ROOT).
# PCL's FindFLANN is in the same Modules dir as this would be found from when PCL adds it.
# We are only used when we are prepended to CMAKE_MODULE_PATH before find_package(pcl_conversions).
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()
# Include PCL's FindFLANN (standard path on Debian/Ubuntu)
include("/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules/FindFLANN.cmake")
