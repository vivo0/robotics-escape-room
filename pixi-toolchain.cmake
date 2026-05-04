set(CMAKE_OSX_SYSROOT "/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk")

# Force CMake to look for all shared libraries (spdlog, Boost, etc.) inside the pixi environment
set(CMAKE_FIND_ROOT_PATH "$ENV{CONDA_PREFIX}")
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Force Coppelia to load the exact version of the dylibs used at build time, instead of letting
# the OSes to resolve a version at runtime with their usual opaque logic.
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH True)
