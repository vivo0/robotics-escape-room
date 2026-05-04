cmake_minimum_required(VERSION 3.16.3)
project(sim_ros2_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
    # fix C1128: number of sections exceeded object file format limit
    add_compile_options(/bigobj)
endif()

if(WIN32)
    # fix LNK1104: cannot open file 'libboost_regex-vc142-mt-x64-1_74.lib'
    set(Boost_USE_STATIC_LIBS ON)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport)

if(NOT COPPELIASIM_INCLUDE_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(COPPELIASIM_INCLUDE_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/include)
    else()
        message(FATAL_ERROR "Environment variable COPPELIASIM_ROOT_DIR is not set")
    endif()
endif()
list(APPEND CMAKE_MODULE_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
    ${COPPELIASIM_INCLUDE_DIR}/cmake)
find_package(CoppeliaSim 4 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Boost REQUIRED)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim_ros2_interface)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include/sim_ros2_interface)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/generated)

set(packages "")
set(interfaces "")

file(GLOB files "${CMAKE_CURRENT_SOURCE_DIR}/meta/*interfaces.txt")

foreach(file IN LISTS files)
    file(STRINGS ${file} _interfaces)
    set(interfaces ${interfaces} ${_interfaces})
endforeach()

foreach(interface IN LISTS interfaces)
    string(FIND ${interface} "/" index)
    string(SUBSTRING ${interface} 0 ${index} pkg)
    if(NOT ${pkg} IN_LIST packages)
        list(APPEND packages ${pkg})
    endif()
endforeach()


foreach(PKG IN LISTS packages)
    find_package(${PKG} REQUIRED)
endforeach()

string(JOIN "\n" interfaces ${interfaces})
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/generated/interfaces.txt ${interfaces})

set(generatedFiles)
file(GLOB templateFiles RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/templates/ ${CMAKE_CURRENT_SOURCE_DIR}/templates/*)
foreach(templateFile ${templateFiles})
    add_custom_command(
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}" 
        COMMAND ${Python3_EXECUTABLE} "${COPPELIASIM_INCLUDE_DIR}/simStubsGen/external/pycpp/pycpp.py" -p "interfaces_file=${CMAKE_CURRENT_BINARY_DIR}/generated/interfaces.txt" -i "${CMAKE_CURRENT_SOURCE_DIR}/templates/${templateFile}" -o "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}" -P "${CMAKE_CURRENT_SOURCE_DIR}/tools" 
        DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tools/parse_interfaces.py" "${CMAKE_CURRENT_BINARY_DIR}/generated/interfaces.txt" "${CMAKE_CURRENT_SOURCE_DIR}/templates/${templateFile}")
    set(generatedFiles ${generatedFiles} "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}")
endforeach()
add_custom_target(generate_ros_code DEPENDS ${generatedFiles})
coppeliasim_generate_stubs(${CMAKE_CURRENT_BINARY_DIR}/generated XML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/meta/callbacks.xml LUA_FILE ${CMAKE_CURRENT_SOURCE_DIR}/lua/simROS2.lua)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/sim_ros2_interface/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/include/sim_ros2_interface/config.h)

set(SOURCES
    src/sim_ros2_interface.cpp
    src/ros_msg_builtin_io.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/generated/callbacks.cpp
)
coppeliasim_add_plugin(simROS2 SOURCES ${SOURCES})
add_dependencies(simROS2 generate_ros_code)
target_include_directories(simROS2 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
# target_link_libraries(simROS2 Boost::boost)

if(APPLE)
    # ROS libraries on macOS leave it to the application to load a Python shared library
    target_link_libraries(simROS2 PRIVATE Python3::Python)
endif()

#ament_target_dependencies(simROS2 rclcpp)
ament_target_dependencies(
    simROS2
    PUBLIC
    "rclcpp"
    "rclcpp_action"
    "tf2_ros"
    "geometry_msgs"
    "sensor_msgs"
)
ament_target_dependencies(simROS2 PUBLIC ${packages})
if(image_transport_FOUND)
    ament_target_dependencies(
        simROS2
        PUBLIC
        image_transport
    )
endif()



# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(simROS2 PRIVATE "SIM_ROS2_INTERFACE_BUILDING_LIBRARY")

ament_package()
