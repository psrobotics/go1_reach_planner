cmake_minimum_required(VERSION 3.5)
project(libmotioncapture)

# define some options
option(LIBMOTIONCAPTURE_BUILD_PYTHON_BINDINGS "Generate Python Bindings" ON)
option(LIBMOTIONCAPTURE_ENABLE_QUALISYS "Enable Qualisys" ON)
option(LIBMOTIONCAPTURE_ENABLE_OPTITRACK "Enable Optitrack" ON)
option(LIBMOTIONCAPTURE_ENABLE_OPTITRACK_CLOSED_SOURCE "Enable Optitrack (Closed Source)" ON)
option(LIBMOTIONCAPTURE_ENABLE_VICON "Enable Vicon" ON)
option(LIBMOTIONCAPTURE_ENABLE_NOKOV "Enable Nokov" OFF)
option(LIBMOTIONCAPTURE_ENABLE_VRPN "Enable VRPN" ON)
option(LIBMOTIONCAPTURE_ENABLE_FZMOTION "Enable FZMOTION" ON)
option(LIBMOTIONCAPTURE_ENABLE_MOTIONANALYSIS "Enable MotionAnalysis" OFF)
option(LIBMOTIONCAPTURE_BUILD_EXAMPLE "Enable Example application" ON)

# Enable C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Boost 1.5 COMPONENTS system REQUIRED) # for optitrack
add_definitions(
  -DBOOST_DATE_TIME_NO_LIB
  -DBOOST_REGEX_NO_LIB
  -D_BUILD_STATIC_LIB
)

# add lcm
find_package(lcm REQUIRED)
# add eigen
find_package(Eigen3 REQUIRED)
# add realsense package
find_package(realsense2 REQUIRED)
# opencv
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV > 3.0 not found.")
endif()

set(VICON_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/vicon-datastream-sdk/)
set(QUALISYS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/qualisys_cpp_sdk/)
set(NATNET_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/NatNetSDKCrossplatform/)
set(NOKOV_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/nokov_sdk/)
set(VRPN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/vrpn/)
set(MOTIONANALYSIS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/cortex_sdk_linux/)


set(CMAKE_POSITION_INDEPENDENT_CODE ON)

###########
## Build ##
###########

## Additional include folders
# lcm
set(my_include_directories
  include
  examples/lcm_type/exlcm # type hpp for lcm
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${LCM_USE_FILE}
)
set(my_link_directories)
set(my_files
  src/motioncapture.cpp
  src/mock.cpp
)

if (LIBMOTIONCAPTURE_ENABLE_VICON)
  message("including vicon")

  add_definitions(-DENABLE_VICON)
  add_subdirectory(deps/vicon-datastream-sdk EXCLUDE_FROM_ALL)
  set(my_include_directories
    ${my_include_directories}
    ${VICON_SDK_DIR}/Vicon/CrossMarket/DataStream
  )
  set(my_files
    ${my_files}
    src/vicon.cpp
  )
  set(my_libraries
    ${my_libraries}
    ViconDataStreamSDK_CPP
  )
endif()

if (LIBMOTIONCAPTURE_ENABLE_OPTITRACK)
  message("including optitrack")

  add_definitions(-DENABLE_OPTITRACK)
  set(my_include_directories
    ${my_include_directories}
  )
  set(my_files
    ${my_files}
    src/optitrack.cpp
  )
endif()

if (LIBMOTIONCAPTURE_ENABLE_OPTITRACK_CLOSED_SOURCE)
  if ((CMAKE_SYSTEM_NAME STREQUAL "Linux") AND (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64"))
    message("Including optitrack_closed_source")

    add_definitions(-DENABLE_OPTITRACK_CLOSED_SOURCE)
    add_subdirectory(deps/NatNetSDKCrossplatform EXCLUDE_FROM_ALL)
    set(my_include_directories
      ${my_include_directories}
      ${NATNET_SDK_DIR}/include
    )
    set(my_files
      ${my_files}
      src/optitrack_closed_source.cpp
    )
    set(my_link_directories
      ${NATNET_SDK_DIR}/lib/ubuntu
    )
    set(my_libraries
      ${my_libraries}
      NatNet
    )
  else()
    message(WARNING "Can not include optitrack_closed_source due to missing SDK. (${CMAKE_SYSTEM_NAME}, ${CMAKE_SYSTEM_PROCESSOR})")
  endif()
endif()

if (LIBMOTIONCAPTURE_ENABLE_QUALISYS)
  message("including qualisys")

  add_definitions(-DENABLE_QUALISYS)
  add_subdirectory(deps/qualisys_cpp_sdk EXCLUDE_FROM_ALL)
  set(my_include_directories
    ${my_include_directories}
    ${QUALISYS_DIR}
  )
  set(my_files
    ${my_files}
    src/qualisys.cpp
  )
  set(my_libraries
    ${my_libraries}
    qualisys_cpp_sdk
  )
endif()

if (LIBMOTIONCAPTURE_ENABLE_NOKOV)
  message("including nokov")

  add_definitions(-DENABLE_NOKOV)
  set(my_include_directories
    ${my_include_directories}
    ${NOKOV_SDK_DIR}/include
  )
  set(my_files
    ${my_files}
    src/nokov.cpp
  )
  set(my_link_directories
    ${my_link_directories}
    ${NOKOV_SDK_DIR}/lib
)
  set(my_libraries
    ${my_libraries}
    SeekerSDKClient
  )
endif()

if (LIBMOTIONCAPTURE_ENABLE_VRPN)
  message("including vrpn")

  add_definitions(-DENABLE_VRPN)
  option(VRPN_BUILD_CLIENT_LIBRARY "" ON)
  add_subdirectory(deps/vrpn EXCLUDE_FROM_ALL)
  set(my_include_directories
    ${my_include_directories}
    ${VRPN_DIR}
  )
  set(my_files
    ${my_files}
    src/vrpn.cpp
  )
  set(my_libraries
    ${my_libraries}
    ${VRPN_CLIENT_LIBRARY}
  )
endif()

if (LIBMOTIONCAPTURE_ENABLE_FZMOTION)
  message("including fzmotion")

  add_definitions(-DENABLE_FZMOTION)
  set(my_files
    ${my_files}
    src/fzmotion.cpp
  )
endif()

if (LIBMOTIONCAPTURE_ENABLE_MOTIONANALYSIS)
  message("including motionanalysis")

  add_definitions(-DENABLE_MOTIONANALYSIS)
  add_subdirectory(deps/cortex_sdk_linux EXCLUDE_FROM_ALL)
  set(my_include_directories
    ${my_include_directories}
    ${MOTIONANALYSIS_DIR}
  )
  set(my_files
    ${my_files}
    src/motionanalysis.cpp
  )
  set(my_libraries
    ${my_libraries}
    cortex_sdk pthread m
  )
endif()

include_directories(
  ${my_include_directories}
)

## Declare a cpp library
add_library(libmotioncapture
  ${my_files}
)

## Specify libraries to link a library or executable target against
target_link_directories(libmotioncapture PUBLIC
  ${my_link_directories}
)
target_link_libraries(libmotioncapture
  ${my_libraries}
)
set_property(TARGET libmotioncapture PROPERTY POSITION_INDEPENDENT_CODE ON)

if (LIBMOTIONCAPTURE_BUILD_PYTHON_BINDINGS)
  # Python bindings
  find_package (Python3 COMPONENTS Interpreter Development)

  add_subdirectory(deps/pybind11 EXCLUDE_FROM_ALL)
  # find_package(Python COMPONENTS Interpreter Development)
  # find_package(pybind11 CONFIG)

  pybind11_add_module(motioncapture
    src/python_bindings.cpp
  )

  target_link_libraries(motioncapture
    PRIVATE
      libmotioncapture
  )
endif()

if (LIBMOTIONCAPTURE_BUILD_EXAMPLE)
  add_executable(vicon_node
    examples/vicon_node/vicon_main.cpp
  )
  # add lcm
  target_link_libraries(vicon_node
    libmotioncapture
    ${LCM_NAMESPACE}lcm
  )

# add realsense node
add_executable(realsense_t265_node
  examples/t265_node/t265_main.cpp
)
# link librealsense2 to the target
target_link_libraries(realsense_t265_node
  ${realsense2_LIBRARY}
  ${LCM_NAMESPACE}lcm
)


# rplidar node
file(GLOB_RECURSE LIDAR_SOURCES
    examples/lidar_node/sdk/src/*.cpp
    examples/lidar_node/sdk/src/**/*.cpp
    examples/lidar_node/sdk/src/**/**/*.cpp
    #examples/lidar_node/sdk/src/**/**/**/*.cpp
)
add_executable(rplidar_publisher
  ${LIDAR_SOURCES}
  examples/lidar_node/lidar_main.cpp
)
# link librealsense2 to the target
target_link_libraries(rplidar_publisher
  ${LCM_NAMESPACE}lcm
  stdc++
  pthread
)
target_include_directories(rplidar_publisher
  PUBLIC
  examples/lidar_node/sdk/include
  examples/lidar_node/sdk/src
)

# lidar to obst map
add_executable(rplidar_to_obst
  examples/lidar_node/lidar_to_map.cpp
)
# link librealsense2 to the target
target_link_libraries(rplidar_to_obst
  ${LCM_NAMESPACE}lcm
)

# lidar visualize
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

add_executable(lidar_visualize
  examples/lidar_node/lidar_visualize.cpp
)
# link librealsense2 to the target
target_link_libraries(lidar_visualize
  ${LCM_NAMESPACE}lcm
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
)


# lidar visualize raw
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

add_executable(lidar_visualize_raw
  examples/lidar_node/lidar_visualize_raw.cpp
)
# link librealsense2 to the target
target_link_libraries(lidar_visualize_raw
  ${LCM_NAMESPACE}lcm
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
)


# rrt node
set(RRT_LIBRARIES_DIR "deps/rrt_sdk")

include_directories(
  examples/rrt_planner
  ${RRT_LIBRARIES_DIR}/include
  examples/rrt_planner/OpenCV-YAMLHelper/include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

link_directories(
  ${RRT_LIBRARIES_DIR}/build)

add_executable(rrt_node
  examples/rrt_planner/rrt_main.cpp
  )

target_link_libraries(rrt_node
  ${EIGEN3_LIBS}
  ${OpenCV_LIBS}
  ${LCM_NAMESPACE}lcm
  planner
  )

endif()

