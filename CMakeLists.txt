cmake_minimum_required(VERSION 3.0.2)
project(picoxr)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  xr_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# nlohmann/json
include(FetchContent)
FetchContent_Declare(
  json
  URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz
)
FetchContent_MakeAvailable(json)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs xr_msgs tf2 tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(ROBOT_SDK_ROOT "/home/dyros20/pico_ws/src/XRoboToolkit-PC-Service/RoboticsService/bin/SDK")
set(PICORobotSDK_LIB_PATH "${ROBOT_SDK_ROOT}/x64")
set(PICORobotSDK_INCLUDE "${ROBOT_SDK_ROOT}/include")
set(PICORobotSDK_LIB_NAME "PXREARobotSDK")

if(NOT EXISTS "${PICORobotSDK_LIB_PATH}/lib${PICORobotSDK_LIB_NAME}.so")
  message(FATAL_ERROR "Robot SDK Library not found at: ${PICORobotSDK_LIB_PATH}")
endif()

if(NOT EXISTS "${PICORobotSDK_INCLUDE}/PXREARobotSDK.h")
  message(FATAL_ERROR "Robot SDK Header not found at: ${PICORobotSDK_INCLUDE}")
endif()


add_executable(talker src/publisher.cpp)
add_dependencies(talker ${catkin_EXPORTED_TARGETS})


target_include_directories(talker PUBLIC ${PICORobotSDK_INCLUDE})
target_link_directories(talker PUBLIC ${PICORobotSDK_LIB_PATH})
target_link_libraries(talker
  ${PICORobotSDK_LIB_NAME}
  nlohmann_json::nlohmann_json
  ${catkin_LIBRARIES}
)

install(TARGETS talker RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES ${PICORobotSDK_LIB_PATH}/lib${PICORobotSDK_LIB_NAME}.so
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
