cmake_minimum_required(VERSION 3.24)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_NAME DemoController)
set(PROJECT_DESCRIPTION "DemoController")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

option(INSTALL_MUJOCO_OBJECTS "Install mc_mujoco objects description" ON)

# Check if the project is built inside mc_rtc
if (NOT TARGET mc_rtc::mc_control)
    find_package(mc_rtc REQUIRED)
endif ()

find_package(baseline_walking_controller REQUIRED)
find_package(baseline_footstep_planner REQUIRED)
if(INSTALL_MUJOCO_OBJECTS)
    find_package(mc_mujoco REQUIRED)
endif()

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/DemoController.in.yaml
        "${CMAKE_CURRENT_BINARY_DIR}/etc/DemoController.yaml"
)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/DemoController.yaml"
        DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc"
)

