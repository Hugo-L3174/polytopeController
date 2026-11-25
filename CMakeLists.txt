cmake_minimum_required(VERSION 3.22)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME PolytopeController)
set(PROJECT_DESCRIPTION "PolytopeController")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
  # find_package(Stabiliplus REQUIRED)
  find_package(sch-core REQUIRED)
  find_package(mc_dynamic_polytopes REQUIRED)
  find_package(DCM_VRPTask CONFIG REQUIRED)
  # find_package(eigen-cdd REQUIRED)
endif()

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/PolytopeController.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/PolytopeController.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/PolytopeController.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
set(MAINROBOT_NAME "RHPS1_forceshoe_forceshoe_lirmm_lirmm")
set(CONTROLLER_ROBOT_NAME "rhps1")
configure_file(etc/mc_rtc.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/mc_rtc_rhps1.yaml" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/mc_rtc_rhps1.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

set(MAINROBOT_NAME "RHPS1_forceshoe_forceshoe_lirmm_lirmm_MuJoCo")
set(CONTROLLER_ROBOT_NAME "rhps1")
configure_file(etc/mc_rtc.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/mc_rtc_rhps1_MuJoCo.yaml" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/mc_rtc_rhps1_MuJoCo.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

set(MAINROBOT_NAME "G1_Revo2")
set(CONTROLLER_ROBOT_NAME "g1_revo2")
configure_file(etc/mc_rtc.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/mc_rtc_g1.yaml" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/mc_rtc_g1.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

set(MAINROBOT_NAME "G1_Revo2_MuJoCo")
set(CONTROLLER_ROBOT_NAME "g1_revo2")
configure_file(etc/mc_rtc.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/mc_rtc_g1_MuJoCo.yaml" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/mc_rtc_g1_MuJoCo.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
