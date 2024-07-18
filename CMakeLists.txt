cmake_minimum_required(VERSION 3.5)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME PolytopeController)
set(PROJECT_DESCRIPTION "PolytopeController")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
  find_package(Stabiliplus REQUIRED)
  find_package(sch-core REQUIRED)
  find_package(eigen-cdd REQUIRED)
  find_package(polito REQUIRED)
endif()

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/PolytopeController.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/PolytopeController.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/PolytopeController.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
