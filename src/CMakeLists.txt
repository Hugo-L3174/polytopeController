set(controller_SRC
  PolytopeController.cpp
  utils/MCStabilityPolytope.cpp
  utils/PointProjector.cpp
  utils/DynamicPolytope.cpp
  utils/WrenchCones.cpp
)

set(controller_HDR
  PolytopeController.h
  utils/MCStabilityPolytope.h
  utils/PointProjector.h
  utils/DynamicPolytope.h
  utils/WrenchCones.h
)

add_library(polito SHARED IMPORTED)
# assuming politopix is at the same level as this folder
set_target_properties(polito PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../../politopix/bin/libpolito.so)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DPolytopeController_EXPORTS")
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  mc_rtc::mc_control_fsm
  Stabiliplus::ProblemDescriptor
  Stabiliplus::StabilityPolytope
  sch-core::sch-core
  polito
  )
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)

add_controller(${PROJECT_NAME}_controller lib.cpp "")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

add_subdirectory(states)
