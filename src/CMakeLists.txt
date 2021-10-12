set(controller_SRC
  controller.cpp
)

set(controller_HDR
  controller.h
)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DHandReacher_EXPORT")
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm mc_rtc::mc_rtc_ros)
target_link_libraries(${PROJECT_NAME} PUBLIC test_controller::ROS)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)


add_controller(${PROJECT_NAME}_controller lib.cpp "")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

add_subdirectory(states)

