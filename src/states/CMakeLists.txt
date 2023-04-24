add_fsm_state_simple(PolicyClient)
target_link_libraries(PolicyClient PUBLIC
  "${TORCH_LIBRARIES}"
)
set_target_properties(PolicyClient
  PROPERTIES
  INSTALL_RPATH "${PROJECT_SOURCE_DIR}/ext/libtorch/lib"
  INSTALL_RPATH_USE_LINK_PATH TRUE
  CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

add_fsm_state_simple(StabilizerConfiguration)

add_fsm_state_simple(ModifyGains)

add_fsm_data_directory(data)
