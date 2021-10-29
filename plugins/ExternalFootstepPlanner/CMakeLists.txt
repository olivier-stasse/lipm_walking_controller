include(FindPkgConfig)
pkg_check_modules(mc_plugin_footstep_plan_msgs REQUIRED IMPORTED_TARGET mc_plugin_footstep_plan_msgs)

set(ExternalFootstepPlanner_data_SRC
  src/Plan.cpp
  src/Request.cpp
)

set(ExternalFootstepPlanner_data_HDR
  include/ExternalFootstepPlanner/Plan.h
  include/ExternalFootstepPlanner/Request.h
)
add_library(ExternalFootstepPlanner_data SHARED "${ExternalFootstepPlanner_data_SRC}" "${ExternaleFootstepPlanner_data_HDR}")
if(TARGET Eigen3::Eigen)
  target_link_libraries(ExternalFootstepPlanner_data PUBLIC Eigen3::Eigen)
else()
  target_include_directories(ExternalFootstepPlanner_data SYSTEM INTERFACE "${EIGEN3_INCLUDE_DIR}")
endif()
target_include_directories(ExternalFootstepPlanner_data PUBLIC $<INSTALL_INTERFACE:include> $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

install(TARGETS ExternalFootstepPlanner_data
  EXPORT "${TARGETS_EXPORT_NAME}"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(
  DIRECTORY include/ExternalFootstepPlanner DESTINATION include)


set(ExternalFootstepPlanner_SRC
  src/ExternalFootstepPlannerPlugin.cpp
  src/ExternalFootstepPlanner.cpp
  src/OnlineFootstepPlanner.cpp
)

set(ExternalFootstepPlanner_HDR
  include/ExternalFootstepPlanner/ExternalFootstepPlannerPlugin.h
  include/ExternalFootstepPlanner/ExternalFootstepPlanner.h
  include/ExternalFootstepPlanner/OnlineFootstepPlanner.h
)


option(AUTOLOAD_ExternalFootstepPlannerPlugin_PLUGIN "Automatically load ExternalFootstepPlanner plugin" OFF)
add_plugin(ExternalFootstepPlannerPlugin "${ExternalFootstepPlanner_SRC}" "${ExternalFootstepPlanner_HDR}")
target_include_directories(ExternalFootstepPlannerPlugin PUBLIC $<INSTALL_INTERFACE:include> $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(ExternalFootstepPlannerPlugin PUBLIC mc_rtc::mc_rtc_ros PkgConfig::mc_plugin_footstep_plan_msgs ExternalFootstepPlanner_data)
set_target_properties(ExternalFootstepPlannerPlugin PROPERTIES OUTPUT_NAME ExternalFootstepPlanner)
install(FILES etc/ExternalFootstepPlanner.yaml DESTINATION ${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc/)