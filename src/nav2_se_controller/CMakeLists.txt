cmake_minimum_required(VERSION 3.10)
project(nav2_se_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# ---------------------------------------------------------------------------
# Two incompatible ABI boundaries force PER-TARGET compile flags:
#
# 1. escape_critic <-> libmppi_controller.so. The installed MPPI binary is
#    built with XTENSOR_USE_XSIMD + AVX2, so its xt::xtensor buffers use
#    xsimd::aligned_allocator<float, 32>. Those tensors cross the pluginlib
#    boundary by reference inside mppi::CriticData (costs, trajectories); a
#    critic compiled WITHOUT the flags reinterprets them with a different
#    storage type — an ODR violation that SIGSEGV'd the whole nav2 container
#    the first time EscapeCritic wrote data.costs live (2026-07-02 U-trap
#    benchmark; unit tests compile everything consistently, so never saw it).
#    Verified:  nm -DC libmppi_controller.so | grep uvector
#      -> xt::uvector<float, xsimd::aligned_allocator<float, 32ul>>
#    => escape_critic (and anything linking its xt-typed API) gets
#       XTENSOR_USE_XSIMD + -mavx2 -mfma. Needs xsimd 7.x in the env (the
#       generation xtensor 0.23 pairs with; 14.x headers do not compile).
#
# 2. se_mppi_core <-> OsqpEigen/Eigen from the env, which are built WITHOUT
#    AVX2. Compiling se_mppi_core with -mavx2 flips Eigen's alignment
#    (EIGEN_MAX_ALIGN_BYTES 16 -> 32) across that boundary and the CBF QP
#    segfaults (reproduced by the CbfSafetyFilter gtest under global -mavx2).
#    => se_mppi_core and safe_escape_controller keep default flags (their
#       flag-free builds are live-proven).
# ---------------------------------------------------------------------------
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
check_cxx_compiler_flag("-mfma" COMPILER_SUPPORTS_FMA)
set(MPPI_ABI_OPTIONS "")
if(COMPILER_SUPPORTS_AVX2)
  list(APPEND MPPI_ABI_OPTIONS -mavx2)
endif()
if(COMPILER_SUPPORTS_FMA)
  list(APPEND MPPI_ABI_OPTIONS -mfma)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

# =============================================================================
# se_mppi_core: plugin-free shared algorithm library (CBF filter, coordinator,
# obstacle tracker/clustering, path progress). Linked by BOTH plugin libraries.
# Keeping plugin factories OUT of this library avoids the class_loader
# "already loaded as a dependency" problem when the controller is loaded.
# =============================================================================
set(core_dependencies nav2_costmap_2d nav_msgs geometry_msgs)
add_library(se_mppi_core SHARED
  src/cbf_safety_filter.cpp
  src/escape_safety_coordinator.cpp
  src/multi_robot_coordinator.cpp
  src/obstacle_clustering.cpp
  src/static_occupancy_filter.cpp
  src/trajectory_predictor.cpp
  src/dynamic_obstacle_tracker.cpp
  src/path_progress.cpp)
target_include_directories(se_mppi_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(se_mppi_core ${core_dependencies})
target_link_libraries(se_mppi_core OsqpEigen::OsqpEigen Eigen3::Eigen)

# =============================================================================
# escape_critic: pluginlib critic for the MPPI optimizer (sampling-time escape).
# =============================================================================
set(critic_dependencies
  nav2_mppi_controller nav2_costmap_2d pluginlib rclcpp rclcpp_lifecycle)
add_library(escape_critic SHARED
  src/escape_critic.cpp
  src/repulsion.cpp
  src/gap_search.cpp)
target_include_directories(escape_critic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(escape_critic ${critic_dependencies})
target_link_libraries(escape_critic se_mppi_core)
# MPPI xtensor-ABI flags (see the boundary note at the top). PUBLIC so that any
# target linking escape_critic's xt-typed API (the gtest binary) compiles with
# the same tensor flavor — the mangled signatures include the allocator.
target_compile_definitions(escape_critic PUBLIC XTENSOR_USE_XSIMD)
target_compile_options(escape_critic PUBLIC ${MPPI_ABI_OPTIONS})

# =============================================================================
# safe_escape_controller: nav2_core controller plugin (subclasses MPPIController).
# Links se_mppi_core only -- NOT escape_critic -- so loading the controller does
# not pre-load the critic factory outside its class loader.
# =============================================================================
set(controller_dependencies
  nav2_mppi_controller nav2_core nav2_costmap_2d pluginlib rclcpp
  rclcpp_lifecycle geometry_msgs nav_msgs tf2 tf2_ros visualization_msgs)
add_library(safe_escape_controller SHARED
  src/safe_escape_controller.cpp)
target_include_directories(safe_escape_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(safe_escape_controller ${controller_dependencies})
target_link_libraries(safe_escape_controller se_mppi_core)

install(TARGETS se_mppi_core escape_critic safe_escape_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# Export the critic plugin (loaded by nav2_mppi_controller's CriticManager) and
# the controller plugin (loaded by nav2_core's controller server).
pluginlib_export_plugin_description_file(nav2_mppi_controller escape_critics.xml)
pluginlib_export_plugin_description_file(nav2_core se_controller_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip xmllint: it fetches the package-format schema from download.ros.org,
  # which is blocked (HTTP 403) by the web container's network policy.
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_nav2_se_controller
    test/test_entrapment_detector.cpp
    test/test_repulsion.cpp
    test/test_cbf_safety_filter.cpp
    test/test_escape_safety_coordinator.cpp
    test/test_dynamic_obstacle_tracker.cpp
    test/test_static_occupancy_filter.cpp
    test/test_trajectory_predictor.cpp
    test/test_conformal_calibrator.cpp
    test/test_multi_robot_coordinator.cpp
    test/test_path_progress.cpp
    test/test_gap_search.cpp
    test/test_escape_injection_live_shapes.cpp)
  target_include_directories(test_nav2_se_controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_nav2_se_controller se_mppi_core escape_critic)
  ament_target_dependencies(test_nav2_se_controller
    nav2_mppi_controller nav2_costmap_2d nav_msgs)

  # Runtime smoke test: load the plugins via pluginlib (needs them installed).
  ament_add_gtest(test_plugin_load test/test_plugin_load.cpp)
  ament_target_dependencies(test_plugin_load pluginlib nav2_core nav2_mppi_controller)
endif()

ament_export_include_directories(include)
ament_export_libraries(se_mppi_core escape_critic safe_escape_controller)
ament_export_dependencies(${controller_dependencies})

ament_package()
