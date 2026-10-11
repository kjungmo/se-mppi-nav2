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
#    => escape_critic (and anything linking its xt-typed API) and
#       mppi_rollout_check get XTENSOR_USE_XSIMD + -mavx2 -mfma. The env must
#       hold the xsimd generation its xtensor pairs with: xtensor 0.25.0 with
#       xsimd 11.2.0 (the pinned Jazzy env, = what its MPPI binary was built
#       with), xtensor 0.23.9 with xsimd 7.6.0 (the pinned Humble env; xsimd
#       14.x headers do not compile with it). scripts/check_simd_abi.py
#       compares the resulting storage type with the MPPI library's.
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
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

# Optional RPATH to the nav2_mppi_controller library this package is built
# against. OFF by default: nothing is added to what the toolchain itself puts
# on the run path, and the installed binaries carry no extra absolute path.
#
# Turn it ON when nav2_mppi_controller comes from an overlay workspace
# (scripts/build_mppi_overlay.sh, the remedy for binaries with negated rollout
# trigonometry). Without it the overlay is found at build time but not at run
# time: the conda toolchain's LDFLAGS give every binary of THIS package a
# DT_RPATH to <environment>/lib, and the loader searches a DT_RPATH before
# LD_LIBRARY_PATH, so the plugin loads the environment's libmppi_controller.so
# even with the overlay sourced. ON prepends the overlay's lib directory to
# that RPATH (to the linker flags themselves: CMake's BUILD_RPATH and
# INSTALL_RPATH entries would come after the toolchain's). Consequences: the
# entry also takes precedence over LD_LIBRARY_PATH; the install is tied to the
# overlay's absolute path (not relocatable, not for binary packages); and if
# the overlay is moved or deleted the loader falls back to the next entry, the
# environment's library, without a message (the rollout check then refuses it
# and names the library that was loaded).
option(NAV2_SE_CONTROLLER_MPPI_RPATH
  "Prepend the directory of the nav2_mppi_controller library found at configure time to the RPATH"
  OFF)
get_filename_component(MPPI_LIBRARY_DIR "${nav2_mppi_controller_DIR}/../../../lib" ABSOLUTE)
string(FIND "${CMAKE_SHARED_LINKER_FLAGS}" "-rpath,${MPPI_LIBRARY_DIR}" MPPI_DIR_ON_RPATH)
if(NAV2_SE_CONTROLLER_MPPI_RPATH)
  if(NOT EXISTS "${MPPI_LIBRARY_DIR}/libmppi_controller.so")
    message(FATAL_ERROR
      "NAV2_SE_CONTROLLER_MPPI_RPATH: libmppi_controller.so not found in ${MPPI_LIBRARY_DIR} "
      "(derived from nav2_mppi_controller_DIR=${nav2_mppi_controller_DIR})")
  endif()
  if(MPPI_DIR_ON_RPATH EQUAL -1)
    message(STATUS "NAV2_SE_CONTROLLER_MPPI_RPATH: RPATH entry ${MPPI_LIBRARY_DIR}")
    foreach(kind SHARED MODULE EXE)
      set(CMAKE_${kind}_LINKER_FLAGS
        "-Wl,-rpath,\"${MPPI_LIBRARY_DIR}\" ${CMAKE_${kind}_LINKER_FLAGS}")
    endforeach()
  else()
    message(STATUS
      "NAV2_SE_CONTROLLER_MPPI_RPATH: ${MPPI_LIBRARY_DIR} is already on the RPATH; nothing added")
  endif()
elseif(NOT "$ENV{CONDA_PREFIX}" STREQUAL "" AND MPPI_DIR_ON_RPATH EQUAL -1)
  message(WARNING
    "nav2_mppi_controller was found in ${MPPI_LIBRARY_DIR}, which is not on this build's RPATH. "
    "At run time the binaries of this package will load libmppi_controller.so from "
    "$ENV{CONDA_PREFIX}/lib instead. If that directory is an overlay you mean to use, configure "
    "with -DNAV2_SE_CONTROLLER_MPPI_RPATH=ON.")
endif()

# Humble (Nav2 1.1.x) API differences, guarded at compile time:
#  - nav2_core::Controller has no virtual reset() (added in 1.2), so
#    SafeEscapeController::reset() cannot be marked override and
#    controller_server never calls it (setPlan() still resets per goal);
#  - mppi::CriticData has no `goal`; EscapeCritic uses the path's last point.
set(SE_MPPI_API_DEFINITIONS "")
if(nav2_core_VERSION VERSION_LESS 1.2.0)
  set(SE_MPPI_API_DEFINITIONS NAV2_SE_CONTROLLER_HUMBLE_API)
endif()

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
target_compile_definitions(escape_critic PUBLIC ${SE_MPPI_API_DEFINITIONS})
target_link_libraries(escape_critic se_mppi_core)
# MPPI xtensor-ABI flags (see the boundary note at the top). PUBLIC so that any
# target linking escape_critic's xt-typed API (the gtest binary) compiles with
# the same tensor flavor — the mangled signatures include the allocator.
target_compile_definitions(escape_critic PUBLIC XTENSOR_USE_XSIMD)
target_compile_options(escape_critic PUBLIC ${MPPI_ABI_OPTIONS})

# =============================================================================
# mppi_rollout_check: asks the installed libmppi_controller where it puts a
# known rollout (configure-time guard against binaries with negated rollout
# trigonometry; mppi_rollout_check.hpp). It sizes and reads mppi's xt::xtensor
# members, so it needs the MPPI xtensor-ABI flags (boundary 1); PRIVATE,
# because its header is plain C++ and its users keep their own flags
# (boundary 2). No -ffast-math here or anywhere else in this package.
# =============================================================================
add_library(mppi_rollout_check SHARED
  src/mppi_rollout_check.cpp)
target_include_directories(mppi_rollout_check PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mppi_rollout_check
  nav2_mppi_controller nav2_costmap_2d rclcpp rclcpp_lifecycle)
target_compile_definitions(mppi_rollout_check PRIVATE XTENSOR_USE_XSIMD)
target_compile_options(mppi_rollout_check PRIVATE ${MPPI_ABI_OPTIONS})
target_link_libraries(mppi_rollout_check ${CMAKE_DL_LIBS})

# The same check from a shell: ros2 run nav2_se_controller check_mppi_rollout
add_executable(check_mppi_rollout src/check_mppi_rollout_main.cpp)
target_link_libraries(check_mppi_rollout mppi_rollout_check)

# =============================================================================
# safe_escape_controller: nav2_core controller plugin (subclasses MPPIController).
# Links se_mppi_core only -- NOT escape_critic -- so loading the controller does
# not pre-load the critic factory outside its class loader.
# =============================================================================
set(controller_dependencies
  nav2_mppi_controller nav2_core nav2_costmap_2d pluginlib rclcpp
  rclcpp_lifecycle geometry_msgs nav_msgs tf2 tf2_ros visualization_msgs diagnostic_msgs)
add_library(safe_escape_controller SHARED
  src/safe_escape_controller.cpp)
target_include_directories(safe_escape_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(safe_escape_controller ${controller_dependencies})
target_compile_definitions(safe_escape_controller PUBLIC ${SE_MPPI_API_DEFINITIONS})
target_link_libraries(safe_escape_controller se_mppi_core mppi_rollout_check)

install(TARGETS se_mppi_core escape_critic mppi_rollout_check safe_escape_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS check_mppi_rollout
  RUNTIME DESTINATION lib/${PROJECT_NAME})
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

  # Deployment tests (test/deployment/): the controller on a lifecycle node with
  # the stock MPPI optimizer, runtime parameter changes. Kept apart from the
  # algorithm unit tests in test/test_*.cpp, which the paper counts.
  ament_add_gtest(test_controller_runtime test/deployment/test_controller_runtime.cpp
    TIMEOUT 120)
  target_link_libraries(test_controller_runtime safe_escape_controller)
  ament_target_dependencies(test_controller_runtime
    nav2_mppi_controller nav2_costmap_2d rclcpp rclcpp_lifecycle tf2_ros diagnostic_msgs)

  ament_add_gtest(test_controller_behaviour test/deployment/test_controller_behaviour.cpp
    TIMEOUT 120)
  target_link_libraries(test_controller_behaviour safe_escape_controller)
  ament_target_dependencies(test_controller_behaviour
    nav2_mppi_controller nav2_costmap_2d rclcpp rclcpp_lifecycle tf2_ros)

  # Rollout direction of the installed libmppi_controller, the sign of the
  # controller's command, and the configure-time guard. Fails on a
  # nav2_mppi_controller binary with negated rollout trigonometry.
  ament_add_gtest(test_rollout_direction test/deployment/test_rollout_direction.cpp
    TIMEOUT 180)
  target_link_libraries(test_rollout_direction safe_escape_controller mppi_rollout_check)
  ament_target_dependencies(test_rollout_direction
    nav2_mppi_controller nav2_costmap_2d rclcpp rclcpp_lifecycle tf2_ros diagnostic_msgs)

  # The shell tool gives the same verdict: exit status 0 on a correct library.
  ament_add_test(check_mppi_rollout_tool
    COMMAND "${CMAKE_CURRENT_BINARY_DIR}/check_mppi_rollout"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)

  ament_add_gtest(test_task_boundary test/deployment/test_task_boundary.cpp)
  target_include_directories(test_task_boundary PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

  ament_add_gtest(test_failure_streak test/deployment/test_failure_streak.cpp)
  target_include_directories(test_failure_streak PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

  ament_add_gtest(test_escape_critic_params test/deployment/test_escape_critic_params.cpp)
  target_link_libraries(test_escape_critic_params escape_critic)
  ament_target_dependencies(test_escape_critic_params
    nav2_mppi_controller nav2_costmap_2d rclcpp rclcpp_lifecycle)
endif()

ament_export_include_directories(include)
ament_export_libraries(se_mppi_core escape_critic mppi_rollout_check safe_escape_controller)
ament_export_dependencies(${controller_dependencies})

ament_package()
