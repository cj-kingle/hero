cmake_minimum_required(VERSION 3.16)
project(gimbal_unit_tests LANGUAGES CXX)
add_executable(variable_kd_test variable_kd_test.cpp ../components/control/pid.cpp)
target_include_directories(variable_kd_test PRIVATE ../components/control)
target_compile_features(variable_kd_test PRIVATE cxx_std_17)
target_compile_options(variable_kd_test PRIVATE -UNDEBUG)
add_test(NAME variable_kd COMMAND variable_kd_test)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()
add_executable(gyro_bias_timing_test gyro_bias_timing_test.cpp ../components/control/gyro_bias.cpp)
target_include_directories(gyro_bias_timing_test PRIVATE ../components/control)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(gyro_bias_timing_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
endif()
add_test(NAME gyro_bias_timing COMMAND gyro_bias_timing_test)
add_executable(upper_imu_recovery_test upper_imu_recovery_test.cpp
  ../Tasks/imu_task.cpp ../components/control/gyro_bias.cpp
  ../components/ahrs/src/mahony.cpp ../components/ahrs/src/ahrs_base.cpp)
target_include_directories(upper_imu_recovery_test BEFORE PRIVATE imu_stubs
  ../Tasks ../Resources/Inc ../components/control ../components/ahrs/inc)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(upper_imu_recovery_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
endif()
add_test(NAME upper_imu_recovery COMMAND upper_imu_recovery_test)
add_executable(dm_motor_test dm_motor_test.cpp
  ../Resources/Src/dm_protocol.cpp ../Resources/Src/dm_motor.cpp ../Resources/Src/dm_stop.cpp)
add_executable(gimbal_logic_test gimbal_logic_test.cpp
  ../Tasks/board_link.cpp ../Tasks/gimbal_task.cpp
  ../components/control/pid.cpp ../components/control/gyro_bias.cpp)
add_executable(single_axis_test single_axis_test.cpp ../Tasks/single_axis_test.cpp
  ../Resources/Src/dm_parameters.cpp ../Resources/Src/dm_protocol.cpp ../components/control/pid.cpp)
add_executable(gimbal_runtime_test gimbal_runtime_test.cpp ../Tasks/gimbal_runtime.cpp
  ../Tasks/gimbal_dual_axis.cpp
  ../Tasks/single_axis_test.cpp ../Tasks/board_link.cpp ../Tasks/gimbal_task.cpp
  ../Resources/Src/dm_parameters.cpp ../Resources/Src/dm_protocol.cpp
  ../Resources/Src/dm_motor.cpp ../Resources/Src/dm_stop.cpp ../components/control/pid.cpp
  ../Resources/Src/dm4340_drv.cpp ../Resources/Src/dm4340_motor.cpp)
target_include_directories(gimbal_runtime_test BEFORE PRIVATE runtime_stubs)
add_executable(dm4340_driver_test dm4340_driver_test.cpp
  ../Resources/Src/dm4340_drv.cpp ../Resources/Src/dm4340_motor.cpp
  ../Resources/Src/dm_motor.cpp ../Resources/Src/dm_protocol.cpp)
target_include_directories(dm4340_driver_test BEFORE PRIVATE runtime_stubs)
foreach(target gimbal_runtime_test dm4340_driver_test)
  target_precompile_headers(${target} PRIVATE runtime_stubs/driver_platform.hpp)
endforeach()
add_executable(gimbal_dual_axis_test gimbal_dual_axis_test.cpp ../Tasks/gimbal_dual_axis.cpp
  ../Tasks/gimbal_task.cpp ../Tasks/board_link.cpp ../Resources/Src/dm_parameters.cpp
  ../Resources/Src/dm_motor.cpp ../Resources/Src/dm_protocol.cpp ../components/control/pid.cpp
  ../Resources/Src/dm4340_motor.cpp ../Resources/Src/dm4340_drv.cpp)
target_include_directories(gimbal_dual_axis_test BEFORE PRIVATE runtime_stubs)
target_precompile_headers(gimbal_dual_axis_test PRIVATE runtime_stubs/driver_platform.hpp)
foreach(target dm_motor_test gimbal_logic_test single_axis_test gimbal_runtime_test gimbal_dual_axis_test dm4340_driver_test)
  target_include_directories(${target} PRIVATE ../Resources/Inc ../Tasks ../components/control)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE -Wall -Wextra -Werror -UNDEBUG)
  elseif(MSVC)
    target_compile_options(${target} PRIVATE /UNDEBUG)
  endif()
  add_test(NAME ${target} COMMAND ${target})
endforeach()
