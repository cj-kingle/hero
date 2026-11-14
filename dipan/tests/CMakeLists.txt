cmake_minimum_required(VERSION 3.16)
project(chassis_logic_tests LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()
add_executable(control_logic_test
  control_logic_test.cpp
  ../Tasks/robot_state.cpp ../Tasks/remote_command.cpp ../Tasks/chassis_task.cpp
  ../Tasks/c620_protocol.cpp
  ../Resources/Src/DT7.cpp ../Resources/Src/m3508.cpp
  ../components/control/pid.cpp ../components/control/gyro_bias.cpp)
target_include_directories(control_logic_test PRIVATE
  ../Tasks ../Resources/Inc ../components/control)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(control_logic_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
elseif(MSVC)
  target_compile_options(control_logic_test PRIVATE /UNDEBUG)
endif()
add_test(NAME control_logic COMMAND control_logic_test)
add_executable(mecanum_kinematics_test
  mecanum_kinematics_test.cpp ../Tasks/chassis_task.cpp ../components/control/pid.cpp)
target_include_directories(mecanum_kinematics_test PRIVATE
  ../Tasks ../Resources/Inc ../components/control)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(mecanum_kinematics_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
elseif(MSVC)
  target_compile_options(mecanum_kinematics_test PRIVATE /UNDEBUG)
endif()
add_test(NAME mecanum_kinematics COMMAND mecanum_kinematics_test)
add_executable(four_wheel_test
  four_wheel_test.cpp ../Tasks/chassis_motor_task.cpp ../Tasks/c620_protocol.cpp
  ../Tasks/chassis_task.cpp ../Tasks/robot_state.cpp ../Resources/Src/m3508.cpp
  ../components/control/pid.cpp)
target_include_directories(four_wheel_test PRIVATE
  ../Tasks ../Resources/Inc ../components/control)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(four_wheel_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
elseif(MSVC)
  target_compile_options(four_wheel_test PRIVATE /UNDEBUG)
endif()
add_test(NAME four_wheel COMMAND four_wheel_test)
add_executable(lower_runtime_test lower_runtime_test.cpp ../Tasks/lower_runtime.cpp
  ../Tasks/chassis_motor_task.cpp ../Tasks/c620_protocol.cpp ../Tasks/chassis_task.cpp
  ../Tasks/remote_command.cpp ../Tasks/robot_state.cpp ../Tasks/board_link.cpp
  ../Resources/Src/DT7.cpp ../Resources/Src/m3508.cpp ../components/control/pid.cpp)
target_sources(lower_runtime_test PRIVATE ../Tasks/yaw_calibration.cpp)
target_include_directories(lower_runtime_test PRIVATE comm_stubs ../Tasks ../Resources/Inc ../components/control)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(lower_runtime_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
endif()
add_test(NAME lower_runtime COMMAND lower_runtime_test)
add_executable(yaw_calibration_test yaw_calibration_test.cpp ../Tasks/yaw_calibration.cpp)
target_include_directories(yaw_calibration_test PRIVATE ../Tasks)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(yaw_calibration_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
endif()
add_test(NAME yaw_calibration COMMAND yaw_calibration_test)
add_executable(chassis_yaw_control_test chassis_yaw_control_test.cpp
  ../Tasks/chassis_motor_task.cpp ../Tasks/c620_protocol.cpp ../Tasks/chassis_task.cpp
  ../Tasks/robot_state.cpp ../Resources/Src/m3508.cpp ../components/control/pid.cpp)
target_include_directories(chassis_yaw_control_test PRIVATE ../Tasks ../Resources/Inc ../components/control)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(chassis_yaw_control_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
endif()
add_test(NAME chassis_yaw_control COMMAND chassis_yaw_control_test)
add_executable(gyro_bias_timing_test gyro_bias_timing_test.cpp ../components/control/gyro_bias.cpp)
target_include_directories(gyro_bias_timing_test PRIVATE ../components/control)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(gyro_bias_timing_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
endif()
add_test(NAME gyro_bias_timing COMMAND gyro_bias_timing_test)
add_executable(chassis_imu_processing_test chassis_imu_processing_test.cpp
  ../Tasks/imu_task.cpp ../components/control/gyro_bias.cpp
  ../components/ahrs/src/mahony.cpp ../components/ahrs/src/ahrs_base.cpp)
target_include_directories(chassis_imu_processing_test BEFORE PRIVATE imu_stubs
  ../Tasks ../components/control ../components/ahrs/inc)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(chassis_imu_processing_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
endif()
add_test(NAME chassis_imu_processing COMMAND chassis_imu_processing_test)

# Offline recorder uses the real core and an in-memory Flash backend.
add_executable(fault_log_test fault_log_test.cpp ../Tasks/fault_log.cpp)
target_include_directories(fault_log_test PRIVATE ../Tasks)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(fault_log_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
endif()
add_test(NAME fault_log COMMAND fault_log_test)
target_sources(lower_runtime_test PRIVATE ../Tasks/fault_log.cpp)
