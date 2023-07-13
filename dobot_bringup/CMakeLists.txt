cmake_minimum_required(VERSION 3.5)
project(dobot_bringup)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED COMPONENTS rclcpp rclcpp_action rclcpp_components)
find_package(Boost REQUIRED COMPONENTS system)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(dobot_bringup
        SHARED
        src/tcp_socket.cpp)

target_include_directories(
        dobot_bringup
        PUBLIC
        include
)

ament_target_dependencies(
        dobot_bringup
        rclcpp
)



# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/RobotStatus.msg"
#   "msg/ToolVectorActual.msg"
#   "srv/EnableRobot.srv"
#   "srv/DisableRobot.srv"
#   "srv/ClearError.srv"
#   "srv/ResetRobot.srv"
#   "srv/SpeedFactor.srv"
#   "srv/User.srv"
#   "srv/Tool.srv"
#   "srv/RobotMode.srv"
#   "srv/PayLoad.srv"
#   "srv/DO.srv"
#   "srv/DOExecute.srv"
#   "srv/ToolDO.srv"
#   "srv/ToolDOExecute.srv"
#   "srv/AO.srv"
#   "srv/AOExecute.srv"
#   "srv/AccJ.srv"
#   "srv/AccL.srv"
#   "srv/SpeedJ.srv"
#   "srv/SpeedL.srv"
#   "srv/Arch.srv"
#   "srv/CP.srv"
#   "srv/LimZ.srv"
#   "srv/SetArmOrientation.srv"
#   "srv/PowerOn.srv"
#   "srv/RunScript.srv"
#   "srv/StopScript.srv"
#   "srv/PauseScript.srv"
#   "srv/ContinueScript.srv"
#   #GetHoldRegs.srv
#   #SetHoldRegs.srv
#   "srv/SetSafeSkin.srv"
#   "srv/SetObstacleAvoid.srv"
#   #GetTraceStartPose.srv
#   #GetPathStartPose.srv
#   #PositiveSolution.srv
#   "srv/SetCollisionLevel.srv"
#   "srv/EmergencyStop.srv"



#   "srv/JointMovJ.srv"
#   "srv/MovJ.srv"
#   "srv/MovL.srv"
#   "srv/Jump.srv"
#   "srv/RelMovJ.srv"
#   "srv/RelMovL.srv"
#   #MovLIO
#   #MovJIO
#   "srv/Arc.srv"
#   "srv/Circle.srv"
#   "srv/ServoJ.srv"
#   "srv/ServoP.srv"
#   "srv/Sync.srv"
#   "srv/StartTrace.srv"
#   "srv/StartPath.srv"
#   "srv/StartFCTrace.srv"
#   "srv/MoveJog.srv"
  
#   DEPENDENCIES std_msgs 
# )


install(
  TARGETS dobot_bringup
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
        include
)
ament_export_libraries(
        dobot_bringup
)
ament_export_dependencies(
        rclcpp
)

ament_package()
