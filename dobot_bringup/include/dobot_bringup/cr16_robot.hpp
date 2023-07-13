/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/07
 *
 * <h2><center>&copy; COPYRIGHT 2021 DOBOT CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

// #include <string>
// #include <memory>
// #include "rclcpp/rclcpp.hpp"
// #include "control_msgs/action/follow_joint_trajectory.hpp"

// #include "rclcpp_action/rclcpp_action.hpp"
// #include <dobot_bringup/commander.hpp>
// #include <dobot_bringup/srv/enable_robot.hpp>

// #include <dobot_bringup/srv/disable_robot.hpp>
// #include <dobot_bringup/srv/clear_error.hpp>
// #include <dobot_bringup/srv/reset_robot.h>
// #include <dobot_bringup/srv/speed_factor.hpp>
// #include <dobot_bringup/srv/user.hpp>
// #include <dobot_bringup/srv/tool.hpp>
// #include <dobot_bringup/srv/robot_mode.hpp>
// #include <dobot_bringup/srv/pay_load.hpp>
// #include <dobot_bringup/srv/do.hpp>
// #include <dobot_bringup/srv/do_execute.hpp>
// #include <dobot_bringup/srv/tool_do.hpp>
// #include <dobot_bringup/srv/tool_do_execute.hpp>
// #include <dobot_bringup/srv/ao.hpp>
// #include <dobot_bringup/srv/ao_execute.hpp>
// #include <dobot_bringup/srv/acc_j.hpp>
// #include <dobot_bringup/srv/acc_l.hpp>
// #include <dobot_bringup/srv/speed_j.hpp>
// #include <dobot_bringup/srv/speed_l.hpp>
// #include <dobot_bringup/srv/arch.hpp>
// #include <dobot_bringup/srv/cp.hpp>
// #include <dobot_bringup/srv/lim_z.hpp>
// #include <dobot_bringup/srv/set_arm_orientation.hpp>
// #include <dobot_bringup/srv/power_on.hpp>
// #include <dobot_bringup/srv/run_script.hpp>
// #include <dobot_bringup/srv/stop_script.hpp>
// #include <dobot_bringup/srv/pause_script.hpp>
// #include <dobot_bringup/srv/continue_script.hpp>
// //#include <dobot_bringup/GethppoldRegs.hpp>
// //#include <dobot_bringup/SethppoldRegs.hpp>
// #include <dobot_bringup/srv/set_safe_skin.hpp>
// #include <dobot_bringup/srv/set_obstacle_avoid.hpp>

// #include <dobot_bringup/srv/set_collision_level.hpp>
// #include <dobot_bringup/srv/emergency_stop.hpp>




// #include <dobot_bringup/srv/mov_j.hpp>
// #include <dobot_bringup/srv/mov_l.hpp>
// #include <dobot_bringup/srv/jump.hpp>
// #include <dobot_bringup/srv/arc.hpp>
// #include <dobot_bringup/srv/sync.hpp>
// #include <dobot_bringup/srv/circle.hpp>
// #include <dobot_bringup/srv/servo_j.hpp>
// #include <dobot_bringup/srv/start_trace.hpp>
// #include <dobot_bringup/srv/start_path.hpp>
// #include <dobot_bringup/srv/start_fc_trace.hpp>
// #include <dobot_bringup/srv/move_jog.hpp>
// #include <dobot_bringup/srv/servo_p.hpp>
// #include <dobot_bringup/srv/rel_mov_j.hpp>
// #include <dobot_bringup/srv/rel_mov_l.hpp>
// #include <dobot_bringup/srv/joint_mov_j.hpp>
// #include <dobot_bringup/msg/robot_status.hpp>

// // using namespace actionlib;
// // using namespace control_msgs;

// /**
//  * CR5Robot
//  */

// auto timer = rclcpp::create_timer();
// class CR5Robot :
// {
// private:
//     double goal_[6];
//     uint32_t index_;
//     rclcpp::Timer
//     ros::Timer timer_;
//     ros::Timer movj_timer_;
//     double trajectory_duration_;
//     ros::NodeHandle control_nh_;
//     std::shared_ptr<CR5Commander> commander_;
//     std::vector<ros::ServiceServer> server_tbl_;

// public:
//     /**
//      * Ctor
//      * @param nh node handle
//      * @param name topic
//      */
//     CR5Robot(ros::NodeHandle& nh, std::string name);

//     /**
//      * CR5Robot
//      */
//     ~CR5Robot() override;

//     /**
//      * init
//      */
//     void init();

//     /**
//      * getJointState
//      * @param point
//      */
//     void getJointState(double* point);

//     /**
//      * getToolVectorActual
//      * @param val value
//      */
//     void getToolVectorActual(double* val);

//     /**
//      * isEnable
//      * @return ture enable, otherwise false
//      */
//     bool isEnable() const;

//     /**
//      * isConnected
//      * @return ture connected, otherwise false
//      */
//     bool isConnected() const;

// protected:
//     bool enableRobot(dobot_bringup::EnableRobot::Request& request, dobot_bringup::EnableRobot::Response& response);
//     bool disableRobot(dobot_bringup::DisableRobot::Request& request, dobot_bringup::DisableRobot::Response& response);
//     bool clearError(dobot_bringup::ClearError::Request& request, dobot_bringup::ClearError::Response& response);
//     bool resetRobot(dobot_bringup::ResetRobot::Request& request, dobot_bringup::ResetRobot::Response& response);
//     bool speedFactor(dobot_bringup::SpeedFactor::Request& request, dobot_bringup::SpeedFactor::Response& response);
//     bool user(dobot_bringup::User::Request& request, dobot_bringup::User::Response& response);
//     bool tool(dobot_bringup::Tool::Request& request, dobot_bringup::Tool::Response& response);
//     bool robotMode(dobot_bringup::RobotMode::Request& request, dobot_bringup::RobotMode::Response& response);
//     bool payload(dobot_bringup::PayLoad::Request& request, dobot_bringup::PayLoad::Response& response);
//     bool DO(dobot_bringup::DO::Request& request, dobot_bringup::DO::Response& response);
//     bool DOExecute(dobot_bringup::DOExecute::Request& request, dobot_bringup::DOExecute::Response& response);
//     bool toolDO(dobot_bringup::ToolDO::Request& request, dobot_bringup::ToolDO::Response& response);
//     bool toolDOExecute(dobot_bringup::ToolDOExecute::Request& request, dobot_bringup::ToolDOExecute::Response& response);
//     bool AO(dobot_bringup::AO::Request& request, dobot_bringup::AO::Response& response);
//     bool AOExecute(dobot_bringup::AOExecute::Request& request, dobot_bringup::AOExecute::Response& response);
//     bool accJ(dobot_bringup::AccJ::Request& request, dobot_bringup::AccJ::Response& response);
//     bool accL(dobot_bringup::AccL::Request& request, dobot_bringup::AccL::Response& response);
//     bool speedJ(dobot_bringup::SpeedJ::Request& request, dobot_bringup::SpeedJ::Response& response);
//     bool speedL(dobot_bringup::SpeedL::Request& request, dobot_bringup::SpeedL::Response& response);
//     bool arch(dobot_bringup::Arch::Request& request, dobot_bringup::Arch::Response& response);
//     bool cp(dobot_bringup::CP::Request& request, dobot_bringup::CP::Response& response);
//     bool limZ(dobot_bringup::LimZ::Request& request, dobot_bringup::LimZ::Response& response);
//     bool setArmOrientation(dobot_bringup::SetArmOrientation::Request& request, dobot_bringup::SetArmOrientation::Response& response);
//     bool powerOn(dobot_bringup::PowerOn::Request& request, dobot_bringup::PowerOn::Response& response);
//     bool runScript(dobot_bringup::RunScript::Request& request, dobot_bringup::RunScript::Response& response);
//     bool stopScript(dobot_bringup::StopScript::Request& request, dobot_bringup::StopScript::Response& response);
//     bool pauseScript(dobot_bringup::PauseScript::Request& request, dobot_bringup::PauseScript::Response& response);
//     bool continueScript(dobot_bringup::ContinueScript::Request& request, dobot_bringup::ContinueScript::Response& response);
// //    bool getHoldRegs(dobot_bringup::SpeedFactor::Request& request, dobot_bringup::SpeedFactor::Response& response);
// //    bool setHoldRegs(dobot_bringup::SpeedFactor::Request& request, dobot_bringup::SpeedFactor::Response& response);
//     bool setSafeSkin(dobot_bringup::SetSafeSkin::Request& request, dobot_bringup::SetSafeSkin::Response& response);
//     bool setObstacleAvoid(dobot_bringup::SetObstacleAvoid::Request& request, dobot_bringup::SetObstacleAvoid::Response& response);
// //    bool getTraceStartPose(dobot_bringup::SpeedFactor::Request& request, dobot_bringup::SpeedFactor::Response& response);
// //    bool getPathStartPose(dobot_bringup::SpeedFactor::Request& request, dobot_bringup::SpeedFactor::Response& response);
// //    bool positiveSolution(dobot_bringup::SpeedFactor::Request& request, dobot_bringup::SpeedFactor::Response& response);
// //    bool inverseSolution(dobot_bringup::SpeedFactor::Request& request, dobot_bringup::SpeedFactor::Response& response);
//     bool setCollisionLevel(dobot_bringup::SetCollisionLevel::Request& request, dobot_bringup::SetCollisionLevel::Response& response);
// //    bool handleTrajPoints(dobot_bringup::SpeedFactor::Request& request, dobot_bringup::SpeedFactor::Response& response);
// //    bool getSixForceData(dobot_bringup::SpeedFactor::Request& request, dobot_bringup::SpeedFactor::Response& response);
// //    bool getAngle(dobot_bringup::SpeedFactor::Request& request, dobot_bringup::SpeedFactor::Response& response);
// //    bool getPose(dobot_bringup::SpeedFactor::Request& request, dobot_bringup::SpeedFactor::Response& response);
//     bool emergencyStop(dobot_bringup::EmergencyStop::Request& request, dobot_bringup::EmergencyStop::Response& response);

//     bool movJ(dobot_bringup::MovJ::Request& request, dobot_bringup::MovJ::Response& response);
//     bool movL(dobot_bringup::MovL::Request& request, dobot_bringup::MovL::Response& response);
//     bool jointMovJ(dobot_bringup::JointMovJ::Request& request, dobot_bringup::JointMovJ::Response& response);
//     bool jump(dobot_bringup::Jump::Request& request, dobot_bringup::Jump::Response& response);
//     bool relMovJ(dobot_bringup::RelMovJ::Request& request, dobot_bringup::RelMovJ::Response& response);
//     bool relMovL(dobot_bringup::RelMovL::Request& request, dobot_bringup::RelMovL::Response& response);
//     //bool MovLIO(dobot_bringup::RelMovL::Request& request, dobot_bringup::RelMovL::Response& response);
//     //bool MovJIO(dobot_bringup::RelMovL::Request& request, dobot_bringup::RelMovL::Response& response);
//     bool arc(dobot_bringup::Arc::Request& request, dobot_bringup::Arc::Response& response);
//     bool circle(dobot_bringup::Circle::Request& request, dobot_bringup::Circle::Response& response);
//     bool servoJ(dobot_bringup::ServoJ::Request& request, dobot_bringup::ServoJ::Response& response);
//     bool servoP(dobot_bringup::ServoP::Request& request, dobot_bringup::ServoP::Response& response);
//     bool sync(dobot_bringup::Sync::Request& request, dobot_bringup::Sync::Response& response);
//     bool startTrace(dobot_bringup::StartTrace::Request& request, dobot_bringup::StartTrace::Response& response);
//     bool startPath(dobot_bringup::StartPath::Request& request, dobot_bringup::StartPath::Response& response);
//     bool startFCTrace(dobot_bringup::StartFCTrace::Request& request, dobot_bringup::StartFCTrace::Response& response);
//     bool moveJog(dobot_bringup::MoveJog::Request& request, dobot_bringup::MoveJog::Response& response);

// private:
//     void feedbackHandle(const ros::TimerEvent& tm,
//                         actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
//     void moveHandle(const ros::TimerEvent& tm, actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
//     void goalHandle(actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
//     void cancelHandle(actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
// };