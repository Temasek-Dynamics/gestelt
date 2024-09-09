/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Trajectory Server (PX4-ROS2)
 * @file trajectory_server.cpp
 * @author John Tan (johntgz@nus.edu.sg) 
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class TrajectoryServer : public rclcpp::Node
{
public:
	TrajectoryServer() : Node("trajectory_server")
	{
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		actuator_cmd_publisher_ = this->create_publisher<ActuatorMotors>("/fmu/in/actuator_motors", 10);
		torque_setpoint_publisher_ = this->create_publisher<VehicleTorqueSetpoint>("/fmu/in/vehicle_torque_setpoint", 10);
		thrust_setpoint_publisher_ = this->create_publisher<VehicleThrustSetpoint>("/fmu/in/vehicle_thrust_setpoint", 10);

		offboard_setpoint_counter_ = 0;

		set_offb_timer_ = this->create_wall_timer(200ms, std::bind(&TrajectoryServer::setOffboardTimerCB, this));
		pub_act_timer_ = this->create_wall_timer(1ms, std::bind(&TrajectoryServer::pubActuatorTimerCmdCB, this));
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr set_offb_timer_;
	rclcpp::TimerBase::SharedPtr pub_act_timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<ActuatorMotors>::SharedPtr actuator_cmd_publisher_;

	rclcpp::Publisher<VehicleTorqueSetpoint>::SharedPtr torque_setpoint_publisher_;
	rclcpp::Publisher<VehicleThrustSetpoint>::SharedPtr thrust_setpoint_publisher_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	int offboard_ctrl_mode_{1}; // offboard control mode

	/* Time callbacks */
	void setOffboardTimerCB();
	void pubActuatorTimerCmdCB();

	/* Publisher methods */
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publishActuatorCmds();
	void publishTorqueSetpoint();
	void publishThrustSetpoint();
};

// Set offboard type
void TrajectoryServer::setOffboardTimerCB()
{
	if (offboard_setpoint_counter_ == 10) {
		// Change to Offboard mode after 10 setpoints

		// VEHICLE_CMD_DO_SET_MODE: |Mode, as defined by ENUM MAV_MODE| Custom mode |
		this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

		// Arm the vehicle
		this->arm();
	}

	// offboard_control_mode needs to be paired with trajectory_setpoint
	publish_offboard_control_mode();

	offboard_setpoint_counter_++;
}

// Publish actuator commands
void TrajectoryServer::pubActuatorTimerCmdCB()
{
	// publishActuatorCmds();

	publishTorqueSetpoint();
	publishThrustSetpoint();
}

/**
 * @brief Send a command to Arm the vehicle
 */
void TrajectoryServer::arm()
{
	// VEHICLE_CMD_COMPONENT_ARM_DISARM: Arms / Disarms a component |1 to arm, 0 to disarm
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, VehicleCommand::ARMING_ACTION_ARM);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void TrajectoryServer::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, VehicleCommand::ARMING_ACTION_DISARM);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}


/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void TrajectoryServer::publish_offboard_control_mode()
{
	OffboardControlMode msg{};

	if (offboard_ctrl_mode_ == 0){ // Position mode
		msg.position = true;
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;
		msg.thrust_and_torque = false;
		msg.direct_actuator = false;
	}
	if (offboard_ctrl_mode_ == 1){ // thrust_and_torque mode
		msg.position = false;
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;
		msg.thrust_and_torque = true;
		msg.direct_actuator = false;
	}
	if (offboard_ctrl_mode_ == 2){ // actuator mode
		msg.position = false;
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;
		msg.thrust_and_torque = false;
		msg.direct_actuator = true;
	}

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void TrajectoryServer::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	msg.position = {0.0, 0.0, -1.0};
	msg.yaw = -3.14; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void TrajectoryServer::publishActuatorCmds()
{
	ActuatorMotors msg{};

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;	// In microseconds

	// msg.reversible_flags     # bitset which motors are configured to be reversible
	// float32[12] control # range: [-1, 1], where 1 means maximum positive thrust,
	// 					# -1 maximum negative (if not supported by the output, <0 maps to NaN),
	// 					# and NaN maps to disarmed (stop the motors)

	msg.control[0] = 0.85;
	msg.control[1] = 0.85;
	msg.control[2] = 0.85;
	msg.control[3] = 0.85;

	actuator_cmd_publisher_->publish(msg);
}

void TrajectoryServer::publishTorqueSetpoint()
{
	VehicleTorqueSetpoint msg{};

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;	// In microseconds

	// xyz: torque setpoint about X, Y, Z body axis (normalized)
	msg.xyz = {0.0, 0.0, 0.1};

	torque_setpoint_publisher_->publish(msg);
}

void TrajectoryServer::publishThrustSetpoint()
{
	VehicleThrustSetpoint msg{};

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;	// In microseconds
	
	// xyz: thrust setpoint along X, Y, Z body axis [-1, 1]
	msg.xyz = {-0.0, -0.0, -0.75};

	thrust_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void TrajectoryServer::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;	// param1 is used to set the base_mode variable. 1 is MAV_MODE_FLAG_CUSTOM_MODE_ENABLED. See mavlink's MAV_MODE_FLAG 
	// For param2 and param3, refer to https://github.com/PX4/PX4-Autopilot/blob/0186d687b2ac3d62789806d341bd868b388c2504/src/modules/commander/px4_custom_mode.h
	msg.param2 = param2;	// custom_main_mode. 6 is PX4_CUSTOM_MAIN_MODE::PX4_CUSTOM_MAIN_MODE_OFFBOARD
	msg.command = command;	// command id
	msg.target_system = 1;		// System which should execute the command
	msg.target_component = 1;	// Component which should execute the command, 0 for all components
	msg.source_system = 1;		// System sending the command
	msg.source_component = 1;	//  Component / mode executor sending the command
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting trajectory server node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;
	auto node = std::make_shared<TrajectoryServer>();
	executor.add_node(node);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
