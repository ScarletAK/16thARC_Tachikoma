/*******************************************************************************
参考URL：
https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_with_open_manipulator/turtlebot3_with_open_manipulator_core/turtlebot3_with_open_manipulator_core_config.h
*******************************************************************************/
#ifndef TACHIKOMA_CORE_CONFIG_H_
#define TACHIKOMA_CORE_CONFIG_H_

#define USE_USBCON

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>

#include "motor_driver.h"

const float max_linear_velocity = 0.22;     // m/s
const float max_angular_velocity = 2.84;    // rad/s
const float min_linear_velocity = -max_linear_velocity;
const float min_angular_velocity = -max_angular_velocity;
const float velocity_linear_x = 0.01;
const float velocity_angular_z = 0.1;
const int scale_velocity_linear_x = 1;
const int scale_velocity_angular_z = 1;

float motor_value_rate = 0.1;
float linear_goal_velocity = 0.0;
float angular_goal_velocity = 0.0;

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
* Subscriber
*******************************************************************************/
/* 車輪駆動 */
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
ros::Subscriber<getmetry_msgs::Twist> cmd_vel_sub("/cmd_vel", commandVelocityCallback);
/* アーム手先移動 */
void commandHandCallback(const std_msgs::Float32MultiArray& cmd_hand_msg);
ros::Subscriber<std_msgs::Float32MultiArray> hand_position_sub("/cmd_hand", commandHandCallback);

/*******************************************************************************
* Publisher
*******************************************************************************/
/* 駆動モーター状態 */
//void motorStateSender(void);
//std_msgs::UInt16 motor_state_msg;
//ros::Publisher motor_state_pub("/motor_state_msg", &motor_state_msg);

/*******************************************************************************
* Declaration for motor
*******************************************************************************/
DrivingMotorDriver driving_driver;
ArmMotorDriver arm_driver;

#endif