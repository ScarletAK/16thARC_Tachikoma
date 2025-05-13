/*******************************************************************************
参考URL：
https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_with_open_manipulator/turtlebot3_with_open_manipulator_core/turtlebot3_with_open_manipulator_core.ino
*******************************************************************************/
#include "motor_core_config.h"

void setup() {
  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(hand_position_sub);
  //nh.advertise(motor_state_pub);

  driving_driver.init();
  arm_driver.init();
}

void loop() {
  nh.spinOnce();
  delay(1);
}

/* 車輪駆動 */
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg) {
  driving_driver.updateLastCommunicatedAt();

  linear_goal_velocity = cmd_vel_msg.linear.x;
  angular_goal_velocity = cmd_vel_msg.angular.z;

  linear_goal_velocity = constrain(linear_goal_velocity, min_linear_velocity, max_linear_velocity);
  angular_goal_velocity = constrain(angular_goal_velocity, min_angular_velocity, max_angular_velocity);

  driving_driver.motorSteering(linear_goal_velocity, angular_goal_velocity);
}

/* アーム手先移動 */
void commandHandCallback(const std_msgs::Float32MultiArray& cmd_hand_msg) {}

/* 駆動モーター状態 */
//void motorStateSender(void) {
//  motor_state_pub.publish(&motor_state_msg);
//}
