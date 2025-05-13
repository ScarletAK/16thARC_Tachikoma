#ifndef TACHIKOMA_OPERATION_H_
#define TACHIKOMA_OPERATION_H_

#include <ros/ros.h>
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
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

/* タチコマ制御メインクラス */
class Operation
{
public:
    Operation();    // コンストラクタ
    ~Operation(){}  // デストラクタ
    
    // Callback functions for subscriber.
    void joy_cmd_callback(const sensor_msgs::Joy &joy_msg);
    void timer_callback(const ros::TimerEvent &event);

    // Send functions for publisher.
    void cmd_vel_send(const sensor_msgs::Joy &joy_msg);
    void cmd_hand_send(const sensor_msgs::Joy &joy_msg);

private:
    bool is_emergency;  // 非常停止の有無

    // コントローラープッシュ操作（PS3/4 DualShock）
    const int ds_axes_stick_left_leftwards = 0;     // L3スティック左右
    const int ds_axes_stick_left_upwards = 1;       // L3スティック上下
    const int ds_axes_stick_right_leftwards = 2;    // R3スティック左右
    const int ds_axes_stick_right_upwards = 3;      // R3スティック上下
    const int ds_button_select_share = 0;   // PS3:SELECT / PS4:SHARE ボタン
    const int ds_button_l3 = 1;             // L3プッシュ
    const int ds_button_r3 = 2;             // R3プッシュ
    const int ds_button_start_options = 3;  // PS3:START / PS4:OPTIONS ボタン
    const int ds_button_up = 4;             // ↑ボタン
    const int ds_button_right = 5;          // →ボタン
    const int ds_button_down = 6;           // ↓ボタン
    const int ds_button_left = 7;           // ←ボタン
    const int ds_button_l2 = 8;             // L2ボタン
    const int ds_button_r2 = 9;             // R2ボタン
    const int ds_button_l1 = 10;            // L1ボタン
    const int ds_button_r1 = 11;            // R1ボタン
    const int ds_button_triangle = 12;      // △ボタン
    const int ds_button_circle = 13;        // 〇ボタン
    const int ds_button_cross = 14;         // ×ボタン
    const int ds_button_square = 15;        // □ボタン
    const int ds_button_ps = 16;            // PSボタン

    // ROS NodeHandle
    ros::NodeHandle nh_;

    // ROS Topic Publishers
    ros::Publisher cmd_vel_pub;
    ros::Publisher cmd_hand_pub;

    // ROS Topic Subscribers
    ros::Subscriber joy_cmd_sub;

    // ROS Others
    ros::Timer timer;
    sensor_msgs::Joy last_joy;
}

#endif