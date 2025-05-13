#include "motor_operation.h"

/* コンストラクタ */
Operation::Operation() : nh_()
{
    ROS_INFO("in class constructor of Operation");
    ROS_INFO("Initializing Subscribers");
    this->joy_cmd_sub = nh_.subscribe("/joy", 10, &Operation::joy_cmd_callback, this);
    ROS_INFO("Initializing Publishers");
    this->cmd_vel_pub = nh_.advertise<getmetry_msgs::Twist>("/cmd_vel", 1, true);
    this->cmd_hand_pub = nh_.advertise<std_msgs::Float32MultiArray>("/cmd_hand", 1, true);
    ROS_INFO("Other Settings");
    this->is_emergency = false;
    this->timer = nh_.createTimer(ros::Duration(0.1), &Operation::timer_callback, this);
}

void Operation::joy_cmd_callback(const sensor_msgs::Joy &joy_msg)
{
    this->last_joy = joy_msg;
    // 非常停止
    if ((this->last_joy.buttons[this->ds_button_l3]==1)&&(this->last_joy.buttons[this->ds_button_r3]==1))
    {
        if (this->is_emergency==false) {
            this->is_emergency = true;
        }
        else {
            this->is_emergency = false;
        }
    }
}

void Operation::timer_callback(const ros::TimerEvent &event)
{
    this->cmd_vel_send(this->last_joy);
    this->cmd_hand_send(this->last_joy);
}

void Operation::cmd_vel_send(const sensor_msgs::Joy &joy_msg)
{
    float linear_x_spd, angular_z_spd;
    getmetry_msgs::Twist cmd_vel;
    
    linear_x_spd = joy_msg.axes[this->ds_axes_stick_left_upwards];
    angular_z_spd = joy_msg.axes[this->ds_axes_stick_left_leftwards];

    if (this->is_emergency==false)
    {
        if (joy_msg.buttons[this->ds_button_l1]==1)
        {
            cmd_vel.linear.x = linear_x_spd * 1.5;
            cmd_vel.angular.z = angular_z_spd * 1.5;
        }
        else if (joy_msg.buttons[this->ds_button_l2]==1)
        {
            cmd_vel.linear.x = linear_x_spd * 0.5;
            cmd_vel.angular.z = angular_z_spd * 0.5;
        }
        else
        {
            cmd_vel.linear.x = linear_x_spd * 1.0;
            cmd_vel.angular.z = angular_z_spd * 1.0;
        }
    }
    else
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
    }
    
    this->cmd_vel_pub.publish(cmd_vel);
}

void Operation::cmd_hand_send(const sensor_msgs::Joy &joy_msg)
{
    std_msgs::Float32MultiArray cmd_hand;
    cmd_hand.resize(7);

    if (this->is_emergency==false)
    {
        // アーム前後移動量
        cmd_hand.data[0] = joy_msg.axes[this->ds_axes_stick_left_leftwards];
        // アーム左右旋回量
        cmd_hand.data[1] = joy_msg.axes[this->ds_axes_stick_right_leftwards];
        // アーム機構上下移動量
        if (joy_msg.buttons[this->ds_button_cross]==1)
        {
            if (joy_msg.buttons[this->ds_button_r1]==1) {
                cmd_hand.data[2] = 1.0;
            }
            else if (joy_msg.buttons[this->ds_button_r2]==1) {
                cmd_hand.data[2] = -1.0;
            }
            else {
                cmd_hand.data[2] = 0.0;
            }
        }
        // 手首単独回転量
        if (joy_msg.buttons[this->ds_button_triangle]==1)
        {
            if (joy_msg.buttons[this->ds_button_r1]==1) {
                cmd_hand.data[3] = 1.0;
            }
            else if (joy_msg.buttons[this->ds_button_r2]==1) {
                cmd_hand.data[3] = -1.0;
            }
            else {
                cmd_hand.data[3] = 0.0;
            }
        }
        // ハンド開閉量
        if (joy_msg.buttons[this->ds_button_circle]==1)
        {
            if (joy_msg.buttons[this->ds_button_r1]==1) {
                cmd_hand.data[4] = 1.0;
            }
            else if (joy_msg.buttons[this->ds_button_r2]==1) {
                cmd_hand.data[4] = -1.0;
            }
            else {
                cmd_hand.data[4] = 0.0;
            }
        }
        // 手首を水平に戻す or 全関節を初期位置に戻す
        if (joy_msg.buttons[this->ds_button_square]==1)
        {
            if (joy_msg.buttons[this->ds_button_r1]==1) {
                cmd_hand.data[5] = 1.0;     // 手首を水平に戻す
                cmd_hand.data[6] = 0.0;
            }
            else if (joy_msg.buttons[this->ds_button_r2]==1) {
                cmd_hand.data[5] = 0.0;
                cmd_hand.data[6] = 1.0;     // 全関節を初期位置に戻す
            }
            else {
                cmd_hand.data[5] = 0.0;
                cmd_hand.data[6] = 0.0;
            }
        }
    }
    else
    {
        cmd_hand.data[0] = 0.0;     // アーム前後移動量
        cmd_hand.data[1] = 0.0;     // アーム左右旋回量
        cmd_hand.data[2] = 0.0;     // アーム機構上下移動量
        cmd_hand.data[3] = 0.0;     // 手首単独回転量
        cmd_hand.data[4] = 0.0;     // ハンド開閉量
        cmd_hand.data[5] = 0.0;     // 手首を水平に戻す
        cmd_hand.data[6] = 0.0;     // 全関節を初期位置に戻す
    }
    
    this->cmd_hand_pub.publish(cmd_hand);
}

int main(int argc, char** argv)
{
    // ROS setup
    ros::init(argc, argv, "/joy_operation");

    ROS_INFO("main: instantiating an object of type ExampleRosClass");
    Operation tachikoma_operation;
    
    ros::spin();
    return 0;
}
