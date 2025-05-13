#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#include "arduino_setting.h"
#include <cmath>

/* 走行機構制御クラス */
class DrivingMotorDriver
{
private:
    const int right_forward_rot_pin = 2;
    const int right_back_rot_pin = 3;
    const int right_pwm_pin = 4;
    const int left_forward_rot_pin = 5;
    const int left_back_rot_pin = 6;
    const int left_pwm_pin = 7;
    const int motor_moving_pin = 13;

    const double wheel_separation = 0.30;   // タイヤの間隔(meter)

    LED enableLED;          // モーター駆動LED
    TimeSetting runTime;    // 経過時間
    DCMotor rightMotor;     // 右モーター
    DCMotor leftMotor;      // 左モーター

    const unsigned long detect_lost_connection_ms = 3000;
    unsigned long lastCommunicatedAt = 0;
    unsigned long detectLostAt = 0;
    int currentMotorL = 0;
    int currentMotorR = 0;

public:
    DrivingMotorDriver(){}      // コンストラクタ
    ~DrivingMotorDriver(){}     // デストラクタ
    bool init(void);            // 初期化
    bool catchException(void);          // 例外検出
    void updateLastCommunicatedAt(void);        // 
    void motorSteering(float linear_x, float angle_z);  // ステアリング

private:
    void setMotorsSpeed(int right, int left);   // 各モーター速度の設定
    void sleepMotors(void);                     // 各モーター停止
}

/* アーム機構制御クラス */
class ArmMotorDriver
{
private:
    const int joint_1_motor_pin = 4;
    const int joint_2_motor_pin = 5;
    const int joint_3_motor_pin = 6;
    const int joint_4_motor_pin = 7;
    const int arm_up_pin = 8;
    const int arm_down_pin = 9;
    const int arm_pwm_pin = 10;
    // 各軸の初期角度を設定
    const int joint_1_def_angle = 90;
    const int joint_2_def_angle = 90;
    const int joint_3_def_angle = 90;
    const int joint_4_def_angle = 90;

    ServoMotor Shoulder;    // Joint1
    ServoMotor Elbow;       // Joint2
    ServoMotor Wrist;       // Joint3
    ServoMotor Hand;        // Joint4
    UpDownMotor updownMotor;    // アーム機構上下モーター

public:
    ArmMotorDriver(){}      // コンストラクタ
    ~ArmMotorDriver(){}     // デストラクタ
    bool init(void);        // 初期化
    void UpDownControl(int linear);     // アーム機構上下（上昇：×+R1, 下降：×+R2）
    void ShoulderControl(int angle);    // アーム旋回（左：R3←, 右：R3→）
    void FrontBackControl(int elbow_angle, int wrist_angle);    // 手先の前後移動（前：R3↑, 後：R3↓）
    void ElbowControl(int angle);       // 肘上下
    void WristControl(int angle);       // 手首上下（上：△+R1, 下：△+R2）
    void HandControl(int angle);        // 手先開閉（開：〇+R1, 閉：〇+R2）
    void AllJointReset(void);           // 全関節を初期角度に戻す
}

#endif