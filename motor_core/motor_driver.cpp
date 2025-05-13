#include "motor_driver.h"

/*******************************************************************************
* DrivingMotorDriver メソッド
*******************************************************************************/
/* 初期化 */
bool DrivingMotorDriver::init(void)
{
    // モーター動作LED
    this->enableLED.setLEDOutput(this->motor_moving_pin);
    // 右モーター
    this->rightMotor.setMotorOutput(this->right_back_rot_pin, this->right_forward_rot_pin, this->right_pwm_pin);
    // 左モーター
    this->leftMotor.setMotorOutput(this->left_back_rot_pin, this->left_forward_rot_pin, this->left_pwm_pin);
    return true;
}

/* 例外検出 */
bool DrivingMotorDriver::catchException(void)
{
    if (this->detectLostAt < this->runTime.milliseconds() && this->currentMotorL != 0 && this->currentMotorR != 0)
    {
        this->currentMotorL = 0;
        this->currentMotorR = 0;
        this->sleepMotors();
        this->enableLED.LEDOff();
        return true;
    }
    return false;
}

/**/
void DrivingMotorDriver::updateLastCommunicatedAt(void)
{
    this->lastCommunicatedAt = this->runTime.milliseconds();
    this->detectLostAt = this->lastCommunicatedAt + this->detect_lost_connection_ms;
}

/* ステアリング */
void DrivingMotorDriver::motorSteering(float linear_x, float angle_z)
{
    float right_wheel_cmd, left_wheel_cmd;
    right_wheel_cmd = linear_x - (angle_z * this->wheel_separation / 2);
    left_wheel_cmd = linear_x + (angle_z * this->wheel_separation / 2);
    this->setMotorsSpeed((int)right_wheel_cmd, (int)left_wheel_cmd);
}

/* 各モーター速度の設定 */
void DrivingMotorDriver::setMotorsSpeed(int right, int left)
{
    // 右モーター
    this->rightMotor.setMotorSpeed(right);
    this->currentMotorR = this->rightMotor.getMotorSpeed();
    // 左モーター
    this->leftMotor.setMotorSpeed(left);
    this->currentMotorL = this->leftMotor.getMotorSpeed();
}

/* 各モーター停止 */
void DrivingMotorDriver::sleepMotors(void)
{
    this->setMotorsSpeed(0, 0);     // モーター停止
}

/*******************************************************************************
* ArmMotorDriver メソッド
*******************************************************************************/
/* 初期化 */
bool ArmMotorDriver::init(void)
{
    this->Shoulder.setMotorOutput(this->joint_1_motor_pin, this->joint_1_def_angle);    // Joint1(Shoulder)
    this->Elbow.setMotorOutput(this->joint_2_motor_pin, this->joint_2_def_angle);       // Joint2(Elbow)
    this->Wrist.setMotorOutput(this->joint_3_motor_pin, this->joint_3_def_angle);       // Joint3(Wrist)
    this->Hand.setMotorOutput(this->joint_4_motor_pin, this->joint_4_def_angle);        // Joint4(Hand)
    this->updownMotor.setMotorOutput(this->arm_up_pin, this->arm_down_pin, this->arm_pwm_pin);
    return true;
}

/* アーム機構上下（上昇：L3↑, 下降：L3↓） */
void ArmMotorDriver::UpDownControl(int linear)
{
    if (linear == 0)
    {
        this->updownMotor.stop(false);
    }
    else
    {
        this->updownMotor.move(linear);
    }
}

/* アーム旋回（左：R3←, 右：R3→） */
void ArmMotorDriver::ShoulderControl(int angle)
{
    this->Shoulder.setMotorAngle(angle);
}

/* 手先の前後移動（前：R3↑, 後：R3↓） */
void ArmMotorDriver::FrontBackControl(int elbow_angle, int wrist_angle)
{
    this->ElbowControl(elbow_angle);
    this->WristControl(wrist_angle);
}

/* 肘上下 */
void ArmMotorDriver::ElbowControl(int angle)
{
    this->Elbow.setMotorAngle(angle);
}

/* 手首上下（上：△+R1, 下：△+R2）*/
void ArmMotorDriver::WristControl(int angle)
{
    this->Wrist.setMotorAngle(angle);
}

/* 手先開閉（開：〇+R1, 閉：〇+R2）*/
void ArmMotorDriver::HandControl(int angle)
{
    this->Hand.setMotorAngle(angle);
}

/* 全関節を初期角度に戻す */
void ArmMotorDriver::AllJointReset(void)
{
    this->Shoulder.setMotorAngle(this->joint_1_def_angle);
    this->Elbow.setMotorAngle(this->joint_2_def_angle);
    this->Wrist.setMotorAngle(this->joint_3_def_angle);
    this->Hand.setMotorAngle(this->joint_4_def_angle);
}
