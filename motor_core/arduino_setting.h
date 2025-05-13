#ifndef ARDUINO_SETTING_H_
#define ARDUINO_SETTING_H_

#include <Arduino.h>
#include <Servo.h>

/*******************************************************************************
* 時間関連設定クラス
*******************************************************************************/
/* 経過時間表示設定クラス */
class TimeSetting
{
public:
    TimeSetting(){}    // コンストラクタ
    ~TimeSetting(){}   // デストラクタ
    unsigned long milliseconds(void){ return millis(); }    // ミリ秒
    unsigned long microseconds(void){ return micros(); }    // マイクロ秒
};

/* 時間停止設定クラス */
class WaitSetting
{
public:
    WaitSetting(){}     // コンストラクタ
    ~WaitSetting(){}    // デストラクタ
    void wait(unsigned long mmsecond){ delay(mmsecond); }   // ミリ秒
    void waitMS(unsigned long microsecond){ delayMicroseconds(microsecond); }   // マイクロ秒
};

/*******************************************************************************
* 各種HW設定クラス
*******************************************************************************/
/* LED設定クラス */
class LED
{
protected:
    int pin_no;     // LEDピン番号

public:
    LED(){}         // コンストラクタ
    ~LED(){}        // デストラクタ
    void setLEDOutput(int led_pin);    // LED出力設定
    void LEDOn(void){ digitalWrite(this->pin_no, HIGH); }   // LED点灯
    void LEDOff(void){ digitalWrite(this->pin_no, LOW); }   // LED消灯
    void LEDflash(unsigned long mmsecond)                   // LED点滅
};

/* サーボモーター設定クラス */
class ServoMotor : public Servo
{
protected:
    int pwm_pin_no;     // PWMピン番号
    int default_angle;  // 基準角度
    
public:
    ServoMotor(){}      // コンストラクタ
    ~ServoMotor(){ this->detach(); }    // デストラクタ
    void setMotorOutput(int pwm_pin, int default_angle);                // モーター出力設定
    void setMotorAngle(int angle){ this->write(angle); }                // モーター角度設定
    void resetMotorAngle(void){ this->write(this->default_angle) };     // モーター角度を基準値に戻す
    int getMotorAngle(void){ return this->read(); }                     // モーター角度取得
};

/* ブラシレスモーター設定クラス */
class BlushlessMotor : public Servo
{
protected:
    int pwm_pin_no;         // PWMピン番号
    const int default_pulse = 1500;     // 基準パルス値
    const int pulse_width = 500;        // パルス幅
    
public:
    BlushlessMotor(){}      // コンストラクタ
    ~BlushlessMotor(){ this->detach(); }    // デストラクタ
    void setMotorOutput(int pwm_pin);   // モーター出力設定
    void setMotorSpeed(int intValue);   // モーター回転速度設定
    int getMotorSpeed(void);            // モーター回転速度取得
};

/* DCモーター設定クラス */
class DCMotor
{
public:
    bool isRealMotor;   // モーター実機の有無

private:
    bool isPwmControl;  // PWM制御の有無
    int pwm_value;      // PWM制御値を出力（テスト用）

protected:
    int b_pin_no;       // 逆回転命令ピン番号
    int f_pin_no;       // 正回転命令ピン番号
    int pwm_pin_no;     // PWMピン番号

public:
    DCMotor(){}     // コンストラクタ
    ~DCMotor(){}    // デストラクタ
    void setMotorOutput(int b_pin, int f_pin, int pwm_pin);     // DCモーター出力設定
    void setMotorSpeed(int intValue);                   // DCモーター速度設定
    void setMotorBreak(void);                           // DCモーターブレーキ
    int getMotorSpeed(void){ return this->pwm_value };  // DCモーター速度を取得
};

/* 上下用モーター設定クラス */
class UpDownMotor : public DCMotor
{
public:
    UpDownMotor(){}     // コンストラクタ
    ~UpDownMotor(){}    // デストラクタ
    void move(int pwm_value);   // 上昇／下降
    void stop(bool emergency);  // 上昇／下降停止
};

#endif