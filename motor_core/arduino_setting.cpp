#include "arduino_setting.h"
#include <cstdlib>

/*******************************************************************************
* LED メソッド
*******************************************************************************/
/* LED出力設定 */
void LED::setLEDOutput(int led_pin)
{
    this->pin_no = led_pin;
    pinMode(this->pin_no, OUTPUT);
    this->LEDOff();         // 出力設定の段階では、LEDは消灯させる
}

/* LED点滅 */
void LED::LEDflash(unsigned long mmsecond)
{
    while(1) {
        this->LEDOn();
        delay(mmsecond);
        this->LEDOff();
        delay(mmsecond);
    }
}

/*******************************************************************************
* ServoMotor メソッド
*******************************************************************************/
/* サーボモーター出力設定 */
void ServoMotor::setMotorOutput(int pwm_pin, int default_angle)
{
    this->pwm_pin_no = pwm_pin;             // 出力ピン番号設定
    this->default_angle = default_angle;    // 基準角度設定
    this->attach(this->pwm_pin_no);
    this->resetMotorAngle();
}

/*******************************************************************************
* BlushlessMotor メソッド
*******************************************************************************/
/* ブラシレスモーター出力設定 */
void BlushlessMotor::setMotorOutput(int pwm_pin)
{
    this->pwm_pin_no = pwm_pin;         // 出力ピン番号設定
    this->attach(this->pwm_pin_no);
    this->setMotorSpeed(0);
}

/* モーター回転速度設定 */
void BlushlessMotor::setMotorSpeed(int intValue)
{
    // 入力値をmicrosecondに変換（microsecond：1000~2000）
    /*
    intValue > 0 -> microsecond < 1500 （正回転）
    intValue = 0 -> microsecond = 1500 （停止）
    intValue < 0 -> microsecond > 1500 （逆回転）
    */
    if (std::abs(intValue) > this->pulse_width) {
        if (intValue > 0) {
            intValue = this->pulse_width;
        } else {
            intValue = this->pulse_width * (-1);
        }
    }
    int microsecond = this->default_pulse - intValue;
    this->writeMicroseconds(microsecond);
}

/* モーター回転速度取得 */
int BlushlessMotor::getMotorSpeed(void)
{
    // サーボ角度を読み込み、microsecondに変換
    /*
    read() < 90 -> microsecond < 1500 （正回転）
    read() = 90 -> microsecond = 1500 （停止）
    read() > 90 -> microsecond > 1500 （逆回転）
    */
    int microsecond = this->default_pulse + ((this->read() - 90) / 90) * this->pulse_width;
    return (this->default_pulse - microsecond);
}

/*******************************************************************************
* DCMotor メソッド
*******************************************************************************/
/* DCモーター出力設定 */
void DCMotor::setMotorOutput(int b_pin, int f_pin, int pwm_pin)
{
    // 出力ピン番号設定
    this->f_pin_no = f_pin;
    this->b_pin_no = b_pin;
    this->pwm_pin_no = pwm_pin;
    
    this->pwm_value = 0;    // PWM制御値（テスト用）

    if ((this->f_pin_no > 0) && (this->b_pin_no > 0)) {
        // 回転ピン番号が両方とも0以上の場合、実機ありと見なす
        this->isRealMotor = true;
        pinMode(this->f_pin_no, OUTPUT);
        pinMode(this->b_pin_no, OUTPUT);
        if (this->pwm_pin_no >= 2) {
            // PWM制御ピンに接続したときのみ有効
            this->isPwmControl = true;
            pinMode(this->pwm_pin_no, OUTPUT);
        } else {
            this->isPwmControl = false;
        }
    } else {
        // 回転ピン番号が両方とも0以上の場合、実機なしと見なす（テスト用）
        this->isRealMotor = false;
        this->isPwmControl = false;
    }
    
    this->setMotorSpeed(0);     // 出力設定の段階では、モーターは停止させる
}

/* DCモーター速度設定（intValue：-255~255） */
void DCMotor::setMotorSpeed(int intValue)
{
    this->pwm_value = intValue;

    if (this->isRealMotor == true)
    {
        if (intValue == 0) {
            // 停止（但し、惰性で回転する）
            digitalWrite(this->f_pin_no, LOW);
            digitalWrite(this->b_pin_no, LOW);
        } else if (intValue > 0) {
            // 正回転
            digitalWrite(this->f_pin_no, HIGH);
            digitalWrite(this->b_pin_no, LOW);
        } else {
            // 逆回転
            digitalWrite(this->f_pin_no, LOW);
            digitalWrite(this->b_pin_no, HIGH);
        }
        // PWM制御が有効の場合
        if (this->isPwmControl == true) {
            analogWrite(this->pwm_pin_no, std::abs(intValue));
        }
    }
}

/* DCモーターブレーキ */
void DCMotor::setMotorBreak(void)
{
    if (this->isRealMotor == true)
    {
        digitalWrite(this->f_pin_no, HIGH);
        digitalWrite(this->b_pin_no, HIGH);
    }
}

/*******************************************************************************
* UpDownMotor メソッド
*******************************************************************************/
/* 上昇／下降 */
void UpDownMotor::move(int pwm_value)
{
    /*
    pwm_value > 0 : 上昇
    pwm_value < 0 : 下降
    */
    this->setMotorSpeed(pwm_value);
}

/* 上昇／下降停止 */
void UpDownMotor::stop(bool emergency)
{
    if (emergency == false) {
        this->setMotorBreak();
    } else {
        // 非常停止した場合、機構を手動で動かせるようにするためブレーキをかけない
        this->setMotorSpeed(0);
    }
}
