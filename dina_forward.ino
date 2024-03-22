#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;

#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x70);  // адрес зависит от перемычек на плате (также попробуйте просканировать адрес: https://github.com/MAKblC/Codes/tree/master/I2C%20scanner)

uint8_t fifoBuffer[45];  // буфер
int error;
int previousAngle;

#define I2C_HUB_ADDR 0x70
#define EN_MASK 0x08
#define DEF_CHANNEL 0x00
#define MAX_CHANNEL 0x08

void setup() {
  Serial.begin(115200);
  Wire.begin();
  setBusChannel(0x03);
  // инициализация DMP
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);

  pwm.begin();
  // Частота (Гц)
  pwm.setPWMFreq(100);
  // Все порты выключены
  pwm.setPWM(8, 0, 4096);
  pwm.setPWM(9, 0, 4096);
  pwm.setPWM(10, 0, 4096);
  pwm.setPWM(11, 0, 4096);
  delay(5000);
  previousAngle = 90;  // ставим изначальный угол как курс
}

void loop() {
  setBusChannel(0x03);
  int currentAngle = angleMeasure();
  error = currentAngle - previousAngle;
  motorA_setpower(50 + error, true);
  motorB_setpower(50 - error, false);
}

// измерение угла в градусах от 0 до 360
int angleMeasure(void) {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // переменные для расчёта (ypr можно вынести в глобальеы переменные)
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    int angle = map(degrees(ypr[0]), -180, 180, 0, 360);
    Serial.println(angle);
    return angle;
  }
}


void motorA_setpower(float pwr, bool invert) {
  // Проверка, инвертирован ли мотор
  if (invert) {
    pwr = -pwr;
  }
  // Проверка диапазонов
  if (pwr < -100) {
    pwr = -100;
  }
  if (pwr > 100) {
    pwr = 100;
  }
  int pwmvalue = fabs(pwr) * 40.95;
  if (pwr < 0) {
    pwm.setPWM(10, 0, 4096);
    pwm.setPWM(11, 0, pwmvalue);
  } else {
    pwm.setPWM(11, 0, 4096);
    pwm.setPWM(10, 0, pwmvalue);
  }
}

// Мощность мотора "B" от -100% до +100% (от знака зависит направление вращения)
void motorB_setpower(float pwr, bool invert) {
  // Проверка, инвертирован ли мотор
  if (invert) {
    pwr = -pwr;
  }
  // Проверка диапазонов
  if (pwr < -100) {
    pwr = -100;
  }
  if (pwr > 100) {
    pwr = 100;
  }
  int pwmvalue = fabs(pwr) * 40.95;
  if (pwr < 0) {
    pwm.setPWM(8, 0, 4096);
    pwm.setPWM(9, 0, pwmvalue);
  } else {
    pwm.setPWM(9, 0, 4096);
    pwm.setPWM(8, 0, pwmvalue);
  }
}

bool setBusChannel(uint8_t i2c_channel) {
  if (i2c_channel >= MAX_CHANNEL) {
    return false;
  } else {
    Wire.beginTransmission(I2C_HUB_ADDR);
    Wire.write(i2c_channel | EN_MASK);  
    //Wire.write(0x01 << i2c_channel);  // Для микросхемы PW548A
    Wire.endTransmission();
    return true;
  }
}