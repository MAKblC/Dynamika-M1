/*
поворот Динамики на 90 градусов с паузой в секунду
Используется MGS-A6 с подключением в 21/22
*/

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;

#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x70);  // адрес зависит от перемычек на плате

uint8_t fifoBuffer[45];  // буфер
int turnSpeed = 30; // скорость поворота
int compensation = 10; // компенсация поворота (зависит от скорости поворота)

void setup() {
  Serial.begin(115200);
  Wire.begin();
 
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
}

void loop() {
  angle(false, 90); // повернуть по часовой на 90
  delay(1000);
}

// повернуть(сторона (направо 0/налево 1), угол)
void angle(bool side, int angle) {
  // измеряем текущий угол
  int currentAngle = angleMeasure();
  // если нужно повернуть по часовой (направо)
  if (side == false) {
    // берем остаток от круга и поворачиваем, пока разница не станет меньше 10, но больше 0
    while (((currentAngle + angle) % 360) - angleMeasure() > compensation or ((currentAngle + angle) % 360) - angleMeasure() < 0) {
      // поворот на одном колесе
      // motorA_setpower(turnSpeed, side); // расскомментировать чтоб поворачивать вокруг оси
      motorB_setpower(turnSpeed, side);
    }
    // другой алгоритм поворота по часовой
    /*if (currentAngle + angle < 360) {
      while (currentAngle + angle - angleMeasure() > 10) {
        motorA_setpower(5, side);
        motorB_setpower(5, side);
      }
    } else {
      while (angle - (360 - currentAngle) - angleMeasure() > 10 or angle - (360 - currentAngle) - angleMeasure() < 0) {
        motorA_setpower(5, side);
        motorB_setpower(5, side);
      }
    }*/
  } else {
    // если поворачиваем налево (против часовой)
    // в простой ситуации, когда нужно повернуться на угол меньший, чем текущий
    if (currentAngle > angle) {
      // поворачиваем пока разница между углами не станет меньше -10
      while (currentAngle - angle - angleMeasure() < -(compensation)) {
        // поворот на одном колесе
        //  motorA_setpower(turnSpeed, side); // расскомментировать чтоб поворачивать вокруг оси
        motorB_setpower(turnSpeed, side);
      }
      // в сложной ситуации, когда нужно осилить переход через 0
    } else {
      while (360 - (angle - currentAngle) - angleMeasure() > 0 or 360 - (angle - currentAngle) - angleMeasure() < -(compensation)) {
        // поворот на одном колесе
        //motorA_setpower(turnSpeed, side); // расскомментировать чтоб поворачивать вокруг оси
        motorB_setpower(turnSpeed, side);
      }
    }
  }
  // резко тормозим с включением на мгновение обратных моторов
  motorA_setpower(turnSpeed, !side);
  motorB_setpower(turnSpeed, !side);
  delay(50);
  motorA_setpower(0, side);
  motorB_setpower(0, side);
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
