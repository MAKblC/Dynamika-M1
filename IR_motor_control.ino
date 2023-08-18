#include "IRremote.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
IRrecv irrecv(27);
decode_results results;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x70);

void setup() {
  Serial.begin(115200);
  irrecv.enableIRIn();
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(100);
  pwm.setPWM(8, 0, 4096);
  pwm.setPWM(9, 0, 4096);
  pwm.setPWM(10, 0, 4096);
  pwm.setPWM(11, 0, 4096);
}

void loop() {
  if ( irrecv.decode( &results )) { // если данные пришли
    if (results.value == 0xFF18E7) { // если это кнопка "2"
      motorA_setpower(100, false); // едем вперед
      motorB_setpower(100, true);
      delay(25);
    }
    if (results.value == 0xFF4AB5) { // если это кнопка "8"
      motorA_setpower(100, true); // едем назад
      motorB_setpower(100, false);
      delay(25);
    }
    if (results.value == 0xFF10EF) { // если это кнопка "4"
      motorA_setpower(100, false); // кружимся влево
      motorB_setpower(-100, true);
      delay(25);
    }
    if (results.value == 0xFF5AA5) {
      motorA_setpower(-100, false); // если это кнопка "6"
      motorB_setpower(100, true); // кружимся вправо
      delay(25);
    }
    if (results.value == 0xFF38C7) { // если это кнопка "5"
      motorA_setpower(0, false); // останавливаемся
      motorB_setpower(0, false);
      delay(25);
    }
    irrecv.resume(); // принимаем следующую команду
  }
}

void motorA_setpower(float pwr, bool invert)
{
  if (invert)
  {
    pwr = -pwr;
  }
  if (pwr < -100)
  {
    pwr = -100;
  }
  if (pwr > 100)
  {
    pwr = 100;
  }
  int pwmvalue = fabs(pwr) * 40.95;
  if (pwr < 0)
  {
    pwm.setPWM(10, 0, 4096);
    pwm.setPWM(11, 0, pwmvalue);
  }
  else
  {
    pwm.setPWM(11, 0, 4096);
    pwm.setPWM(10, 0, pwmvalue);
  }
}
void motorB_setpower(float pwr, bool invert)
{
  if (invert)
  {
    pwr = -pwr;
  }
  if (pwr < -100)
  {
    pwr = -100;
  }
  if (pwr > 100)
  {
    pwr = 100;
  }
  int pwmvalue = fabs(pwr) * 40.95;
  if (pwr < 0)
  {
    pwm.setPWM(8, 0, 4096);
    pwm.setPWM(9, 0, pwmvalue);
  }
  else
  {
    pwm.setPWM(9, 0, 4096);
    pwm.setPWM(8, 0, pwmvalue);
  }
}

