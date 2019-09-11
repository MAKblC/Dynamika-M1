#include <Wire.h> //библиотека для I2C интерфейса

#include <Adafruit_PWMServoDriver.h> // библиотека для моторной платы
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x70); // адрес платы

#include <VL53L0X.h> // библиотека для датчика MGS-D20
VL53L0X lox1;

#define I2C_HUB_ADDR        0x70 // настройки I2C для платы MGB-I2C63EN
#define EN_MASK             0x08
#define DEF_CHANNEL         0x00
#define MAX_CHANNEL         0x08
// I2C порт 0x07 - выводы D4 (SDA), D5 (SCL)
// I2C порт 0x06 - выводы D6 (SDA), D7 (SCL)
// I2C порт 0x05 - выводы D8 (SDA), D9 (SCL)
// I2C порт 0x04 - выводы D10 (SDA), D11 (SCL)
// I2C порт 0x03 - выводы D12 (SDA), D13 (SCL)

#define LONG_RANGE // режим дальности для датчика расстояния

#include "TLC59108.h" // библиотека для модуля MGL-RGB1
#define HW_RESET_PIN 0 // Только програмнный сброс
#define I2C_ADDR TLC59108::I2C_ADDR::BASE
TLC59108 leds(I2C_ADDR + 7); // Без перемычек добавляется 3 бита адреса

#include <Adafruit_MCP4725.h>    // библиотека для MGB-BUZ1
Adafruit_MCP4725 buzzer;
int ton;
int vol1 = 4095; // Уровень громкости = vol1-vol2
int vol2 = 0;

void setup() {
  // Инициализация последовательного порта
  Serial.begin(115200);
  // Инициализация драйвера
  Wire.begin();
  pwm.begin();
  // Частота (Гц)
  pwm.setPWMFreq(100);
  // Все порты выключены
  pwm.setPWM(8, 0, 4096);
  pwm.setPWM(9, 0, 4096);
  pwm.setPWM(10, 0, 4096);
  pwm.setPWM(11, 0, 4096);

  setBusChannel(0x07); // 7ой канал
  buzzer.begin(0x61); // С перемычкой адрес будет 0x60
  buzzer.setVoltage(0, false);   // выключение звука

  setBusChannel(0x05); //5ый канал
  lox1.init();
  lox1.setTimeout(500);
#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  lox1.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  lox1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  lox1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif
#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  lox1.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  lox1.setMeasurementTimingBudget(200000);
#endif

  setBusChannel(0x01); // первый канал
  leds.init(HW_RESET_PIN);
  leds.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);

}

void loop() {
  motorA_setpower(0x20, true); // едем вперед
  motorB_setpower(0x20, false);
  setBusChannel(0x05);
  float dist2 = lox1.readRangeSingleMillimeters(); // снимаем данные с датчика расстояния
  if (dist2 < 100) { // если меньше 100 мм
    motorA_setpower(0x00, false); // то останавливаемся
    motorB_setpower(0x00, false);
    signalization(); // включаем сигнализацию
    setBusChannel(0x01);
    leds.setBrightness(3, 0xff); // моргнем красным цветом 2 раза
    delay(100);
    leds.setBrightness(3, 0x00);
    delay(100);
    leds.setBrightness(3, 0xff);
    delay(100);
    leds.setBrightness(3, 0x00);
    motorA_setpower(0x20, false); // повернемся вокруг оси в течение 300 мс
    motorB_setpower(0x20, false);
    delay(300);
  }
}

// Мощность мотора "A" от -100% до +100% (от знака зависит направление вращения)
void motorA_setpower(float pwr, bool invert)
{
  // Проверка, инвертирован ли мотор
  if (invert)
  {
    pwr = -pwr;
  }
  // Проверка диапазонов
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

// Мощность мотора "B" от -100% до +100% (от знака зависит направление вращения)
void motorB_setpower(float pwr, bool invert)
{
  // Проверка, инвертирован ли мотор
  if (invert)
  {
    pwr = -pwr;
  }
  // Проверка диапазонов
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

bool setBusChannel(uint8_t i2c_channel)
{
  if (i2c_channel >= MAX_CHANNEL)
  {
    return false;
  }
  else
  {
    Wire.beginTransmission(I2C_HUB_ADDR);
    Wire.write(i2c_channel | EN_MASK);
    Wire.endTransmission();
    return true;
  }
}
void signalization() { // для модуля динамика
  setBusChannel(0x07);
  for (int i = 0; i < 400; i++) {
    buzzer.setVoltage(vol1, false);
    buzzer.setVoltage(vol2, false);
    delayMicroseconds(190);
  }
  for (int i = 0; i < 400; i++) {
    buzzer.setVoltage(vol1, false);
    buzzer.setVoltage(vol2, false);
    delayMicroseconds(500);
  }
  buzzer.setVoltage(0, false);
}
