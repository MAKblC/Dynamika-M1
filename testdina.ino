#include <Wire.h> //библиотека для I2C интерфейса

#include <Adafruit_PWMServoDriver.h> // библиотека для моторной платы
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x70); // адрес платы

#include <VL53L0X.h> // библиотека для датчика MGS-D20
VL53L0X lox1;

// Выберите гироскоп или датчик цвета в вашей сборке (ненужные занесите в комментарии)
#define MGS_A9 1
//#define MGS_CLM60 1
//#define MGS_A6 1

/////////////////// гироскоп и датчик цвета ///////////////////
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_APDS9960.h"

#ifdef MGS_A9
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
#endif
#ifdef MGS_A6
Adafruit_MPU6050 mpu;
#endif
#ifdef MGS_CLM60 
Adafruit_APDS9960 apds9960;
#endif

// Выберите модуль светодиодов в вашей сборке (ненужные занесите в комментарии)
//#define MGL_RGB1EN 1
//#define MGL_RGB2 1
#define MGL_RGB3 1

/////////////////// модуль светодиодов ///////////////////
#ifdef MGL_RGB1EN
#include "TLC59108.h" // библиотека для модуля MGL-RGB1
#define HW_RESET_PIN 0 // Только програмнный сброс
#define I2C_ADDR TLC59108::I2C_ADDR::BASE
TLC59108 leds(I2C_ADDR + 7); // Без перемычек добавляется 3 бита адреса
#endif
#ifdef MGL_RGB2
#include <PCA9634.h>
PCA9634 leds1(0x4D); // (также попробуйте просканировать адрес: https://github.com/MAKblC/Codes/tree/master/I2C%20scanner)
#endif
#ifdef MGL_RGB3
#include <PCA9634.h>
PCA9634 testModule(0x08); // (также попробуйте просканировать адрес: https://github.com/MAKblC/Codes/tree/master/I2C%20scanner)
#endif

#define I2C_HUB_ADDR        0x70 // настройки I2C для платы MGB-I2C63EN
#define EN_MASK             0x08
#define DEF_CHANNEL         0x00
#define MAX_CHANNEL         0x08

/*
  I2C порт 0x07 - выводы GP16 (SDA), GP17 (SCL)
  I2C порт 0x06 - выводы GP4 (SDA), GP13 (SCL)
  I2C порт 0x05 - выводы GP14 (SDA), GP15 (SCL)
  I2C порт 0x04 - выводы GP5 (SDA), GP23 (SCL)
  I2C порт 0x03 - выводы GP18 (SDA), GP19 (SCL)
*/

#define LONG_RANGE // режим дальности для датчика расстояния

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

  setBusChannel(0x06);
#ifdef MGS_A9 
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
  }
  Serial.println("Found LSM9DS1 9DOF");
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
#endif
#ifdef MGS_A6
  if (!mpu.begin(0x69)) { // (0x68) (также попробуйте просканировать адрес: https://github.com/MAKblC/Codes/tree/master/I2C%20scanner)
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
#endif  
#ifdef MGS_CLM60 
  if (!apds9960.begin()) {
    Serial.println("Failed to initialize device!");
  }
  Serial.println("CLM60");
  // Инициализация режимов работы датчика
  apds9960.enableColor(true);
  apds9960.enableProximity(true);
#endif

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
#ifdef MGL_RGB1EN
  leds.init(HW_RESET_PIN);
  leds.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);
#endif
#ifdef MGL_RGB2
  leds1.begin();
  for (int channel = 0; channel < leds1.channelCount(); channel++)
  {
    leds1.setLedDriverMode(channel, PCA9634_LEDOFF); // выключить все светодиоды в режиме 0/1
    leds1.write1(channel, 0x00); // выключить все светодиоды в режиме ШИМ
    delay(200);
    leds1.setLedDriverMode(channel, PCA9634_LEDPWM); // установка режима ШИМ
  }
#endif
#ifdef MGL_RGB3
  testModule.begin();
  for (int channel = 0; channel < testModule.channelCount(); channel++)
  {
    testModule.setLedDriverMode(channel, PCA9634_LEDOFF); // выключить все светодиоды в режиме 0/1
  }
#endif
}

void loop() {
  Serial.println("going clockwise"); // едем в одну сторону
  motorA_setpower(0x20, true);
  motorB_setpower(0x20, true);
  delay(3000);
  Serial.println("going anti-clockwise"); // едем в обратную
  motorA_setpower(0x20, false);
  motorB_setpower(0x20, false);
  delay(3000);
  motorA_setpower(0x00, false);
  motorB_setpower(0x00, false);
  Serial.println("stop");
  setBusChannel(0x05);
  delay(1000);
  float dist = lox1.readRangeSingleMillimeters(); // снимаем данные с датчика расстояния
  Serial.println("distance = " + String(dist, 1) + " mm");
  delay(1000);

  setBusChannel(0x06);
#ifdef MGS_A9
  lsm.read(); // данные гироскопа, акселерометра и магнетометра
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  Serial.println("accel x = " + String(a.acceleration.x, 1));
  Serial.println("accel y = " + String(a.acceleration.y, 1));
  Serial.println("accel z = " + String(a.acceleration.z, 1));
#endif
#ifdef MGS_A6
 sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  /* Вывод значений */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");
  Serial.println("");
#endif
#ifdef MGS_CLM60
  uint16_t red_data   = 0;
  uint16_t green_data = 0;
  uint16_t blue_data  = 0;
  uint16_t clear_data = 0;
  uint16_t prox_data  = 0;
  // Определение цвета
  while (!apds9960.colorDataReady()) {
    delay(5);
  }
  apds9960.getColorData(&red_data, &green_data, &blue_data, &clear_data);
  // Определение близости препятствия
  prox_data = apds9960.readProximity();
  // Вывод измеренных значений в терминал
  Serial.println("RED   = " + String(red_data));
  Serial.println("GREEN = " + String(green_data));
  Serial.println("BLUE  = " + String(blue_data));
  Serial.println("CLEAR = " + String(clear_data));
  Serial.println("PROX  = " + String(prox_data));
#endif
  delay(1000);
#ifdef MGL_RGB1EN
  leds.setBrightness(6, 0xff); // белые
  leds.setBrightness(0, 0xff);
  Serial.println("white");
  delay(2000);
  leds.setBrightness(6, 0x00);
  leds.setBrightness(0, 0x00);
  leds.setBrightness(1, 0xff); // УФ
  leds.setBrightness(4, 0xff);
  Serial.println("ultraviolet");
  delay(2000);
  leds.setBrightness(1, 0x00);
  leds.setBrightness(4, 0x00);
  leds.setBrightness(2, 0xff);// зеленый
  Serial.println("green");
  delay(2000);
  leds.setBrightness(2, 0x00);
  leds.setBrightness(3, 0xff); // красный
  Serial.println("red");
  delay(2000);
  leds.setBrightness(3, 0x00);
  leds.setBrightness(5, 0xff); // синий
  Serial.println("blue");
  delay(2000);
  leds.setBrightness(5, 0x00);
#endif
#ifdef MGL_RGB2
  for (int channel = 0; channel < 8; channel++)
  {
    for (byte pwm = 0; pwm < 0xff; pwm++) {
      leds1.write1(channel, pwm);
      delay(1);
    }
    for (byte pwm = 0xfe; pwm < 0xff; pwm--) {
      leds1.write1(channel, pwm);
      delay(1);
    }
  }
#endif
#ifdef MGL_RGB3
  for (int channel = 0; channel < testModule.channelCount(); channel++)
  {
    testModule.setLedDriverMode(channel, PCA9634_LEDON);
    Serial.println(channel);
    delay(500);
    testModule.setLedDriverMode(channel, PCA9634_LEDOFF);
    delay(500);
  }
  for (int channel = 0; channel < testModule.channelCount(); channel++)
  {
    testModule.setLedDriverMode(channel, PCA9634_LEDPWM); // установка режима ШИМ (0-255)
  }
  testModule.write1(3, 0x90);
  testModule.write1(2, 0x00);
  testModule.write1(5, 0x90);
  Serial.println("розовый");
  delay(1000);
  testModule.write1(3, 0x00);
  testModule.write1(2, 0x90);
  testModule.write1(5, 0x90);
  Serial.println("голубой");
  delay(1000);
  testModule.write1(3, 0x90);
  testModule.write1(2, 0x90);
  testModule.write1(5, 0x00);
  Serial.println("желтый");
  delay(1000);
  testModule.write1(3, 0x00);
  testModule.write1(2, 0x00);
  testModule.write1(5, 0x00);
#endif
  delay(1000);


  setBusChannel(0x07);
  Serial.println("sound!");
  for (int i = 0; i < 400; i++) { // звук
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

bool setBusChannel(uint8_t i2c_channel) // смена I2C порта
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
