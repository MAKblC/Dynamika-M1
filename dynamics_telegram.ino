#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>

#include <Wire.h>

// параметры сети
#define WIFI_SSID "XXXXXX"
#define WIFI_PASSWORD "XXXXXXXXXX"
// токен вашего бота
#define BOT_TOKEN "XXXXXXXX:XXXXXXXXXXXXXXX"

const unsigned long BOT_MTBS = 1000; // период обновления сканирования новых сообщений

WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);
unsigned long bot_lasttime;

// ссылка для поста фотографии
String test_photo_url = "https://mgbot.ru/upload/logo-r.png";

// отобразить кнопки перехода на сайт с помощью InlineKeyboard
String keyboardJson1 = "[[{ \"text\" : \"Ваш сайт\", \"url\" : \"https://mgbot.ru\" }],[{ \"text\" : \"Перейти на сайт IoTik.ru\", \"url\" : \"https://www.iotik.ru\" }]]";

#include <Adafruit_PWMServoDriver.h> // библиотека для моторной платы
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x70); // адрес платы

#include <VL53L0X.h> // библиотека для датчика MGS-D20
VL53L0X lox1;

#include <Adafruit_LSM9DS1.h>                       // гироскоп
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

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

#include "TLC59108.h" // библиотека для модуля MGL-RGB1
#define HW_RESET_PIN 0 // Только програмнный сброс
#define I2C_ADDR TLC59108::I2C_ADDR::BASE
TLC59108 leds(I2C_ADDR + 7); // Без перемычек добавляется 3 бита адреса

#include <Adafruit_MCP4725.h>    // библиотека для MGB-BUZ1
Adafruit_MCP4725 buzzer;
int ton;
int vol1 = 4095; // Уровень громкости = vol1-vol2
int vol2 = 0;

void setup()
{
  Serial.begin(115200);
  delay(512);
  Serial.println();
  Serial.print("Connecting to Wifi SSID ");
  Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.print("\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());

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
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
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

// функция обработки новых сообщений
void handleNewMessages(int numNewMessages)
{
  Serial.print("handleNewMessages ");
  Serial.println(numNewMessages);

  for (int i = 0; i < numNewMessages; i++)
  {
    String chat_id = bot.messages[i].chat_id;
    String text = bot.messages[i].text;
    text.toLowerCase();
    String from_name = bot.messages[i].from_name;
    if (from_name == "")
      from_name = "Guest";

    // выполняем действия в зависимости от пришедшей команды
    if ((text == "/sensors") || (text == "sensors")) // измеряем данные
    {
      setBusChannel(0x05);
      float dist = lox1.readRangeSingleMillimeters(); // снимаем данные с датчика расстояния
      setBusChannel(0x06);
      lsm.read(); // данные гироскопа, акселерометра и магнетометра
      sensors_event_t a, m, g, temp;
      lsm.getEvent(&a, &m, &g, &temp);

      String welcome = "Показания датчиков:\n";
      welcome += "Distance: " + String(dist, 0) + " mm\n";
      welcome += "aX: " + String(a.acceleration.x, 1) + " \n";
      welcome += "aY: " + String(a.acceleration.y, 1) + " \n";
      welcome += "aZ: " + String(a.acceleration.z, 1) + " \n";
      bot.sendMessage(chat_id, welcome, "Markdown");
    }

    if (text == "/photo") { // пост фотографии
      bot.sendPhoto(chat_id, test_photo_url, "а вот и фотка!");
    }

    if ((text == "/clockwise") || (text == "clockwise"))
    {
      motorA_setpower(0x20, true);
      motorB_setpower(0x20, true);
      bot.sendMessage(chat_id, "Едем по часовой", "");
    }
    if ((text == "/anticlockwise") || (text == "anticlockwise"))
    {
      motorA_setpower(0x20, false);
      motorB_setpower(0x20, false);
      bot.sendMessage(chat_id, "Едем против часовой", "");
    }
    if ((text == "/light") || (text == "light"))
    {
      setBusChannel(0x01);
      leds.setBrightness(6, 0x99);
      leds.setBrightness(0, 0x99);
      bot.sendMessage(chat_id, "Свет включен", "");
    }
    if ((text == "/off") || (text == "off"))
    {
      setBusChannel(0x01);
      leds.setBrightness(6, 0x00);
      leds.setBrightness(0, 0x00);
      leds.setBrightness(3, 0x00);
      leds.setBrightness(2, 0x00);
      leds.setBrightness(5, 0x00);
      bot.sendMessage(chat_id, "Свет выключен", "");
    }
    if ((text == "/color") || (text == "color"))
    {
      setBusChannel(0x01);
      leds.setBrightness(3, random(0, 255));
      leds.setBrightness(2, random(0, 255));
      leds.setBrightness(5, random(0, 255));
      bot.sendMessage(chat_id, "Включен случайный цвет", "");
    }
    if ((text == "/stop") || (text == "stop"))
    {
       bot.sendMessage(chat_id, "Моторы остановлены", "");
      motorA_setpower(0x00, false);
      motorB_setpower(0x00, false);
    }
    if ((text == "/sound") || (text == "sound"))
    {
       bot.sendMessage(chat_id, "Бииип бууп", "");
      setBusChannel(0x07);
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
    if (text == "/site") // отобразить кнопки в диалоге для перехода на сайт
    {
      bot.sendMessageWithInlineKeyboard(chat_id, "Выберите действие", "", keyboardJson1);
    }
    if (text == "/options") // клавиатура для управления теплицей
    {
      String keyboardJson = "[[\"/light\", \"/off\"],[\"/color\",\"/sensors\"],[\"/clockwise\", \"/anticlockwise\",\"/stop\"],[\"/sound\"]]";
      bot.sendMessageWithReplyKeyboard(chat_id, "Выберите команду", "", keyboardJson, true);
    }

    if ((text == "/start") || (text == "start") || (text == "/help") || (text == "help")) // команда для вызова помощи
    {
      bot.sendMessage(chat_id, "Привет, " + from_name + "!", "");
      bot.sendMessage(chat_id, "Я контроллер Йотик 32. Команды смотрите в меню слева от строки ввода", "");
      String sms = "Команды:\n";
      sms += "/options - пульт управления\n";
      sms += "/site - перейти на сайт\n";
      sms += "/photo - запостить фото\n";
      sms += "/help - вызвать помощь\n";
      bot.sendMessage(chat_id, sms, "Markdown");
    }
  }
}



void loop() // вызываем функцию обработки сообщений через определенный период
{
  if (millis() - bot_lasttime > BOT_MTBS)
  {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while (numNewMessages)
    {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }

    bot_lasttime = millis();
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
