// Скетч к проекту "Широкоформатная WiFi панель / гирлянда"
// Гайд по постройке матрицы: https://alexgyver.ru/matrix_guide/
// Страница проекта на GitHub: https://github.com/vvip-68/GyverLampWiFi
// Автор: AlexGyver Technologies, 2019
// Дальнейшее развитие: vvip, 2019
// https://AlexGyver.ru/

// http://arduino.esp8266.com/stable/pspackage_esp8266com_index.json
// https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json


// ************************ WIFI ПАНЕЛЬ *************************

#define FIRMWARE_VER F("GyverPanel-WiFi v.1.00.2020.0801")

#include "a_def_hard.h"     // Определение параметров мвтрицы, пинов подключения и т.п
#include "a_def_soft.h"     // Определение параметров эффектов, переменных программы и т.п.

void setup() {
#if defined(ESP8266)
  ESP.wdtEnable(WDTO_8S);
#endif
  Serial.begin(115200);
  delay(10);

  Serial.println();
  Serial.println();
  Serial.println(FIRMWARE_VER);

  // Инициализация EEPROM и загрузка начальных значений переменных и параметров
  EEPROM.begin(4096);
  loadSettings();

  // Настройки ленты
  FastLED.addLeds<WS2812, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(globalBrightness);
  if (CURRENT_LIMIT > 0) {
    FastLED.setMaxPowerInVoltsAndMilliamps(5, CURRENT_LIMIT);
  }
  FastLED.clear();
  FastLED.show();

  randomSeed(analogRead(0) ^ millis());    // пинаем генератор случайных чисел

  // Первый этап инициализации плеера - подключение и основные настройки
  #if (USE_MP3 == 1)
    InitializeDfPlayer1();
  #endif
     
  #if defined(ESP8266)
    WiFi.setSleepMode(WIFI_NONE_SLEEP);
  #endif

  // Подключение к сети
  connectToNetwork();

  // UDP-клиент на указанном порту
  udp.begin(localPort);

  // Настройка кнопки
  butt.setStepTimeout(100);
  butt.setClickTimeout(500);

  // Настройка внешнего дисплея TM1637
  display.setBrightness(7);
  display.displayByte(_empty, _empty, _empty, _empty);
  
  // Таймер бездействия
  if (idleTime == 0) // Таймер Idle  отключен
    idleTimer.setInterval(4294967295);
  else  
    idleTimer.setInterval(idleTime);

  // Таймер рассвета
  dawnTimer.setInterval(4294967295);
  
  // Второй этап инициализации плеера - проверка наличия файлов звуков на SD карте
  #if (USE_MP3 == 1)
    InitializeDfPlayer2();
    if (!isDfPlayerOk) {
      Serial.println(F("MP3 плеер недоступен."));
    }
  #endif

  // Проверить соответствие позиции вывода часов размерам матрицы
  // При необходимости параметры отображения часов корректируются в соответствии с текущими аппаратными возможностями
  checkClockOrigin();
  
  // Если был задан спец.режим во время предыдущего сеанса работы матрицы - включить его
  // Номер спец-режима запоминается при его включении и сбрасывается при включении обычного режима или игры
  // Это позволяет в случае внезапной перезагрузки матрицы (например по wdt), когда был включен спец-режим (например ночные часы или выкл. лампы)
  // снова включить его, а не отображать случайный обычный после включения матрицы
  int8_t spc_mode = getCurrentSpecMode();
  if (spc_mode >= 0 && spc_mode < MAX_SPEC_EFFECT) {
    setSpecialMode(spc_mode);
    isTurnedOff = spc_mode == 0;
    isNightClock = spc_mode == 8;
  } else {
    thisMode = getCurrentManualMode();
    if (thisMode < 0 || AUTOPLAY) {
      setRandomMode2();
    } else {
      while (1) {
        // Если режим отмечен флагом "использовать" - используем его, иначе берем следующий (и проверяем его)
        if (getEffectUsage(thisMode)) break;
        thisMode++;
        if (thisMode >= MAX_EFFECT) {
          thisMode = 0;
          break;
        }
      }
      setEffect(thisMode);        
    }
  }
  autoplayTimer = millis();
}

void loop() {
  process();
}

// -----------------------------------------

void startWiFi() { 
  
  WiFi.disconnect(true);
  wifi_connected = false;
  
  delay(10);               // Иначе получаем Core 1 panic'ed (Cache disabled but cached memory region accessed)
  WiFi.mode(WIFI_STA);
 
  // Пытаемся соединиться с роутером в сети
  if (strlen(ssid) > 0) {
    Serial.print(F("\nПодключение к "));
    Serial.print(ssid);

    if (IP_STA[0] + IP_STA[1] + IP_STA[2] + IP_STA[3] > 0) {
      WiFi.config(IPAddress(IP_STA[0], IP_STA[1], IP_STA[2], IP_STA[3]),  // 192.168.0.106
                  IPAddress(IP_STA[0], IP_STA[1], IP_STA[2], 1),          // 192.168.0.1
                  IPAddress(255, 255, 255, 0),                            // Mask
                  IPAddress(IP_STA[0], IP_STA[1], IP_STA[2], 1),          // DNS1 192.168.0.1
                  IPAddress(8, 8, 8, 8));                                 // DNS2 8.8.8.8                  
    }              
    WiFi.begin(ssid, pass);
  
    // Проверка соединения (таймаут 5 секунд)
    for (int j = 0; j < 10; j++ ) {
      wifi_connected = WiFi.status() == WL_CONNECTED; 
      if (wifi_connected) {
        // Подключение установлено
        Serial.println();
        Serial.print(F("WiFi подключен. IP адрес: "));
        Serial.println(WiFi.localIP());
        break;
      }
      delay(500);
      Serial.print(".");
    }
    Serial.println();

    if (!wifi_connected)
      Serial.println(F("Не удалось подключиться к сети WiFi."));
  }  
}

void startSoftAP() {
  WiFi.softAPdisconnect(true);
  ap_connected = false;

  Serial.print(F("Создание точки доступа "));
  Serial.print(apName);
  
  ap_connected = WiFi.softAP(apName, apPass);

  for (int j = 0; j < 10; j++ ) {    
    if (ap_connected) {
      Serial.println();
      Serial.print(F("Точка доступа создана. Сеть: '"));
      Serial.print(apName);
      // Если пароль совпадает с паролем по умолчанию - печатать для информации,
      // если был изменен пользователем - не печатать
      if (strcmp(apPass, "12341234") == 0) {
        Serial.print(F("'. Пароль: '"));
        Serial.print(apPass);
      }
      Serial.println(F("'."));
      Serial.print(F("IP адрес: "));
      Serial.println(WiFi.softAPIP());
      break;
    }    
    
    WiFi.enableAP(false);
    WiFi.softAPdisconnect(true);
    delay(500);
    
    Serial.print(".");
    ap_connected = WiFi.softAP(apName, apPass);
  }  
  Serial.println();  

  if (!ap_connected) 
    Serial.println(F("Не удалось создать WiFi точку доступа."));
}

void printNtpServerName() {
  Serial.print(F("NTP-сервер "));
  Serial.print(ntpServerName);
  Serial.print(F(" -> "));
  Serial.println(timeServerIP);
}

void connectToNetwork() {
  // Подключиться к WiFi сети
  startWiFi();

  // Если режим точки тоступане используется и к WiFi сети подключиться не удалось - создать точку доступа
  if (!wifi_connected){
    WiFi.mode(WIFI_AP);
    startSoftAP();
  }

  if (useSoftAP && !ap_connected) startSoftAP();    

  // Сообщить UDP порт, на который ожидаются подключения
  if (wifi_connected || ap_connected) {
    Serial.print(F("UDP-сервер на порту "));
    Serial.println(localPort);
  }
}
