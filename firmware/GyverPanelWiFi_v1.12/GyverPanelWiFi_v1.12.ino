// Страница проекта на GitHub: https://github.com/vvip-68/GyverPanelWiFi
// Страница данного форка: https://github.com/frol-aleksan/GyverPanelWiFi
// Автор идеи, начальный проект - GyverMatrixBT: AlexGyver Technologies, 2019 (https://alexgyver.ru/gyvermatrixbt/)
// Дальнейшее развитие: vvip-68, 2019-2022
// Добавленs эффекты из прошивок от gunner47, Shaitan и alvikskor
// https://AlexGyver.ru/
//
// Дополнительные ссылки для Менеджера плат ESP8266 и ESP32 в Файл -> Настройки
// https://raw.githubusercontent.com/esp8266/esp8266.github.io/master/stable/package_esp8266com_index.json
// https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

// -------------------------------------------------------------------------------------------------------
// Внимание!!! 
// -------------------------------------------------------------------------------------------------------

// Начиная с версии 1.12 изменилась карта распределения хранения настроек в постоянной памяти EEPROM
// После сборки и загрузки скомпилированной прошивки 1.12 вам придется заново перенастраивать все эффекты
// и прочие настройки программы, в том числе набор текстов бегущей строки. Ввиду несовместимости расположения 
// сохраненных настроек, их восстановление из файла резервной копии также недоступно или приведет к
// сбоям в работе прошивки.

// -------------------------------------------------------------------------------------------------------

// На версии ядра ESP8266 v3.x.x и FastLED 3.4 работать не будет! 
//  -> Постоянно горит самый первый светодиод в цепочке синим или зелёным
//  -> некоторые эффекты сильно мерцают.
//  -> пин ленты с D2 переназначается ядром на D4
// На версии ядра ESP32 1.0.5 или 2.x.x и FastLED 3.4 работать не будет! 
//  -> 1.0.5 - При USE_E131 == 1 - при переключении с MASTER на STANDALONE или SLAVE - crash  и перезагрузка. 
//             Для исправления - в файле C:\Users\<user>\AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.5\libraries\AsyncUDP\src\AsyncUDP.cpp
//             найти деструктор AsyncUDP::~AsyncUDP() и перенести вызов close(); из начала процедуры в конец  
//  -> 2.0.0 - Нет вывода изображения на ленту ни на одном из пинов

// Версия ядра ESP8266 - 2.7.4    
// Версия ядра ESP32   - 1.0.6
// Версия FastLED      - 3.4.0

// -------------------------------------------------------------------------------------------------------

// Библиотеку TM1637 следует обязательно устанавливать из папки проекта. В ней сделаны исправления, позволяющие
// компилироваться проекту для микроконтроллера ESP32. Со стандартной версией библиотеки из менеджера библиотек
// проект не будет компилироваться

// Библиотеку ESPAsyncE131 следует обязательно устанавливать из папки проекта. В ней исправлены ошибки стандартной
// библиотеки (добавлен деструктор и освобождение выделяемых ресурсов), а также добавлен ряд функций, позволяющих
// осуществлять передачу сформированных пакетов в сеть. Со стандартной версией библиотеки из менеджера библиотек
// проект не будет компилироваться
//
//Доработки от меня:
//Добавлены игры Flappy Bird и Runner из первой версии проекта Gyver BT Matrix с адаптацией под текущий проект (играбельны только на максимальной скорости).
//Добавлен эффект "Синусоиды" из первой версии проекта Gyver BT Matrix с адаптацией под текущий проект.
//Раскомментирована строка (showTextNow && !fullTextFlag) в файле custom.h, чтобы эффект не менялся, пока не пробежит вся строка.
//Максимальный размер кубика в соответствующем эффекте увеличен до 5*5, что хорошо будет смотреться на больших матрицах.
//Исправлен глюк с яркостью при переходе к предыдущему эффекту (не присваивалось значение globalbrightness после вызова эффектов, где значение яркости отличается от глобального).
//Исправлен глюк с невозвратом игр в демо-режим после их запуска в игровом режиме.
//Закомментирована строчка setManualModeTo(true) в a_main.h т. к. задолбало каждый раз ставить галку автопереключения режимов после выбора режима вручную.
//Добавлены эффекты из прошивки gunner47_v2. Имеющийся эффект PRIZMATA (Синусы) объединен с одноименным эффектом из прошивки gunner47. Теперь их два в списке Вариант в параметрах эффекта.
//Некоторые эффекты объединены по смыслу и если они представляют собой вызов одной функции с разными параметрами, поэтому фактически добавленных эффектов стало еще больше.
//Изменен порядок эффектов, чтобы анимация и SD-карта шли в самом конце. Стало меньше строк в списке эффектов (их было больше из-за дублирования в блоках #ifdef) и удобнее их нумеровать.
//Добавлен ползунок варианта на изменение количества червячков в пределах 1..2*pWIDTH в эффекте NEXUS.
//Добавлен эффект "Плазмолампа".
//Исправлено зависание игр в игровом режиме, когда подключен SD-модуль (объявлен #define USE_SD 1).
//Добавлена очистка экрана при вызове рисовалки и загрузки изображения.
//В бегущую строку добавлены макросы на вывод IP-адреса, SSID и версии прошивки.
//Исправлено нахождение середины в эффекте Драже.
//Наведен порядок в эффекте Sinusoid (код распихан по отдельным функциям, после чего нормально заработали все 8 вариантов).
//Также разбит на отдельные функции эффект "Волны". 
//В эффекте Пикассо добавлен параметр Варианты. Теперь доступен список из 3 позиций.
//К эффекту Nexus вторым вариантом добавлен эффект Змейки, тем более, что эти два эффекта похожи между собой. 
//Добавлен эффект "Аврора".
//Добавлены Часы с циферблатом, Салют и Firework (в составе одного эффекта), Swirl, Watercolor, Свеча, Кубик Рубика от vvip-68, Color Frizzles, Lotus Flower и Елка.
//Произведены некоторые мелкие доработки. Включен ползунок скорости у камина, поскольку эффект работал слишком быстро.
//В эффекте Узоры добавлены Гвардейская лента на зеленом фоне и российский флаг.
//Переделано пламя свечи. Для низких матриц оно было странное, теперь больше напоминает настоящее (но все равно стремное).
//Уменьшен нижний предел высоты матрицы для анимированного пламени до 13.
//Свеча немного перерисована. Цветов стало меньше, зато нормально отрисовывается на малых яркостях.
//Часы с циферблатом теперь всегда занимают квадратную область с размерами, соответствующими короткой стороне матрицы.
//Добавлены ползунки варианта на изменение количества частиц в эффектах Мячики/Прыгуны и Попкорн.
//В эффекте Prizmata добавлен алгоритм выбора направления синусов в зависимости от размеров матрицы.
//Добавлены эффекты Дизайн, Контакт, Звёзды и Штора, перенесена часть правок по поводу бегущей строки из оригинальной прошивки. 
//Изменен алгоритм на выбор направления эффекта ДНК в зависимости от размеров матрицы аналогично эффекту Prizmata.
//Добавлены еще 5 эффектов: Песок, Лента, Stars, Жидкая лампа и Лавовая лампа.
//Эффект Вышиванка переименован в Узор, Палитра - в Одеяло, Притяжение - в Quantum с целью экономии места в списке эффектов.
//"Шумовые" эффекты с префиксом MC_NOISE_ (Облака, Лава, Шумящий лес и вот это все) собраны в едином эффекте Переливы со списком выбора вариантов.
//Эффекты Жидкая лампа и Лавовая лампа объединены в один эффект Лампа со списком выбора вариантов.
//На освободившееся место в списке добавлен эффект Питон.
//Добавлено большое количество новых палитр (всего их теперь порядка 6 десятков).
//Новые палитры подключены к эффектам Переливы, Синус, Попкорн, Прыжки, Спирали, Ветер, Питон, Куб2D, Quantum, Осциллятор, Кипение, Волны, Круг,
//Люменьер, Шары, Вихри, Паук, Звезды, Лампа (1 вариант), Замок, в том числе, со случайным выбором и автоперебором,где есть такая возможность.
//Пока дополнительные варианты палитр в списке имеют по большей части корявые названия (названия палитр вбиты "как есть").
//К эффекту Фейерверк добавлен третий вариант.
//В эффекте Елка добавлены чекбокс, запускающий движение елок, и ползунок, задающий направление падения снега.
//В эффектах Синусоид, Пульс, Узоры и Волны добавлен автоперебор вариантов.
//Добавлен выбор порядка цветов в гирлянде через приложение PanelSetup. Проверено на гирляндах с порядком цветов GRB, RGB и BGR - все четко.
//Добавлен служебный эффект, запускаемый через приложение PanelWiFi для проверки правильности и удобства выбора подходящего порядка цветов.
//Уменьшен таймаут на подключение к Wi-Fi до 1 минуты. Ждать 3 минуты до запуска собственной точки доступа в автономном режиме слишком долго.
//В эффекте Водопад убран вариант 4в1, вместо него прикручены палитры, добавлены ползунки параметра 1 и контраста (применен как параметр 2).
//В варианте 3 эффекта Салют добавлен перебор цветов залпов.
//К эффекту Контакт добавлен вариант 2.
//Добавлена предварительная очистка матрицы во всех эффектах, где ее не было.
//В эффекте Треки заданы начальные значения переменных, после чего эффект заиграл новыми красками.
//Добавлен параметр YYY+ макроса D из оригинального проекта.
//Добавлены эффекты Tixy Land (26 вариантов, случайный выбор, автоперебор) и Серпантин.
//В эффекте Контакт (Вариант 2) добавлен перебор из 8 доступных вариантов.
//Исправлена работа ползунка контраста в эффекте "Водопад". Теперь эффект получает значение параметра плотности потока правильно.
//В эффекте "Елка" задействован блок кода наряженной елки, который включается с 24 декабря по 14 января, снег идет с декабря по февраль.
//В эффекте "Лента" добавлен рандом (2 варианта). Вроде что-то делает, а там ХЗ.
//Исправлен вывод эффектов "Елка" и "Лента" на всю ширину матрицы.
//Добавлен алгоритм выбора направления эффекта "Серпантин" в зависимости от размеров матрицы по аналогии с эффектами Prizmata и "ДНК".

//Итого в деморежиме доступно 89/88 эффектов с/без SD. Со всеми вариантами имеем свыше 160 эффектов, если не считать анимации для эффекта SD.
// -------------------------------------------------------------------------------------------------------

// ************************ WIFI ПАНЕЛЬ *************************

#define FIRMWARE_VER F("WiFiPanel v.1.12.2023.12.01") //переходим на обозначение версий по дате

// --------------------------------------------------------

#include "a_def_hard.h"     // Определение параметров матрицы, пинов подключения и т.п
#include "a_def_soft.h"     // Определение параметров эффектов, переменных программы и т.п.

#if (USE_MQTT == 1)

// ------------------ MQTT CALLBACK -------------------

void callback(char* topic, uint8_t* payload, uint32_t length) {
  if (stopMQTT) return;
  // проверяем из нужного ли нам топика пришли данные
  DEBUG("MQTT << topic='" + String(topic) + "'");
  if (strcmp(topic, mqtt_topic(TOPIC_CMD).c_str()) == 0) {
    memset(incomeMqttBuffer, 0, BUF_MAX_SIZE);
    memcpy(incomeMqttBuffer, payload, length);
    
    DEBUG(F("; cmd='"));
    DEBUG(incomeMqttBuffer);
    DEBUG("'");
    
    // В одном сообщении может быть несколько команд. Каждая команда начинается с '$' и заканчивается ';'/ Пробелы между ';' и '$' НЕ допускаются.
    String command = String(incomeMqttBuffer);    
    command.replace("\n", "~");
    command.replace(";$", "\n");
    uint32_t count = CountTokens(command, '\n');
    
    for (uint8_t i=1; i<=count; i++) {
      String cmd = GetToken(command, i, '\n');
      cmd.replace('~', '\n');
      cmd.trim();
      // После разделения команд во 2 и далее строке '$' (начало команды) удален - восстановить
      if (!cmd.startsWith("$")) {
        cmd = "$" + cmd;
      }
      // После разделения команд во 2 и далее строке ';' (конец команды) удален - восстановить
      // Команда '$6 ' не может быть в пакете и признак ';' (конец команды) не используется - не восстанавливать
      if (!cmd.endsWith(";") && !cmd.startsWith(F("$6 "))) {
        cmd += ";";
      }        
      if (cmd.length() > 0 && queueLength < QSIZE_IN) {
        queueLength++;
        cmdQueue[queueWriteIdx++] = cmd;
        if (queueWriteIdx >= QSIZE_IN) queueWriteIdx = 0;
      }
    }    
  }
  DEBUGLN();
}
#endif

// ---------------------------------------------------------------

void setup() {
  #if defined(ESP8266)
    ESP.wdtEnable(WDTO_8S);
  #endif

  // Инициализация EEPROM и загрузка начальных значений переменных и параметров
  #if (EEPROM_MAX <= EEPROM_TEXT)
    #pragma message "Не выделено памяти для хранения строк эффекта 'Бегущая строка'"
    EEPROM.begin(EEPROM_TEXT);
  #else  
    EEPROM.begin(EEPROM_MAX);
  #endif

  #if (DEBUG_SERIAL == 1)
    Serial.begin(115200);
    delay(300);
  #endif

  // пинаем генератор случайных чисел
  #if defined(ESP8266) && defined(TRUE_RANDOM)
  uint32_t seed = (int)RANDOM_REG32;
  #else
  uint32_t seed = (int)(analogRead(0) ^ micros());
  #endif
  randomSeed(seed);
  random16_set_seed(seed);

  #ifdef DEVICE_ID
    host_name = String(HOST_NAME) + "-" + String(DEVICE_ID);
  #else
    host_name = String(HOST_NAME);
  #endif

  DEBUGLN();
  DEBUGLN(FIRMWARE_VER);
  DEBUGLN();
  DEBUGLN(F("\nИнициализация файловой системы... "));
  
  spiffs_ok = LittleFS.begin();
  if (!spiffs_ok) {
    DEBUGLN(F("\nВыполняется разметка файловой системы... "));
    LittleFS.format();
    spiffs_ok = LittleFS.begin();    
  }

  if (spiffs_ok) {
    DEBUG(F("FS: "));
    #if defined(ESP32)
      spiffs_total_bytes = LittleFS.totalBytes();
      spiffs_used_bytes  = LittleFS.usedBytes();
      DEBUGLN(String(F("Использовано ")) + String(spiffs_used_bytes) + " из " + String(spiffs_total_bytes) + " байт");
    #else
      FSInfo fs_info;
      if (LittleFS.info(fs_info)) {
        spiffs_total_bytes = fs_info.totalBytes;
        spiffs_used_bytes  = fs_info.usedBytes;
        DEBUGLN(String(F("Использовано ")) + String(spiffs_used_bytes) + " из " + String(spiffs_total_bytes) + " байт");
      } else {
        DEBUGLN(F("Ошибка получения сведений о файловой системе."));
      }
    #endif
  } else {
    DEBUGLN(F("Файловая система недоступна."));
  }

  loadSettings();

  // -----------------------------------------
  // В этом блоке можно принудительно устанавливать параметры, которые должны быть установлены при старте микроконтроллера
  // -----------------------------------------
  
  //переменные для эффекта Спирали
  spirotheta1 = 0;
  spirotheta2 = 0;
  spiroradiusx = round(pWIDTH/4);// - 1;
  spiroradiusy = round(pHEIGHT/4);// - 1;
  spirocenterX = round(pWIDTH/2);
  spirocenterY = round(pHEIGHT/2);
  spirominx = spirocenterX - spiroradiusx;
  spiromaxx = spirocenterX + spiroradiusx - (pWIDTH%2 == 0 ? 1:0);//+ 1;
  spirominy = spirocenterY - spiroradiusy;
  spiromaxy = spirocenterY + spiroradiusy - (pHEIGHT%2 == 0 ? 1:0);//+ 1;
  spirocount = 1;
  spirooffset = 256 / spirocount;
  spiroincrement = false;
  spirohandledChange = false;  

  //Переменные для эффекта Волны
  waveThetaUpdate = 0;
  waveThetaUpdateFrequency = 0;
  waveTheta = 0;
  hueUpdate = 0;
  hueUpdateFrequency = 0;
  waveRotation = 0;
  waveScale = 256 / pWIDTH;
  waveCount = 1;

  //переменная для эффекта Свеча
  delta_X = floor(pWIDTH * 0.5) - 4; // position img

  //Трехмерный динамический массив, необходимый для работы ряда эффектов из прошивки Gunner47
  noise_3d = new uint8_t**[numlayersmax];
  for (int i = 0; i < numlayersmax; ++i) {
    noise_3d[i] = new uint8_t*[pWIDTH];
    for (int j = 0; j < pWIDTH; ++j)
      noise_3d[i][j] = new uint8_t[pHEIGHT];
  }

  //переменные для 3 варианта фейерверка
  model_border = pHEIGHT - 4U;  // как далеко за экран может вылетить снаряд, если снаряд вылетает за экран, то вспышка белого света (не особо логично)
  model_pWIDTH = model_border + pWIDTH  + model_border; // не трогать, - математика
  model_pHEIGHT = model_border + pHEIGHT + model_border; // -//-
  pixel_x_offset = (model_pWIDTH  - pWIDTH ) / 2; // -//-
  pixel_y_offset = (model_pHEIGHT - pHEIGHT) / 2; // -//-
  num_sparks = pWIDTH; // количество разлетающихся петард (частей снаряда)
  gSparks = new Dot[num_sparks];

  DEBUGLN();
  DEBUGLN("Host: '" + host_name + "'" + String(F(" >> ")) + String(pWIDTH) + "x" + String(pHEIGHT));
  DEBUGLN();

  // Настройки ленты
  leds =  new CRGB[NUM_LEDS];     
  ledsbuff =  new CRGB[NUM_LEDS];         
  overlayLEDs = new CRGB[OVERLAY_SIZE];

  //Задаем порядок цветов
  switch(mCOLOR_ORDER) {
    case 0: FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip ); break;
    case 1: FastLED.addLeds<WS2812, LED_PIN, RGB>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip ); break;
    case 2: FastLED.addLeds<WS2812, LED_PIN, BGR>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip ); break;
    case 3: FastLED.addLeds<WS2812, LED_PIN, BRG>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip ); break;
    case 4: FastLED.addLeds<WS2812, LED_PIN, GBR>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip ); break;
    case 5: FastLED.addLeds<WS2812, LED_PIN, RBG>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip ); break;
  }
  
  FastLED.setBrightness(globalBrightness);
  if (CURRENT_LIMIT > 0) {
    FastLED.setMaxPowerInVoltsAndMilliamps(5, CURRENT_LIMIT);
  }
  FastLED.clear();
  FastLED.show();
  
  // Инициализация SD-карты
  #if (USE_SD == 1)
    InitializeSD1();
  #endif

  // Проверить наличие резервной копии настроек EEPROM в файловой системе MK и/или на SD-карте
  eeprom_backup = checkEepromBackup();
  if ((eeprom_backup & 0x01) > 0) {
    DEBUGLN(F("Найдены сохраненные настройки: FS://eeprom.bin"));
  }
  if ((eeprom_backup & 0x02) > 0) {
    DEBUGLN(F("Найдены сохраненные настройки: SD://eeprom.bin"));
  }
    
  // Инициализация SD-карты
  #if (USE_SD == 1)
    InitializeSD2();
  #endif

  #if (USE_POWER == 1)
    pinMode(POWER_PIN, OUTPUT);
  #endif
     
  // Первый этап инициализации плеера - подключение и основные настройки
  #if (USE_MP3 == 1)
    InitializeDfPlayer1();
  #endif

  #if defined(ESP8266)
    WiFi.setSleepMode(WIFI_NONE_SLEEP);
  #endif

  // Настройка кнопки
  butt.setStepTimeout(100);
  butt.setClickTimeout(500);

  // Второй этап инициализации плеера - проверка наличия файлов звуков на SD карте
  #if (USE_MP3 == 1)
    InitializeDfPlayer2();
    if (!isDfPlayerOk) {
      DEBUGLN(F("MP3 плеер недоступен."));
    }
  #endif

  // Подключение к сети
  connectToNetwork();

  #if (USE_E131 == 1)
    InitializeE131();
  #endif
  
  #if (USE_MQTT == 1)
  // Настройка соединения с MQTT сервером
  stopMQTT = !useMQTT;
  changed_keys = "";
  mqtt_client_name = host_name + "-" + String(random16(), HEX);
  last_mqtt_server = mqtt_server;
  last_mqtt_port = mqtt_port;
  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(callback);
  mqtt.setSocketTimeout(1);
  uint32_t t = millis();
  checkMqttConnection();
  if (millis() - t > MQTT_CONNECT_TIMEOUT) {
    nextMqttConnectTime = millis() + MQTT_RECONNECT_PERIOD;
  }
  String msg = F("START");
  SendMQTT(msg, TOPIC_STA);
  #endif

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);
 
  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(host_name.c_str());
 
  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");
 
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = F("скетча...");
    else // U_SPIFFS
      type = F("файловой системы SPIFFS...");
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    DEBUG(F("Начато обновление "));    
    DEBUGLN(type);    
  });

  ArduinoOTA.onEnd([]() {
    DEBUGLN(F("\nОбновление завершено"));
  });

  ArduinoOTA.onProgress([](uint32_t progress, uint32_t total) {
    #if (DEBUG_SERIAL == 1)
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    #endif
  });

  ArduinoOTA.onError([](ota_error_t error) {
    DEBUG(F("Ошибка: "));
    DEBUGLN(error);
    if      (error == OTA_AUTH_ERROR)    DEBUGLN(F("Неверное имя/пароль сети"));
    else if (error == OTA_BEGIN_ERROR)   DEBUGLN(F("Не удалось запустить обновление"));
    else if (error == OTA_CONNECT_ERROR) DEBUGLN(F("Не удалось установить соединение"));
    else if (error == OTA_RECEIVE_ERROR) DEBUGLN(F("Не удалось получить данные"));
    else if (error == OTA_END_ERROR)     DEBUGLN(F("Ошибка завершения сессии"));
  });

  ArduinoOTA.begin();

  // UDP-клиент на указанном порту
  udp.begin(localPort);

  // Настройка внешнего дисплея TM1637
  #if (USE_TM1637 == 1)
  display.setBrightness(7);
  display.displayByte(_empty, _empty, _empty, _empty);
  #endif

  // Таймер бездействия
  if (idleTime == 0) // Таймер Idle  отключен
    idleTimer.setInterval(4294967295);
  else  
    idleTimer.setInterval(idleTime);

  // Таймер синхронизации часов
  ntpSyncTimer.setInterval(1000 * 60 * SYNC_TIME_PERIOD);

  #if (USE_WEATHER == 1)     
  // Таймер получения погоды
  weatherTimer.setInterval(1000 * 60 * SYNC_WEATHER_PERIOD);
  #endif

  // Таймер рассвета
  dawnTimer.setInterval(4294967295);
  
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
    set_isTurnedOff(spc_mode == 0);
    set_isNightClock(spc_mode == 8);
  } else {
    set_thisMode(getCurrentManualMode());
    if (thisMode < 0 || thisMode == MC_TEXT || thisMode >= SPECIAL_EFFECTS_START) {
      setRandomMode2();
    } else {
      setEffect(thisMode);        
    }
  }
  autoplayTimer = millis();
  #if (USE_MQTT == 1)
    if (!stopMQTT) mqttSendStartState();
  #endif
  if (manualMode || specialMode) {
    idleTimer.setInterval(4294967295);
  } else {
    idleTimer.setInterval(idleTime);    
  }
  idleTimer.reset();
}

void loop() {
  if (wifi_connected) {
    ArduinoOTA.handle();
    #if (USE_MQTT == 1)
      if (!stopMQTT) {
        checkMqttConnection();
        mqtt.loop();
      }
    #endif
  }
  process();
}

// -----------------------------------------

void startWiFi(uint32_t waitTime) { 
  
  WiFi.disconnect(true);
  set_wifi_connected(false);
  
  delay(10);               // Иначе получаем Core 1 panic'ed (Cache disabled but cached memory region accessed)
  WiFi.mode(WIFI_STA);
 
  // Пытаемся соединиться с роутером в сети
  if (ssid.length() > 0) {
    DEBUG(F("\nПодключение к "));
    DEBUG(ssid);

    if (IP_STA[0] + IP_STA[1] + IP_STA[2] + IP_STA[3] > 0) {
      WiFi.config(IPAddress(IP_STA[0], IP_STA[1], IP_STA[2], IP_STA[3]),  // 192.168.0.106
                  IPAddress(IP_STA[0], IP_STA[1], IP_STA[2], 1),          // 192.168.0.1
                  IPAddress(255, 255, 255, 0),                            // Mask
                  IPAddress(IP_STA[0], IP_STA[1], IP_STA[2], 1),          // DNS1 192.168.0.1
                  IPAddress(8, 8, 8, 8));                                 // DNS2 8.8.8.8                  
      DEBUG(F(" -> "));
      DEBUG(IP_STA[0]);
      DEBUG(".");
      DEBUG(IP_STA[1]);
      DEBUG(".");
      DEBUG(IP_STA[2]);
      DEBUG(".");
      DEBUG(IP_STA[3]);                  
    }              
    WiFi.begin(ssid.c_str(), pass.c_str());
  
    // Проверка соединения (таймаут 180 секунд, прерывается при необходимости нажатием кнопки)
    // Такой таймаут нужен в случае, когда отключают электричество, при последующем включении устройство стартует быстрее
    // чем роутер успеет загрузиться и создать сеть. При коротком таймауте устройство не найдет сеть и создаст точку доступа,
    // не сможет получить время, погоду и т.д.
    bool     stop_waiting = false;
    uint32_t start_wifi_check = millis();
    uint32_t last_wifi_check = 0;
    int16_t  cnt = 0;
    while (!(stop_waiting || wifi_connected)) {
      delay(0);
      if (millis() - last_wifi_check > 250) {
        last_wifi_check = millis();
        set_wifi_connected(WiFi.status() == WL_CONNECTED); 
        if (wifi_connected) {
          // Подключение установлено
          DEBUGLN();
          DEBUG(F("WiFi подключен. IP адрес: "));
          DEBUGLN(WiFi.localIP());
          break;
        }
        if (cnt % 50 == 0) {
          DEBUGLN();
        }
        DEBUG(".");
        cnt++;
      }
      if (millis() - start_wifi_check > waitTime) {
        // Время ожидания подключения к сети вышло
        break;
      }
      delay(0);
      // Опрос состояния кнопки
      butt.tick();
      if (butt.hasClicks()) {
        butt.getClicks();
        DEBUGLN();
        DEBUGLN(F("Нажата кнопка.\nОжидание подключения к сети WiFi прервано."));  
        stop_waiting = true;
        break;
      }
      delay(0);
    }
    DEBUGLN();

    if (!wifi_connected && !stop_waiting)
      DEBUGLN(F("Не удалось подключиться к сети WiFi."));
  }  
}

void startSoftAP() {
  WiFi.softAPdisconnect(true);
  ap_connected = false;

  DEBUG(F("Создание точки доступа "));
  DEBUG(apName);
  
  ap_connected = WiFi.softAP(apName, apPass);

  for (uint8_t j = 0; j < 10; j++ ) {    
    delay(0);
    if (ap_connected) {
      DEBUGLN();
      DEBUG(F("Точка доступа создана. Сеть: '"));
      DEBUG(apName);
      // Если пароль совпадает с паролем по умолчанию - печатать для информации,
      // если был изменен пользователем - не печатать
      if (strcmp(apPass, "12341234") == 0) {
        DEBUG(F("'. Пароль: '"));
        DEBUG(apPass);
      }
      DEBUGLN(F("'."));
      DEBUG(F("IP адрес: "));
      DEBUGLN(WiFi.softAPIP());
      break;
    }    
    
    WiFi.enableAP(false);
    WiFi.softAPdisconnect(true);
    delay(500);
    
    DEBUG(".");
    ap_connected = WiFi.softAP(apName, apPass);
  }  
  DEBUGLN();  

  if (!ap_connected) 
    DEBUGLN(F("Не удалось создать WiFi точку доступа."));
}

void connectToNetwork() {
  // Подключиться к WiFi сети, ожидать подключения 180 сек пока, например, после отключения электричества роутер загрузится и поднимет сеть
  startWiFi(60000);    //было 180, поставил 60; 3 минуты на мой взгляд слишком долго

  // Если режим точки доступа не используется и к WiFi сети подключиться не удалось - создать точку доступа
  if (!wifi_connected){
    WiFi.mode(WIFI_AP);
    startSoftAP();
  }

  if (useSoftAP && !ap_connected) startSoftAP();    

  // Сообщить UDP порт, на который ожидаются подключения
  if (wifi_connected || ap_connected) {
    DEBUG(F("UDP-сервер на порту "));
    DEBUGLN(localPort);
  }
}
