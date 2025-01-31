// игра "бегалка прыгалка"

// **************** НАСТРОЙКИ RUNNER ****************
// 1 пиксель = 10 мм
#define POP_SPEED 420     // скорость подскока (в мм/с)
#define GRAVITY 1400      // ускорение падения (в мм/с2)
#define DT 20             // период интегрирования (мс)
#define OBST_SPEED 50     // скорость препятствий
#define MIN_GAP 8         // минимальное расстояние между препятствиями
#define OBST_HEIGHT 4     // макс. высота препятствия
#define OBST_PROB 10      // шанс появления препятствия
#define DEMO_JUMP 6       // за какое расстояние делать прыжок в демо режиме

timerMinim runnerTimer(DT);
timerMinim obstTimer(OBST_SPEED);

int posRun = 0;
int velRun = 0;  // в мм и мм/с
byte prevPosRun;
byte obstCounter;
int runnerScore;

void runnerRoutine() {
  if (loadingFlag) {
    FastLED.clear();
    loadingFlag = false;
  }
  if (gameDemo) {
    if (getPixColorXY(DEMO_JUMP, 0) == GLOBAL_COLOR_1) buttons = 0;   // автопрыжок
  }

  if (checkButtons()) {
    if (buttons == 0) {   // кнопка нажата
      if (posRun == 0) {
        velRun = POP_SPEED;        
      }
      buttons = 4;
    }
  }

  if (runnerTimer.isReady()) {
    if (obstTimer.isReady()) {
      for (byte i = 0; i < pWIDTH - 1; i++) {
        for (byte j = 0; j < pHEIGHT; j++) {
          leds[getPixelNumber(i, j)] = getPixColorXY(i + 1, j);
        }
      }

      obstCounter++;
      if (obstCounter >= MIN_GAP && random(0, OBST_PROB) == 0) {
        obstCounter = 0;
        runnerScore++;
        byte thisHeight = random(1, OBST_HEIGHT + 1);
        for (byte i = 0; i < thisHeight; i++)
          drawPixelXY(pWIDTH - 1, i, GLOBAL_COLOR_1);
      } else {
        for (byte i = 0; i < pHEIGHT; i++)
          drawPixelXY(pWIDTH - 1, i, 0);
      }
    }

    velRun -= (float)GRAVITY * DT / 1000;
    posRun += (float)velRun * DT / 1000;
    if (posRun < 0) {
      posRun = 0;
      velRun = 0;
    }
    if (getPixColorXY(1, posRun / 10) == GLOBAL_COLOR_1) {
    for (uint8_t bright = 0; bright < 15; bright++) {
        FastLED.setBrightness(bright);
        for (uint16_t i = 0; i < NUM_LEDS; i++) {
          leds[i] = CRGB::Red;
        }
        FastLEDshow();
        delay(10);
      }
      delay(100);
      FastLED.clear();
      FastLED.setBrightness(globalBrightness);
      displayScore(runnerScore);
      FastLEDshow();
      delay(1500);
      runnerScore = 0;
      loadingFlag = true;
      return;
    }
    drawPixelXY(1, prevPosRun / 10, 0x000000);
    drawPixelXY(1, posRun / 10, GLOBAL_COLOR_2);
    prevPosRun = posRun;
    FastLED.show();
  }
}
