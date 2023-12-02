#include "patterns.h"

int8_t patternIdx = -1;
int8_t lineIdx = 0;
int8_t columnIdx = 0;
bool   isWhite = false;

// Заполнение матрицы указанным паттерном
// ptrn - индекс узора в массив узоров patterns[] в patterns.h
// W   - ширина паттерна
// H   - высота паттерна
void drawPattern(uint8_t ptrn, uint8_t W, uint8_t H) {

  // Идея "подвижного" смещения узора - (С) Stepko
  // https://editor.soulmatelights.com/gallery/761-patterns

  uint8_t y_offs = 0;
  uint8_t x_offs = 0;
    
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  uint8_t variant = map8(getEffectScaleParamValue(MC_PATTERNS),0,4);
  uint8_t spd1 = map8(255-getEffectSpeedValue(MC_PATTERNS), 6, 15);
  uint8_t spd2 = map8(255-getEffectSpeedValue(MC_PATTERNS), 7, 20);

  switch(variant) {
    case 1:
      // снизу вверх
      columnIdx = -((pWIDTH % W) / 2) + 1;      
      lineIdx++;      
      if (lineIdx >= H) lineIdx = 0;
      y_offs = lineIdx;
      x_offs = columnIdx;
      break;
    case 2:
      // сверху вниз
      columnIdx = -((pWIDTH % W) / 2) + 1;      
      lineIdx--;      
      if (lineIdx < 0) lineIdx = H - 1;
      y_offs = lineIdx;
      x_offs = columnIdx;
      break;
    case 3:
      // справа налево
      lineIdx = 0;      
      columnIdx++;      
      if (columnIdx >= W) columnIdx = 0;
      y_offs = lineIdx;
      x_offs = columnIdx;
      break;
    case 4:
      // слева направо
      lineIdx = 0;      
      columnIdx--;      
      if (columnIdx < 0) columnIdx = W - 1;
      y_offs = lineIdx;
      x_offs = columnIdx;
      break;
    default:
      // Переменное движение
      y_offs = beatsin8(spd1, 1, 32); // for X and Y texture move
      x_offs = beatsin8(spd2, 1, 32); // for X and Y texture move
      break;
  }
  
  for (uint8_t y = 0; y < pHEIGHT; y++) {
    for (uint8_t x = 0; x < pWIDTH; x++) {
      uint8_t in = (uint8_t) pgm_read_byte( & (patterns[ptrn][(y_offs + y) % H][(x_offs + x) % W]));
      CHSV color = colorMR[in];
      CHSV color2 = color.v != 0 ? CHSV(color.h, color.s, effectBrightness) : color;
      drawPixelXY(x, pHEIGHT - y - 1, color2); 
    }
  }
}

// Отрисовка указанной картинки с размерами WxH в позиции XY
void drawPicture_XY(uint8_t iconIdx, uint8_t X, uint8_t Y, uint8_t W, uint8_t H) {
  if (loadingFlag) {
    loadingFlag = false;
  }

  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  for (uint8_t x = 0; x < W; x++) {
    for (uint8_t y = 0; y < H; y++) {
      uint8_t in = (uint8_t)pgm_read_byte(&(patterns[iconIdx][y][x])); 
      if (in != 0) {
        CHSV color = colorMR[in];        
        CHSV color2 = color.v != 0 ? CHSV(color.h, color.s, effectBrightness) : color;
        drawPixelXY(X+x,Y+H-y, color2); 
      }
    }
  }
}

void patternRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    palette_number = getEffectScaleParamValue2(MC_PATTERNS);
    if (palette_number == 0 || palette_number == MAX_PATTERN + 2) patternIdx = random8(0,MAX_PATTERN); //Если Случайный выбор или Авто, задать произвольный вариант (в Авто от него начинается отсчет)
    else if (palette_number > 0 || palette_number < MAX_PATTERN + 2) patternIdx = palette_number - 1;  //Если что-то из вариантов 1-33, берем только это значение
    
    lineIdx = 9;         // Картинка спускается сверху вниз - отрисовка с нижней строки паттерна (паттерн 10x10) dir='d'
 // lineIdx = 0;         // Картинка поднимается сверху вниз - отрисовка с верхней строки паттерна dir='u'

    isWhite = false;
    hue = random8();
    colorMR[6] = CHSV(hue, 255, 255);
    colorMR[7] = CHSV(hue + 80, 255, 255);
    if (random8() % 10 == 0) {
      colorMR[6] = CHSV(0,0,255);
      isWhite = true;
    }
  }
  
  if (palette_number == MAX_PATTERN + 2) {  //автоперебор вариантов, если выбран вариант Авто, дается 10 сек на эффект
    if (millis()- color_timer > 10000) { 
      color_timer = millis();
      patternIdx++;
      if (patternIdx > MAX_PATTERN) patternIdx = 0;
    }
  }
  
  hue++;
  if (!isWhite) colorMR[6] = CHSV(hue, 255, 255);
  colorMR[7] = CHSV(hue + 80, 255, 255);
  colorMR[8] = CHSV(hue + 160, 255, 255);
  drawPattern(patternIdx, 10, 10);  
}
