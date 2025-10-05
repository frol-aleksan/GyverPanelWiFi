// ***************** ДЛЯ РАЗРАБОТЧИКОВ ******************
// The 16 bit version of our coordinates
static uint16_t x;
static uint16_t y;
static uint16_t z;


uint8_t **noise;
uint8_t ihue = 0;

void madnessNoise() {
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100);
  fillnoise8();
  for (uint8_t i = 0; i < pWIDTH; i++) {
    for (uint8_t j = 0; j < pHEIGHT; j++) {
      CRGB thisColor = CHSV(noise[j][i], 255, map8(noise[i][j], effectBrightness / 2, effectBrightness));
      drawPixelXY(i, j, thisColor);
    }
  }
  ihue += 1;
}

void rainbowNoise() {
  currentPalette = RainbowColors_p;
  colorLoop = 1;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void rainbowStripeNoise() {
  currentPalette = RainbowStripeColors_p;
  colorLoop = 1;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void zebraNoise() {
  fill_solid( currentPalette, 16, CRGB::Black);
  // and set every fourth one to white.
  currentPalette[0] = CRGB::White;
  currentPalette[4] = CRGB::White;
  currentPalette[8] = CRGB::White;
  currentPalette[12] = CRGB::White;
  colorLoop = 1;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void forestNoise() {
  currentPalette = ForestColors_p;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void oceanNoise() {
  currentPalette = OceanColors_p;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void plasmaNoise() {
  currentPalette = PartyColors_p;
  colorLoop = 1;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void cloudNoise() {
  currentPalette = CloudColors_p;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void lavaNoise() {
  currentPalette = LavaColors_p;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void alcoholFireNoise() {
  currentPalette = AlcoholFireColors_p;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void heatColorsNoise() {
  currentPalette = HeatColors_p;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void waterfallColors4in1Noise() {
  currentPalette = WaterfallColors4in1_p;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void woodFireColorsNoise() {
  currentPalette = WoodFireColors_p;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void normalFireNoise() {
  currentPalette = NormalFire_p;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void normalFire2Noise() {
  currentPalette = NormalFire2_p;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void lithiumFireColorsNoise() {
  currentPalette = LithiumFireColors_p;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void sodiumFireColorsNoise() {
  currentPalette = SodiumFireColors_p;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void copperFireColorsNoise() {
  currentPalette = CopperFireColors_p;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void rubidiumFireColorsNoise() {
  currentPalette = RubidiumFireColors_p;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void potassiumFireColorsNoise() {
  currentPalette = PotassiumFireColors_p;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void SunsetRealNoise() {
  currentPalette = Sunset_Real_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void dkblueredNoise() {
  currentPalette = dkbluered_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void OptimusPrimeNoise() {
  currentPalette = Optimus_Prime_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void warmGradNoise() {
  currentPalette = warmGrad_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void coldGradNoise() {
  currentPalette = coldGrad_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void hotGradNoise() {
  currentPalette = hotGrad_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void pinkGradNoise() {
  currentPalette = pinkGrad_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void comfyNoise() {
  currentPalette = comfy_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void cyperpunkNoise() {
  currentPalette = cyperpunk_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void girlNoise() {
  currentPalette = girl_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void xmasNoise() {
  currentPalette = xmas_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void acidNoise() {
  currentPalette = acid_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void blueSmokeNoise() {
  currentPalette = blueSmoke_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void gummyNoise() {
  currentPalette = gummy_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void leoNoise() {
  currentPalette = leo_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void auroraNoise() {
  currentPalette = aurora_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void rainCloudsNoise() {
  currentPalette = rainClouds_p;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void redwhiteNoise() {
  currentPalette = redwhite_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void ib_jul01Noise() {
  currentPalette = ib_jul01_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void rgi_15Noise() {
  currentPalette = rgi_15_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void retro2_16Noise() {
  currentPalette = retro2_16_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void Analogous_1Noise() {
  currentPalette = Analogous_1_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void pinksplash_08Noise() {
  currentPalette = pinksplash_08_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void pinksplash_07Noise() {
  currentPalette = pinksplash_07_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void Coral_reefNoise() {
  currentPalette = Coral_reef_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void ocean_breezeNoise() {
  currentPalette = ocean_breeze_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void landscape_64Noise() {
  currentPalette = landscape_64_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void landscape_33Noise() {
  currentPalette = landscape_33_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void rainbowsherbetNoise() {
  currentPalette = rainbowsherbet_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void gr65_hultNoise() {
  currentPalette = gr65_hult_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void GMT_drywetNoise() {
  currentPalette = GMT_drywet_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void emerald_dragonNoise() {
  currentPalette = emerald_dragon_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void ColorfullNoise() {
  currentPalette = Colorfull_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void Pink_PurpleNoise() {
  currentPalette = Pink_Purple_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void autumn_19Noise() {
  currentPalette = autumn_19_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void daybreakNoise() {
  currentPalette = daybreak_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void Blue_Cyan_YellowNoise() {
  currentPalette = Blue_Cyan_Yellow_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void bhw1_28Noise() {
  currentPalette = bhw1_28_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void rbwNoise() {
  currentPalette = rbw_gp;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}
/* функции рабочие, но в качестве вариантов для эффекта Переливы данные палитры выглядят не очень
void pacifica1Noise() {
  currentPalette = pacifica_palette_1;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void pacifica2Noise() {
  currentPalette = pacifica_palette_2;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void pacifica3Noise() {
  currentPalette = pacifica_palette_3;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}

void WaterfallColorsNoise() {
  currentPalette = WaterfallColors_p;
  colorLoop = 0;
  scale = map8(getEffectScaleParamValue(MC_NOISE_EFFECTS),0,100); 
  fillNoiseLED();
}*/

// ******************* СЛУЖЕБНЫЕ *******************
void fillNoiseLED() {
  uint8_t dataSmoothing = 0;
  if ( speed < 50) {
    dataSmoothing = 200 - (speed * 4);
  }

  for (uint8_t i = 0; i < maxDim; i++) {
    uint16_t ioffset = scale * i;
    for (uint8_t j = 0; j < maxDim; j++) {
      uint16_t joffset = scale * j;

      uint8_t data = inoise8(x + ioffset, y + joffset, z);

      data = qsub8(data, 16);
      data = qadd8(data, scale8(data, 39));

      if ( dataSmoothing ) {
        uint8_t olddata = noise[i][j];
        uint8_t newdata = scale8( olddata, dataSmoothing) + scale8( data, 256 - dataSmoothing);
        data = newdata;
      }

      noise[i][j] = data;
    }
  }
  z += speed;

  // apply slow drift to X and Y, just for visual variation.
  x += speed / 8;
  y -= speed / 16;

  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));

  for (uint8_t i = 0; i < pWIDTH; i++) {
    for (uint8_t j = 0; j < pHEIGHT; j++) {
      uint8_t index = noise[j][i];
      uint8_t bri =   noise[i][j];
      // if this palette is a 'loop', add a slowly-changing base value
      if ( colorLoop) {
        index += ihue;
      }
      // brighten up, as the color palette itself often contains the
      // light/dark dynamic range desired
      if ( bri > map8(effectBrightness,0,127) ) { 
        bri = effectBrightness; // 255;
      } else {
        bri = dim8_raw( bri * 2);
      }
      CRGB color = ColorFromPalette( currentPalette, index, bri);      
      drawPixelXY(i, j, color);   //leds[getPixelNumber(i, j)] = color;
    }
  }
  ihue += 1;
}

void fillnoise8() {
  for (uint8_t i = 0; i < maxDim; i++) {
    int16_t ioffset = scale * i;
    for (uint8_t j = 0; j < maxDim; j++) {
      int16_t joffset = scale * j;
      noise[i][j] = inoise8(x + ioffset, y + joffset, z);
    }
  }
  z += speed;
}

void createNoise() {
  if (noise == NULL) { noise = new uint8_t*[maxDim]; for (uint8_t i = 0; i < maxDim; i++) noise[i] = new uint8_t[maxDim];  }    
}

void releaseNoise() {
  if (noise != NULL) { for (uint8_t i = 0; i < maxDim; i++) delete[] noise[i]; delete[] noise; noise = NULL; }
}
