// Эта функция в FastLED объявлена как forward;
// линкуется с библиотекой FastLed, которая использует её для определения индекса светодиода в массиве leds[]
// ври вызове функций типа blur2d() и т.п.
float sqrt3(const float x) {
  union {
    int i;
    float x;
  } u;
  u.x = x;
  u.i = (1 << 29) + (u.i >> 1) - (1 << 22);
  return u.x;
}

#define SQRT_VARIANT sqrt3   // выбор основной функции для вычисления квадратного корня sqrtf или sqrt3 для ускорения

void blurScreen(fract8 blur_amount, CRGB *LEDarray = leds) {
  blur2d(LEDarray, pWIDTH, pHEIGHT, blur_amount);
}
struct ModeType {
  uint8_t Brightness = 50U;
  uint8_t Speed = 225U;
  uint8_t Scale = 40U;
};

template <class T>
class Vector2 {
  public:
    T x, y;
    Vector2() : x(0), y(0) {}
    Vector2(T x, T y) : x(x), y(y) {}
    Vector2(const Vector2& v) : x(v.x), y(v.y) {}
    Vector2& operator=(const Vector2& v) {
      x = v.x;
      y = v.y;
      return *this;
    }
    bool isEmpty() {
      return x == 0 && y == 0;
    }
    bool operator==(Vector2& v) {
      return x == v.x && y == v.y;
    }
    bool operator!=(Vector2& v) {
      return !(x == y);
    }
    Vector2 operator+(Vector2& v) {
      return Vector2(x + v.x, y + v.y);
    }
    Vector2 operator-(Vector2& v) {
      return Vector2(x - v.x, y - v.y);
    }
    Vector2& operator+=(Vector2& v) {
      x += v.x;
      y += v.y;
      return *this;
    }
    Vector2& operator-=(Vector2& v) {
      x -= v.x;
      y -= v.y;
      return *this;
    }
    Vector2 operator+(double s) {
      return Vector2(x + s, y + s);
    }
    Vector2 operator-(double s) {
      return Vector2(x - s, y - s);
    }
    Vector2 operator*(double s) {
      return Vector2(x * s, y * s);
    }
    Vector2 operator/(double s) {
      return Vector2(x / s, y / s);
    }
    Vector2& operator+=(double s) {
      x += s;
      y += s;
      return *this;
    }
    Vector2& operator-=(double s) {
      x -= s;
      y -= s;
      return *this;
    }
    Vector2& operator*=(double s) {
      x *= s;
      y *= s;
      return *this;
    }
    Vector2& operator/=(double s) {
      x /= s;
      y /= s;
      return *this;
    }
    void set(T x, T y) {
      this->x = x;
      this->y = y;
    }
    void rotate(double deg) {
      double theta = deg / 180.0 * M_PI;
      double c = cos(theta);
      double s = sin(theta);
      double tx = x * c - y * s;
      double ty = x * s + y * c;
      x = tx;
      y = ty;
    }
    Vector2& normalize() {
      if (length() == 0) return *this;
      *this *= (1.0 / length());
      return *this;
    }
    float dist(Vector2 v) const {
      Vector2 d(v.x - x, v.y - y);
      return d.length();
    }
    float length() const {
      return sqrt(x * x + y * y);
    }
    float mag() const {
      return length();
    }
    float magSq() {
      return (x * x + y * y);
    }
    void truncate(double length) {
      double angle = atan2f(y, x);
      x = length * cos(angle);
      y = length * sin(angle);
    }
    Vector2 ortho() const {
      return Vector2(y, -x);
    }
    static float dot(Vector2 v1, Vector2 v2) {
      return v1.x * v2.x + v1.y * v2.y;
    }
    static float cross(Vector2 v1, Vector2 v2) {
      return (v1.x * v2.y) - (v1.y * v2.x);
    }
    void limit(float max) {
      if (magSq() > max * max) {
        normalize();
        *this *= max;
      }
    }
};

typedef Vector2<float> PVector;
class Boid {
  public:
    PVector location;
    PVector velocity;
    PVector acceleration;
    float maxforce;    // Maximum steering force
    float maxspeed;    // Maximum speed
    float desiredseparation = 4;
    float neighbordist = 8;
    byte colorIndex = 0;
    float mass;
    boolean enabled = true;
    Boid() {}
    Boid(float x, float y) {
      acceleration = PVector(0, 0);
      velocity = PVector(randomf(), randomf());
      location = PVector(x, y);
      maxspeed = 1.5;
      maxforce = 0.05;
    }
    static float randomf() {
      return mapfloat(random(0, 255), 0, 255, -.5, .5);
    }
    static float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    void run(Boid boids [], uint8_t boidCount) {
      flock(boids, boidCount);
      update();
    }
    // Method to update location
    void update() {
      // Update velocity
      velocity += acceleration;
      // Limit speed
      velocity.limit(maxspeed);
      location += velocity;
      // Reset acceleration to 0 each cycle
      acceleration *= 0;
    }
    void applyForce(PVector force) {
      // We could add mass here if we want A = F / M
      acceleration += force;
    }
    void repelForce(PVector obstacle, float radius) {
      //Force that drives boid away from obstacle.
      PVector futPos = location + velocity; //Calculate future position for more effective behavior.
      PVector dist = obstacle - futPos;
      float d = dist.mag();
      if (d <= radius) {
        PVector repelVec = location - obstacle;
        repelVec.normalize();
        if (d != 0) { //Don't divide by zero.
          repelVec.normalize();
          repelVec *= (maxforce * 7);
          if (repelVec.mag() < 0) { //Don't let the boids turn around to avoid the obstacle.
            repelVec.y = 0;
          }
        }
        applyForce(repelVec);
      }
    }
    // We accumulate a new acceleration each time based on three rules
    void flock(Boid boids [], uint8_t boidCount) {
      PVector sep = separate(boids, boidCount);   // Separation
      PVector ali = align(boids, boidCount);      // Alignment
      PVector coh = cohesion(boids, boidCount);   // Cohesion
      // Arbitrarily weight these forces
      sep *= 1.5;
      ali *= 1.0;
      coh *= 1.0;
      // Add the force vectors to acceleration
      applyForce(sep);
      applyForce(ali);
      applyForce(coh);
    }
    // Separation
    // Method checks for nearby boids and steers away
    PVector separate(Boid boids [], uint8_t boidCount) {
      PVector steer = PVector(0, 0);
      int count = 0;
      // For every boid in the system, check if it's too close
      for (int i = 0; i < boidCount; i++) {
        Boid other = boids[i];
        if (!other.enabled)
          continue;
        float d = location.dist(other.location);
        // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
        if ((d > 0) && (d < desiredseparation)) {
          // Calculate vector pointing away from neighbor
          PVector diff = location - other.location;
          diff.normalize();
          diff /= d;        // Weight by distance
          steer += diff;
          count++;            // Keep track of how many
        }
      }
      // Average -- divide by how many
      if (count > 0) {
        steer /= (float) count;
      }
      // As long as the vector is greater than 0
      if (steer.mag() > 0) {
        // Implement Reynolds: Steering = Desired - Velocity
        steer.normalize();
        steer *= maxspeed;
        steer -= velocity;
        steer.limit(maxforce);
      }
      return steer;
    }
    // Alignment
    // For every nearby boid in the system, calculate the average velocity
    PVector align(Boid boids [], uint8_t boidCount) {
      PVector sum = PVector(0, 0);
      int count = 0;
      for (int i = 0; i < boidCount; i++) {
        Boid other = boids[i];
        if (!other.enabled)
          continue;
        float d = location.dist(other.location);
        if ((d > 0) && (d < neighbordist)) {
          sum += other.velocity;
          count++;
        }
      }
      if (count > 0) {
        sum /= (float) count;
        sum.normalize();
        sum *= maxspeed;
        PVector steer = sum - velocity;
        steer.limit(maxforce);
        return steer;
      }
      else {
        return PVector(0, 0);
      }
    }
    // Cohesion
    // For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
    PVector cohesion(Boid boids [], uint8_t boidCount) {
      PVector sum = PVector(0, 0);   // Start with empty vector to accumulate all locations
      int count = 0;
      for (int i = 0; i < boidCount; i++) {
        Boid other = boids[i];
        if (!other.enabled)
          continue;
        float d = location.dist(other.location);
        if ((d > 0) && (d < neighbordist)) {
          sum += other.location; // Add location
          count++;
        }
      }
      if (count > 0) {
        sum /= count;
        return seek(sum);  // Steer towards the location
      }
      else {
        return PVector(0, 0);
      }
    }
    // A method that calculates and applies a steering force towards a target
    // STEER = DESIRED MINUS VELOCITY
    PVector seek(PVector target) {
      PVector desired = target - location;  // A vector pointing from the location to the target
      // Normalize desired and scale to maximum speed
      desired.normalize();
      desired *= maxspeed;
      // Steering = Desired minus Velocity
      PVector steer = desired - velocity;
      steer.limit(maxforce);  // Limit to maximum steering force
      return steer;
    }
    // A method that calculates a steering force towards a target
    // STEER = DESIRED MINUS VELOCITY
    void arrive(PVector target) {
      PVector desired = target - location;  // A vector pointing from the location to the target
      float d = desired.mag();
      // Normalize desired and scale with arbitrary damping within 100 pixels
      desired.normalize();
      if (d < 4) {
        float m = map(d, 0, 100, 0, maxspeed);
        desired *= m;
      }
      else {
        desired *= maxspeed;
      }
      // Steering = Desired minus Velocity
      PVector steer = desired - velocity;
      steer.limit(maxforce);  // Limit to maximum steering force
      applyForce(steer);
    }
    void wrapAroundBorders() {
      if (location.x < 0) location.x = pWIDTH - 1;
      if (location.y < 0) location.y = pHEIGHT - 1;
      if (location.x >= pWIDTH) location.x = 0;
      if (location.y >= pHEIGHT) location.y = 0;
    }
    void avoidBorders() {
      PVector desired = velocity;
      if (location.x < 8) desired = PVector(maxspeed, velocity.y);
      if (location.x >= pWIDTH - 8) desired = PVector(-maxspeed, velocity.y);
      if (location.y < 8) desired = PVector(velocity.x, maxspeed);
      if (location.y >= pHEIGHT - 8) desired = PVector(velocity.x, -maxspeed);
      if (desired != velocity) {
        PVector steer = desired - velocity;
        steer.limit(maxforce);
        applyForce(steer);
      }
      if (location.x < 0) location.x = 0;
      if (location.y < 0) location.y = 0;
      if (location.x >= pWIDTH) location.x = pWIDTH - 1;
      if (location.y >= pHEIGHT) location.y = pHEIGHT - 1;
    }
    bool bounceOffBorders(float bounce) {
      bool bounced = false;
      if (location.x >= pWIDTH) {
        location.x = pWIDTH - 1;
        velocity.x *= -bounce;
        bounced = true;
      }
      else if (location.x < 0) {
        location.x = 0;
        velocity.x *= -bounce;
        bounced = true;
      }
      if (location.y >= pHEIGHT) {
        location.y = pHEIGHT - 1;
        velocity.y *= -bounce;
        bounced = true;
      }
      else if (location.y < 0) {
        location.y = 0;
        velocity.y *= -bounce;
        bounced = true;
      }
      return bounced;
    }
    void render() {

    }
};

#define trackingOBJECT_MAX_COUNT                         (100U)  // максимальное количество отслеживаемых объектов (очень влияет на расход памяти)
float   trackingObjectPosX[trackingOBJECT_MAX_COUNT];
float   trackingObjectPosY[trackingOBJECT_MAX_COUNT];
float   trackingObjectSpeedX[trackingOBJECT_MAX_COUNT];
float   trackingObjectSpeedY[trackingOBJECT_MAX_COUNT];
float   trackingObjectShift[trackingOBJECT_MAX_COUNT];
uint8_t trackingObjectHue[trackingOBJECT_MAX_COUNT];
uint8_t trackingObjectState[trackingOBJECT_MAX_COUNT];
bool    trackingObjectIsShift[trackingOBJECT_MAX_COUNT];
#define enlargedOBJECT_MAX_COUNT          (pWIDTH * 2) // максимальное количество сложных отслеживаемых объектов (меньше, чем trackingOBJECT_MAX_COUNT)
#define NUM_LAYERSMAX (numlayersmax)
uint16_t ff_x, ff_y, ff_z;                             // большие счётчики
uint8_t enlargedObjectNUM;                             // используемое в эффекте количество объектов
uint8_t* shiftHue = new uint8_t[pHEIGHT];                             // свойство пикселей в размер столбца матрицы
uint8_t* shiftValue = new uint8_t[pHEIGHT];                           // свойство пикселей в размер столбца матрицы ещё одно
long* enlargedObjectTime = new long[enlargedOBJECT_MAX_COUNT];
float* liquidLampHot = new float[enlargedOBJECT_MAX_COUNT];
float* liquidLampSpf = new float[enlargedOBJECT_MAX_COUNT];
unsigned* liquidLampMX = new unsigned[enlargedOBJECT_MAX_COUNT];
unsigned* liquidLampSC = new unsigned[enlargedOBJECT_MAX_COUNT];
unsigned* liquidLampTR = new unsigned[enlargedOBJECT_MAX_COUNT];  //объявил массивы так.

//константы размера матрицы вычисляется только здесь и не меняется в эффектах
const uint8_t CENTER_X_MINOR =  (pWIDTH / 2) -  ((pWIDTH - 1) & 0x01); // центр матрицы по ИКСУ, сдвинутый в меньшую сторону, если ширина чётная
const uint8_t CENTER_Y_MINOR = (pHEIGHT / 2) - ((pHEIGHT - 1) & 0x01); // центр матрицы по ИГРЕКУ, сдвинутый в меньшую сторону, если высота чётная
const uint8_t CENTER_X_MAJOR =   pWIDTH / 2  + (pWIDTH % 2);           // центр матрицы по ИКСУ, сдвинутый в большую сторону, если ширина чётная
const uint8_t CENTER_Y_MAJOR =  pHEIGHT / 2  + (pHEIGHT % 2);          // центр матрицы по ИГРЕКУ, сдвинутый в большую сторону, если высота чётная
float speedfactor;                                 // регулятор скорости в эффектах реального времени
ModeType modes[MAX_EFFECT];
uint8_t currentMode = 0;
uint8_t step;                                      // какой-нибудь счётчик кадров или последовательностей операций
uint8_t hue, hue2;                                 // постепенный сдвиг оттенка или какой-нибудь другой цикличный счётчик
uint8_t deltaHue, deltaHue2;                       // ещё пара таких же, когда нужно много
uint8_t deltaValue;                                // просто повторно используемая переменная
float emitterX, emitterY;                          // какие-то динамичные координаты
uint32_t noise32_x[NUM_LAYERSMAX];
uint32_t noise32_y[NUM_LAYERSMAX];
uint32_t noise32_z[NUM_LAYERSMAX];
uint32_t scale32_x[NUM_LAYERSMAX];
uint32_t scale32_y[NUM_LAYERSMAX];
uint8_t noisesmooth;
int8_t zD;
int8_t zF;

// -------основные палитры--------------
extern const TProgmemRGBPalette16 WaterfallColors_p FL_PROGMEM = {0x000000, 0x060707, 0x101110, 0x151717, 0x1C1D22, 0x242A28, 0x363B3A, 0x313634, 0x505552, 0x6B6C70, 0x98A4A1, 0xC1C2C1, 0xCACECF, 0xCDDEDD, 0xDEDFE0, 0xB2BAB9};
static const uint8_t AVAILABLE_BOID_COUNT = 20U;
Boid boids[AVAILABLE_BOID_COUNT];
const TProgmemRGBPalette16 *palette_arr[] = {
  &PartyColors_p,
  &OceanColors_p,
  &LavaColors_p,
  &HeatColors_p,
  &WaterfallColors_p,
  &CloudColors_p,
  &ForestColors_p,
  &RainbowColors_p,
  &RainbowStripeColors_p
};
const TProgmemRGBPalette16 *curPalette = palette_arr[0];

// -------массив "огненных" палитр--------------
extern const TProgmemRGBPalette16 WoodFireColors_p FL_PROGMEM = {CRGB::Black, 0x330e00, 0x661c00, 0x992900, 0xcc3700, CRGB::OrangeRed, 0xff5800, 0xff6b00, 0xff7f00, 0xff9200, CRGB::Orange, 0xffaf00, 0xffb900, 0xffc300, 0xffcd00, CRGB::Gold};             //* Orange
extern const TProgmemRGBPalette16 NormalFire_p FL_PROGMEM = {CRGB::Black, 0x330000, 0x660000, 0x990000, 0xcc0000, CRGB::Red, 0xff0c00, 0xff1800, 0xff2400, 0xff3000, 0xff3c00, 0xff4800, 0xff5400, 0xff6000, 0xff6c00, 0xff7800};                             // пытаюсь сделать что-то более приличное
extern const TProgmemRGBPalette16 NormalFire2_p FL_PROGMEM = {CRGB::Black, 0x560000, 0x6b0000, 0x820000, 0x9a0011, CRGB::FireBrick, 0xc22520, 0xd12a1c, 0xe12f17, 0xf0350f, 0xff3c00, 0xff6400, 0xff8300, 0xffa000, 0xffba00, 0xffd400};                      // пытаюсь сделать что-то более приличное
extern const TProgmemRGBPalette16 LithiumFireColors_p FL_PROGMEM = {CRGB::Black, 0x240707, 0x470e0e, 0x6b1414, 0x8e1b1b, CRGB::FireBrick, 0xc14244, 0xd16166, 0xe08187, 0xf0a0a9, CRGB::Pink, 0xff9ec0, 0xff7bb5, 0xff59a9, 0xff369e, CRGB::DeepPink};        //* Red
extern const TProgmemRGBPalette16 SodiumFireColors_p FL_PROGMEM = {CRGB::Black, 0x332100, 0x664200, 0x996300, 0xcc8400, CRGB::Orange, 0xffaf00, 0xffb900, 0xffc300, 0xffcd00, CRGB::Gold, 0xf8cd06, 0xf0c30d, 0xe9b913, 0xe1af1a, CRGB::Goldenrod};           //* Yellow
extern const TProgmemRGBPalette16 CopperFireColors_p FL_PROGMEM = {CRGB::Black, 0x001a00, 0x003300, 0x004d00, 0x006600, CRGB::Green, 0x239909, 0x45b313, 0x68cc1c, 0x8ae626, CRGB::GreenYellow, 0x94f530, 0x7ceb30, 0x63e131, 0x4bd731, CRGB::LimeGreen};     //* Green
extern const TProgmemRGBPalette16 AlcoholFireColors_p FL_PROGMEM = {CRGB::Black, 0x000033, 0x000066, 0x000099, 0x0000cc, CRGB::Blue, 0x0026ff, 0x004cff, 0x0073ff, 0x0099ff, CRGB::DeepSkyBlue, 0x1bc2fe, 0x36c5fd, 0x51c8fc, 0x6ccbfb, CRGB::LightSkyBlue};  //* Blue
extern const TProgmemRGBPalette16 RubidiumFireColors_p FL_PROGMEM = {CRGB::Black, 0x0f001a, 0x1e0034, 0x2d004e, 0x3c0068, CRGB::Indigo, CRGB::Indigo, CRGB::Indigo, CRGB::Indigo, CRGB::Indigo, CRGB::Indigo, 0x3c0084, 0x2d0086, 0x1e0087, 0x0f0089, CRGB::DarkBlue};        //* Indigo
extern const TProgmemRGBPalette16 PotassiumFireColors_p FL_PROGMEM = {CRGB::Black, 0x0f001a, 0x1e0034, 0x2d004e, 0x3c0068, CRGB::Indigo, 0x591694, 0x682da6, 0x7643b7, 0x855ac9, CRGB::MediumPurple, 0xa95ecd, 0xbe4bbe, 0xd439b0, 0xe926a1, CRGB::DeepPink}; //* Violet
const TProgmemRGBPalette16 *firePalettes[] = {
  &WoodFireColors_p,
  &NormalFire_p,
  &NormalFire2_p,
  &LithiumFireColors_p,
  &SodiumFireColors_p,
  &CopperFireColors_p,
  &AlcoholFireColors_p,
  &RubidiumFireColors_p,
  &PotassiumFireColors_p
};

//и еще кучка крутых палитр
DEFINE_GRADIENT_PALETTE( Sunset_Real_gp ) {
  0,    120,  0,    0,
  22,   179,  22,   0,
  51,   255,  104,  0,
  85,   167,  22,   18,
  135,  100,  0,    103,
  198,  16,   0,    130,
  255,  0,    0,    160
};

DEFINE_GRADIENT_PALETTE( dkbluered_gp ) {
  0,    1,    0,    4,
  8,    1,    0,    13,
  17,   1,    0,    29,
  25,   1,    0,    52,
  33,   1,    0,    83,
  42,   1,    0,    123,
  51,   1,    0,    174,
  59,   1,    0,    235,
  68,   1,    2,    255,
  76,   4,    17,   255,
  84,   16,   45,   255,
  93,   37,   82,   255,
  102,  69,   127,  255,
  110,  120,  168,  255,
  119,  182,  217,  255,
  127,  255,  255,  255,
  135,  255,  217,  184,
  144,  255,  168,  123,
  153,  255,  127,  73,
  161,  255,  82,   40,
  170,  255,  45,   18,
  178,  255,  17,   5,
  186,  255,  2,    1,
  195,  234,  0,    1,
  204,  171,  0,    1,
  212,  120,  0,    1,
  221,  79,   0,    1,
  229,  48,   0,    1,
  237,  26,   0,    1,
  246,  12,   0,    1,
  255,  4,    0,    1
};
DEFINE_GRADIENT_PALETTE( Optimus_Prime_gp ) {
  0,    5,    16,   18,
  25,   5,    16,   18,
  51,   7,    25,   39,
  76,   8,    38,   71,
  102,  64,   99,   106,
  127,  194,  189,  151,
  153,  182,  63,   42,
  178,  167,  6,    2,
  204,  100,  3,    1,
  229,  53,   1,    1,
  255,  53,   1,    1
};

DEFINE_GRADIENT_PALETTE( warmGrad_gp ) {
  0,    252,  252,  172,
  25,   239,  255,  61,
  53,   247,  45,   17,
  76,   197,  82,   19,
  96,   239,  255,  61,
  124,  83,   4,    1,
  153,  247,  45,   17,
  214,  23,   15,   17,
  255,  1,    1,    1
};

DEFINE_GRADIENT_PALETTE( coldGrad_gp ) {
  0,    66,   186,  192,
  43,   1,    22,   71,
  79,   2,    104,  142,
  117,  66,   186,  192,
  147,  2,    104,  142,
  186,  1,    22,   71,
  224,  2,    104,  142,
  255,  4,    27,   28
};

DEFINE_GRADIENT_PALETTE( hotGrad_gp ) {
  0,    157,  21,   2,
  35,   229,  244,  16,
  73,   255,  44,   7,
  107,  142,  7,    1,
  153,  229,  244,  16,
  206,  142,  7,    1,
  255,  135,  36,   0
};

DEFINE_GRADIENT_PALETTE( pinkGrad_gp ) {
  0,    249,  32,   145,
  28,   208,  1,    7,
  43,   249,  1,    19,
  56,   126,  152,  10,
  73,   234,  23,   84,
  89,   224,  45,   119,
  107,  232,  127,  158,
  127,  244,  13,   89,
  150,  188,  6,    52,
  175,  177,  70,   14,
  221,  194,  1,    8,
  255,  112,  0,    1
};

DEFINE_GRADIENT_PALETTE( comfy_gp ) {
  0,    255,  255,  45,
  43,   208,  93,   1,
  137,  224,  1,    242,
  181,  159,  1,    29,
  255,  63,   4,    68
};

DEFINE_GRADIENT_PALETTE( cyperpunk_gp ) {
  0,    3,    6,    72,
  38,   12,   50,   188,
  109,  217,  35,   1,
  135,  242,  175,  12,
  178,  161,  32,   87,
  255,  24,   6,    108
};

DEFINE_GRADIENT_PALETTE( girl_gp ) {
  0,    103,  1,    10,
  33,   109,  1,    12,
  76,   159,  5,    48,
  119,  175,  55,   103,
  127,  175,  55,   103,
  178,  159,  5,    48,
  221,  109,  1,    12,
  255,  103,  1,    10
};

DEFINE_GRADIENT_PALETTE( xmas_gp ) {
  0,    0,    12,   0,
  40,   0,    55,   0,
  66,   1,    117,  2,
  77,   1,    84,   1,
  81,   0,    55,   0,
  119,  0,    12,   0,
  153,  42,   0,    0,
  181,  121,  0,    0,
  204,  255 , 12,   8,
  224,  121,  0,    0,
  244,  42,   0,    0,
  255,  42,   0,    0
};

DEFINE_GRADIENT_PALETTE( acid_gp ) {
  0,    0,    12,   0,
  61,   153,  239,  112,
  127,  0,    12,   0,
  165,  106,  239,  2,
  196,  167,  229,  71,
  229,  106,  239,  2,
  255,  0,    12,   0
};

DEFINE_GRADIENT_PALETTE( blueSmoke_gp ) {
  0,    0,    0,    0,
  12,   1,    1,    3,
  53,   8,    1,    22,
  80,   4,    6,    89,
  119,  2,    25,   216,
  145,  7,    10,   99,
  186,  15,   2,    31,
  233,  2,    1,    5,
  255,  0,    0,    0
};

DEFINE_GRADIENT_PALETTE( gummy_gp ) {
  0,    8,    47,   5,
  31,   77,   122,  6,
  63,   249,  237,  7,
  95,   232,  51,   1,
  127,  215,  0,    1,
  159,  47,   1,    3,
  191,  1,    7,    16,
  223,  52,   22,   6,
  255,  239,  45,   1,
};

DEFINE_GRADIENT_PALETTE( leo_gp ) {
  0,    0,    0,    0,
  16,   0,    0,    0,
  32,   0,    0,    0,
  18,   0,    0,    0,
  64,   16,   8,    0,
  80,   80,   40,   0,
  96,   16,   8,    0,
  112,  0,    0,    0,
  128,  0,    0,    0,
  144,  0,    0,    0,
  160,  0,    0,    0,
  176,  0,    0,    0,
  192,  0,    0,    0,
  208,  0,    0,    0,
  224,  0,    0,    0,
  240,  0,    0,    0,
  255,  0,    0,    0,
};

DEFINE_GRADIENT_PALETTE ( aurora_gp ) {
  0,    17,   177,  13,   //Greenish
  64,   121,  242,  5,    //Greenish
  128,  25,   173,  121,  //Turquoise
  192,  250,  77,   127,  //Pink
  255,  171,  101,  221   //Purple
};

DEFINE_GRADIENT_PALETTE ( redwhite_gp ) {
  0,    255,  0,    0,
  25,   255,  255,  255,
  51,   255,  0,    0,
  76,   255,  255,  255,
  102,  255,  0,    0,
  127,  255,  255,  255,
  153,  255,  0,    0,
  178,  255,  255,  255,
  204,  255,  0,    0,
  229,  255,  255,  255,
  255,  255,  0,    0,
};

DEFINE_GRADIENT_PALETTE( ib_jul01_gp ) {
  0,    194,  1,    1,
  94,   1,    29,   18,
  132,  57,   131,  28,
  255,  113,  1,    1
};

DEFINE_GRADIENT_PALETTE( rgi_15_gp ) {
  0,    4,    1,    31,
  31,   55 ,  1,    16,
  63,   197,  3,    7,
  95,   59,   2,    17,
  127,  6,    2,    34,
  159,  39,   6,    33,
  191,  112,  13,   32,
  223,  56,   9,    35,
  255,  22,   6,    38
};

DEFINE_GRADIENT_PALETTE( retro2_16_gp ) {
  0,    188,  135,  1,
  255,  46,   7,    1
};

DEFINE_GRADIENT_PALETTE( Analogous_1_gp ) {
  0,    3,    0,    255,
  63,   23,   0,    255,
  127,  67,   0,    255,
  191,  142,  0,    45,
  255,  255,  0,    0
};

DEFINE_GRADIENT_PALETTE( pinksplash_08_gp ) {
  0,    126,  11,   255,
  127,  197,  1,    22,
  175,  210,  157,  172,
  221,  157,  3,    112,
  255,  157,  3,    112
};

DEFINE_GRADIENT_PALETTE( pinksplash_07_gp ) {
  0,    229,  1,    1,
  61,   242,  4,    63,
  101,  255,  12,   255,
  127,  249,  81,   252,
  153,  255,  11,   235,
  193,  244,  5,    68,
  255,  232,  1,    5
};

DEFINE_GRADIENT_PALETTE( Coral_reef_gp ) {
  0,    40,   199,  197,
  50,   10,   152,  155,
  96,   1,    111,  120,
  96,   43,   127,  162,
  139,  10,   73,   111,
  255,  1,    34,   71
};

DEFINE_GRADIENT_PALETTE( ocean_breeze_gp ) {
  0,    1,    6,    7,
  89,   1,    99,   111,
  153,  144,  209,  255,
  255,  0,    73,   82
};

DEFINE_GRADIENT_PALETTE( landscape_64_gp ) {
  0,    0,    0,    0,
  37,   2,    25,   1,
  76,   15,   115,  5,
  127,  79,   213,  1,
  128,  126,  211,  47,
  130,  188,  209,  247,
  153,  144,  182,  205,
  204,  59,   117,  250,
  255,  1,    37,   192
};

DEFINE_GRADIENT_PALETTE( landscape_33_gp ) {
  0,    1,    5,    0,
  19,   32,   23,   1,
  38,   161,  55,   1,
  63,   229,  144,  1,
  66,   39,   142,  74,
  255,  1,    4,    1
};

DEFINE_GRADIENT_PALETTE( rainbowsherbet_gp ) {
  0,    255,  33,   4,
  43,   255,  68,   25,
  86,   255,  7,    25,
  127,  255,  82,   103,
  170,  255,  255,  242,
  209,  42,   255,  22,
  255,  87,   255,  65
};

DEFINE_GRADIENT_PALETTE( gr65_hult_gp ) {
  0,    247,  176,  247,
  48,   255,  136,  255,
  89,   220,  29,   226,
  160,  7,    82,   178,
  216,  1,    124,  109,
  255,  1,    124,  109
};

/*DEFINE_GRADIENT_PALETTE( gr64_hult_gp ) {
  0,    1,    124,  109,
  66,   1,    93,   79,
  104,  52,   65,   1,
  130,  115,  127,  1,
  150,  52,   65,   1,
  201,  1,    86,   72,
  239,  0,    55,   45,
  255,  0,    55,   45
  };*/

DEFINE_GRADIENT_PALETTE( GMT_drywet_gp ) {
  0,    47,   30,   2,
  42,   213,  147,  24,
  84,   103,  219,  52,
  127,  3,    219,  207,
  170,  1,    48,   214,
  212,  1,    1,    111,
  255,  1,    7,    33
};

DEFINE_GRADIENT_PALETTE( emerald_dragon_gp ) {
  0,    97,   255,  1,
  101,  47,   133,  1,
  178,  13,   43,   1,
  255,  2,    10,   1
};

DEFINE_GRADIENT_PALETTE( Colorfull_gp ) {
  0,    10,   85,   5,
  25,   29,   109,  18,
  60,   59,   138,  42,
  93,   83,   99,   52,
  106,  110,  66,   64,
  109,  123,  49,   65,
  113,  139,  35,   66,
  116,  192,  117,  98,
  124,  255,  255,  137,
  168,  100,  180,  155,
  255,  22,   121,  174
};

DEFINE_GRADIENT_PALETTE( Pink_Purple_gp ) {
  0,    19,   2,    39,
  25,   26,   4,    45,
  51,   33,   6,    52,
  76,   68,   62,   125,
  102,  118,  187,  240,
  109,  163,  215,  247,
  114,  217,  244,  255,
  122,  159,  149,  221,
  149,  113,  78,   188,
  183,  128,  57,   155,
  255,  146,  40,   123
};

DEFINE_GRADIENT_PALETTE( autumn_19_gp ) {
  0,    26,   1,    1,
  51,   67,   4,    1,
  84,   118,  14,   1,
  104,  137,  152,  52,
  112,  113,  65,   1,
  122,  133,  149,  59,
  124,  137,  152,  52,
  135,  113,  65,   1,
  142,  139,  154,  46,
  163,  113,  13,   1,
  204,  55,   3,    1,
  249,  17,   1,    1,
  255,  17,   1,    1
};

DEFINE_GRADIENT_PALETTE( daybreak_gp ) {
  0,    0,    0,    0,
  42,   0,    0,    45,
  84,   0,    0,    255,
  127,  42,   0,    255,
  170,  255,  0,    255,
  212,  255,  55,   255,
  255,  255,  255,  255
};

DEFINE_GRADIENT_PALETTE( Blue_Cyan_Yellow_gp ) {
  0,    0,    0,    255,
  63,   0,    55,   255,
  127,  0,    255,  255,
  191,  42,   255,  45,
  255,  255,  255,  0
};

DEFINE_GRADIENT_PALETTE( bhw1_28_gp ) {
  0,    75,   1,    221,
  30,   252,  73,   255,
  48,   169,  0,    242,
  119,  0,    149,  242,
  170,  43,   0,    242,
  206,  252,  73,   255,
  232,  78,   12,   214,
  255,  0,    149,  242
};

DEFINE_GRADIENT_PALETTE( rbw_gp ) {
  0,    255,  0,    0,
  78,   255,  0,    0,
  83,   0,    0,    255,
  168,  0,    0,    255,
  173,  255,  255,  255,
  255,  255,  255,  255
};

float fmap(const float x, const float in_min, const float in_max, const float out_min, const float out_max) {
  return (out_max - out_min) * (x - in_min) / (in_max - in_min) + out_min;
}
void setCurrentPalette() {
  if (modes[currentMode].Scale > 100U) modes[currentMode].Scale = 100U; // чтобы не было проблем при прошивке без очистки памяти
  curPalette = palette_arr[(uint8_t)(modes[currentMode].Scale / 100.0F * ((sizeof(palette_arr) / sizeof(TProgmemRGBPalette16 *)) - 0.01F))];
}
void MoveFractionalNoiseX(int8_t amplitude = 1, float shift = 0) {
  for (uint8_t y = 0; y < pHEIGHT; y++) {
    int16_t amount = ((int16_t)noise_3d[0][0][y] - 128) * 2 * amplitude + shift * 256  ;
    int8_t delta = abs(amount) >> 8 ;
    int8_t fraction = abs(amount) & 255;
    for (uint8_t x = 0 ; x < pWIDTH; x++) {
      if (amount < 0) {
        zD = x - delta; zF = zD - 1;
      } else {
        zD = x + delta; zF = zD + 1;
      }
      CRGB PixelA = CRGB::Black  ;
      if ((zD >= 0) && (zD < pWIDTH)) PixelA = leds[XY(zD, y)];
      CRGB PixelB = CRGB::Black ;
      if ((zF >= 0) && (zF < pWIDTH)) PixelB = leds[XY(zF, y)];
      ledsbuff[XY(x, y)] = (PixelA.nscale8(ease8InOutApprox(255 - fraction))) + (PixelB.nscale8(ease8InOutApprox(fraction)));   // lerp8by8(PixelA, PixelB, fraction );
    }
  }
  memcpy(leds, ledsbuff, sizeof(CRGB)* NUM_LEDS);
}
void MoveFractionalNoiseY(int8_t amplitude = 1, float shift = 0) {
  for (uint8_t x = 0; x < pWIDTH; x++) {
    int16_t amount = ((int16_t)noise_3d[0][x][0] - 128) * 2 * amplitude + shift * 256 ;
    int8_t delta = abs(amount) >> 8 ;
    int8_t fraction = abs(amount) & 255;
    for (uint8_t y = 0 ; y < pHEIGHT; y++) {
      if (amount < 0) {
        zD = y - delta; zF = zD - 1;
      } else {
        zD = y + delta; zF = zD + 1;
      }
      CRGB PixelA = CRGB::Black ;
      if ((zD >= 0) && (zD < pHEIGHT)) PixelA = leds[XY(x, zD)];
      CRGB PixelB = CRGB::Black ;
      if ((zF >= 0) && (zF < pHEIGHT)) PixelB = leds[XY(x, zF)];
      ledsbuff[XY(x, y)] = (PixelA.nscale8(ease8InOutApprox(255 - fraction))) + (PixelB.nscale8(ease8InOutApprox(fraction)));
    }
  }
  memcpy(leds, ledsbuff, sizeof(CRGB)* NUM_LEDS);
}
void dimAll(uint8_t value, CRGB *LEDarray = leds) {
  // теперь короткий вариант
  nscale8(LEDarray, NUM_LEDS, value);
}
void particlesUpdate2(uint8_t i) {
  //age
  trackingObjectState[i]--; //ttl // ещё и сюда надо speedfactor вкорячить. удачи там!
  //apply velocity
  trackingObjectPosX[i] += trackingObjectSpeedX[i];
  trackingObjectPosY[i] += trackingObjectSpeedY[i];
  if (trackingObjectState[i] == 0 || trackingObjectPosX[i] <= -1 || trackingObjectPosX[i] >= pWIDTH || trackingObjectPosY[i] <= -1 || trackingObjectPosY[i] >= pHEIGHT)
    trackingObjectIsShift[i] = false;
}
void FillNoise(int8_t layer) {
  for (uint8_t i = 0; i < pWIDTH; i++) {
    int32_t ioffset = scale32_x[layer] * (i - CENTER_X_MINOR);
    for (uint8_t j = 0; j < pHEIGHT; j++) {
      int32_t joffset = scale32_y[layer] * (j - CENTER_Y_MINOR);
      int8_t data = inoise16(noise32_x[layer] + ioffset, noise32_y[layer] + joffset, noise32_z[layer]) >> 8;
      int8_t olddata = noise_3d[layer][i][j];
      int8_t newdata = scale8( olddata, noisesmooth ) + scale8( data, 255 - noisesmooth );
      data = newdata;
      noise_3d[layer][i][j] = data;
    }
  }
}

// ------------------------------ Дополнительные функции рисования ----------------------
void DrawLine(int x1, int y1, int x2, int y2, CRGB color)
{
  int deltaX = abs(x2 - x1);
  int deltaY = abs(y2 - y1);
  int signX = x1 < x2 ? 1 : -1;
  int signY = y1 < y2 ? 1 : -1;
  int error = deltaX - deltaY;
  drawPixelXY(x2, y2, color);
  while (x1 != x2 || y1 != y2) {
    drawPixelXY(x1, y1, color);
    int error2 = error * 2;
    if (error2 > -deltaY) {
      error -= deltaY;
      x1 += signX;
    }
    if (error2 < deltaX) {
      error += deltaX;
      y1 += signY;
    }
  }
}
void DrawLineF(float x1, float y1, float x2, float y2, CRGB color) {
  float deltaX = std::fabs(x2 - x1);
  float deltaY = std::fabs(y2 - y1);
  float error = deltaX - deltaY;
  float signX = x1 < x2 ? 0.5 : -0.5;
  float signY = y1 < y2 ? 0.5 : -0.5;
  while (x1 != x2 || y1 != y2) { // (true) - а я то думаю - "почему функция часто вызывает вылет по вачдогу?" А оно вон оно чё, Михалычь!
    if ((signX > 0 && x1 > x2 + signX) || (signX < 0 && x1 < x2 + signX)) break;
    if ((signY > 0 && y1 > y2 + signY) || (signY < 0 && y1 < y2 + signY)) break;
    drawPixelXYF(x1, y1, color); // интересно, почему тут было обычное drawPixelXY() ???
    float error2 = error;
    if (error2 > -deltaY) {
      error -= deltaY;
      x1 += signX;
    }
    if (error2 < deltaX) {
      error += deltaX;
      y1 += signY;
    }
  }
}

uint8_t getEffectSpeedValue(int8_t eff) {
#if (USE_E131 == 1)
  if (workMode == SLAVE && e131_streaming) {
    return syncEffectSpeed;
  }
#endif
  return effectSpeed[eff];
}
uint8_t getEffectContrastValue(int8_t eff) {
#if (USE_E131 == 1)
  if (workMode == SLAVE && e131_streaming) {
    return syncEffectContrast;
  }
#endif
  return effectContrast[eff];
}
uint8_t getEffectScaleParamValue(int8_t eff) {
#if (USE_E131 == 1)
  if (workMode == SLAVE && e131_streaming) {
    return syncEffectParam1;
  }
#endif
  return effectScaleParam[eff];
}
uint8_t getEffectScaleParamValue2(int8_t eff) {
#if (USE_E131 == 1)
  if (workMode == SLAVE && e131_streaming) {
    return syncEffectParam2;
  }
#endif
  return effectScaleParam2[eff];
}

// ---------------------------------------------
// *********** снегопад 2.0 ***********

void snowRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    // modeCode = MC_SNOW;
    FastLED.clear();  // очистить
  }
  // сдвигаем всё вниз
  shiftDown();
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  for (uint8_t x = 0; x < pWIDTH; x++) {
    // заполняем случайно верхнюю строку
    // а также не даём двум блокам по вертикали вместе быть
    if (getPixColorXY(x, pHEIGHT - 2) == 0 && (random8(0, map8(255 - getEffectScaleParamValue(MC_SNOW), 5, 15)) == 0)) {
      CRGB color = CRGB(effectBrightness, effectBrightness, effectBrightness); /*0xE0FFFF*/
      if (color.r > 0x20 && random8(0, 4) == 0) color = color - CRGB(0x10, 0x10, 0x10);
      drawPixelXY(x, pHEIGHT - 1, color);
    } else {
      drawPixelXY(x, pHEIGHT - 1, 0x000000);
    }
  }
}

// ------------- ПЕЙНТБОЛ -------------
uint8_t USE_SEGMENTS_PAINTBALL = 0;
uint8_t BorderWidth = 0;
uint8_t dir_mx, seg_num, seg_size, seg_offset, seg_offset_x, seg_offset_y;
uint16_t idx;
void lightBallsRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    FastLED.clear();  // очистить
    dir_mx = pWIDTH > pHEIGHT ? 0 : 1;                                   // 0 - квадратные сегменты расположены горизонтально, 1 - вертикально
    seg_num = dir_mx == 0 ? (pWIDTH / pHEIGHT) : (pHEIGHT / pWIDTH);     // вычисляем количество сегментов, умещающихся на матрице
    seg_size = dir_mx == 0 ? pHEIGHT : pWIDTH;                           // Размер квадратного сегмента (высота и ширина равны)
    seg_offset = ((dir_mx == 0 ? pWIDTH : pHEIGHT) - seg_size * seg_num) / (seg_num + 1); // смещение от края матрицы и между сегментами
    BorderWidth = 0;
    USE_SEGMENTS_PAINTBALL = getEffectScaleParamValue2(MC_PAINTBALL);
  }
  // Если совсем задержки нет - матрица мерцает от постоянного обновления
  delay(5);
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  // Apply some blurring to whatever's already on the matrix
  // Note that we never actually clear the matrix, we just constantly
  // blur it repeatedly.  Since the blurring is 'lossy', there's
  // an automatic trend toward black -- by design.
  uint8_t blurAmount = map(effectBrightness, 32, 255, 65, 91);
  uint8_t actualBrightness = map(effectBrightness, 32, 255, 125, 250);
  blur2d(leds, pWIDTH, pHEIGHT, blurAmount);
  // The color of each point shifts over time, each at a different speed.
  uint32_t ms = millis();
  uint8_t  cnt = map8(255 - getEffectScaleParamValue(MC_PAINTBALL), 1, 4); // 1..4 шариков
  float spd = (map8(255 - getEffectSpeedValue(MC_PAINTBALL), 50, 100) / 100.0) / (USE_SEGMENTS_PAINTBALL != 0 ? 1 : (float)seg_num);
  // Отрисовка режима происходит на максимальной скорости. Значение effectSpeed влияет на параметр BPM функции beatsin8
  // The easiest way to construct this is to multiply a floating point BPM value (e.g. 120.3) by 256, (e.g. resulting in 30796 in this case), and pass that as the 16-bit BPM argument.
  uint8_t m1 = ( 91.0 * spd) + 0.51;
  uint8_t m2 = (109.0 * spd) + 0.51;
  uint8_t m3 = ( 73.0 * spd) + 0.51;
  uint8_t m4 = (123.0 * spd) + 0.51;
  // Для неквадратных - вычленяем квадратные сегменты, которые равномерно распределяем по ширине / высоте матрицы
  if (USE_SEGMENTS_PAINTBALL != 0) {
    uint8_t  i = beatsin8(m1, 0, seg_size - BorderWidth - 1);
    uint8_t  j = beatsin8(m2, 0, seg_size - BorderWidth - 1);
    uint8_t  k = beatsin8(m3, 0, seg_size - BorderWidth - 1);
    uint8_t  m = beatsin8(m4, 0, seg_size - BorderWidth - 1);
    uint8_t d1 = ms / 29;
    uint8_t d2 = ms / 41;
    uint8_t d3 = ms / 73;
    uint8_t d4 = ms / 97;
    for (uint8_t ii = 0; ii < seg_num; ii++) {
      delay(0); // Для предотвращения ESP8266 Watchdog Timer
      uint8_t cx = dir_mx == 0 ? (seg_offset * (ii + 1) + seg_size * ii) : 0;
      uint8_t cy = dir_mx == 0 ? 0 : (seg_offset * (ii + 1) + seg_size * ii);
      uint8_t color_shift = ii * 50;
      if (cnt <= 1) {
        idx = XY(i + cx, j + cy);
        leds[idx] += CHSV( color_shift + d1, 200, actualBrightness);
      }
      if (cnt <= 2) {
        idx = XY(j + cx, k + cy);
        leds[idx] += CHSV( color_shift + d2, 200, actualBrightness);
      }
      if (cnt <= 3) {
        idx = XY(k + cx, m + cy);
        leds[idx] += CHSV( color_shift + d3, 200, actualBrightness);
      }
      if (cnt <= 4) {
        idx = XY(m + cx, i + cy);
        leds[idx] += CHSV( color_shift + d4, 200, actualBrightness);
      }
      // При соединении матрицы из угла вверх или вниз почему-то слева и справа узора остаются полосы, которые
      // не гаснут обычным blur - гасим полоски левой и правой стороны дополнительно.
      // При соединении из угла влево или вправо или на неквадратных матрицах такого эффекта не наблюдается
      uint8_t fade_step = map8(effectBrightness, 1, 15);
      for (uint8_t i2 = cy; i2 < cy + seg_size; i2++) {
        fadePixel(cx + BorderWidth, i2, fade_step);
        fadePixel(cx + seg_size - BorderWidth - 1, i2, fade_step);
      }
    }
  }
  else {
    uint8_t  i = beatsin8(m1, BorderWidth, pWIDTH - BorderWidth - 1);
    uint8_t  j = beatsin8(m1, BorderWidth, pHEIGHT - BorderWidth - 1);
    uint8_t  k = beatsin8(m3, BorderWidth, pWIDTH - BorderWidth - 1);
    uint8_t  m = beatsin8(m4, BorderWidth, pHEIGHT - BorderWidth - 1);
    if (cnt <= 1) {
      idx = XY(i, j);
      leds[idx] += CHSV( ms / 29, 200, actualBrightness);
    }
    if (cnt <= 2) {
      idx = XY(k, j);
      leds[idx] += CHSV( ms / 41, 200, actualBrightness);
    }
    if (cnt <= 3) {
      idx = XY(k, m);
      leds[idx] += CHSV( ms / 73, 200, actualBrightness);
    }
    if (cnt <= 4) {
      idx = XY(i, m);
      leds[idx] += CHSV( ms / 97, 200, actualBrightness);
    }
    if (pWIDTH == pHEIGHT) {
      // При соединении матрицы из угла вверх или вниз почему-то слева и справа узора остаются полосы, которые
      // не гаснут обычным blur - гасим полоски левой и правой стороны дополнительно.
      // При соединении из угла влево или вправо или на неквадратных матрицах такого эффекта не наблюдается
      uint8_t fade_step = map8(effectBrightness, 1, 15);
      for (uint8_t i = 0; i < pHEIGHT; i++) {
        fadePixel(0, i, fade_step);
        fadePixel(pWIDTH - 1, i, fade_step);
      }
    }
  }
}

// ------------- ВОДОВОРОТ -------------
uint8_t USE_SEGMENTS_SWIRL = 0;
void swirlRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    FastLED.clear();  // очистить
    dir_mx = pWIDTH > pHEIGHT ? 0 : 1;                                   // 0 - квадратные сегменты расположены горизонтально, 1 - вертикально
    seg_num = dir_mx == 0 ? (pWIDTH / pHEIGHT) : (pHEIGHT / pWIDTH);     // вычисляем количество сегментов, умещающихся на матрице
    seg_size = dir_mx == 0 ? pHEIGHT : pWIDTH;                           // Размер квадратного сегмента (высота и ширина равны)
    seg_offset = ((dir_mx == 0 ? pWIDTH : pHEIGHT) - seg_size * seg_num) / (seg_num + 1); // смещение от края матрицы и между сегментами
    BorderWidth = seg_num == 1 ? 0 : 1;
    USE_SEGMENTS_SWIRL = getEffectScaleParamValue2(MC_SWIRL);
  }
  // Если совсем задержки нет - матрица мерцает от постоянного обновления
  delay(5);
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  // Apply some blurring to whatever's already on the matrix
  // Note that we never actually clear the matrix, we just constantly
  // blur it repeatedly.  Since the blurring is 'lossy', there's
  // an automatic trend toward black -- by design.

  uint8_t blurAmount = map(effectBrightness, 32, 255, 65, 91);
  uint8_t actualBrightness = map(effectBrightness, 32, 255, 125, 250);
  blur2d(leds, pWIDTH, pHEIGHT, blurAmount);
  uint32_t ms = millis();
  float spd = (map8(255 - getEffectSpeedValue(MC_SWIRL), 50, 100) / 100.0) / (USE_SEGMENTS_SWIRL != 0 ? 1 : (float)seg_num);
  // Отрисовка режима происходит на максимальной скорости. Знеачение effectSpeed влияет на параметр BPM функции beatsin8
  // The easiest way to construct this is to multiply a floating point BPM value (e.g. 120.3) by 256, (e.g. resulting in 30796 in this case), and pass that as the 16-bit BPM argument.
  uint8_t m1 = (41.0 * spd) + 0.51;
  uint8_t m2 = (27.0 * spd) + 0.51;
  if (USE_SEGMENTS_SWIRL != 0) {
    // Use two out-of-sync sine waves
    uint8_t  i = beatsin8(m1, 0, seg_size - BorderWidth - 1);
    uint8_t  j = beatsin8(m2, 0, seg_size - BorderWidth - 1);
    // Also calculate some reflections
    uint8_t ni = (seg_size - 1) - i;
    uint8_t nj = (seg_size - 1) - j;
    uint8_t d1 = ms / 11;
    uint8_t d2 = ms / 13;
    uint8_t d3 = ms / 17;
    uint8_t d4 = ms / 29;
    uint8_t d5 = ms / 37;
    uint8_t d6 = ms / 41;
    for (uint8_t ii = 0; ii < seg_num; ii++) {
      delay(0); // Для предотвращения ESP8266 Watchdog Timer
      uint8_t cx = dir_mx == 0 ? (seg_offset * (ii + 1) + seg_size * ii) : 0;
      uint8_t cy = dir_mx == 0 ? 0 : (seg_offset * (ii + 1) + seg_size * ii);
      uint8_t color_shift = ii * 50;
      // The color of each point shifts over time, each at a different speed.
      idx = XY( i + cx, j + cy); leds[idx] += CHSV( color_shift + d1, 200, actualBrightness);
      idx = XY(ni + cx, nj + cy); leds[idx] += CHSV( color_shift + d2, 200, actualBrightness);
      idx = XY( i + cx, nj + cy); leds[idx] += CHSV( color_shift + d3, 200, actualBrightness);
      idx = XY(ni + cx, j + cy); leds[idx] += CHSV( color_shift + d4, 200, actualBrightness);
      idx = XY( j + cx, i + cy); leds[idx] += CHSV( color_shift + d5, 200, actualBrightness);
      idx = XY(nj + cx, ni + cy); leds[idx] += CHSV( color_shift + d6, 200, actualBrightness);
      // При соединении матрицы из угла вверх или вниз почему-то слева и справа узора остаются полосы, которые
      // не гаснут обычным blur - гасим полоски левой и правой стороны дополнительно.
      // При соединении из угла влево или вправо или на неквадратных матрицах такого эффекта не наблюдается
      uint8_t fade_step = map8(effectBrightness, 1, 15);
      for (uint8_t i2 = cy; i2 < cy + seg_size; i2++) {
        fadePixel(cx, i2, fade_step);
        fadePixel(cx + BorderWidth, i2, fade_step);
        fadePixel(cx + seg_size - 1, i2, fade_step);
        fadePixel(cx + seg_size - BorderWidth - 1, i2, fade_step);
      }
    }
  }
  else
  {
    // Use two out-of-sync sine waves
    uint8_t  i = beatsin8(m1, BorderWidth, pWIDTH - BorderWidth - 1);
    uint8_t  j = beatsin8(m2, BorderWidth, pHEIGHT - BorderWidth - 1);
    // Also calculate some reflections
    uint8_t ni = (pWIDTH - 1) - i;
    uint8_t nj = (pHEIGHT - 1) - j;
    // The color of each point shifts over time, each at a different speed.
    idx = XY( i, j); leds[idx] += CHSV( ms / 11, 200, actualBrightness);
    idx = XY(ni, nj); leds[idx] += CHSV( ms / 13, 200, actualBrightness);
    idx = XY( i, nj); leds[idx] += CHSV( ms / 17, 200, actualBrightness);
    idx = XY(ni, j); leds[idx] += CHSV( ms / 29, 200, actualBrightness);
    if (pHEIGHT == pWIDTH) {
      // для квадратных матриц - 6 точек создают более красивую картину
      idx = XY( j, i); leds[idx] += CHSV( ms / 37, 200, actualBrightness);
      idx = XY(nj, ni); leds[idx] += CHSV( ms / 41, 200, actualBrightness);
      // При соединении матрицы из угла вверх или вниз почему-то слева и справа узора остаются полосы, которые
      // не гаснут обычным blur - гасим полоски левой и правой стороны дополнительно.
      // При соединении из угла влево или вправо или на неквадратных матрицах такого эффекта не наблюдается
      uint8_t fade_step = map8(effectBrightness, 1, 15);
      for (uint8_t i = 0; i < pHEIGHT; i++) {
        fadePixel(0, i, fade_step);
        fadePixel(pWIDTH - 1, i, fade_step);
      }
    }
  }
}

// ***************************** БЛУДНЫЙ КУБИК *****************************
#define RANDOM_COLOR 1    // случайный цвет при отскоке
int16_t coordB[2];
int8_t  vectorB[2];
int8_t  ballSize;
CRGB    ballColor;
void ballRoutine() {
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  if (loadingFlag) {
    for (uint8_t i = 0; i < 2; i++) {
      coordB[i] = pWIDTH / 2 * 10;
      vectorB[i] = random8(8, 20);
      ballColor = CHSV(random8(0, 9) * 28, 255, effectBrightness);
    }
    loadingFlag = false;
  }
  ballSize = map8(getEffectScaleParamValue(MC_BALL), 2, 5); //размер кубика можно выбрать от 2*2 до 5*5
  for (uint8_t i = 0; i < 2; i++) {
    coordB[i] += vectorB[i];
    if (coordB[i] < 0) {
      coordB[i] = 0;
      vectorB[i] = -vectorB[i];
      if (RANDOM_COLOR) ballColor = CHSV(random8(0, 9) * 28, 255, effectBrightness);
    }
  }
  if (coordB[0] > (pWIDTH - ballSize) * 10) {
    coordB[0] = (pWIDTH - ballSize) * 10;
    vectorB[0] = -vectorB[0];
    if (RANDOM_COLOR) ballColor = CHSV(random8(0, 9) * 28, 255, effectBrightness);
  }
  if (coordB[1] > (pHEIGHT - ballSize) * 10) {
    coordB[1] = (pHEIGHT - ballSize) * 10;
    vectorB[1] = -vectorB[1];
    if (RANDOM_COLOR) ballColor = CHSV(random8(0, 9) * 28, 255, effectBrightness);
  }
  FastLED.clear();
  for (uint8_t i = 0; i < ballSize; i++)
    for (uint8_t j = 0; j < ballSize; j++)
      leds[getPixelNumber(coordB[0] / 10 + i, coordB[1] / 10 + j)] = ballColor;
}

// ***************************** РАДУГА *****************************
uint8_t rainbow_type = 0;
void rainbowRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    rainbow_type = (specialTextEffectParam >= 0) ? specialTextEffectParam : getEffectScaleParamValue2(MC_RAINBOW);
    // Если авто - генерировать один из типов - 1-Вертикальная радуга, 2-Горизонтальная радуга, 3-Диагональная радуга, 4-Вращающаяся радуга
    if (rainbow_type == 0 || rainbow_type > 4) {
      rainbow_type = random8(1, 5);
    }
    FastLED.clear();  // очистить
  }
  switch (rainbow_type) {
    case 1:  rainbowVertical(); break;
    case 2:  rainbowHorizontal(); break;
    case 3:  rainbowDiagonal(); break;
    default: rainbowRotate(); break;
  }
}

// *********** радуга дигональная ***********
void rainbowDiagonal() {
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  hue += 2;
  for (uint8_t x = 0; x < pWIDTH; x++) {
    for (uint8_t y = 0; y < pHEIGHT; y++) {
      float dx = (pWIDTH >= pHEIGHT)
                 ? (float)(pWIDTH / pHEIGHT * x + y)
                 : (float)(pHEIGHT / pWIDTH * y + x);
      CRGB thisColor = CHSV((uint8_t)(hue + dx * (float)(255 / maxDim)), 255, effectBrightness);
      drawPixelXY(x, y, thisColor);
    }
  }
}

// *********** радуга горизонтальная ***********
void rainbowHorizontal() {
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  hue += 2;
  for (uint8_t j = 0; j < pHEIGHT; j++) {
    CHSV thisColor = CHSV((uint8_t)(hue + j * map8(getEffectScaleParamValue(MC_RAINBOW), 1, pWIDTH)), 255, effectBrightness);
    for (uint8_t i = 0; i < pWIDTH; i++)
      drawPixelXY(i, j, thisColor);
  }
}

// *********** радуга вертикальная ***********
void rainbowVertical() {
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  hue += 2;
  for (uint8_t i = 0; i < pWIDTH; i++) {
    CHSV thisColor = CHSV((uint8_t)(hue + i * map8(getEffectScaleParamValue(MC_RAINBOW), 1, pHEIGHT)), 255, effectBrightness);
    for (uint8_t j = 0; j < pHEIGHT; j++)
      drawPixelXY(i, j, thisColor);
  }
}

// *********** радуга вращающаяся ***********
void rainbowRotate() {
  uint32_t ms = millis();
  int32_t  yHueDelta32 = ((int32_t)cos16( ms * (27 / 1) ) * (350 / pWIDTH));
  int32_t  xHueDelta32 = ((int32_t)cos16( ms * (39 / 1) ) * (310 / pHEIGHT));
  uint8_t  lineStartHue = ms / 65536;
  int8_t   yHueDelta8   = yHueDelta32 / 32768;
  int8_t   xHueDelta8   = xHueDelta32 / 32768;
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  for (uint8_t y = 0; y < pHEIGHT; y++) {
    lineStartHue += yHueDelta8;
    uint8_t pixelHue = lineStartHue;
    for (uint8_t x = 0; x < pWIDTH; x++) {
      pixelHue += xHueDelta8;
      leds[ XY(x, y)]  = CHSV( pixelHue, 255, effectBrightness);
    }
  }
}

// ---------------------------------------- ЦВЕТА ------------------------------------------
void colorsRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    FastLED.clear();  // очистить
  }
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  hue += map8(getEffectScaleParamValue(MC_COLORS), 1, 10);
  CHSV hueColor = CHSV(hue, 255, effectBrightness);
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = hueColor;
  }
}

// ---------------------------------------- ЦИКЛОН ------------------------------------------
int16_t cycle_x, cycle_y; // могут уходить в минус при смене направления
uint8_t move_dir, fade_divider, inc_cnt, USE_SEGMENTS_CYCLON;
void cyclonRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    USE_SEGMENTS_CYCLON = getEffectScaleParamValue2(MC_CYCLON);
    dir_mx = pWIDTH > pHEIGHT ? 0 : 1;                                                                      // 0 - сегменты расположены горизонтально, 1 - вертикально
    seg_num = dir_mx == 0 ? (pWIDTH / pHEIGHT) : (pHEIGHT / pWIDTH);                                        // вычисляем количество сегментов, умещающихся на матрице, в режиме без сигментов ширина одной полоски будет равна кол-ву сегментов
    seg_size = dir_mx == 0 ? pHEIGHT : pWIDTH;                                                              // Размер квадратного сегмента (высота и ширина равны)
    seg_offset_y = USE_SEGMENTS_CYCLON == 1 ? (dir_mx == 1 ? pHEIGHT - seg_size * seg_num : 0) / 2 : 0;     // смещение от низа/верха матрицы
    seg_offset_x = USE_SEGMENTS_CYCLON == 1 ? (dir_mx == 0 ? pWIDTH - seg_size * seg_num : 0) / 2 : 0;      // смещение от левого/правого края матрицы
    hue = 0;
    cycle_x = USE_SEGMENTS_CYCLON == 1 ? (seg_offset_x + seg_size - 1) : pWIDTH - 1; // начало - от правого края к левому
    cycle_y = USE_SEGMENTS_CYCLON == 1 ?  seg_offset_y : 0;
    move_dir = 1;
    fade_divider = 0;
    inc_cnt = NUM_LEDS / 312;
    if (inc_cnt == 0) inc_cnt = 1;
    FastLED.clear();  // очистить
  }
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  uint8_t actualBrightness = map(effectBrightness, 32, 255, 125, 250);
  // Использовать отрисовку по сегментам
  // Если сегменты не используется - ширина одной полоски - кол-во сегментов
  for (uint8_t i = 0; i < seg_num; i++) {
    for (uint8_t k = 0; k < inc_cnt; k++) {
      if (USE_SEGMENTS_CYCLON == 1) {
        if (cycle_y + k - seg_offset_y >= seg_size) continue;
        idx = dir_mx == 0
              ? getPixelNumber(cycle_x + i * seg_size, cycle_y + k)
              : getPixelNumber(cycle_x, cycle_y + i * seg_size + k);
      } else {
        if (cycle_y + k  >= pHEIGHT) continue;
        idx = getPixelNumber(cycle_x + i, cycle_y + k);
      }
      if (idx < NUM_LEDS)
        leds[idx] = CHSV(hue + k + (USE_SEGMENTS_CYCLON == 1 ? i * 85 : 0), 255, actualBrightness);
    }
  }
  hue += inc_cnt;
  // Затухание - не на каждый цикл, а регулируется параметром эффекта
  uint8_t fader_param = map8(255 - getEffectScaleParamValue(MC_CYCLON), 0, 5);
  fade_divider++;
  if (fade_divider > fader_param) {
    fade_divider = 0;
    fader(5);
  }
  cycle_y += inc_cnt;
  if (USE_SEGMENTS_CYCLON) {
    if (cycle_y - seg_offset_y >= seg_size) {
      cycle_y = seg_offset_y;
      if (move_dir == 0) {
        // Слева направо
        cycle_x++;
        if (cycle_x - seg_offset_x >= seg_size) {
          move_dir = 1;
          cycle_x = seg_size - 1 + seg_offset_x;
        }
      } else {
        // Справа налево
        cycle_x--;
        if (cycle_x < seg_offset_x) {
          move_dir = 0;
          cycle_x = seg_offset_x;
        }
      }
    }
  } else {
    if (cycle_y >= pHEIGHT) {
      cycle_y = 0;
      if (move_dir == 0) {
        // Слева направо
        cycle_x += seg_num;
        if (cycle_x >= pWIDTH) {
          move_dir = 1;
          cycle_x = pWIDTH - 1;
        }
      } else {
        // Справа налева
        cycle_x -= seg_num;
        if (cycle_x < 0) {
          move_dir = 0;
          cycle_x = 0;
        }
      }
    }
  }
}

// ********************** огонь **********************
#define SPARKLES 1        // вылетающие угольки вкл выкл
uint8_t matrixValue[8][16];
uint8_t *line;
uint8_t pcnt = 0;
//these values are substracetd from the generated values to give a shape to the animation
const uint8_t valueMask[8][16] PROGMEM = {
  {32 , 0  , 0  , 0  , 0  , 0  , 0  , 32 , 32 , 0  , 0  , 0  , 0  , 0  , 0  , 32 },
  {64 , 0  , 0  , 0  , 0  , 0  , 0  , 64 , 64 , 0  , 0  , 0  , 0  , 0  , 0  , 64 },
  {96 , 32 , 0  , 0  , 0  , 0  , 32 , 96 , 96 , 32 , 0  , 0  , 0  , 0  , 32 , 96 },
  {128, 64 , 32 , 0  , 0  , 32 , 64 , 128, 128, 64 , 32 , 0  , 0  , 32 , 64 , 128},
  {160, 96 , 64 , 32 , 32 , 64 , 96 , 160, 160, 96 , 64 , 32 , 32 , 64 , 96 , 160},
  {192, 128, 96 , 64 , 64 , 96 , 128, 192, 192, 128, 96 , 64 , 64 , 96 , 128, 192},
  {255, 160, 128, 96 , 96 , 128, 160, 255, 255, 160, 128, 96 , 96 , 128, 160, 255},
  {255, 192, 160, 128, 128, 160, 192, 255, 255, 192, 160, 128, 128, 160, 192, 255}
};

//these are the hues for the fire,
//should be between 0 (red) to about 25 (yellow)
const uint8_t hueMask[8][16] PROGMEM = {
  {1 , 11, 19, 25, 25, 22, 11, 1 , 1 , 11, 19, 25, 25, 22, 11, 1 },
  {1 , 8 , 13, 19, 25, 19, 8 , 1 , 1 , 8 , 13, 19, 25, 19, 8 , 1 },
  {1 , 8 , 13, 16, 19, 16, 8 , 1 , 1 , 8 , 13, 16, 19, 16, 8 , 1 },
  {1 , 5 , 11, 13, 13, 13, 5 , 1 , 1 , 5 , 11, 13, 13, 13, 5 , 1 },
  {1 , 5 , 11, 11, 11, 11, 5 , 1 , 1 , 5 , 11, 11, 11, 11, 5 , 1 },
  {0 , 1 , 5 , 8 , 8 , 5 , 1 , 0 , 0 , 1 , 5 , 8 , 8 , 5 , 1 , 0 },
  {0 , 0 , 1 , 5 , 5 , 1 , 0 , 0 , 0 , 0 , 1 , 5 , 5 , 1 , 0 , 0 },
  {0 , 0 , 0 , 1 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 1 , 1 , 0 , 0 , 0 }
};
void fireRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    // modeCode = MC_FIRE;
    FastLED.clear();
    if (line == NULL) line = new uint8_t[pWIDTH];
    generateLine();
    memset(matrixValue, 0, sizeof(matrixValue));
  }
  if (pcnt >= 100) {
    shiftFireUp();
    generateLine();
    pcnt = 0;
  }
  drawFrame(pcnt);
  pcnt += 30;
}
void fireRoutineRelease() {
  if (line != NULL) {
    delete [] line;
    line = NULL;
  }
}
// Randomly generate the next line (matrix row)
void generateLine() {
  for (uint8_t x = 0; x < pWIDTH; x++) {
    line[x] = random8(64, 255);
  }
}

//shift all values in the matrix up one row
void shiftFireUp() {
  for (uint8_t y = pHEIGHT - 1; y > 0; y--) {
    for (uint8_t x = 0; x < pWIDTH; x++) {
      uint8_t newX = x;
      if (x > 15) newX = x % 16;
      if (y > 7) continue;
      matrixValue[y][newX] = matrixValue[y - 1][newX];
    }
  }
  for (uint8_t x = 0; x < pWIDTH; x++) {
    uint8_t newX = x;
    if (x > 15) newX = x % 16;
    matrixValue[0][newX] = line[newX];
  }
}
// draw a frame, interpolating between 2 "key frames"
// @param pcnt percentage of interpolation
void drawFrame(uint8_t pcnt) {
  int nextv;
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  //each row interpolates with the one before it
  for (uint8_t y = pHEIGHT - 1; y > 0; y--) {
    for (uint8_t x = 0; x < pWIDTH; x++) {
      uint8_t newX = x;
      if (x > 15) newX = x % 16;
      if (y < 8) {
        nextv =
          (((100.0 - pcnt) * matrixValue[y][newX]
            + pcnt * matrixValue[y - 1][newX]) / 100.0)
          - pgm_read_byte(&(valueMask[y][newX]));
        CRGB color = CHSV(
                       map8(getEffectScaleParamValue(MC_FIRE), 0, 230) + pgm_read_byte(&(hueMask[y][newX])), // H
                       255, // S
                       (uint8_t)max(0, nextv) // V
                     );
        CRGB color2 = color.nscale8_video(effectBrightness);
        leds[getPixelNumber(x, y)] = color2;
      } else if (y == 8 && SPARKLES) {
        if (random8(0, 20) == 0 && getPixColorXY(x, y - 1) != 0)
          drawPixelXY(x, y, getPixColorXY(x, y - 1));
        else
          drawPixelXY(x, y, 0);
      } else if (SPARKLES) {
        // старая версия для яркости
        if (getPixColorXY(x, y - 1) > 0)
          drawPixelXY(x, y, getPixColorXY(x, y - 1));
        else
          drawPixelXY(x, y, 0);
      }
    }
  }
  //first row interpolates with the "next" line
  for (uint8_t x = 0; x < pWIDTH; x++) {
    uint8_t newX = x;
    if (x > 15) newX = x % 16;
    CRGB color = CHSV(
                   map8(getEffectScaleParamValue(MC_FIRE), 0, 230) + pgm_read_byte(&(hueMask[0][newX])), // H
                   255,           // S
                   (uint8_t)(((100.0 - pcnt) * matrixValue[0][newX] + pcnt * line[newX]) / 100.0) // V
                 );
    CRGB color2 = color.nscale8_video(effectBrightness);
    leds[getPixelNumber(x, 0)] = color2;      // на матрицах шире 16 столбцов нижний правый угол неработает
  }
}

// **************** МАТРИЦА *****************
void matrixRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    FastLED.clear();
  }
  uint8_t  effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  uint32_t cut_out = pHEIGHT < 10 ? 0x40 : 0x20; // на 0x004000 хвосты мматрицы короткие (4 точки), на 0x002000 - длиннее (8 точек)
  for (uint8_t x = 0; x < pWIDTH; x++) {
    // заполняем случайно верхнюю строку
    CRGB thisColor = getPixColorXY(x, pHEIGHT - 1);
    if (thisColor.g == 0) {
      leds[getPixelNumber(x, pHEIGHT - 1)] = random8(0, map8(255 - getEffectScaleParamValue(MC_MATRIX), 5, 15)) == 0 ? CRGB(0, effectBrightness, 0) : CRGB(0, 0, 0);
    } else if (thisColor.g < cut_out)
      drawPixelXY(x, pHEIGHT - 1, 0);
    else
      drawPixelXY(x, pHEIGHT - 1, thisColor - CRGB(cut_out, cut_out, cut_out));
  }
  // сдвигаем всё вниз
  shiftDown();
}

// ********************************* ШАРИКИ *********************************
#define BALLS_AMOUNT_MAX 6 // максимальное количество "шариков"
#define CLEAR_PATH 1       // очищать путь
#define BALL_TRACK 1       // (0 / 1) - вкл/выкл следы шариков
#define TRACK_STEP 70      // длина хвоста шарика (чем больше цифра, тем хвост короче)
int8_t  BALLS_AMOUNT;
int16_t coord[BALLS_AMOUNT_MAX][2];
int8_t  vector[BALLS_AMOUNT_MAX][2];
uint8_t ballColors[BALLS_AMOUNT_MAX];
void ballsRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    FastLED.clear();
    // Текущее количество шариков из настроек
    BALLS_AMOUNT = map8(getEffectScaleParamValue(MC_BALLS), 3, 6);
    for (uint8_t j = 0; j < BALLS_AMOUNT; j++) {
      int8_t sign;
      // забиваем случайными данными
      coord[j][0] = pWIDTH / 2 * 10;
      random8(0, 2) ? sign = 1 : sign = -1;
      vector[j][0] = random8(4, 15) * sign;
      coord[j][1] = pHEIGHT / 2 * 10;
      random8(0, 2) ? sign = 1 : sign = -1;
      vector[j][1] = random8(4, 15) * sign;
      ballColors[j] = random8(0, 255);
    }
  }
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  if (!BALL_TRACK)    // если режим БЕЗ следов шариков
    FastLED.clear();  // очистить
  else {              // режим со следами
    fader(map8(effectBrightness, 4, TRACK_STEP));
  }
  // движение шариков
  for (uint8_t j = 0; j < BALLS_AMOUNT; j++) {
    // движение шариков
    for (uint8_t i = 0; i < 2; i++) {
      coord[j][i] += vector[j][i];
      if (coord[j][i] < 0) {
        coord[j][i] = 0;
        vector[j][i] = -vector[j][i];
      }
    }
    if (coord[j][0] > (pWIDTH - 1) * 10) {
      coord[j][0] = (pWIDTH - 1) * 10;
      vector[j][0] = -vector[j][0];
    }
    if (coord[j][1] > (pHEIGHT - 1) * 10) {
      coord[j][1] = (pHEIGHT - 1) * 10;
      vector[j][1] = -vector[j][1];
    }
    leds[getPixelNumber(coord[j][0] / 10, coord[j][1] / 10)] =  CHSV(ballColors[j], 255, effectBrightness);
  }
}

// ********************* ЗВЕЗДОПАД ******************
#define TAIL_STEP  80     // длина хвоста кометы (чем больше цифра, тем хвост короче)
#define SATURATION 150    // насыщенность кометы (от 0 до 255)
int8_t STAR_DENSE;     // плотность комет 30..90
void starfallRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    FastLED.clear();  // очистить
  }
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  STAR_DENSE = map8(getEffectScaleParamValue(MC_SPARKLES), 30, 90);
  // заполняем головами комет левую и верхнюю линию
  for (uint8_t i = 4; i < pHEIGHT; i++) {
    if (getPixColorXY(0, i) == 0
        && (random8(0, STAR_DENSE) == 0)
        && getPixColorXY(0, i + 1) == 0
        && getPixColorXY(0, i - 1) == 0)
      leds[getPixelNumber(0, i)] = CHSV(random8(0, 200), SATURATION, effectBrightness);
  }
  for (uint8_t i = 0; i < pWIDTH - 4; i++) {
    if (getPixColorXY(i, pHEIGHT - 1) == 0
        && (random8(0, map8(getEffectScaleParamValue(MC_STARFALL), 10, 120)) == 0)
        && getPixColorXY(i + 1, pHEIGHT - 1) == 0
        && getPixColorXY(i - 1, pHEIGHT - 1) == 0)
      leds[getPixelNumber(i, pHEIGHT - 1)] = CHSV(random8(0, 200), SATURATION, effectBrightness);
  }
  // сдвигаем по диагонали
  shiftDiag();
  // уменьшаем яркость левой и верхней линии, формируем "хвосты"
  for (uint8_t i = 4; i < pHEIGHT; i++) {
    fadePixel(0, i, TAIL_STEP);
  }
  for (uint8_t i = 0; i < pWIDTH - 4; i++) {
    fadePixel(i, pHEIGHT - 1, TAIL_STEP);
  }
}

// *********************  КОНФЕТТИ ******************
#define BRIGHT_STEP 70    // шаг уменьшения яркости
void sparklesRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    FastLED.clear();  // очистить
  }
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  for (uint8_t i = 0; i < map8(getEffectScaleParamValue(MC_SPARKLES), 1, 25); i++) {
    uint8_t x = random8(0, pWIDTH);
    uint8_t y = random8(0, pHEIGHT);
    if (getPixColorXY(x, y) == 0)
      leds[getPixelNumber(x, y)] = CHSV(random8(0, 255), 255, effectBrightness);
  }
  fader(map8(effectBrightness, 4, BRIGHT_STEP));
}

uint8_t loopCounter;
uint8_t loopCounter2;

// ********************* СВЕТЛЯКИ *********************

#define LIGHTERS_AM 100

int8_t  **lightersPos;   // Позиции светляков
int8_t  **lightersSpeed; // Скорость движения светляков
uint8_t *lightersColor;  // Цвета светляков

void lightersRoutine() {

  if (loadingFlag) {
    loadingFlag = false;
    // modeCode = MC_LIGHTERS;

    if (lightersPos == NULL) {
      lightersPos = new int8_t*[2];
      for (uint8_t i = 0; i < 2; i++) {
        lightersPos[i] = new int8_t [LIGHTERS_AM];
      }
    }
    if (lightersSpeed == NULL) {
      lightersSpeed = new int8_t*[2];
      for (uint8_t i = 0; i < 2; i++) {
        lightersSpeed[i] = new int8_t [LIGHTERS_AM];
      }
    }
    if (lightersColor == NULL) {
      lightersColor = new uint8_t [LIGHTERS_AM];
    }

    FOR_i (0, LIGHTERS_AM) {
      lightersPos[0][i] = random16(0, pWIDTH);
      lightersPos[1][i] = random16(0, pHEIGHT);
      lightersSpeed[0][i] = random8(0, 4) - 2;
      lightersSpeed[1][i] = random8(0, 4) - 2;
      lightersColor[i] = random8(0, 255);
    }
  }

  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  FastLED.clear();

  if (++loopCounter > 20) loopCounter = 0;

  FOR_i (0, map8(getEffectScaleParamValue(MC_LIGHTERS), 5, 100)) {
    if (loopCounter == 0) {     // меняем скорость каждые 20 отрисовок
      while (lightersSpeed[0][i] == 0 && lightersSpeed[1][i] == 0) {
        lightersSpeed[0][i] += random8(0, 4) - 2;
        lightersSpeed[1][i] += random8(0, 4) - 2;
        lightersSpeed[0][i] = constrain(lightersSpeed[0][i], -5, 5);
        lightersSpeed[1][i] = constrain(lightersSpeed[1][i], -5, 5);
      }
    }

    lightersPos[0][i] += lightersSpeed[0][i];
    lightersPos[1][i] += lightersSpeed[1][i];

    if (lightersPos[0][i] < 0) lightersPos[0][i] = pWIDTH - 1;
    if (lightersPos[0][i] >= pWIDTH) lightersPos[0][i] = 0;

    if (lightersPos[1][i] < 0) {
      lightersPos[1][i] = 0;
      lightersSpeed[1][i] = -lightersSpeed[1][i];
    }
    if (lightersPos[1][i] >= pHEIGHT - 1) {
      lightersPos[1][i] = pHEIGHT - 1;
      lightersSpeed[1][i] = -lightersSpeed[1][i];
    }
    drawPixelXY(lightersPos[0][i], lightersPos[1][i], CHSV(lightersColor[i], 255, effectBrightness));
  }
}

void lighters2RoutineRelease() {
  if (lightersPos   != NULL) {
    for ( uint8_t i = 0; i < 2; i++ ) {
      delete [] lightersPos[i];
    }   delete [] lightersPos;
    lightersPos   = NULL;
  }
  if (lightersSpeed != NULL) {
    for ( uint8_t i = 0; i < 2; i++ ) {
      delete [] lightersSpeed[i];
    } delete [] lightersSpeed;
    lightersSpeed = NULL;
  }

  if (lightersColor == NULL) {
    delete [] lightersColor;
    lightersColor = NULL;
  }
}

// ********************* БУДИЛЬНИК-РАССВЕТ *********************
int8_t   row, col;                 // Для эффекта спирали  - точка "головы" змейки, бегающей по спирали (первая змейка для круговой спирали)
int8_t   row2, col2;               // Для эффекта спирали  - точка "головы" змейки, бегающей по спирали (вторая змейка для плоской спирали)
int8_t   dir, dir2;                // Для эффекта спирали на плоскости - направление движениия змейки: 0 - вниз; 1 - влево; 2 - вверх; 3 - вправо;
int8_t   range[4], range2[4];      // Для эффекта спирали на плоскости - границы разворачивания спирали;
uint16_t tail[8], tail2[8];        // Для эффекта спирали на плоскости - позиции хвоста змейки. HiByte = x, LoByte=y
CHSV     tailColor;                // Цвет последней точки "хвоста" змейки. Этот же цвет используется для предварительной заливки всей матрицы
CHSV     tailColor2;               // Предварительная заливка нужна для корректного отображения часов поверх специальных эффектов будильника
bool     firstRowFlag;             // Флаг начала самого первого ряда первого кадра, чтобы не рисовать "хвост" змейки в предыдущем кадре, которого не было.
uint8_t  dawnBrightness;           // Текущая яркость будильника "рассвет"
uint8_t  tailBrightnessStep;       // Шаг приращения яркости будильника "рассвет"
uint8_t  dawnColorIdx;             // Индекс в массиве цвета "заливки" матрицы будильника "рассвет" (голова змейки)
uint8_t  dawnColorPrevIdx;         // Предыдущий индекс - нужен для корректного цвета отрисовки "хвоста" змейки,
// когда голова начинает новый кадр внизу матрицы, а хвост - вверху от предыдущего кадра
uint8_t  step_cnt;                 // Номер шага эффекта, чтобы определить какой длины "хвост" у змейки
// "Рассвет" - от красного к желтому - белому - голубому с плавным увеличением яркости;
// Яркость меняется по таймеру - на каждое срабатывание таймера - +1 к яркости.
// Диапазон изменения яркости - от MIN_DAWN_BRIGHT до MAX_DAWN_BRIGHT (количество шагов)
// Цветовой тон матрицы меняется каждые 16 шагов яркости 255 / 16 -> дает 16 индексов в массиве цветов
// Время таймера увеличения яркости - время рассвета DAWN_NINUTES на количество шагов приращения яркости
uint8_t dawnColorHue[16]  PROGMEM = {0, 16, 28, 36, 44, 52, 57, 62, 64, 66, 66, 64, 62, 60, 128, 128};              // Цвет заполнения - HUE змейки 1
uint8_t dawnColorSat[16]  PROGMEM = {255, 250, 245, 235, 225, 210, 200, 185, 170, 155, 130, 105, 80, 50, 25, 80};   // Цвет заполнения - SAT змейки 1
uint8_t dawnColorHue2[16] PROGMEM = {0, 16, 28, 36, 44, 52, 57, 62, 64, 66, 66, 64, 62, 60, 128, 128};              // Цвет заполнения - HUE змейки 2
uint8_t dawnColorSat2[16] PROGMEM = {255, 250, 245, 235, 225, 210, 200, 185, 170, 155, 130, 105, 80, 50, 25, 80};   // Цвет заполнения - SAT змейки 2
#define MIN_DAWN_BRIGHT   2        // Минимальное значение яркости будильника (с чего начинается)
#define MAX_DAWN_BRIGHT   255      // Максимальное значение яркости будильника (чем заканчивается)
uint8_t DAWN_NINUTES = 20;            // Продолжительность рассыета в минутах
void dawnProcedure() {
  if (loadingFlag) {
    dawnBrightness = MIN_DAWN_BRIGHT;
    // modeCode = MC_DAWN_ALARM;
    FastLED.clear();  // очистить
    FastLED.setBrightness(dawnBrightness);
    if (realDawnDuration <= 0 || realDawnDuration > dawnDuration) realDawnDuration = dawnDuration;
    uint32_t interval = realDawnDuration * 60000UL / (MAX_DAWN_BRIGHT - MIN_DAWN_BRIGHT);
    dawnTimer.setInterval(interval);
  }
  // Пришло время увеличить яркость рассвета?
  if (dawnTimer.isReady() && dawnBrightness < 255) {
    dawnBrightness++;
    FastLED.setBrightness(dawnBrightness);
  }
  uint8_t effect = isAlarming ? alarmEffect : MC_DAWN_ALARM;
  if (effect == MC_DAWN_ALARM) {
    // Если устройство лампа (DEVICE_TYPE == 0) - матрица свернута в "трубу" - рассвет - огонек, бегущий вкруговую по спирали
    // Если устройство плоская матрица в рамке (DEVICE_TYPE == 1) - рассвет - огонек, бегущий по спирали от центра матрицы к краям на плоскости
    effect = DEVICE_TYPE == 0 ? MC_DAWN_ALARM_SPIRAL : MC_DAWN_ALARM_SQUARE;
  }
  // Если эффект "Лампа" и цвет - черный (остался от "выключено" - выбрать цвет лампы из сохраненных эффектов "Цветная лампа"
  if (effect == MC_FILL_COLOR && globalColor == 0) {
    set_globalColor(getColorInt(CHSV(getEffectSpeedValue(MC_FILL_COLOR), getEffectScaleParamValue(MC_FILL_COLOR), 255)));
  }
  if (effect == MC_FILL_COLOR && globalColor == 0) {
    set_globalColor(0xFFFFFF);
  }
  // Сформировать изображение эффекта
  processEffect(effect);
  // Сбрасывать флаг нужно ПОСЛЕ того как инициализированы: И процедура рассвета И применяемый эффект,
  // используемый в качестве рассвета
  loadingFlag = false;
}
// "Рассвет" по спирали, для ламп на круговой матрице (свернутой в трубу)
void dawnLampSpiral() {
  if (loadingFlag) {
    row = 0, col = 0;
    dawnBrightness = MIN_DAWN_BRIGHT;
    tailBrightnessStep = 16;
    firstRowFlag = true;
    dawnColorIdx = 0;
    dawnColorPrevIdx = 0;
    tailColor = CHSV(0, 255, 255 - 8 * tailBrightnessStep);
  }
  bool flag = true;
  int8_t x = col, y = row;
  if (!firstRowFlag) {
    fillAll(tailColor);
  }
  uint8_t tail_len = min(8, pWIDTH - 1);
  for (uint8_t i = 0; i < tail_len; i++) {
    x--;
    if (x < 0) {
      x = pWIDTH - 1;
      y--;
    }
    if (y < 0) {
      y = pHEIGHT - 1;
      flag = false;
      if (firstRowFlag) break;
    }
    uint8_t idx = y > row ? dawnColorPrevIdx : dawnColorIdx;
    uint8_t dawnHue = pgm_read_byte(&(dawnColorHue[idx]));
    uint8_t dawnSat = pgm_read_byte(&(dawnColorSat[idx]));
    tailColor = CHSV(dawnHue, dawnSat, 255 - i * tailBrightnessStep);
    drawPixelXY(x, y, tailColor);
  }
  if (flag) {
    firstRowFlag = false;
    dawnColorPrevIdx = dawnColorIdx;
  }
  if (dawnBrightness == 255 && tailBrightnessStep > 8) tailBrightnessStep -= 2;
  col++;
  if (col >= pWIDTH) {
    col = 0; row++;
  }
  if (row >= pHEIGHT) row = 0;
  if (col == 0 && row == 0) {
    // Кол-во элементов массива - 16; Шагов яркости - 255; Изменение индекса каждые 16 шагов яркости.
    dawnColorIdx = dawnBrightness >> 4;
  }
}

// "Рассвет" по спирали на плоскости, для плоских матриц
void dawnLampSquare() {
  if (loadingFlag) {
    dir_mx = pWIDTH > pHEIGHT ? 0 : 1;                                   // 0 - квадратные сегменты расположены горизонтально, 1 - вертикально
    seg_num = dir_mx == 0 ? (pWIDTH / pHEIGHT) : (pHEIGHT / pWIDTH);     // вычисляем количество сегментов, умещающихся на матрице
    seg_size = dir_mx == 0 ? pHEIGHT : pWIDTH;                           // Размер квадратного сегмента (высота и ширина равны)
    seg_offset = ((dir_mx == 0 ? pWIDTH : pHEIGHT) - seg_size * seg_num) / (seg_num + 1); // смещение от края матрицы и между сегментами
    SetStartPos();
    dawnBrightness = MIN_DAWN_BRIGHT;
    tailBrightnessStep = 16;
    dawnColorIdx = 0;
    step_cnt = 0;
    memset(tail, 0, sizeof(uint16_t) * 8);
    memset(tail2, 0, sizeof(uint16_t) * 8);
    tailColor = CHSV(0, 255, 255 - 8 * tailBrightnessStep);
  }
  int8_t x = col, y = row;
  int8_t x2 = col2, y2 = row2;
  fillAll(tailColor);
  step_cnt++;
  for (uint8_t i = 7; i > 0; i--) {
    tail[i]  = tail[i - 1];
    tail2[i] = tail2[i - 1];
  }
  tail[0]  = (uint)((int)x << 8 | (int)y);
  tail2[0] = (uint)((int)x2 << 8 | (int)y2);
  uint8_t dawnHue  = pgm_read_byte(&(dawnColorHue[dawnColorIdx]));
  uint8_t dawnSat  = pgm_read_byte(&(dawnColorSat[dawnColorIdx]));
  uint8_t dawnHue2 = pgm_read_byte(&(dawnColorHue2[dawnColorIdx]));
  uint8_t dawnSat2 = pgm_read_byte(&(dawnColorSat2[dawnColorIdx]));
  for (uint8_t i = 0; i < 8; i++) {
    tailColor  = CHSV(dawnHue, dawnSat, 255 - i * tailBrightnessStep);
    tailColor2 = CHSV(dawnHue2, dawnSat2, 255 - i * tailBrightnessStep);
    if (i <= step_cnt) {
      x  =  tail[i] >> 8;
      y  = tail[i]  & 0xff;
      x2 =  tail2[i] >> 8;
      y2 = tail2[i] & 0xff;
      for (uint8_t n = 0; n < seg_num; n++) {
        uint8_t cx = dir_mx == 0 ? (seg_offset * (n + 1) + seg_size * n) : 0;
        uint8_t cy = dir_mx == 0 ? 0 : (seg_offset * (n + 1) + seg_size * n);
        drawPixelXY(x + cx,  y + cy,  tailColor);
        drawPixelXY(x2 + cx, y2 + cy, tailColor2);
      }
    }
  }
  if (dawnBrightness == 255 && tailBrightnessStep > 8) tailBrightnessStep -= 2;
  switch (dir) {
    case 0: // вниз;
      row--;
      if (row <= range[dir]) {
        range[dir] = row - 2;
        dir++;
      }
      break;
    case 1: // влево;
      col--;
      if (col <= range[dir]) {
        range[dir] = col - 2;
        dir++;
      }
      break;
    case 2: // вверх;
      row++;
      if (row >= range[dir]) {
        range[dir] = row + 2;
        dir++;
      }
      break;
    case 3: // вправо;
      col++;
      if (col >= range[dir]) {
        range[dir] = col + 2;
        dir = 0;
      }
      break;
  }
  switch (dir2) {
    case 0: // вниз;
      row2--;
      if (row2 <= range2[dir2]) {
        range2[dir2] = row2 - 2;
        dir2++;
      }
      break;
    case 1: // влево;
      col2--;
      if (col2 <= range2[dir2]) {
        range2[dir2] = col2 - 2;
        dir2++;
      }
      break;
    case 2: // вверх;
      row2++;
      if (row2 >= range2[dir2]) {
        range2[dir2] = row2 + 2;
        dir2++;
      }
      break;
    case 3: // вправо;
      col2++;
      if (col2 >= range2[dir2]) {
        range2[dir2] = col2 + 2;
        dir2 = 0;
      }
      break;
  }
  bool out  = (col  < 0 || col  >= seg_size) && (row  < 0 || row  >= seg_size);
  bool out2 = (col2 < 0 || col2 >= seg_size) && (row2 < 0 || row2 >= seg_size);
  if (out && out2) {
    // Кол-во элементов массива - 16; Шагов яркости - 255; Изменение индекса каждые 16 шагов яркости.
    dawnColorIdx = dawnBrightness >> 4;
    SetStartPos();
    step_cnt = 0;
  }
}
void SetStartPos() {
  if (seg_size % 2 == 1) {
    col = seg_size / 2 + 1;
    col2 = col;
    row = seg_size / 2 + 1;
    row2 = row;
  } else {
    col = seg_size / 2 - 1;
    col2 = seg_size - col - 1;
    row = seg_size / 2 - 1;
    row2 = seg_size - row - 1;
  }
  dir = 2; dir2 = 0;
  // 0 - вниз; 1 - влево; 2 - вверх; 3 - вправо;
  range[0] = row - 2; range[1] = col - 2; range[2] = row + 2; range[3] = col + 2;
  range2[0] = row2 - 2; range2[1] = col2 - 2; range2[2] = row2 + 2; range2[3] = col2 + 2;
}

//тестовый эффект для проверки правильности порядка цветов
//просто "в лоб" зажигаем три верхних ряда в порядке RGB
void testColorOrder() {
  if (loadingFlag) {
    loadingFlag = false;
    FastLED.clear();
  }
  for (uint8_t i = 0; i < pWIDTH; i++) {
    leds[XY(i, pHEIGHT - 1)] = CHSV(0, 255, 255);
    leds[XY(i, pHEIGHT - 2)] = CHSV(96, 255, 255);
    leds[XY(i, pHEIGHT - 3)] = CHSV(160, 255, 255);
  }
}

// ******************* ЛАМПА ********************
void fillColorProcedure() {
  if (loadingFlag) {
    loadingFlag = false;
  }
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  uint8_t bright = isAlarming && !isAlarmStopped
                   ? dawnBrightness
                   : (specialMode ? specialBrightness : effectBrightness);
  if (globalColor == 0) {
    fillAll(CRGB::Black);
  } else {
    CRGB color = globalColor;
    color.nscale8_video(bright);
    fillAll(color);
  }
}

// ******************* МЕРЦАНИЕ ********************
uint32_t xf, yf, v_time, hue_time, hxy;
// Play with the values of the variables below and see what kinds of effects they
// have!  More octaves will make things slower.
// how many octaves to use for the brightness and hue functions
uint8_t octaves = 1;
uint8_t hue_octaves = 3;
// the 'distance' between points on the x and y axis
int32_t  xscale = 57771;
int32_t  yscale = 57771;
// the 'distance' between x/y points for the hue noise
int32_t  hue_scale = 1;
// how fast we move through time & hue noise
int32_t  time_speed = 1111;
uint8_t  hue_speed = 1;
// adjust these values to move along the x or y axis between frames
uint16_t x_speed, y_speed;
void flickerRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    x_speed = (pWIDTH > pHEIGHT ? 1111 : 331);
    y_speed = (pWIDTH > pHEIGHT ? 331 : 1111);
    hxy = (uint32_t)((uint32_t)random16() << 16) + (uint32_t)random16();
    xf = (uint32_t)((uint32_t)random16() << 16) + (uint32_t)random16();
    yf = (uint32_t)((uint32_t)random16() << 16) + (uint32_t)random16();
    v_time = (uint32_t)((uint32_t)random16() << 16) + (uint32_t)random16();
    hue_time = (uint32_t)((uint32_t)random16() << 16) + (uint32_t)random16();
  }
  // Если совсем задержки нет - матрица мерцает от постоянного обновления
  delay(5);
  // uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  // fill the led array 2/16-bit noise values
  fill_2dnoise16(leds, pWIDTH, pHEIGHT, (sMATRIX_TYPE == 0),
                 octaves, xf, xscale, yf, yscale, v_time,
                 hue_octaves, hxy, hue_scale, hxy, hue_scale, hue_time,
                 false);
  // adjust the intra-frame time values
  hue_speed  = map8(255 - getEffectSpeedValue(MC_FLICKER), 1, 10);
  xf += x_speed;
  yf += y_speed;
  v_time += time_speed;
  hue_time += hue_speed;
}

// ******************* PACIFICA ********************
//////////////////////////////////////////////////////////////////////////
//
// In this animation, there are four "layers" of waves of light.
//
// Each layer moves independently, and each is scaled separately.
//
// All four wave layers are added together on top of each other, and then
// another filter is applied that adds "whitecaps" of brightness where the
// waves line up with each other more.  Finally, another pass is taken
// over the led array to 'deepen' (dim) the blues and greens.
//
// The speed and scale and motion each layer varies slowly within independent
// hand-chosen ranges, which is why the code has a lot of low-speed 'beatsin8' functions
// with a lot of oddly specific numeric ranges.
//
// These three custom blue-green color palettes were inspired by the colors found in
// the waters off the southern coast of California, https://goo.gl/maps/QQgd97jjHesHZVxQ7
//
CRGBPalette16 pacifica_palette_1 =
{ 0x000507, 0x000409, 0x00030B, 0x00030D, 0x000210, 0x000212, 0x000114, 0x000117,
  0x000019, 0x00001C, 0x000026, 0x000031, 0x00003B, 0x000046, 0x14554B, 0x28AA50
};
CRGBPalette16 pacifica_palette_2 =
{ 0x000507, 0x000409, 0x00030B, 0x00030D, 0x000210, 0x000212, 0x000114, 0x000117,
  0x000019, 0x00001C, 0x000026, 0x000031, 0x00003B, 0x000046, 0x0C5F52, 0x19BE5F
};
CRGBPalette16 pacifica_palette_3 =
{ 0x000208, 0x00030E, 0x000514, 0x00061A, 0x000820, 0x000927, 0x000B2D, 0x000C33,
  0x000E39, 0x001040, 0x001450, 0x001860, 0x001C70, 0x002080, 0x1040BF, 0x2060FF
};
void pacificaRoutine()
{
  if (loadingFlag) {
    // modeCode = MC_PACIFICA;
    loadingFlag = false;
  }
  // Если совсем задержки нет - матрица мерцает от постоянного обновления
  delay(5);
  // Increment the four "color index start" counters, one for each wave layer.
  // Each is incremented at a different speed, and the speeds vary over time.
  static uint16_t sCIStart1, sCIStart2, sCIStart3, sCIStart4;
  static uint32_t sLastms = 0;
  uint32_t ms = GET_MILLIS();
  uint32_t deltams = ms - sLastms;
  sLastms = ms;
  uint16_t speedfactor1 = beatsin16(3, 179, 269);
  uint16_t speedfactor2 = beatsin16(4, 179, 269);
  uint32_t deltams1 = (deltams * speedfactor1) / 256;
  uint32_t deltams2 = (deltams * speedfactor2) / 256;
  uint32_t deltams21 = (deltams1 + deltams2) / 2;
  sCIStart1 += (deltams1 * beatsin88(1011, 10, 13));
  sCIStart2 -= (deltams21 * beatsin88(777, 8, 11));
  sCIStart3 -= (deltams1 * beatsin88(501, 5, 7));
  sCIStart4 -= (deltams2 * beatsin88(257, 4, 6));
  // Clear out the LED array to a dim background blue-green
  fill_solid( leds, NUM_LEDS, CRGB( 2, 6, 10));
  // Render each of four layers, with different scales and speeds, that vary over time
  pacifica_one_layer( pacifica_palette_1, sCIStart1, beatsin16( 3, 11 * 256, 14 * 256), beatsin8( 10, 70, 130), 0 - beat16( 301) );
  pacifica_one_layer( pacifica_palette_2, sCIStart2, beatsin16( 4,  6 * 256,  9 * 256), beatsin8( 17, 40,  80), beat16( 401) );
  pacifica_one_layer( pacifica_palette_3, sCIStart3, 6 * 256, beatsin8( 9, 10, 38), 0 - beat16(503));
  pacifica_one_layer( pacifica_palette_3, sCIStart4, 5 * 256, beatsin8( 8, 10, 28), beat16(601));
  // Add brighter 'whitecaps' where the waves lines up more
  pacifica_add_whitecaps();
  // Deepen the blues and greens a bit
  pacifica_deepen_colors();
}
// Add one layer of waves into the led array
void pacifica_one_layer( CRGBPalette16& p, uint16_t cistart, uint16_t wavescale, uint8_t bri, uint16_t ioff)
{
  uint16_t ci = cistart;
  uint16_t waveangle = ioff;
  uint16_t wavescale_half = (wavescale / 2) + 20;
  for ( uint16_t i = 0; i < NUM_LEDS; i++) {
    waveangle += 250;
    uint16_t s16 = sin16( waveangle ) + 32768;
    uint16_t cs = scale16( s16 , wavescale_half ) + wavescale_half;
    ci += cs;
    uint16_t sindex16 = sin16( ci) + 32768;
    uint8_t sindex8 = scale16( sindex16, 240);
    CRGB c = ColorFromPalette( p, sindex8, bri, LINEARBLEND);
    leds[i] += c;
  }
}
// Add extra 'white' to areas where the four layers of light have lined up brightly
void pacifica_add_whitecaps() {
  uint8_t basethreshold = beatsin8( 9, 55, 65);
  uint8_t wave = beat8( 7 );
  for ( uint16_t i = 0; i < NUM_LEDS; i++) {
    uint8_t threshold = scale8( sin8( wave), 20) + basethreshold;
    wave += 7;
    uint8_t l = leds[i].getAverageLight();
    if ( l > threshold) {
      uint8_t overage = l - threshold;
      uint8_t overage2 = qadd8( overage, overage);
      leds[i] += CRGB( overage, overage2, qadd8( overage2, overage2));
    }
  }
}
// Deepen the blues and greens
void pacifica_deepen_colors() {
  for ( uint16_t i = 0; i < NUM_LEDS; i++) {
    leds[i].blue = scale8( leds[i].blue,  145);
    leds[i].green = scale8( leds[i].green, 200);
    leds[i] |= CRGB( 2, 5, 7);
  }
}


// ********************** SHADOWS ***********************

void shadowsRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
  }
  // Если совсем задержки нет - матрица мерцает от постоянного обновления
  delay(5);
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;
  uint8_t  sat8 = beatsin88( 87, 220, 250);
  uint8_t  brightdepth = beatsin88( 341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
  uint8_t  msmultiplier = beatsin88(147, 23, 60);
  uint16_t hue16 = sHue16;//gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 1, 3000);
  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis ;
  uint8_t  effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  sLastMillis  = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88( 400, 5, 9);
  uint16_t brightnesstheta16 = sPseudotime;
  for ( uint16_t i = 0 ; i < NUM_LEDS; i++) {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;
    brightnesstheta16  += brightnessthetainc16;
    uint16_t b16 = sin16( brightnesstheta16  ) + 32768;
    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);
    CRGB newcolor = CHSV( hue8, sat8, map8(bri8, map(effectBrightness, 32, 255, 32, 125), map(effectBrightness, 32, 255, 125, 250)));
    uint16_t pixelnumber = i;
    pixelnumber = (NUM_LEDS - 1) - pixelnumber;
    nblend( leds[pixelnumber], newcolor, 64);
  }
}

// ***************************** ПАЛИТРА *****************************
#define BLOCK_SIZE 4       // Размер квадратика палитры
#define FADE_IN_STEPS 16   // За сколько шагов плашка появляется на экране    
#define FADE_OUT_STEPS 32  // За сколько шагов плашка убирается с экрана    
#define BLOCK_ON_START 5   // Сколько блоков сразу появлять в начале эффекта
uint8_t num_x, num_y, off_x, off_y;
uint8_t **palette_h; // Н in CHSV
uint8_t **palette_s; // S in CHSV
uint8_t **block_sta; // Block state: // 0 - появление; 1 - исчезновение; 2 - пауза перед появлением 3 - пауза перед удалением
uint8_t **block_dur; // время паузы блока
void paletteRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    num_x = pWIDTH / BLOCK_SIZE;
    num_y = pHEIGHT / BLOCK_SIZE;
    off_x = (pWIDTH - BLOCK_SIZE * num_x) / 2;
    off_y = (pHEIGHT - BLOCK_SIZE * num_y) / 2;
    dir_mx = pWIDTH > pHEIGHT ? 0 : 1;                                   // 0 - квадратные сегменты расположены горизонтально, 1 - вертикально
    seg_num = dir_mx == 0 ? (pWIDTH / pHEIGHT) : (pHEIGHT / pWIDTH);     // вычисляем количество сегментов, умещающихся на матрице
    if (palette_h == NULL) {
      palette_h = new uint8_t*[num_x];
      for (uint8_t i = 0; i < num_x; i++) {
        palette_h[i] = new uint8_t [num_y];
      }
    }
    if (palette_s == NULL) {
      palette_s = new uint8_t*[num_x];
      for (uint8_t i = 0; i < num_x; i++) {
        palette_s[i] = new uint8_t [num_y];
      }
    }
    if (block_sta == NULL) {
      block_sta = new uint8_t*[num_x];
      for (uint8_t i = 0; i < num_x; i++) {
        block_sta[i] = new uint8_t [num_y];
      }
    }
    if (block_dur == NULL) {
      block_dur = new uint8_t*[num_x];
      for (uint8_t i = 0; i < num_x; i++) {
        block_dur[i] = new uint8_t [num_y];
      }
    }
    // Для всех блоков определить состояние - "ожидание появления
    for (uint8_t c = 0; c < num_x; c++) {
      for (uint8_t r = 0; r < num_y; r++) {
        block_sta[c][r] = 2;                // Состояние - пауза перед появлением
        block_dur[c][r] = random8(25, 125); // Длительность паузы
      }
    }
    // Для некоторого количества начальных - установить "За шаг до появления"
    // При первом же проходе состояние переключится на "появление"
    for (uint8_t i = 0; i < BLOCK_ON_START * seg_num; i++) {
      uint8_t c = random8(0, num_x - 1);
      uint8_t r = random8(0, num_y - 1);
      block_dur[c][r] = 1;                  // Счетчик до начала появления
    }
    FastLED.clear();
  }
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  for (uint8_t c = 0; c < num_x; c++) {
    uint8_t block_x = off_x + c * BLOCK_SIZE;
    for (uint8_t r = 0; r < num_y; r++) {
      uint8_t block_y = off_y + r * BLOCK_SIZE;
      uint8_t h = palette_h[c][r];
      uint8_t s = palette_s[c][r];
      // Проверить состояние блока
      if (block_sta[c][r] > 1) {
        // Одна из пауз (2 или 3) - пауза перед появлением или перед исчезновением
        // Уменьшить время паузы. Если стало 0 - переключить с паузы на появление / исчезновение
        block_dur[c][r] -= 1;
        if (block_dur[c][r] == 0) {
          block_sta[c][r] -= 2;     // 3->1 - исчезать; 2->0 появлять за указанное количество шагов
          if (block_sta[c][r] == 0) {
            block_dur[c][r] = FADE_IN_STEPS;    // Количество шагов появления блока
            palette_h[c][r] = random8(0, 255);  // Цвет нового блока
            palette_s[c][r] = random8(112, 254); // Насыщенность цвета нового блока
          } else {
            block_dur[c][r] = FADE_OUT_STEPS;  // Кол-во шагов убирания блока
          }
        }
      }
      if (block_sta[c][r] < 2) {
        // В процессе появления или исчезновения (0 или 1)
        // Выполнить один шаг появления / исчезновения блока
        uint8_t fade_dir = block_sta[c][r]; // 0 - появляться, 1 - исчезать
        uint8_t fade_step = block_dur[c][r];

        // Яркость блока
        uint8_t bri = fade_dir == 0
                      ? map(fade_step, 0, FADE_IN_STEPS,  0, effectBrightness)
                      : map(fade_step, 0, FADE_OUT_STEPS, effectBrightness, 0);
        // Нарисовать блок
        for (uint8_t i = 0; i < BLOCK_SIZE; i++) {
          for (uint8_t j = 0; j < BLOCK_SIZE; j++) {
            CHSV color = CHSV(h, s, bri); // bri2
            uint8_t xx = block_x + j;
            uint8_t yy = block_y + BLOCK_SIZE - i - 1;
            if (xx < pWIDTH && yy < pHEIGHT) {
              uint16_t idx = getPixelNumber(xx, yy);
              leds[idx] = color;
            }
          }
        }
        // Шаг появления - обработан
        block_dur[c][r] -= 1;
        // Весь процесс появления / исчезновения выполнен?
        // Сменить статус блока
        if (block_dur[c][r] == 0) {
          // Появление / исчезновение закончено
          block_sta[c][r] = block_sta[c][r] == 0 ? 3 : 2; // вкл паузу перед исчезновением после появления или паузу перед появлением после исчезновения
          block_dur[c][r] = random8(25, 125);             // Длительность паузы (циклов обращения палитры)
        }
      }
    }
  }
}
void paletteRoutineRelease() {
  if (block_dur != NULL) {
    for ( uint8_t i = 0; i < num_x; i++ ) {
      delete [] block_dur[i];
    } delete [] block_dur;
    block_dur = NULL;
  }
  if (block_sta != NULL) {
    for ( uint8_t i = 0; i < num_x; i++ ) {
      delete [] block_sta[i];
    } delete [] block_sta;
    block_sta = NULL;
  }
  if (palette_s != NULL) {
    for ( uint8_t i = 0; i < num_x; i++ ) {
      delete [] palette_s[i];
    } delete [] palette_s;
    palette_s = NULL;
  }
  if (palette_h != NULL) {
    for ( uint8_t i = 0; i < num_x; i++ ) {
      delete [] palette_h[i];
    } delete [] palette_h;
    palette_h = NULL;
  }
}

// ****************************** ANALYZER *****************************
// цвета высоты полос спектра.
#define COLOR1    HUE_GREEN
#define COLOR2    HUE_YELLOW
#define COLOR3    HUE_ORANGE
#define COLOR4    HUE_RED
#define MAX_COLOR HUE_RED // цвет точек максимума
// анимация
#define SMOOTH 0.3        // плавность движения столбиков (0 - 1)
// точки максимума
#define MAX_DOTS 1        // включить/выключить отрисовку точек максимума (1 вкл, 0 выкл)
#define FALL_DELAY 50     // скорость падения точек максимума (задержка, миллисекунды)
#define FALL_PAUSE 700    // пауза перед падением точек максимума, миллисекунды
uint32_t gainTimer, fallTimer;
uint8_t  maxValue;
bool     fallFlag;
uint32_t *timeLevel;
uint8_t  *posOffset;   // Массив данных для отображения на матрице
int16_t  *maxLevel;
uint8_t  *posLevel_old;
uint8_t st = 0;
uint8_t phase = 0;          // фаза эффекта

// -------------------------------------------------------------------------------------
void analyzerRoutine() {
  static int16_t MAX_LEVEL = (pHEIGHT + pHEIGHT / 4);
  static uint8_t SIN_WIDTH = (pWIDTH / 8);
  if (loadingFlag) {
    loadingFlag = false;
    if (timeLevel    == NULL) {
      timeLevel    = new uint32_t[pWIDTH];
    }
    if (posOffset    == NULL) {
      posOffset    = new uint8_t[pWIDTH];
    }
    if (maxLevel     == NULL) {
      maxLevel     = new int16_t[pWIDTH];
    }
    if (posLevel_old == NULL) {
      posLevel_old = new uint8_t[pWIDTH];
    }
    for (uint8_t i = 0; i < pWIDTH; i++) {
      maxLevel[i] = 0;
      posLevel_old[i] = 0;
    }
    st = 0;
    phase = 0;
    FastLED.clear();
  }
  if (phase == 0) {
    // Движение волны слева направо
    for (uint8_t i = 0; i < pWIDTH; i++) {
      posOffset[i] = (i < st || i >= st + SIN_WIDTH - (SIN_WIDTH / 4))
                     ? 0
                     : map8(sin8(map(i, st, st + SIN_WIDTH, 0, 255)), 1, pHEIGHT + pHEIGHT / 2);
    }
  } else if (phase == 2) {
    // Движение волны справа налево
    for (uint8_t i = 0; i < pWIDTH; i++) {
      posOffset[i] = (i < pWIDTH - st || i > pWIDTH - st + SIN_WIDTH)
                     ? 0
                     : map8(sin8(map(i, pWIDTH - st, pWIDTH - st + SIN_WIDTH, 0, 255)), 1, pHEIGHT + pHEIGHT / 2);
    }
  } else if (phase == 1 || phase == 3) {
    // Пауза, даем "отстояться" пикам
    for (uint8_t i = 0; i < pWIDTH; i++) {
      posOffset[i] = 0;
    }
  } else if (phase >= 4) {
    // Случайные двиижения - "музыка"
    for (uint8_t i = 0; i < pWIDTH; i++) {
      posOffset[i] = random8(1, MAX_LEVEL);
    }
  }
  st++;
  if (st >= pWIDTH && phase < 4) {
    phase++;
    st = phase % 2 == 1 ? pWIDTH / 2 : 0;
  }
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  maxValue = 0;
  FastLED.clear();  // очистить матрицу
  for (uint8_t pos = 0; pos < pWIDTH; pos++) {    // для каждого столбца матрицы
    uint8_t posLevel = posOffset[pos];
    // найти максимум из пачки тонов
    if (posLevel > maxValue) maxValue = posLevel;
    // фильтрация длины столбиков, для их плавного движения
    posLevel = posLevel * SMOOTH + posLevel_old[pos] * (1 - SMOOTH);
    posLevel_old[pos] = posLevel;
    // преобразовать значение величины спектра в диапазон 0..pHEIGHT с учётом настроек
    posLevel = constrain(posLevel, 1, pHEIGHT - 1);
    if (posLevel > 0) {
      for (uint8_t j = 0; j < posLevel; j++) {
        CHSV color;
        if      (j < map( 5, 0, 16, 0, pHEIGHT)) color = CHSV(COLOR1, 255, effectBrightness);
        else if (j < map(10, 0, 16, 0, pHEIGHT)) color = CHSV(COLOR2, 255, effectBrightness);
        else if (j < map(13, 0, 16, 0, pHEIGHT)) color = CHSV(COLOR3, 255, effectBrightness);
        else if (j < map(15, 0, 16, 0, pHEIGHT)) color = CHSV(COLOR4, 255, effectBrightness);
        drawPixelXY(pos, j, color);
      }
    }
    if (posLevel > 0 && posLevel > maxLevel[pos]) {    // если для этой полосы есть максимум, который больше предыдущего
      maxLevel[pos] = posLevel;                        // запомнить его
      timeLevel[pos] = millis();                       // запомнить время
    }
    // если точка максимума выше нуля (или равна ему) - включить пиксель
    if (maxLevel[pos] >= 0 && MAX_DOTS) {
      drawPixelXY(pos, maxLevel[pos], CHSV(MAX_COLOR, 255, effectBrightness));
    }
    if (fallFlag) {                                           // если падаем на шаг
      if ((uint32_t)millis() - timeLevel[pos] > FALL_PAUSE) {     // если максимум держался на своей высоте дольше FALL_PAUSE
        if (maxLevel[pos] >= 0) maxLevel[pos]--;              // уменьшить высоту точки на 1
        // внимание! Принимает минимальное значение -1 !
      }
    }
  }
  fallFlag = 0;                                 // сбросить флаг падения
  if (millis() - fallTimer > FALL_DELAY) {      // если настало время следующего падения
    fallFlag = 1;                               // поднять флаг
    fallTimer = millis();
  }
}
void analyzerRoutineRelease() {
  if (posLevel_old != NULL) {
    delete[] posLevel_old;
    posLevel_old = NULL;
  }
  if (maxLevel != NULL)     {
    delete[] maxLevel;
    maxLevel = NULL;
  }
  if (posOffset != NULL)    {
    delete[] posOffset;
    posOffset = NULL;
  }
  if (timeLevel != NULL)    {
    delete[] timeLevel;
    timeLevel = NULL;
  }
}

// *************************** ВЫШИВАНКА **************************
// ------ Эффект "Вышиванка"
// (с) проект Aurora "Munch"
// adopted/updated by kostyamat
int8_t  count = 0;
uint8_t flip = 0;
uint8_t generation = 0;
uint8_t rnd = 4; //1-8
uint8_t mic[2];
uint8_t minDimLocal = maxDim > 32 ? 32 : 16;
const uint8_t width_adj = (pWIDTH < pHEIGHT ? (pHEIGHT - pWIDTH) / 2 : 0);
const uint8_t height_adj = (pHEIGHT < pWIDTH ? (pWIDTH - pHEIGHT) / 2 : 0);
const uint8_t maxDim_steps = 256 / maxDim;
void munchRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    generation = 0;
    dir = 1;
    count = 0;
    flip = 0;
    FastLED.clear();
  }
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  for (uint8_t x = 0; x < minDimLocal; x++) {
    for (uint8_t y = 0; y < minDimLocal; y++) {
      CRGB color = (x ^ y ^ flip) < count ? ColorFromPalette(RainbowColors_p, ((x ^ y) << rnd) + generation, effectBrightness) : CRGB::Black;
      if (x < pWIDTH and y < pHEIGHT) leds[XY(x, y)] = color;
      if (x + minDimLocal < pWIDTH and y < pHEIGHT) leds[XY(x + minDimLocal, y)] = color;
      if (y + minDimLocal < pHEIGHT and x < pWIDTH) leds[XY(x, y + minDimLocal)] = color;
      if (x + minDimLocal < pWIDTH and y + minDimLocal < pHEIGHT) leds[XY(x + minDimLocal, y + minDimLocal)] = color;
    }
  }
  count += dir;
  if (count <= 0 || count >= mic[0]) {
    dir = -dir;
    if (count <= 0) {
      mic[0] = mic[1];
      if (flip == 0)
        flip = mic[1] - 1;
      else
        flip = 0;
    }
  }
  generation++;
  mic[1] = minDimLocal;
}

// *************************** ДОЖДЬ **************************
CRGB rainColor = CRGB(60, 80, 90);
CRGB lightningColor = CRGB(72, 72, 80);
CRGBPalette16 rain_p( CRGB::Black, rainColor);
CRGBPalette16 rainClouds_p( CRGB::Black, CRGB(15, 24, 24), CRGB(9, 15, 15), CRGB::Black );
uint8_t cloudHeight = pHEIGHT * 0.2 + 1;
uint8_t **noise3d;
uint8_t *cloud;
void rain(uint8_t backgroundDepth, uint8_t spawnFreq, uint8_t tailLength, bool splashes, bool clouds, bool storm) {
  static uint16_t noiseX = random16();
  static uint16_t noiseY = random16();
  static uint16_t noiseZ = random16();
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  fadeToBlackBy( leds, NUM_LEDS, 255 - tailLength);
  // Loop for each column individually
  for (uint8_t x = 0; x < pWIDTH; x++) {
    // Step 1.  Move each dot down one cell
    for (uint8_t i = 0; i < pHEIGHT; i++) {
      if (noise3d[x][i] >= backgroundDepth) {  // Don't move empty cells
        if (i > 0) noise3d[x][wrapY(i - 1)] = noise3d[x][i];
        noise3d[x][i] = 0;
      }
    }
    // Step 2.  Randomly spawn new dots at top
    if (random8() < spawnFreq) {
      noise3d[x][pHEIGHT - 1] = random(backgroundDepth, effectBrightness);
    }
    // Step 3. Map from tempMatrix cells to LED colors
    for (uint8_t y = 0; y < pHEIGHT; y++) {
      if (noise3d[x][y] >= backgroundDepth) {  // Don't write out empty cells
        leds[XY(x, y)] = ColorFromPalette(rain_p, noise3d[x][y], effectBrightness);
      }
    }
    // Step 4. Add splash if called for
    if (splashes) {
      // FIXME, this is broken
      uint8_t j = line[x];
      uint8_t v = noise3d[x][0];
      if (j >= backgroundDepth) {
        leds[XY(wrapX(x - 2), 0)] = ColorFromPalette(rain_p, j / 3, effectBrightness);
        leds[XY(wrapX(x + 2), 0)] = ColorFromPalette(rain_p, j / 3, effectBrightness);
        line[x] = 0;   // Reset splash
      }
      if (v >= backgroundDepth) {
        leds[XY(wrapX(x - 1), 1)] = ColorFromPalette(rain_p, v / 2, effectBrightness);
        leds[XY(wrapX(x + 1), 1)] = ColorFromPalette(rain_p, v / 2, effectBrightness);
        line[x] = v; // Prep splash for next frame
      }
    }
    // Step 5. Add lightning if called for
    if (storm && random16() < 72) {
      uint8_t *lightning = (uint8_t *) malloc(pWIDTH * pHEIGHT);
      if (lightning != NULL) {
        lightning[scale8(random8(), pWIDTH - 1) + (pHEIGHT - 1) * pWIDTH] = 255; // Random starting location
        for (uint8_t ly = pHEIGHT - 1; ly > 1; ly--) {
          for (uint8_t lx = 1; lx < pWIDTH - 1; lx++) {
            if (lightning[lx + ly * pWIDTH] == 255) {
              lightning[lx + ly * pWIDTH] = 0;
              uint8_t dir = random8(4);
              switch (dir) {
                case 0:
                  leds[XY(lx + 1, ly - 1)] = lightningColor;
                  lightning[(lx + 1) + (ly - 1) * pWIDTH] = 255; // move down and right
                  break;
                case 1:
                  leds[XY(lx, ly - 1)] = CRGB(128, 128, 128); // я без понятия, почему у верхней молнии один оттенок, а у остальных - другой
                  lightning[lx + (ly - 1) * pWIDTH] = 255;  // move down
                  break;
                case 2:
                  leds[XY(lx - 1, ly - 1)] = CRGB(128, 128, 128);
                  lightning[(lx - 1) + (ly - 1) * pWIDTH] = 255; // move down and left
                  break;
                case 3:
                  leds[XY(lx - 1, ly - 1)] = CRGB(128, 128, 128);
                  lightning[(lx - 1) + (ly - 1) * pWIDTH] = 255; // fork down and left
                  leds[XY(lx - 1, ly - 1)] = CRGB(128, 128, 128);
                  lightning[(lx + 1) + (ly - 1) * pWIDTH] = 255; // fork down and right
                  break;
              }
            }
          }
        }
        free(lightning);
      } else {
        DEBUGLN("lightning malloc failed");
      }
    }
    // Step 6. Add clouds if called for
    if (clouds) {
      uint16_t noiseScale = 250;  // A value of 1 will be so zoomed in, you'll mostly see solid colors. A value of 4011 will be very zoomed out and shimmery
      if (cloud != NULL) {
        int16_t xoffset = noiseScale * x + hue;
        for (uint8_t z = 0; z < cloudHeight; z++) {
          int16_t yoffset = noiseScale * z - hue;
          uint8_t dataSmoothing = 192;
          uint8_t noiseData = qsub8(inoise8(noiseX + xoffset, noiseY + yoffset, noiseZ), 16);
          noiseData = qadd8(noiseData, scale8(noiseData, 39));
          cloud[x * cloudHeight + z] = scale8( cloud[x * cloudHeight + z], dataSmoothing) + scale8( noiseData, 256 - dataSmoothing);
          nblend(leds[XY(x, pHEIGHT - z - 1)], ColorFromPalette(rainClouds_p, cloud[x * cloudHeight + z], effectBrightness), (cloudHeight - z) * (250 / cloudHeight));
        }
      } else {
        DEBUGLN("cloud malloc failed");
      }
      noiseZ++;
    }
  }
}

void rainRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    cloudHeight = pHEIGHT * 0.2 + 1; // это уже 20% c лишеним, но на высоких матрицах будет чуть меньше
    if (noise3d == NULL) {
      noise3d = new uint8_t*[pWIDTH];
      for (uint8_t i = 0; i < pWIDTH; i++) {
        noise3d[i] = new uint8_t [pHEIGHT];
      }
    }
    if (line == NULL)    {
      line = new uint8_t[pWIDTH];
    }
    if (cloud == NULL)   {
      cloud = new uint8_t[pWIDTH * cloudHeight];
    }
  }
  uint8_t intensity = beatsin8(map8(getEffectScaleParamValue(MC_RAIN), 2, 6), 4, 60);
  // ( Depth of dots, frequency of new dots, length of tails, splashes, clouds, ligthening )
  if (intensity <= 35)
    // Lightweight
    rain(60, intensity, 10, true, true, false);
  else
    // Stormy
    rain(0, intensity, 10, true, true, true);
}
void rainRoutineRelease() {
  if (cloud != NULL)   {
    delete[] cloud;
    cloud = NULL;
  }
  if (line != NULL)    {
    delete[] line;
    line = NULL;
  }
  if (noise3d != NULL) {
    for (uint8_t i = 0; i < pWIDTH; i++) delete[] noise3d[i];
    delete[] noise3d;
    noise3d = NULL;
  }
}

// ********************** ОГОНЬ-2 (КАМИН) *********************
void fire2Routine() {
  if (loadingFlag) {
    loadingFlag = false;
    if (noise3d == NULL) {
      noise3d = new uint8_t*[pWIDTH];
      for (uint8_t i = 0; i < pWIDTH; i++) {
        noise3d[i] = new uint8_t [pHEIGHT];
      }
    }
  }
  // Если совсем задержки нет - матрица мерцает от постоянного обновления
  delay(5);
  static uint8_t FIRE_BASE = pHEIGHT / 6 > 6 ? 6 : pHEIGHT / 6 + 1;
  // COOLING: How much does the air cool as it rises?
  // Less cooling = taller flames.  More cooling = shorter flames.
  uint8_t cooling = map8(getEffectSpeedValue(MC_FIRE2), 70, 100);
  // SPARKING: What chance (out of 255) is there that a new spark will be lit?
  // Higher chance = more roaring fire.  Lower chance = more flickery fire.
  uint8_t sparking = map8(getEffectScaleParamValue(MC_FIRE2), 90, 150);
  // SMOOTHING; How much blending should be done between frames
  // Lower = more blending and smoother flames. Higher = less blending and flickery flames
  const uint8_t fireSmoothing = 80;
  // Add entropy to random number generator; we use a lot of it.
  random16_add_entropy(random(256));
  uint8_t effectBrightness = map8(getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode)), 32, 128);
  // Loop for each column individually
  for (uint8_t x = 0; x < pWIDTH; x++) {
    // Step 1.  Cool down every cell a little
    for (uint8_t i = 0; i < pHEIGHT; i++) {
      noise3d[x][i] = qsub8(noise3d[x][i], random(0, ((cooling * 10) / pHEIGHT) + 2));
    }
    // Step 2.  Heat from  cell drifts 'up' and diffuses a little
    for (uint8_t k = pHEIGHT; k > 1; k--) {
      noise3d[x][wrapY(k)] = (noise3d[x][k - 1] + noise3d[x][wrapY(k - 2)] + noise3d[x][wrapY(k - 2)]) / 3;
    }
    // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
    if (random8() < sparking) {
      uint8_t j = random8(FIRE_BASE);
      noise3d[x][j] = qadd8(noise3d[x][j], random(160, 255));
    }
    // Step 4.  Map from heat cells to LED colors
    // Blend new data with previous frame. Average data between neighbouring pixels
    for (uint8_t y = 0; y < pHEIGHT; y++)
      nblend(leds[XY(x, y)], ColorFromPalette(HeatColors_p, ((noise3d[x][y] * 0.7) + (noise3d[wrapX(x + 1)][y] * 0.3)), effectBrightness), fireSmoothing);
  }
}
void fire2RoutineRelease() {
  if (noise3d != NULL) {
    for (uint8_t i = 0; i < pWIDTH; i++) delete[] noise3d[i];
    delete[] noise3d;
    noise3d = NULL;
  }
}

// ************************** СТРЕЛКИ *************************
int8_t   arrow_x[4], arrow_y[4], stop_x[4], stop_y[4];
uint8_t  arrow_direction;            // 0x01 - слева направо; 0x02 - снизу вверх; 0х04 - справа налево; 0х08 - сверху вниз
uint8_t  arrow_mode, arrow_mode_orig;// 0 - по очереди все варианты
// 1 - по очереди от края до края экрана;
// 2 - одновременно по горизонтали навстречу к ентру, затем одновременно по вертикали навстречу к центру
// 3 - одновременно все к центру
// 4 - по два (горизонталь / вертикаль) все от своего края к противоположному, стрелки смещены от центра на 1/3
// 5 - одновременно все от своего края к противоположному, стрелки смещены от центра на 1/3
bool     arrow_complete, arrow_change_mode;
uint8_t  arrow_hue[4];
uint8_t  arrow_play_mode_count[6];        // Сколько раз проигрывать полностью каждый режим если вариант 0 - текущий счетчик
uint8_t  arrow_play_mode_count_orig[6];   // Сколько раз проигрывать полностью каждый режим если вариант 0 - исходные настройки
void arrowsRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    FastLED.clear();
    arrow_complete = false;
    arrow_mode_orig = (specialTextEffectParam >= 0) ? specialTextEffectParam : getEffectScaleParamValue2(MC_ARROWS);
    arrow_mode = (arrow_mode_orig == 0 || arrow_mode_orig > 5) ? random8(1, 5) : arrow_mode_orig;
    Serial.print(arrow_mode);
    arrow_play_mode_count_orig[0] = 0;
    arrow_play_mode_count_orig[1] = 4;  // 4 фазы - все стрелки показаны по кругу один раз - переходить к следующему ->
    arrow_play_mode_count_orig[2] = 4;  // 2 фазы - гориз к центру (1), затем верт к центру (2) - обе фазы повторить по 2 раза -> 4
    arrow_play_mode_count_orig[3] = 4;  // 1 фаза - все к центру (1) повторить по 4 раза -> 4
    arrow_play_mode_count_orig[4] = 4;  // 2 фазы - гориз к центру (1), затем верт к центру (2) - обе фазы повторить по 2 раза -> 4
    arrow_play_mode_count_orig[5] = 4;  // 1 фаза - все сразу (1) повторить по 4 раза -> 4
    for (uint8_t i = 0; i < 6; i++) {
      arrow_play_mode_count[i] = arrow_play_mode_count_orig[i];
    }
    arrowSetupForMode(arrow_mode, true);
  }
  uint8_t effectBrightness = map8(getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode)), 32, 255);
  fader(65);
  CHSV color;
  // движение стрелки - cлева направо
  if ((arrow_direction & 0x01) > 0) {
    color = CHSV(arrow_hue[0], 255, effectBrightness);
    for (int8_t x = 0; x <= 4; x++) {
      for (int8_t y = 0; y <= x; y++) {
        if (arrow_x[0] - x >= 0 && arrow_x[0] - x <= stop_x[0]) {
          CHSV clr = (x < 4 || (x == 4 && y < 2)) ? color : CHSV(0, 0, 0);
          drawPixelXY(arrow_x[0] - x, arrow_y[0] - y, clr);
          drawPixelXY(arrow_x[0] - x, arrow_y[0] + y, clr);
        }
      }
    }
    arrow_x[0]++;
  }
  // движение стрелки - cнизу вверх
  if ((arrow_direction & 0x02) > 0) {
    color = CHSV(arrow_hue[1], 255, effectBrightness);
    for (int8_t y = 0; y <= 4; y++) {
      for (int8_t x = 0; x <= y; x++) {
        if (arrow_y[1] - y >= 0 && arrow_y[1] - y <= stop_y[1]) {
          CHSV clr = (y < 4 || (y == 4 && x < 2)) ? color : CHSV(0, 0, 0);
          drawPixelXY(arrow_x[1] - x, arrow_y[1] - y, clr);
          drawPixelXY(arrow_x[1] + x, arrow_y[1] - y, clr);
        }
      }
    }
    arrow_y[1]++;
  }
  // движение стрелки - cправа налево
  if ((arrow_direction & 0x04) > 0) {
    color = CHSV(arrow_hue[2], 255, effectBrightness);
    for (int8_t x = 0; x <= 4; x++) {
      for (int8_t y = 0; y <= x; y++) {
        if (arrow_x[2] + x >= stop_x[2] && arrow_x[2] + x < pWIDTH) {
          CHSV clr = (x < 4 || (x == 4 && y < 2)) ? color : CHSV(0, 0, 0);
          drawPixelXY(arrow_x[2] + x, arrow_y[2] - y, clr);
          drawPixelXY(arrow_x[2] + x, arrow_y[2] + y, clr);
        }
      }
    }
    arrow_x[2]--;
  }
  // движение стрелки - cверху вниз
  if ((arrow_direction & 0x08) > 0) {
    color = CHSV(arrow_hue[3], 255, effectBrightness);
    for (int8_t y = 0; y <= 4; y++) {
      for (int8_t x = 0; x <= y; x++) {
        if (arrow_y[3] + y >= stop_y[3] && arrow_y[3] + y < pHEIGHT) {
          CHSV clr = (y < 4 || (y == 4 && x < 2)) ? color : CHSV(0, 0, 0);
          drawPixelXY(arrow_x[3] - x, arrow_y[3] + y, clr);
          drawPixelXY(arrow_x[3] + x, arrow_y[3] + y, clr);
        }
      }
    }
    arrow_y[3]--;
  }
  // Проверка завершения движения стрелки, переход к следующей фазе или режиму
  switch (arrow_mode) {
    case 1:
      // Последовательно - слева-направо -> снизу вверх -> справа налево -> сверху вниз и далее по циклу
      // В каждый сомент времени сктивна только одна стрелка, если она дошла до края - переключиться на следующую и задать ее начальные координаты
      arrow_complete = false;
      switch (arrow_direction) {
        case 1: arrow_complete = arrow_x[0] > stop_x[0]; break;
        case 2: arrow_complete = arrow_y[1] > stop_y[1]; break;
        case 4: arrow_complete = arrow_x[2] < stop_x[2]; break;
        case 8: arrow_complete = arrow_y[3] < stop_y[3]; break;
      }
      arrow_change_mode = false;
      if (arrow_complete) {
        arrow_direction = (arrow_direction << 1) & 0x0F;
        if (arrow_direction == 0) arrow_direction = 1;
        if (arrow_mode_orig == 0) {
          arrow_play_mode_count[1]--;
          if (arrow_play_mode_count[1] == 0) {
            arrow_play_mode_count[1] = arrow_play_mode_count_orig[1];
            arrow_mode = random8(1, 5);
            arrow_change_mode = true;
          }
        }
        arrowSetupForMode(arrow_mode, arrow_change_mode);
      }
      break;
    case 2:
      // Одновременно горизонтальные навстречу до половины экрана
      // Затем одновременно вертикальные до половины экрана. Далее - повторять
      arrow_complete = false;
      switch (arrow_direction) {
        case  5: arrow_complete = arrow_x[0] > stop_x[0]; break;   // Стрелка слева и справа встречаются в центре одновременно - проверять только стрелку слева
        case 10: arrow_complete = arrow_y[1] > stop_y[1]; break;   // Стрелка снизу и сверху встречаются в центре одновременно - проверять только стрелку снизу
      }
      arrow_change_mode = false;
      if (arrow_complete) {
        arrow_direction = arrow_direction == 5 ? 10 : 5;
        if (arrow_mode_orig == 0) {
          arrow_play_mode_count[2]--;
          if (arrow_play_mode_count[2] == 0) {
            arrow_play_mode_count[2] = arrow_play_mode_count_orig[2];
            arrow_mode = random8(1, 5);
            arrow_change_mode = true;
          }
        }
        arrowSetupForMode(arrow_mode, arrow_change_mode);
      }
      break;
    case 3:
      // Одновременно со всех сторон к центру
      // Завершение кадра режима - когда все стрелки собрались в центре.
      // Проверять стрелки по самой длинной стороне
      if (pWIDTH >= pHEIGHT)
        arrow_complete = arrow_x[0] > stop_x[0];
      else
        arrow_complete = arrow_y[1] > stop_y[1];
      arrow_change_mode = false;
      if (arrow_complete) {
        if (arrow_mode_orig == 0) {
          arrow_play_mode_count[3]--;
          if (arrow_play_mode_count[3] == 0) {
            arrow_play_mode_count[3] = arrow_play_mode_count_orig[3];
            arrow_mode = random8(1, 5);
            arrow_change_mode = true;
          }
        }
        arrowSetupForMode(arrow_mode, arrow_change_mode);
      }
      break;
    case 4:
      // Одновременно слева/справа от края до края со смещением горизонтальной оси на 1/3 высоты, далее
      // одновременно снизу/сверху от края до края со смещением вертикальной оси на 1/3 ширины
      // Завершение кадра режима - когда все стрелки собрались в центре.
      // Проверять стрелки по самой длинной стороне
      switch (arrow_direction) {
        case  5: arrow_complete = arrow_x[0] > stop_x[0]; break;   // Стрелка слева и справа движутся и достигают края одновременно - проверять только стрелку слева
        case 10: arrow_complete = arrow_y[1] > stop_y[1]; break;   // Стрелка снизу и сверху движутся и достигают края одновременно - проверять только стрелку снизу
      }
      arrow_change_mode = false;
      if (arrow_complete) {
        arrow_direction = arrow_direction == 5 ? 10 : 5;
        if (arrow_mode_orig == 0) {
          arrow_play_mode_count[4]--;
          if (arrow_play_mode_count[4] == 0) {
            arrow_play_mode_count[4] = arrow_play_mode_count_orig[4];
            arrow_mode = random8(1, 5);
            arrow_change_mode = true;
          }
        }
        arrowSetupForMode(arrow_mode, arrow_change_mode);
      }
      break;
    case 5:
      // Одновременно со всех сторон от края до края со смещением горизонтальной оси на 1/3 высоты, далее
      // Проверять стрелки по самой длинной стороне
      if (pWIDTH >= pHEIGHT)
        arrow_complete = arrow_x[0] > stop_x[0];
      else
        arrow_complete = arrow_y[1] > stop_y[1];
      arrow_change_mode = false;
      if (arrow_complete) {
        if (arrow_mode_orig == 0) {
          arrow_play_mode_count[5]--;
          if (arrow_play_mode_count[5] == 0) {
            arrow_play_mode_count[5] = arrow_play_mode_count_orig[5];
            arrow_mode = random8(1, 5);
            arrow_change_mode = true;
          }
        }
        arrowSetupForMode(arrow_mode, arrow_change_mode);
      }
      break;
  }
}
void arrowSetupForMode(uint8_t mode, bool change) {
  switch (mode) {
    case 1:
      if (change) arrow_direction = 1;
      arrowSetup_mode1();    // От края матрицы к краю, по центру гориз и верт
      break;
    case 2:
      if (change) arrow_direction = 5;
      arrowSetup_mode2();    // По центру матрицы (гориз / верт) - ограничение - центр матрицы
      break;
    case 3:
      if (change) arrow_direction = 15;
      arrowSetup_mode2();    // как и в режиме 2 - по центру матрицы (гориз / верт) - ограничение - центр матрицы
      break;
    case 4:
      if (change) arrow_direction = 5;
      arrowSetup_mode4();    // От края матрицы к краю, верт / гориз
      break;
    case 5:
      if (change) arrow_direction = 15;
      arrowSetup_mode4();    // как и в режиме 4 от края матрицы к краю, на 1/3
      break;
  }
}
void arrowSetup_mode1() {
  // Слева направо
  if ((arrow_direction & 0x01) > 0) {
    arrow_hue[0] = random8();
    arrow_x[0] = 0;
    arrow_y[0] = pHEIGHT / 2;
    stop_x [0] = pWIDTH + 7;      // скрывается за экраном на 7 пикселей
    stop_y [0] = 0;              // неприменимо
  }
  // снизу вверх
  if ((arrow_direction & 0x02) > 0) {
    arrow_hue[1] = random8();
    arrow_y[1] = 0;
    arrow_x[1] = pWIDTH / 2;
    stop_y [1] = pHEIGHT + 7;     // скрывается за экраном на 7 пикселей
    stop_x [1] = 0;              // неприменимо
  }
  // справа налево
  if ((arrow_direction & 0x04) > 0) {
    arrow_hue[2] = random8();
    arrow_x[2] = pWIDTH - 1;
    arrow_y[2] = pHEIGHT / 2;
    stop_x [2] = -7;             // скрывается за экраном на 7 пикселей
    stop_y [2] = 0;              // неприменимо
  }
  // сверху вниз
  if ((arrow_direction & 0x08) > 0) {
    arrow_hue[3] = random8();
    arrow_y[3] = pHEIGHT - 1;
    arrow_x[3] = pWIDTH / 2;
    stop_y [3] = -7;             // скрывается за экраном на 7 пикселей
    stop_x [3] = 0;              // неприменимо
  }
}
void arrowSetup_mode2() {
  // Слева направо до половины экрана
  if ((arrow_direction & 0x01) > 0) {
    arrow_hue[0] = random8();
    arrow_x[0] = 0;
    arrow_y[0] = pHEIGHT / 2;
    stop_x [0] = pWIDTH / 2 - 1;  // до центра экрана
    stop_y [0] = 0;              // неприменимо
  }
  // снизу вверх до половины экрана
  if ((arrow_direction & 0x02) > 0) {
    arrow_hue[1] = random8();
    arrow_y[1] = 0;
    arrow_x[1] = pWIDTH / 2;
    stop_y [1] = pHEIGHT / 2 - 1; // до центра экрана
    stop_x [1] = 0;              // неприменимо
  }
  // справа налево до половины экрана
  if ((arrow_direction & 0x04) > 0) {
    arrow_hue[2] = random8();
    arrow_x[2] = pWIDTH - 1;
    arrow_y[2] = pHEIGHT / 2;
    stop_x [2] = pWIDTH / 2;      // до центра экрана
    stop_y [2] = 0;              // неприменимо
  }
  // сверху вниз до половины экрана
  if ((arrow_direction & 0x08) > 0) {
    arrow_hue[3] = random8();
    arrow_y[3] = pHEIGHT - 1;
    arrow_x[3] = pWIDTH / 2;
    stop_y [3] = pHEIGHT / 2;     // до центра экрана
    stop_x [3] = 0;              // неприменимо
  }
}
void arrowSetup_mode4() {
  // Слева направо
  if ((arrow_direction & 0x01) > 0) {
    arrow_hue[0] = random8();
    arrow_x[0] = 0;
    arrow_y[0] = (pHEIGHT / 3) * 2;
    stop_x [0] = pWIDTH + 7;      // скрывается за экраном на 7 пикселей
    stop_y [0] = 0;              // неприменимо
  }
  // снизу вверх
  if ((arrow_direction & 0x02) > 0) {
    arrow_hue[1] = random8();
    arrow_y[1] = 0;
    arrow_x[1] = (pWIDTH / 3) * 2;
    stop_y [1] = pHEIGHT + 7;     // скрывается за экраном на 7 пикселей
    stop_x [1] = 0;              // неприменимо
  }
  // справа налево
  if ((arrow_direction & 0x04) > 0) {
    arrow_hue[2] = random8();
    arrow_x[2] = pWIDTH - 1;
    arrow_y[2] = pHEIGHT / 3;
    stop_x [2] = -7;             // скрывается за экраном на 7 пикселей
    stop_y [2] = 0;              // неприменимо
  }
  // сверху вниз
  if ((arrow_direction & 0x08) > 0) {
    arrow_hue[3] = random8();
    arrow_y[3] = pHEIGHT - 1;
    arrow_x[3] = pWIDTH / 3;
    stop_y [3] = -7;             // скрывается за экраном на 7 пикселей
    stop_x [3] = 0;              // неприменимо
  }
}

// ******************************** СИНУСОИДЫ *******************************
#define WAVES_AMOUNT_MAX 4  //максимальное количество синусоид
#define DEG_TO_RAD 0.017453
int8_t  WAVES_AMOUNT;
int t;
byte w[WAVES_AMOUNT_MAX];
byte phi[WAVES_AMOUNT_MAX];
byte A[WAVES_AMOUNT_MAX];
uint8_t waveColors[WAVES_AMOUNT_MAX];
void sinwavesRoutine() {
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  if (loadingFlag) {
    loadingFlag = false;
    WAVES_AMOUNT = map8(getEffectScaleParamValue(MC_SINWAVES), 1, 4);
    for (byte j = 0; j < WAVES_AMOUNT; j++) {
      // забиваем случайными данными
      w[j] = random(17, 25);
      phi[j] = random(0, 360);
      A[j] = pHEIGHT / 2 * random(4, 11) / 10;
      waveColors[j] = random(0, 9) * 28;
    }
  }
  uint8_t timerperiod = 10000;
  uint8_t wavetimer;
  wavetimer = millis();
  if (millis() - wavetimer >= timerperiod) {
    wavetimer = millis();
    // сдвигаем все пиксели вправо
    for (int i = pWIDTH - 1; i > 0; i--)
      for (int j = 0; j < pHEIGHT; j++)
        drawPixelXY(i, j, getPixColorXY(i - 1, j));
    // увеличиваем "угол"
    t++;
    if (t > 360) t = 0;
    // заливаем чёрным левую линию
    for (byte i = 0; i < pHEIGHT; i++) {
      drawPixelXY(0, i, 0x000000);
    }
    // генерируем позицию точки через синус
    for (byte j = 0; j < WAVES_AMOUNT; j++) {
      float value = pHEIGHT / 2 + (float)A[j] * sin((float)w[j] * t * DEG_TO_RAD + (float)phi[j] * DEG_TO_RAD);
      leds[getPixelNumber(0, (byte)value)] = CHSV(waveColors[j], 255, effectBrightness);
    }
  }
}

// ------------- Nexus --------------
// (c) kostyamat
// https://github.com/DmytroKorniienko/FireLamp_JeeUI/blob/master/src/effects.cpp
uint8_t wormscount;
uint8_t nexus_type = 0;
void nexusRoutine() {
  if (loadingFlag)
  {
    loadingFlag = false;
    speedfactor = fmap(modes[currentMode].Speed, 1, 255, 0.1, .33);//(float)modes[currentMode].Speed / 555.0f + 0.001f;
    wormscount = map8(getEffectScaleParamValue(MC_NEXUS), 1, 2 * pWIDTH); //количество червячков, которое можно задавать от 1 до 2*pWIDTH через ползунок варианта
    enlargedObjectNUM = (modes[currentMode].Scale - 1U) / 99.0 * (wormscount - 1U) + 1U;
    if (enlargedObjectNUM > wormscount) enlargedObjectNUM = wormscount;
    for (uint8_t i = 0; i < enlargedObjectNUM; i++) {
      enlargedObjectTime[i] = 0;
      trackingObjectPosX[i] = random8(pWIDTH);
      trackingObjectPosY[i] = random8(pHEIGHT);
      trackingObjectSpeedX[i] = (255. + random8()) / 255.;
      trackingObjectSpeedY[i] = 0;
      trackingObjectHue[i] = random8();
      trackingObjectState[i] = random8(4);//     B00           направление головы змейки
      // B10     B11
      //     B01
    }
    nexus_type = (specialTextEffectParam >= 0) ? specialTextEffectParam : getEffectScaleParamValue2(MC_NEXUS);
    // Если авто - генерировать один из типов - Вариант 1, Вариант 2
    if (nexus_type == 0 || nexus_type > 2) {
      nexus_type = random8(1, 3);
    }
  }
  switch (nexus_type) {
    case 1:  nexus(); break;
    default: snake(); break;
  }
}

void nexusReset(uint8_t i) {
  trackingObjectHue[i] = random8();
  trackingObjectState[i] = random8(4);
  trackingObjectSpeedX[i] = (float)random8(5, 11) / 70 + speedfactor; // делаем частицам немного разное ускорение и сразу пересчитываем под общую скорость
  switch (trackingObjectState[i]) {
    case B01:
      trackingObjectPosY[i] = pHEIGHT;
      trackingObjectPosX[i] = random8(pWIDTH);
      break;
    case B00:
      trackingObjectPosY[i] = -1;
      trackingObjectPosX[i] = random8(pWIDTH);
      break;
    case B10:
      trackingObjectPosX[i] = pWIDTH;
      trackingObjectPosY[i] = random8(pHEIGHT);
      break;
    case B11:
      trackingObjectPosX[i] = -1;
      trackingObjectPosY[i] = random8(pHEIGHT);
      break;
  }
}

void nexus() {
  deltaValue = 255U - map(modes[currentMode].Speed, 1, 255, 11, 33);
  dimAll(deltaValue);
  for (uint8_t i = 0; i < enlargedObjectNUM; i++) {
    switch (trackingObjectState[i]) {
      case B01:
        trackingObjectPosY[i] -= trackingObjectSpeedX[i];
        if (trackingObjectPosY[i] <= -1)
          nexusReset(i);
        break;
      case B00:
        trackingObjectPosY[i] += trackingObjectSpeedX[i];
        if (trackingObjectPosY[i] >= pHEIGHT)
          nexusReset(i);
        break;
      case B10:
        trackingObjectPosX[i] -= trackingObjectSpeedX[i];
        if (trackingObjectPosX[i] <= -1)
          nexusReset(i);
        break;
      case B11:
        trackingObjectPosX[i] += trackingObjectSpeedX[i];
        if (trackingObjectPosX[i] >= pWIDTH)
          nexusReset(i);
        break;
    }
    drawPixelXYF(trackingObjectPosX[i], trackingObjectPosY[i],  CHSV(trackingObjectHue[i], 255U, 255));
  }
}

#define SNAKES_LENGTH (8U)
void snake() {
  FastLED.clear();
  int8_t dx, dy;
  for (uint8_t i = 0; i < enlargedObjectNUM; i++) {
    trackingObjectSpeedY[i] += trackingObjectSpeedX[i] * speedfactor;
    if (trackingObjectSpeedY[i] >= 1)
    {
      trackingObjectSpeedY[i] = trackingObjectSpeedY[i] - (int)trackingObjectSpeedY[i];
      if (random8(9U) == 0U) // вероятность поворота
        if (random8(2U)) { // <- поворот налево
          enlargedObjectTime[i] = (enlargedObjectTime[i] << 2) | B01; // младший бит = поворот
          switch (trackingObjectState[i]) {
            case B10:
              trackingObjectState[i] = B01;
              if (trackingObjectPosY[i] == 0U)
                trackingObjectPosY[i] = pHEIGHT - 1U;
              else
                trackingObjectPosY[i]--;
              break;
            case B11:
              trackingObjectState[i] = B00;
              if (trackingObjectPosY[i] >= pHEIGHT - 1U)
                trackingObjectPosY[i] = 0U;
              else
                trackingObjectPosY[i]++;
              break;
            case B00:
              trackingObjectState[i] = B10;
              if (trackingObjectPosX[i] == 0U)
                trackingObjectPosX[i] = pWIDTH - 1U;
              else
                trackingObjectPosX[i]--;
              break;
            case B01:
              trackingObjectState[i] = B11;
              if (trackingObjectPosX[i] >= pWIDTH - 1U)
                trackingObjectPosX[i] = 0U;
              else
                trackingObjectPosX[i]++;
              break;
          }
        }
        else { // -> поворот направо
          enlargedObjectTime[i] = (enlargedObjectTime[i] << 2) | B11; // младший бит = поворот, старший = направо
          switch (trackingObjectState[i]) {
            case B11:
              trackingObjectState[i] = B01;
              if (trackingObjectPosY[i] == 0U)
                trackingObjectPosY[i] = pHEIGHT - 1U;
              else
                trackingObjectPosY[i]--;
              break;
            case B10:
              trackingObjectState[i] = B00;
              if (trackingObjectPosY[i] >= pHEIGHT - 1U)
                trackingObjectPosY[i] = 0U;
              else
                trackingObjectPosY[i]++;
              break;
            case B01:
              trackingObjectState[i] = B10;
              if (trackingObjectPosX[i] == 0U)
                trackingObjectPosX[i] = pWIDTH - 1U;
              else
                trackingObjectPosX[i]--;
              break;
            case B00:
              trackingObjectState[i] = B11;
              if (trackingObjectPosX[i] >= pWIDTH - 1U)
                trackingObjectPosX[i] = 0U;
              else
                trackingObjectPosX[i]++;
              break;
          }
        }
      else { // двигаем без поворота
        enlargedObjectTime[i] = (enlargedObjectTime[i] << 2);
        switch (trackingObjectState[i]) {
          case B01:
            if (trackingObjectPosY[i] == 0U)
              trackingObjectPosY[i] = pHEIGHT - 1U;
            else
              trackingObjectPosY[i]--;
            break;
          case B00:
            if (trackingObjectPosY[i] >= pHEIGHT - 1U)
              trackingObjectPosY[i] = 0U;
            else
              trackingObjectPosY[i]++;
            break;
          case B10:
            if (trackingObjectPosX[i] == 0U)
              trackingObjectPosX[i] = pWIDTH - 1U;
            else
              trackingObjectPosX[i]--;
            break;
          case B11:
            if (trackingObjectPosX[i] >= pWIDTH - 1U)
              trackingObjectPosX[i] = 0U;
            else
              trackingObjectPosX[i]++;
            break;
        }
      }
    }
    switch (trackingObjectState[i]) {
      case B01:
        dy = 1;
        dx = 0;
        break;
      case B00:
        dy = -1;
        dx = 0;
        break;
      case B10:
        dy = 0;
        dx = 1;
        break;
      case B11:
        dy = 0;
        dx = -1;
        break;
    }
    long temp = enlargedObjectTime[i];
    uint8_t x = trackingObjectPosX[i];
    uint8_t y = trackingObjectPosY[i];
    leds[XY(x, y)] += CHSV(trackingObjectHue[i], 255U, trackingObjectSpeedY[i] * 255); // тут рисуется голова
    for (uint8_t m = 0; m < SNAKES_LENGTH; m++) { // 16 бит распаковываем, 14 ещё остаётся без дела в запасе, 2 на хвостик
      x = (pWIDTH + x + dx) % pWIDTH;
      y = (pHEIGHT + y + dy) % pHEIGHT;
      leds[XY(x, y)] += CHSV(trackingObjectHue[i] + (m + trackingObjectSpeedY[i]) * 4U, 255U, 255U); // тут рисуется тело
      if (temp & B01) { // младший бит = поворот, старший = направо
        temp = temp >> 1;
        if (temp & B01) { // старший бит = направо
          if (dx == 0) {
            dx = 0 - dy;
            dy = 0;
          }
          else {
            dy = dx;
            dx = 0;
          }
        }
        else { // иначе налево
          if (dx == 0) {
            dx = dy;
            dy = 0;
          }
          else {
            dy = 0 - dx;
            dx = 0;
          }
        }
        temp = temp >> 1;
      }
      else { // если без поворота
        temp = temp >> 2;
      }
    }
    x = (pWIDTH + x + dx) % pWIDTH;
    y = (pHEIGHT + y + dy) % pHEIGHT;
    leds[XY(x, y)] += CHSV(trackingObjectHue[i] + (SNAKES_LENGTH + trackingObjectSpeedY[i]) * 4U, 255U, (1 - trackingObjectSpeedY[i]) * 255); // хвостик
  }
}

// ***** SINUSOID3 / СИНУСОИД3 ***** + попытка повторить все остальные версии
/*
  Sinusoid3 by Stefan Petrick (mod by Palpalych for GyverLamp 27/02/2020)
  read more about the concept: https://www.youtube.com/watch?v=mubH-w_gwdA
  https://gist.github.com/StefanPetrick/dc666c1b4851d5fb8139b73719b70149
*/
// v1.7.0 - Updating for GuverLamp v1.7 by PalPalych 12.03.2020
// 2nd upd by Stepko https://wokwi.com/arduino/projects/287675911209222664
// 3rd proper by SottNick
uint8_t sinusoid_type = 0;
uint8_t sinusoid_num;
uint32_t sinus_timer;
void Sinusoid3Routine() {
  if (loadingFlag) {
    loadingFlag = false;
    sinusoid_type = (specialTextEffectParam >= 0) ? specialTextEffectParam : getEffectScaleParamValue2(MC_SINUSOID3);
    if (sinusoid_type == 0 || sinusoid_type == 9) sinusoid_num = random8(1, 9); //Если Случайный выбор или Авто, задать произвольный вариант (в Авто от него начинается отсчет)
    else if (sinusoid_type > 0 || sinusoid_type < 9) sinusoid_num = sinusoid_type;  //Если что-то из вариантов 1-8, берем только это значение
    FastLED.clear();  // очистить
    emitterX = pWIDTH * 0.5;
    emitterY = pHEIGHT * 0.5;
    speedfactor = 0.00145 * modes[currentMode].Speed + 0.015;
  }
  float e_s3_size = 3. * modes[currentMode].Scale / 100.0 + 2;    // amplitude of the curves
  uint32_t time_shift = millis() & 0xFFFFFF; // overflow protection
  uint16_t _scale = (((modes[currentMode].Scale - 1U) % 9U) * 10U + 80U) << 7U; // = fmap(scale, 1, 255, 0.1, 3);
  float _scale2 = (float)((modes[currentMode].Scale - 1U) % 9U) * 0.2 + 0.4; // для спиралей на sinf
  uint16_t _scale3 = ((modes[currentMode].Scale - 1U) % 9U) * 1638U + 3276U; // для спиралей на sin16
  CRGB color;
  float center1x = float(e_s3_size * sin16(speedfactor * 72.0874 * time_shift)) / 0x7FFF - emitterX;
  float center1y = float(e_s3_size * cos16(speedfactor * 98.301  * time_shift)) / 0x7FFF - emitterY;
  float center2x = float(e_s3_size * sin16(speedfactor * 68.8107 * time_shift)) / 0x7FFF - emitterX;
  float center2y = float(e_s3_size * cos16(speedfactor * 65.534  * time_shift)) / 0x7FFF - emitterY;
  float center3x = float(e_s3_size * sin16(speedfactor * 134.3447 * time_shift)) / 0x7FFF - emitterX;
  float center3y = float(e_s3_size * cos16(speedfactor * 170.3884 * time_shift)) / 0x7FFF - emitterY;
  if (sinusoid_type == 9) {  //автоперебор вариантов, если выбран вариант Авто, дается 10 сек на эффект
    if (millis() - sinus_timer > 10000) {
      sinus_timer = millis();
      sinusoid_num++;
      if (sinusoid_num > 8) sinusoid_num = 1;
    }
  }
  switch (sinusoid_num) {
    case 1:  SinusoidI(center1x, center2x, center1y, center2y, color, _scale);                                    break;
    case 2:  SinusoidII(center1x, center2x, center3x, center1y, center2y, center3y, color, _scale);               break;
    case 3:  SinusoidIII(center1x, center2x, center3x, center1y, center2y, center3y, color, _scale, time_shift);  break;
    case 4:  SinusoidIV(center1x, center1y, color, _scale, time_shift);                                           break;
    case 5:  SinusoidV(center1x, center1y, color, _scale, time_shift);                                            break;
    case 6:  SinusoidVI(center1x, center2x, center3x, center1y, center2y, center3y, color, _scale2, _scale3);     break;
    case 7:  SinusoidVII(center1x, center2x, center3x, center1y, center2y, center3y, color, _scale3);             break;
    case 8:  SinusoidVIII(center1x, center3x, center1y, center3y, color, _scale);                                 break;
  }
}

// распихал код эффектов по отдельным функциям
//Sinusoid I
void SinusoidI(float center1x, float center2x, float center1y, float center2y, CRGB color, uint16_t _scale)
{
  for (uint8_t y = 0; y < pHEIGHT; y++) {
    for (uint8_t x = 0; x < pWIDTH; x++) {
      float cx = x + center1x;
      float cy = y + center1y;
      uint8_t v = 127 * (1 + float(sin16(_scale * SQRT_VARIANT(cx * cx + cy * cy))) / 0x7FFF);
      color.r = v;
      cx = x + center2x;
      cy = y + center2y;
      v = 127 * (1 + float(sin16(_scale * SQRT_VARIANT(cx * cx + cy * cy))) / 0x7FFF);
      color.g = (v - (min(v, color.r) >> 1)) >> 1;
      color.b = color.g >> 2;
      color.r = max(v, color.r);
      drawPixelXY(x, y, color);
    }
  }
}

//Sinusoid II
void SinusoidII(float center1x, float center2x, float center3x, float center1y, float center2y, float center3y, CRGB color, uint16_t _scale)
{
  for (uint8_t y = 0; y < pHEIGHT; y++) {
    for (uint8_t x = 0; x < pWIDTH; x++) {
      float cx = x + center1x;
      float cy = y + center1y;
      int8_t v = 127 * (1 + float(sin16(_scale * SQRT_VARIANT(cx * cx + cy * cy))) / 0x7FFF);
      color.r = v;
      cx = x + center2x;
      cy = y + center2y;
      v = 127 * (1 + float(sin16(_scale * SQRT_VARIANT(cx * cx + cy * cy))) / 0x7FFF);
      color.b = v;
      cx = x + center3x;
      cy = y + center3y;
      v = 127 * (1 + float(sin16(_scale * SQRT_VARIANT(cx * cx + cy * cy))) / 0x7FFF);
      color.g = v;
      drawPixelXY(x, y, color);
    }
  }
}

//Sinusoid III
void SinusoidIII(float center1x, float center2x, float center3x, float center1y, float center2y, float center3y, CRGB color, uint16_t _scale, uint32_t time_shift)
{
  for (uint8_t y = 0; y < pHEIGHT; y++) {
    for (uint8_t x = 0; x < pWIDTH; x++) {
      float cx = x + center1x;
      float cy = y + center1y;
      int8_t v = 127 * (1 + float(sin16(_scale * SQRT_VARIANT(cx * cx + cy * cy) + time_shift * speedfactor * 100)) / 0x7FFF);
      color.r = ~v;
      cx = x + center2x;
      cy = y + center2y;
      v = 127 * (1 + float(sin16(_scale * SQRT_VARIANT(cx * cx + cy * cy) + time_shift * speedfactor * 100)) / 0x7FFF);
      color.g = ~v;
      cx = x + center3x;
      cy = y + center3y;
      v = 127 * (1 + float(sin16(_scale * SQRT_VARIANT(cx * cx + cy * cy) + time_shift * speedfactor * 100)) / 0x7FFF);
      color.b = ~v;
      drawPixelXY(x, y, color);
    }
  }
}

//changed by stepko
//colored sinusoid
//Sinusoid IV
void SinusoidIV(float center1x, float center1y, CRGB color, uint16_t _scale, uint32_t time_shift)
{
  for (uint8_t y = 0; y < pHEIGHT; y++) {
    for (uint8_t x = 0; x < pWIDTH; x++) {
      float cx = x + center1x;
      float cy = y + center1y;
      int8_t v = 127 * (1 + float(sin16(_scale * (beatsin16(2, 1000, 1750) / 2550.) * SQRT_VARIANT(cx * cx + cy * cy))) / 0x7FFF); // + time_shift * speedfactor * 5 // mass colors plus by SottNick
      color.r = v;
      v = 127 * (1 + float(sin16(_scale * (beatsin16(1, 570, 1050) / 2250.) * SQRT_VARIANT(((cx * cx) + (cy * cy)))  + 13 * time_shift * speedfactor)) / 0x7FFF); // вместо beatsin сперва ставил просто * 0.41
      color.b = v;
      v = 127 * (1 + float(cos16(_scale * (beatsin16(3, 1900, 2550) / 2550.) * SQRT_VARIANT(((cx * cx) + (cy * cy)))  + 41 * time_shift * speedfactor)) / 0x7FFF); // вместо beatsin сперва ставил просто * 0.53
      color.g = v;
      drawPixelXY(x, y, color);
    }
  }
}

//changed by stepko
//sinusoid in net
//Sinusoid V
void SinusoidV(float center1x, float center1y, CRGB color, uint16_t _scale, uint32_t time_shift)
{
  for (uint8_t y = 0; y < pHEIGHT; y++) {
    for (uint8_t x = 0; x < pWIDTH; x++) {
      float cx = x + center1x;
      float cy = y + center1y;
      int8_t v = 127 * (1 + float(sin16(_scale * SQRT_VARIANT(cx * cx + cy * cy) + time_shift * speedfactor * 5)) / 0x7FFF);
      color.g = ~v;
      v = 127 * (1 + float(sin16(_scale * (x + 0.005 * time_shift * speedfactor))) / 0x7FFF); // proper by SottNick
      color.b = ~v;
      v = 127 * (1 + float(sin16(_scale * (y + 0.0055 * time_shift * speedfactor))) / 0x7FFF); // proper by SottNick
      color.r = ~v;
      drawPixelXY(x, y, color);
    }
  }
}

//changed by stepko
//spiral
//Sinusoid VI
void SinusoidVI(float center1x, float center2x, float center3x, float center1y, float center2y, float center3y, CRGB color, uint16_t _scale2, uint16_t _scale3)
{
  for (uint8_t y = 0; y < pHEIGHT; y++) {
    for (uint8_t x = 0; x < pWIDTH; x++) {
      float cx = x + center1x;
      float cy = y + center1y;
      uint8_t v = 127 * (1 + sinf (3 * atan2(cy, cx)  + _scale2 *  hypot(cy, cx))); // proper by SottNick
      //вырезаем центр спирали - proper by SottNick
      float d = SQRT_VARIANT(cx * cx + cy * cy) / 10.; // 10 - это радиус вырезаемого центра в каких-то условных величинах. 10 = 1 пиксель, 20 = 2 пикселя. как-то так
      if (d < 0.06) d = 0.06;
      if (d < 1) // просто для ускорения расчётов
        v = constrain(v - int16_t(1 / d / d), 0, 255);
      //вырезали
      color.r = v;
      cx = x + center2x;
      cy = y + center2y;
      v = 127 * (1 + sinf (3 * atan2(cy, cx)  + _scale2 *  hypot(cy, cx))); // proper by SottNick
      //вырезаем центр спирали
      d = SQRT_VARIANT(cx * cx + cy * cy) / 10.; // 10 - это радиус вырезаемого центра в каких-то условных величинах. 10 = 1 пиксель, 20 = 2 пикселя. как-то так
      if (d < 0.06) d = 0.06;
      if (d < 1) // просто для ускорения расчётов
        v = constrain(v - int16_t(1 / d / d), 0, 255);
      //вырезали
      color.b = v;
      cx = x + center3x;
      cy = y + center3y;
      v = 127 * (1 + float(sin16(atan2(cy, cx) * 31255  + _scale3 *  hypot(cy, cx))) / 0x7FFF); // proper by SottNick
      //вырезаем центр спирали
      d = SQRT_VARIANT(cx * cx + cy * cy) / 10.; // 10 - это радиус вырезаемого центра в каких-то условных величинах. 10 = 1 пиксель, 20 = 2 пикселя. как-то так
      if (d < 0.06) d = 0.06;
      if (d < 1) // просто для ускорения расчётов
        v = constrain(v - int16_t(1 / d / d), 0, 255);
      //вырезали
      color.g = v;
      drawPixelXY(x, y, color);
    }
  }
}

//variant by SottNick
//Sinusoid VII
void SinusoidVII(float center1x, float center2x, float center3x, float center1y, float center2y, float center3y, CRGB color, uint16_t _scale3)
{
  for (uint8_t y = 0; y < pHEIGHT; y++) {
    for (uint8_t x = 0; x < pWIDTH; x++) {
      float cx = x + center1x;
      float cy = y + center1y;
      uint8_t v = 127 * (1 + float(sin16(atan2(cy, cx) * 31255  + _scale3 *  hypot(cy, cx))) / 0x7FFF); // proper by SottNick
      //вырезаем центр спирали
      float d = SQRT_VARIANT(cx * cx + cy * cy) / 10.; // 10 - это радиус вырезаемого центра в каких-то условных величинах. 10 = 1 пиксель, 20 = 2 пикселя. как-то так
      if (d < 0.06) d = 0.06;
      if (d < 1) // просто для ускорения расчётов
        v = constrain(v - int16_t(1 / d / d), 0, 255);
      //вырезали
      color.g = v;
      cx = x + center3x;
      cy = y + center3y;
      v = 127 * (1 + float(sin16(atan2(cy, cx) * 31255  + _scale3 *  hypot(cy, cx))) / 0x7FFF); // proper by SottNick
      //вырезаем центр спирали
      d = SQRT_VARIANT(cx * cx + cy * cy) / 10.; // 10 - это радиус вырезаемого центра в каких-то условных величинах. 10 = 1 пиксель, 20 = 2 пикселя. как-то так
      if (d < 0.06) d = 0.06;
      if (d < 1) // просто для ускорения расчётов
        v = constrain(v - int16_t(1 / d / d), 0, 255);
      //вырезали
      color.r = v;
      drawPixelXY(x, y, color);
    }
  }
}

//Sinusoid VIII
void SinusoidVIII(float center1x, float center3x, float center1y, float center3y, CRGB color, uint16_t _scale)
{
  for (uint8_t y = 0; y < pHEIGHT; y++) {
    for (uint8_t x = 0; x < pWIDTH; x++) {
      float cx = x + center1x;
      float cy = y + center1y;
      int8_t v = 127 * (1 + float(sin16(_scale * SQRT_VARIANT(cx * cx + cy * cy))) / 0x7FFF);
      color.r = v;
      cx = x + center3x;
      cy = y + center3y;
      v = 127 * (1 + float(sin16(_scale * SQRT_VARIANT(cx * cx + cy * cy))) / 0x7FFF);
      color.b = v;
      drawPixelXY(x, y, color);
    }
  }
}

// ============= ЭФФЕКТ ФЕЯ ===============
// (c) SottNick
#define FAIRY_BEHAVIOR //типа сложное поведение

void fairyEmit(uint8_t i) {  //particlesEmit(Particle_Abstract *particle, ParticleSysConfig *g)
  if (deltaHue++ & 0x01)
    if (hue++ & 0x01)
      hue2++;//counter++;
  trackingObjectPosX[i] = boids[0].location.x;
  trackingObjectPosY[i] = boids[0].location.y;
  //хотите навставлять speedfactor? - тут не забудьте
  trackingObjectSpeedX[i] = ((float)random8() - 127.) / 512.; // random(_hVar)-_constVel; // particle->vx
  trackingObjectSpeedY[i] = SQRT_VARIANT(0.0626 - trackingObjectSpeedX[i] * trackingObjectSpeedX[i]); // SQRT_VARIANT(pow(_constVel,2)-pow(trackingObjectSpeedX[i],2)); // particle->vy зависит от particle->vx - не ошибка
  if (random8(2U)) {
    trackingObjectSpeedY[i] = -trackingObjectSpeedY[i];
  }
  trackingObjectState[i] = random8(20, 80); // random8(minLife, maxLife);// particle->ttl
  trackingObjectHue[i] = hue2;// (counter/2)%255; // particle->hue
  trackingObjectIsShift[i] = true; // particle->isAlive
}
void fairyRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    deltaValue = 10; // количество зарождающихся частиц за 1 цикл //perCycle = 1;
    enlargedObjectNUM = (modes[currentMode].Scale - 1U) / 99.0 * (trackingOBJECT_MAX_COUNT - 1U) + 1U;
    if (enlargedObjectNUM > trackingOBJECT_MAX_COUNT) enlargedObjectNUM = trackingOBJECT_MAX_COUNT;
    for (int i = 0; i < enlargedObjectNUM; i++)
      trackingObjectIsShift[i] = false; // particle->isAlive
    // лень было придумывать алгоритм для траектории феи, поэтому это будет нулевой "бойд" из эффекта Притяжение
    boids[0] = Boid(random8(pWIDTH), random8(pHEIGHT));//pWIDTH - 1, pHEIGHT - 1);
    boids[0].mass = 0.5; // random(0.1, 2); // сюда можно поставить регулятор разлёта. чем меньше число, тем дальше от центра будет вылет
    boids[0].velocity.x = ((float) random8(46U, 100U)) / 500.0;
    if (random8(2U)) boids[0].velocity.x = -boids[0].velocity.x;
    boids[0].velocity.y = 0;
    hue = random8();//boids[0].colorIndex =
#ifdef FAIRY_BEHAVIOR
    deltaHue2 = 1U;
#endif;
  }
  step = deltaValue; //счётчик количества частиц в очереди на зарождение в этом цикле
#ifdef FAIRY_BEHAVIOR
  if (!deltaHue && deltaHue2 && fabs(boids[0].velocity.x) + fabs(boids[0].velocity.y) < 0.15) {
    deltaHue2 = 0U;
    boids[1].velocity.x = ((float)random8() + 255.) / 4080.;
    boids[1].velocity.y = ((float)random8() + 255.) / 2040.;
    if (boids[0].location.x > pWIDTH * 0.5) boids[1].velocity.x = -boids[1].velocity.x;
    if (boids[0].location.y > pHEIGHT * 0.5) boids[1].velocity.y = -boids[1].velocity.y;
  }
  if (!deltaHue2) {
    step = 1U;
    boids[0].location.x += boids[1].velocity.x;
    boids[0].location.y += boids[1].velocity.y;
    deltaHue2 = (boids[0].location.x <= 0 || boids[0].location.x >= pWIDTH - 1 || boids[0].location.y <= 0 || boids[0].location.y >= pHEIGHT - 1);
  }
  else
#endif // FAIRY_BEHAVIOR
  {
    PVector attractLocation = PVector(pWIDTH * 0.5, pHEIGHT * 0.5);
    // перемножаем и получаем 5.
    Boid boid = boids[0];
    PVector force = attractLocation - boid.location;      // Calculate direction of force
    float d = force.mag();                                // Distance between objects
    d = constrain(d, 5.0f, pHEIGHT);//видео снято на 5.0f  // Limiting the distance to eliminate "extreme" results for very close or very far objects
    force.normalize();                                    // Normalize vector (distance doesn't matter here, we just want this vector for direction)
    float strength = (5. * boid.mass) / (d * d);          // Calculate gravitional force magnitude 5.=attractG*attractMass
    force *= strength;                                    // Get force vector --> magnitude * direction
    boid.applyForce(force);
    boid.update();
    if (boid.location.x <= -1) boid.location.x = -boid.location.x;
    else if (boid.location.x >= pWIDTH) boid.location.x = -boid.location.x + pWIDTH + pWIDTH;
    if (boid.location.y <= -1) boid.location.y = -boid.location.y;
    else if (boid.location.y >= pHEIGHT) boid.location.y = -boid.location.y + pHEIGHT + pHEIGHT;
    boids[0] = boid;
    if (!deltaHue) {
      if (random8(3U)) {
        d = ((random8(2U)) ? boids[0].velocity.x : boids[0].velocity.y) * ((random8(2U)) ? .2 : -.2);
        boids[0].velocity.x += d;
        boids[0].velocity.y -= d;
      }
      else {
        if (fabs(boids[0].velocity.x) < 0.02)
          boids[0].velocity.x = -boids[0].velocity.x;
        else if (fabs(boids[0].velocity.y) < 0.02)
          boids[0].velocity.y = -boids[0].velocity.y;
      }
    }
  }
  dimAll(127);
  //go over particles and update matrix cells on the way
  for (int i = 0; i < enlargedObjectNUM; i++) {
    if (!trackingObjectIsShift[i] && step) {
      fairyEmit(i);
      step--;
    }
    if (trackingObjectIsShift[i]) { // particle->isAlive
      if (modes[currentMode].Scale & 0x01 && trackingObjectSpeedY[i] > -1) trackingObjectSpeedY[i] -= 0.05; //apply acceleration
      particlesUpdate2(i);
      //generate RGB values for particle
      CRGB baseRGB = CHSV(trackingObjectHue[i], 255, 255); // particles[i].hue
      baseRGB.nscale8(trackingObjectState[i]);//эквивалент
      drawPixelXYF(trackingObjectPosX[i], trackingObjectPosY[i], baseRGB);
    }
  }
  drawPixelXYF(boids[0].location.x, boids[0].location.y, CHSV(hue, 160U, 255U));//boid.colorIndex + hue
}

// ============= ЭФФЕКТ Капли на стекле ===============
// https://github.com/DmytroKorniienko/FireLamp_JeeUI/blob/master/src/effects.cpp
void newMatrixRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    setCurrentPalette();
    enlargedObjectNUM = map(modes[currentMode].Speed, 1, 255, 1, trackingOBJECT_MAX_COUNT);
    speedfactor = 0.136f; // фиксируем хорошую скорость
    for (uint8_t i = 0U; i < enlargedObjectNUM; i++)
    {
      trackingObjectPosX[i] = random8(pWIDTH);
      trackingObjectPosY[i] = random8(pHEIGHT);
      trackingObjectSpeedY[i] = random8(150, 250) / 100.;
      trackingObjectState[i] = random8(127U, 255U);
    }
    hue = modes[currentMode].Scale * 2.55;
  }
  dimAll(246); // для фиксированной скорости
  CHSV color;
  for (uint8_t i = 0U; i < enlargedObjectNUM; i++) {
    trackingObjectPosY[i] -= trackingObjectSpeedY[i] * speedfactor;
    if (modes[currentMode].Scale == 100U) {
      color = rgb2hsv_approximate(CRGB::Gray);
      color.val = trackingObjectState[i];
    } else if (modes[currentMode].Scale == 1U) {
      color = CHSV(++hue, 255, trackingObjectState[i]);
    } else {
      color = CHSV(hue, 255, trackingObjectState[i]);
    }
    drawPixelXYF(trackingObjectPosX[i], trackingObjectPosY[i], color);
#define GLUK 20 // вероятность горизонтального сдвига капли
    if (random8() < GLUK) {
      trackingObjectPosX[i] = (uint8_t)(trackingObjectPosX[i] + pWIDTH - 1U + random8(3U)) % pWIDTH ;
      trackingObjectState[i] = random8(196, 255);
    }
    if (trackingObjectPosY[i] < -1) {
      trackingObjectPosX[i] = random8(pWIDTH);
      trackingObjectPosY[i] = random8(pHEIGHT - pHEIGHT / 2, pHEIGHT);
      trackingObjectSpeedY[i] = random8(150, 250) / 100.;
      trackingObjectState[i] = random8(127U, 255U);
    }
  }
}

// ============= Эффект Цветные драже ===============
// (c) SottNick
//по мотивам визуала эффекта by Yaroslaw Turbin 14.12.2020
//https://vk.com/ldirko программный код которого он запретил брать
void sandRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    pcnt = 0U;// = pHEIGHT;
  }
  // если насыпалось уже достаточно, бахаем рандомные песчинки
  uint8_t temp = map8(random8(), modes[currentMode].Scale * 2.55, 255U);
  if (pcnt >= map8(temp, 2U, pHEIGHT - 3U)) {
    temp = pHEIGHT + 1U - pcnt;
    if (!random8(4U)) // иногда песка осыпается до половины разом
      if (random8(2U))
        temp = 2U;
      else
        temp = 3U;
    for (uint8_t y = 0; y < pcnt; y++)
      for (uint8_t x = 0; x < pWIDTH; x++)
        if (!random8(temp))
          leds[XY(x, y)] = 0;
  }
  pcnt = 0U;
  // осыпаем всё, что есть на экране
  for (uint8_t y = 1; y < pHEIGHT; y++)
    for (uint8_t x = 0; x < pWIDTH; x++)
      if (leds[XY(x, y)])                                                          // проверяем для каждой песчинки
        if (!leds[XY(x, y - 1)]) {                                                 // если под нами пусто, просто падаем
          leds[XY(x, y - 1)] = leds[XY(x, y)];
          leds[XY(x, y)] = 0;
        }
        else if (x > 0U && !leds[XY(x - 1, y - 1)] && x < pWIDTH - 1 && !leds[XY(x + 1, y - 1)]) { // если под нами пик
          if (random8(2U))
            leds[XY(x - 1, y - 1)] = leds[XY(x, y)];
          else
            leds[XY(x - 1, y - 1)] = leds[XY(x, y)];
          leds[XY(x, y)] = 0;
          pcnt = y - 1;
        }
        else if (x > 0U && !leds[XY(x - 1, y - 1)]) {                              // если под нами склон налево
          leds[XY(x - 1, y - 1)] = leds[XY(x, y)];
          leds[XY(x, y)] = 0;
          pcnt = y - 1;
        }
        else if (x < pWIDTH - 1 && !leds[XY(x + 1, y - 1)]) {                       // если под нами склон направо
          leds[XY(x + 1, y - 1)] = leds[XY(x, y)];
          leds[XY(x, y)] = 0;
          pcnt = y - 1;
        }
        else                                                                       // если под нами плато
          pcnt = y;
  // эмиттер новых песчинок в оригинале тут было что-то непонятное, поэтому чтобы точно найти центр, тупо делим ширину пополам и округляем (округление нужно для матриц с нечетным числом колонок)
  if (!leds[XY(round(pWIDTH / 2), pHEIGHT - 2)] && !leds[XY(round(pWIDTH / 2), pHEIGHT - 2)] && !random8(3)) {
    temp = round(pWIDTH / 2);
    leds[XY(temp, pHEIGHT - 1)] = CHSV(random8(), 255U, 255U);
  }
}

// ------------------------------ ЭФФЕКТЫ ПИКАССО ----------------------
// взято откуда-то by @obliterator или им написано
// https://github.com/DmytroKorniienko/FireLamp_JeeUI/blob/templ/src/effects.cpp
void PicassoGenerate(bool reset) {
  if (loadingFlag)
  {
    loadingFlag = false;
    if (enlargedObjectNUM > enlargedOBJECT_MAX_COUNT) enlargedObjectNUM = enlargedOBJECT_MAX_COUNT;
    if (enlargedObjectNUM < 2U) enlargedObjectNUM = 2U;
    double minSpeed = 0.2, maxSpeed = 0.8;
    for (uint8_t i = 0 ; i < enlargedObjectNUM ; i++) {
      trackingObjectPosX[i] = random8(pWIDTH);
      trackingObjectPosY[i] = random8(pHEIGHT);
      trackingObjectHue[i] = random8();
      trackingObjectSpeedY[i] = +((-maxSpeed / 3) + (maxSpeed * (float)random8(1, 100) / 100));
      trackingObjectSpeedY[i] += trackingObjectSpeedY[i] > 0 ? minSpeed : -minSpeed;
      trackingObjectShift[i] = +((-maxSpeed / 2) + (maxSpeed * (float)random8(1, 100) / 100));
      trackingObjectShift[i] += trackingObjectShift[i] > 0 ? minSpeed : -minSpeed;
      trackingObjectState[i] = trackingObjectHue[i];
    }
  }
  for (uint8_t i = 0 ; i < enlargedObjectNUM ; i++) {
    if (reset) {
      trackingObjectState[i] = random8();
      trackingObjectSpeedX[i] = (trackingObjectState[i] - trackingObjectHue[i]) / 25;
    }
    if (trackingObjectState[i] != trackingObjectHue[i] && trackingObjectSpeedX[i]) {
      trackingObjectHue[i] += trackingObjectSpeedX[i];
    }
  }
}
void PicassoPosition() {
  for (uint8_t i = 0 ; i < enlargedObjectNUM ; i++) {
    if (trackingObjectPosX[i] + trackingObjectSpeedY[i] > pWIDTH || trackingObjectPosX[i] + trackingObjectSpeedY[i] < 0) {
      trackingObjectSpeedY[i] = -trackingObjectSpeedY[i];
    }
    if (trackingObjectPosY[i] + trackingObjectShift[i] > pHEIGHT || trackingObjectPosY[i] + trackingObjectShift[i] < 0) {
      trackingObjectShift[i] = -trackingObjectShift[i];
    }
    trackingObjectPosX[i] += trackingObjectSpeedY[i];
    trackingObjectPosY[i] += trackingObjectShift[i];
  };
}
void PicassoRoutine() {
  PicassoGenerate(false);
  PicassoPosition();
  for (uint8_t i = 0 ; i < enlargedObjectNUM - 2U ; i += 2)
    DrawLine(trackingObjectPosX[i], trackingObjectPosY[i], trackingObjectPosX[i + 1U], trackingObjectPosY[i + 1U], CHSV(trackingObjectHue[i], 255U, 255U));
  EVERY_N_MILLIS(20000) {
    PicassoGenerate(true);
  }
  blurScreen(80);
}
void PicassoRoutine2() {
  PicassoGenerate(false);
  PicassoPosition();
  dimAll(180);
  for (uint8_t i = 0 ; i < enlargedObjectNUM - 1U ; i++)
    DrawLineF(trackingObjectPosX[i], trackingObjectPosY[i], trackingObjectPosX[i + 1U], trackingObjectPosY[i + 1U], CHSV(trackingObjectHue[i], 255U, 255U));
  EVERY_N_MILLIS(20000) {
    PicassoGenerate(true);
  }
  blurScreen(80);
}
void PicassoRoutine3() {
  PicassoGenerate(false);
  PicassoPosition();
  dimAll(180);
  for (uint8_t i = 0 ; i < enlargedObjectNUM - 2U ; i += 2)
    drawCircleF(fabs(trackingObjectPosX[i] - trackingObjectPosX[i + 1U]), fabs(trackingObjectPosY[i] - trackingObjectPosX[i + 1U]), fabs(trackingObjectPosX[i] - trackingObjectPosY[i]), CHSV(trackingObjectHue[i], 255U, 255U));
  EVERY_N_MILLIS(20000) {
    PicassoGenerate(true);
  }
  blurScreen(80);
}
uint8_t picselect = 0;
void picassoSelector() {
  if (loadingFlag)
  {
    if (modes[currentMode].Scale < 34U)           // если масштаб до 34
      enlargedObjectNUM = (modes[currentMode].Scale - 1U) / 32.0 * (enlargedOBJECT_MAX_COUNT - 3U) + 3U;
    else if (modes[currentMode].Scale >= 68U)      // если масштаб больше 67
      enlargedObjectNUM = (modes[currentMode].Scale - 68U) / 32.0 * (enlargedOBJECT_MAX_COUNT - 3U) + 3U;
    else                                          // для масштабов посередине
      enlargedObjectNUM = (modes[currentMode].Scale - 34U) / 33.0 * (enlargedOBJECT_MAX_COUNT - 1U) + 1U;
    picselect = (specialTextEffectParam >= 0) ? specialTextEffectParam : getEffectScaleParamValue2(MC_PICASSO);
    if (picselect == 1 || picselect > 3) {
      picselect = random8(1, 4);
    }
  }
  switch (picselect) {
    case 1:    PicassoRoutine();       break;
    case 2:    PicassoRoutine2();      break;
    default:   PicassoRoutine3();      break;
  }
}

// ------------- цвет + вода в бассейне ------------------
// (с) SottNick. 03.2020
// эффект иммеет шов на стыке краёв матрицы (сзади лампы, как и у других эффектов), зато адаптирован для нестандартных размеров матриц.
// можно было бы сделать абсолютно бесшовный вариант для конкретной матрицы (16х16), но уже была бы заметна зацикленность анимации.
// далее идёт массив из 25 кадров анимации с маской бликов на воде (размер картинки больше размера матрицы, чтобы повторяемость картинки была незаметной)
// бесшовную анимированную текстуру бликов делал в программе Substance Designer (30 дней бесплатно работает) при помощи плагина Bruno Caustics Generator
// но сразу под такой мелкий размер текстура выходит нечёткой, поэтому пришлось делать крупную и потом в фотошопе доводить её до ума
// конвертировал в массив через сервис https://littlevgl.com/image-to-c-array,
// чтобы из ч/б картинки получить массив для коррекции параметра насыщенности цвета, использовал настройки True color -> C array
// последовательность замен полученных блоков массива в ворде: "^p  0x"->"^p  {0x"  ...  ", ^p"->"},^p" ... "},^p#endif"->"}^p },^p {"
static const uint8_t aquariumGIF[25][32][32] PROGMEM =
{
  {
    {0x00, 0x00, 0x00, 0x00, 0x34, 0x8f, 0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x89, 0xe1, 0x77, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x37, 0x9b, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x7f, 0xdd, 0x77, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x37, 0x94, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x88, 0x78, 0x7a, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x4a, 0x71, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x17, 0x38, 0x87, 0x62, 0x00, 0x21, 0x67, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x77, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x20, 0x1b, 0x19, 0x46, 0x2e, 0x00, 0x00, 0x3e, 0x73, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x02, 0x55, 0xc7, 0x61, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x1f, 0x07, 0x00, 0x00, 0x02, 0x14, 0x00, 0x00, 0x1a, 0xa6, 0x8f, 0x4c, 0x22, 0x04, 0x00, 0x00, 0x00},
    {0x0d, 0x20, 0x62, 0xb3, 0xc8, 0x97, 0x47, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x00, 0x00, 0x47, 0x74, 0x6a, 0x70, 0x77, 0x6c, 0x4d, 0x29, 0x11},
    {0x7b, 0x95, 0x9b, 0x60, 0x31, 0x42, 0x61, 0x67, 0x5d, 0x28, 0x00, 0x00, 0x02, 0x23, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f, 0x59, 0x4a, 0x3e, 0x0e, 0x01, 0x0a, 0x27, 0x4d, 0x6a, 0x79, 0x7c},
    {0xc5, 0xb1, 0x2e, 0x00, 0x00, 0x00, 0x00, 0x2a, 0x69, 0x82, 0x44, 0x0f, 0x2e, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0xb6, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x25, 0x6c},
    {0x82, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x82, 0x83, 0x79, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0xc2, 0x83, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e},
    {0x5e, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0xab, 0xec, 0x8f, 0x31, 0x21, 0x28, 0x39, 0x59, 0xa2, 0xc4, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x5d, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6d, 0xed, 0xb9, 0x7d, 0x6e, 0x6d, 0x65, 0x55, 0x43, 0x66, 0x5e, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x5c, 0x4d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6d, 0x99, 0x3d, 0x1d, 0x18, 0x10, 0x02, 0x00, 0x00, 0x13, 0x5a, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x56, 0x52, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x83, 0x59, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x50, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x4d, 0x58, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0x98, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4f, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x41, 0x5d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3a, 0xb4, 0x97, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x54, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x31, 0x61, 0x07, 0x00, 0x00, 0x00, 0x00, 0x12, 0x5b, 0x90, 0x77, 0x60, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3a, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x38, 0x78, 0x2d, 0x00, 0x00, 0x0b, 0x43, 0x7e, 0x79, 0x35, 0x06, 0x0d, 0x35, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x54, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x70, 0xc3, 0x96, 0x48, 0x4d, 0x79, 0x81, 0x4c, 0x0b, 0x00, 0x00, 0x00, 0x02, 0x2f, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x7b, 0x54, 0x2f, 0x27, 0x22, 0x1f, 0x30},
    {0xbf, 0xc9, 0xa6, 0xad, 0xae, 0x72, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x33, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x35, 0xa3, 0xac, 0x81, 0x7a, 0x78, 0x78, 0x8e},
    {0x92, 0x5a, 0x22, 0x5b, 0x83, 0x2e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x35, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x74, 0x71, 0x3a, 0x27, 0x27, 0x2d, 0x37, 0x61},
    {0x46, 0x18, 0x00, 0x00, 0x4d, 0x2e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x34, 0x07, 0x00, 0x00, 0x00, 0x00, 0x26, 0x62, 0x51, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10},
    {0x2f, 0x09, 0x00, 0x00, 0x32, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x33, 0x03, 0x00, 0x04, 0x30, 0x50, 0x3a, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01},
    {0x2f, 0x0d, 0x00, 0x00, 0x1e, 0x48, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x3b, 0x22, 0x3d, 0x39, 0x1e, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03},
    {0x46, 0x1f, 0x00, 0x00, 0x0f, 0x4f, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x65, 0x6c, 0x2f, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01},
    {0x74, 0x5f, 0x10, 0x00, 0x16, 0x69, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5f, 0x59, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19},
    {0x4b, 0x42, 0x3a, 0x39, 0x5b, 0x91, 0x6f, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x5b, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x4e},
    {0x00, 0x00, 0x09, 0x2e, 0x79, 0x81, 0x75, 0x72, 0x53, 0x43, 0x3d, 0x37, 0x2f, 0x29, 0x24, 0x1e, 0x20, 0x4e, 0x7b, 0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x2f, 0x3f, 0x1f},
    {0x00, 0x00, 0x00, 0x00, 0x37, 0x65, 0x60, 0x81, 0x72, 0x67, 0x69, 0x6b, 0x6d, 0x6f, 0x6f, 0x6c, 0x71, 0x98, 0xb2, 0x55, 0x17, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x3e, 0x30, 0x06, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x11, 0x6c, 0x75, 0x32, 0x0a, 0x08, 0x0a, 0x0d, 0x12, 0x18, 0x1f, 0x24, 0x2d, 0x4a, 0x8e, 0x93, 0x4a, 0x16, 0x00, 0x00, 0x00, 0x16, 0x36, 0x39, 0x16, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x0e, 0x7d, 0x67, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x88, 0x9c, 0x45, 0x11, 0x18, 0x32, 0x3a, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x1f, 0x84, 0x4a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x35, 0xb1, 0xa5, 0x60, 0x46, 0x26, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
  },
  {
    {0x00, 0x00, 0x00, 0x00, 0x38, 0x59, 0x0a, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x93, 0x8d, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x46, 0x73, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a, 0xa5, 0xa1, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x5f, 0x75, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x22, 0x30, 0x58, 0xc6, 0xd3, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x70, 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x32, 0x28, 0x18, 0x19, 0x6d, 0xd7, 0x8b, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x1b, 0x7f, 0x2c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x34, 0x28, 0x08, 0x00, 0x00, 0x00, 0x09, 0x8d, 0xc3, 0x7f, 0x2a, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x6d, 0xc6, 0x4a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x50, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x67, 0xca, 0xa2, 0x7a, 0x6e, 0x54, 0x2c, 0x08, 0x00, 0x00, 0x00},
    {0x0b, 0x24, 0x70, 0xba, 0xc5, 0x95, 0x44, 0x07, 0x00, 0x00, 0x00, 0x00, 0x53, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7b, 0xa6, 0x36, 0x2c, 0x4e, 0x70, 0x7f, 0x76, 0x57, 0x2f, 0x13},
    {0x8e, 0xa8, 0x9b, 0x51, 0x28, 0x44, 0x68, 0x67, 0x43, 0x12, 0x00, 0x26, 0x71, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x91, 0x5c, 0x00, 0x00, 0x00, 0x05, 0x24, 0x4c, 0x6c, 0x80, 0x88},
    {0xd6, 0xac, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x2b, 0x58, 0x64, 0x66, 0x9e, 0x72, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x9c, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x74},
    {0x8b, 0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x6c, 0xeb, 0xa4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0xb7, 0x7a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15},
    {0x6b, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x92, 0xef, 0x77, 0x1e, 0x15, 0x1d, 0x30, 0x53, 0xab, 0xd5, 0x51, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03},
    {0x6b, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4a, 0xea, 0xd1, 0x7f, 0x73, 0x76, 0x72, 0x68, 0x5f, 0x7f, 0x5c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04},
    {0x6a, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5c, 0xaf, 0x5d, 0x2d, 0x26, 0x1e, 0x0c, 0x00, 0x00, 0x23, 0x62, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02},
    {0x65, 0x4e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x76, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3d, 0x4b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x5d, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2b, 0x94, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x59, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x50, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x8a, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a, 0x54, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x43, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5a, 0xc3, 0x67, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x4c, 0x90, 0x34, 0x00, 0x00, 0x00, 0x00, 0x20, 0x6a, 0x9b, 0x8b, 0x5a, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x57, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x8f, 0xe2, 0xac, 0x63, 0x43, 0x3e, 0x65, 0x8b, 0x73, 0x31, 0x11, 0x30, 0x32, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x5b, 0x12, 0x00, 0x00, 0x00, 0x00, 0x20},
    {0xb5, 0x93, 0x7f, 0x84, 0xa4, 0xc5, 0xa5, 0x52, 0x06, 0x00, 0x00, 0x00, 0x21, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x69, 0x80, 0x4c, 0x3a, 0x41, 0x5a, 0x99},
    {0x3d, 0x03, 0x00, 0x07, 0x39, 0x9d, 0x6e, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x75, 0xaa, 0x82, 0x73, 0x88, 0xba, 0xab},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x58, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x55, 0x6e, 0x34, 0x15, 0x14, 0x34, 0x7a, 0x4a},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x55, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x2f, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x4f, 0x5d, 0x12, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x21},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x52, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x32, 0x08, 0x00, 0x00, 0x00, 0x03, 0x4e, 0x5a, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0x20},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x43, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x30, 0x02, 0x00, 0x06, 0x4a, 0x5c, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4e, 0x49},
    {0x2c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x4e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f, 0x32, 0x1c, 0x4b, 0x5f, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x71, 0x7c},
    {0x4a, 0x3a, 0x1e, 0x04, 0x00, 0x01, 0x6b, 0x94, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x5d, 0x6d, 0x5d, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x46, 0x36, 0x34},
    {0x02, 0x21, 0x39, 0x41, 0x45, 0x6a, 0xa9, 0xac, 0x84, 0x63, 0x4d, 0x3f, 0x36, 0x30, 0x2c, 0x29, 0x2d, 0x41, 0x81, 0x81, 0x3a, 0x03, 0x00, 0x00, 0x00, 0x00, 0x18, 0x35, 0x30, 0x0e, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x1c, 0x5d, 0xa5, 0x7b, 0x33, 0x4e, 0x7a, 0x72, 0x60, 0x5f, 0x63, 0x65, 0x64, 0x65, 0x73, 0xa6, 0x94, 0x42, 0x07, 0x00, 0x00, 0x13, 0x30, 0x35, 0x1a, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x24, 0x67, 0x23, 0x00, 0x00, 0x1d, 0x13, 0x05, 0x07, 0x0d, 0x12, 0x17, 0x1c, 0x29, 0x5c, 0xa7, 0x7e, 0x28, 0x1c, 0x2f, 0x37, 0x22, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x1e, 0x4b, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x69, 0xb7, 0x80, 0x50, 0x28, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x2a, 0x4d, 0x01, 0x00, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0xb0, 0xa5, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
  },
  {
    {0x00, 0x00, 0x00, 0x0a, 0x54, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0f, 0x04, 0x00, 0x00, 0x00, 0x00, 0x02, 0x50, 0x9c, 0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x0f, 0x5c, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x0c, 0x25, 0x42, 0x85, 0xd7, 0x81, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x13, 0x67, 0x3a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x2f, 0x41, 0x40, 0x34, 0x2b, 0x40, 0xb6, 0xbe, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x17, 0x7b, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x7a, 0x47, 0x0f, 0x00, 0x00, 0x00, 0x37, 0xcd, 0x8b, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x33, 0x8b, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4d, 0x58, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x95, 0xed, 0x7b, 0x2d, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x0b, 0x8a, 0xc9, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x5f, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x69, 0xe5, 0x9d, 0x75, 0x6f, 0x59, 0x31, 0x0c, 0x00, 0x00, 0x00},
    {0x11, 0x33, 0x84, 0xbe, 0xbc, 0x8e, 0x41, 0x07, 0x00, 0x00, 0x00, 0x00, 0x46, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x9d, 0x32, 0x25, 0x47, 0x6c, 0x7e, 0x77, 0x5b, 0x35, 0x19},
    {0x9c, 0xb1, 0x8f, 0x40, 0x21, 0x43, 0x68, 0x68, 0x44, 0x11, 0x00, 0x18, 0x6b, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x8d, 0x5c, 0x00, 0x00, 0x00, 0x02, 0x1e, 0x45, 0x67, 0x7f, 0x8d},
    {0xd8, 0x99, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x2d, 0x5a, 0x65, 0x62, 0x96, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x9c, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x76},
    {0x8a, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x69, 0xe8, 0xab, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xac, 0x7b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c},
    {0x70, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8a, 0xea, 0x72, 0x14, 0x0a, 0x11, 0x23, 0x47, 0xaa, 0xd6, 0x4a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b},
    {0x71, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0xe3, 0xe0, 0x80, 0x6f, 0x75, 0x75, 0x73, 0x75, 0x8e, 0x51, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c},
    {0x71, 0x3d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0xb6, 0x7c, 0x3e, 0x34, 0x2b, 0x17, 0x04, 0x01, 0x34, 0x62, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a},
    {0x6d, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6d, 0x8a, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05},
    {0x65, 0x4b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x90, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x5b, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x58, 0x51, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0x8f, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x4b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x52, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x90, 0x61, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x51, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x6c, 0xaf, 0x48, 0x05, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x83, 0xa6, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x57, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0xba, 0xe8, 0xb8, 0x85, 0x62, 0x45, 0x2f, 0x44, 0x81, 0xa7, 0x82, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x4a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47},
    {0x93, 0x5e, 0x5a, 0x6b, 0x83, 0x9f, 0xbd, 0xb7, 0x7f, 0x37, 0x2f, 0x3b, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x57, 0x40, 0x06, 0x03, 0x1b, 0x68, 0xaa},
    {0x07, 0x00, 0x00, 0x00, 0x04, 0x2d, 0x97, 0x9e, 0x2c, 0x00, 0x00, 0x17, 0x2a, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x7e, 0x72, 0x6a, 0x9b, 0xa8, 0x52},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0x68, 0x20, 0x00, 0x00, 0x00, 0x16, 0x26, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x88, 0x8e, 0xa5, 0xb2, 0x3e, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x52, 0x30, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a, 0x61, 0x3b, 0x1d, 0x55, 0x5c, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x5a, 0x29, 0x00, 0x00, 0x2e, 0x47, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x56, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x2c, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x59, 0x28, 0x00, 0x00, 0x00, 0x44, 0x7a, 0x1c, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3a, 0x89, 0x2c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x2d, 0x04, 0x00, 0x00, 0x00, 0x20, 0x59, 0x30, 0x00, 0x00, 0x02, 0x28, 0x51, 0x67, 0x5e, 0x1d},
    {0x38, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x7d, 0xb8, 0x84, 0x43, 0x17, 0x00, 0x00, 0x00, 0x02, 0x32, 0x37, 0x0d, 0x03, 0x24, 0x5a, 0x3b, 0x04, 0x01, 0x1a, 0x2f, 0x2c, 0x13, 0x0b, 0x31, 0x4f},
    {0x36, 0x45, 0x38, 0x23, 0x14, 0x33, 0x79, 0x7b, 0x50, 0x50, 0x64, 0x6f, 0x5e, 0x43, 0x38, 0x3d, 0x4e, 0x72, 0x6a, 0x54, 0x6e, 0x50, 0x1c, 0x22, 0x35, 0x2f, 0x14, 0x00, 0x00, 0x00, 0x00, 0x0c},
    {0x00, 0x0d, 0x32, 0x5c, 0x8a, 0xa1, 0x5c, 0x0c, 0x00, 0x00, 0x07, 0x38, 0x74, 0x6f, 0x58, 0x57, 0x5f, 0x74, 0xa0, 0xbf, 0x8b, 0x4a, 0x42, 0x38, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x19, 0x82, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x1f, 0x0a, 0x0b, 0x12, 0x1d, 0x4a, 0xa9, 0xb1, 0x6c, 0x33, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x51, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x5d, 0xa9, 0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x03, 0x4d, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x85, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
  },
  {
    {0x00, 0x00, 0x00, 0x36, 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x3f, 0x7a, 0x28, 0x00, 0x00, 0x00, 0x0b, 0x70, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x3a, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x9c, 0x94, 0x2a, 0x0f, 0x07, 0x33, 0x8c, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x3d, 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x97, 0x69, 0x3a, 0x2c, 0x3f, 0x95, 0xcb, 0x5d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x40, 0x53, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x65, 0x37, 0x00, 0x00, 0x00, 0x10, 0x62, 0xe5, 0xa0, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x54, 0x78, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3b, 0x4d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x98, 0xf4, 0x84, 0x31, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x1f, 0xa9, 0xcf, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x5c, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x62, 0xe1, 0x9d, 0x74, 0x70, 0x5c, 0x36, 0x10, 0x00, 0x00, 0x00},
    {0x18, 0x45, 0x97, 0xbf, 0xb6, 0x8b, 0x42, 0x07, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6e, 0x9d, 0x30, 0x21, 0x41, 0x66, 0x7b, 0x78, 0x5e, 0x3b, 0x1f},
    {0xa8, 0xb5, 0x80, 0x2f, 0x1b, 0x42, 0x68, 0x69, 0x44, 0x11, 0x00, 0x11, 0x6a, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x8a, 0x5f, 0x00, 0x00, 0x00, 0x00, 0x19, 0x3f, 0x61, 0x7e, 0x92},
    {0xd7, 0x83, 0x02, 0x00, 0x00, 0x00, 0x00, 0x2e, 0x5b, 0x66, 0x60, 0x8e, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x9b, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x78},
    {0x88, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0x66, 0xe4, 0xb1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0xa2, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24},
    {0x73, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0xe5, 0x6d, 0x0b, 0x01, 0x07, 0x16, 0x3a, 0xa4, 0xd5, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14},
    {0x75, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a, 0xd9, 0xea, 0x80, 0x6a, 0x70, 0x74, 0x7a, 0x8a, 0x99, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15},
    {0x76, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0xbc, 0x9e, 0x55, 0x42, 0x39, 0x23, 0x0d, 0x09, 0x44, 0x5d, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12},
    {0x73, 0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x62, 0x9c, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x52, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d},
    {0x6c, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x85, 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x58, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06},
    {0x60, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5e, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x63, 0x67, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x7f, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x55, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x96, 0xd0, 0x65, 0x1e, 0x03, 0x00, 0x00, 0x00, 0x05, 0x6d, 0x5d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x51, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12},
    {0xd1, 0xd9, 0xb6, 0x9d, 0x88, 0x66, 0x40, 0x2d, 0x65, 0x91, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x18, 0x7d},
    {0x64, 0x2e, 0x33, 0x4b, 0x6e, 0x8c, 0x9e, 0xb9, 0xe2, 0x94, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x55, 0x2e, 0x01, 0x06, 0x3b, 0x94, 0xa7},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x2e, 0x8d, 0xd7, 0x95, 0x3b, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2d, 0x65, 0x47, 0x68, 0xa6, 0x7d, 0x1f},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x78, 0x74, 0x44, 0x32, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x66, 0xba, 0xbc, 0x61, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x5e, 0x15, 0x20, 0x27, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x7a, 0xda, 0x7e, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f, 0x5f, 0x0e, 0x00, 0x1a, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x50, 0x8d, 0xac, 0x51, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2b, 0x7d, 0x33, 0x00, 0x00, 0x29, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x55, 0x7f, 0x76, 0x7d, 0x6b, 0x13, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5a, 0xb8, 0x84, 0x3b, 0x17, 0x1f, 0x39, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x69, 0x94, 0x6a, 0x3a, 0x23, 0x46, 0x58, 0x1c, 0x00},
    {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4b, 0x8e, 0x75, 0x63, 0x68, 0x66, 0x54, 0x52, 0x54, 0x27, 0x0a, 0x01, 0x02, 0x0a, 0x2d, 0x81, 0xa2, 0x61, 0x1b, 0x00, 0x00, 0x01, 0x2e, 0x57, 0x39},
    {0x4e, 0x2e, 0x0b, 0x00, 0x00, 0x20, 0x68, 0x77, 0x34, 0x05, 0x02, 0x13, 0x3a, 0x6a, 0x87, 0x9c, 0x9c, 0x7c, 0x66, 0x65, 0x76, 0xa6, 0xab, 0x51, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x47},
    {0x1f, 0x43, 0x55, 0x5a, 0x74, 0x92, 0x5d, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x5f, 0xa7, 0xb6, 0x8a, 0x6d, 0x85, 0xcd, 0xcb, 0x55, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x20, 0x79, 0xb9, 0x5c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x6b, 0x53, 0x0a, 0x00, 0x0d, 0x71, 0x93, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x37, 0x6d, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x4f, 0x17, 0x00, 0x00, 0x00, 0x1f, 0x74, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x2d, 0x4e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x56, 0x0c, 0x00, 0x00, 0x00, 0x0c, 0x69, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
  },
  {
    {0x00, 0x00, 0x07, 0x51, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x68, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x7b, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x0c, 0x53, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x7c, 0x7b, 0x01, 0x00, 0x00, 0x00, 0x00, 0x29, 0x7e, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x0e, 0x51, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0xc2, 0x61, 0x23, 0x16, 0x0a, 0x05, 0x56, 0x90, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x0d, 0x51, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x74, 0x6d, 0x33, 0x29, 0x2d, 0x33, 0x5c, 0xc1, 0xbf, 0x3a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x15, 0x64, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x06, 0x30, 0xbf, 0xfe, 0x91, 0x38, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x45, 0xb2, 0x96, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x56, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0xdb, 0x99, 0x6e, 0x69, 0x5b, 0x38, 0x14, 0x00, 0x00, 0x00},
    {0x1f, 0x57, 0xa3, 0xb8, 0xad, 0x8f, 0x49, 0x09, 0x00, 0x00, 0x00, 0x00, 0x33, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x62, 0x94, 0x2c, 0x1b, 0x37, 0x5b, 0x72, 0x72, 0x5c, 0x3d, 0x23},
    {0xaa, 0xad, 0x6a, 0x25, 0x1e, 0x40, 0x68, 0x68, 0x40, 0x0f, 0x00, 0x0a, 0x63, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x5c, 0x00, 0x00, 0x00, 0x00, 0x13, 0x35, 0x56, 0x76, 0x90},
    {0xc7, 0x67, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2d, 0x58, 0x61, 0x58, 0x7f, 0x7c, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x92, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x75},
    {0x7e, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x61, 0xd6, 0xb2, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x90, 0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a},
    {0x6d, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0xe7, 0x80, 0x11, 0x00, 0x00, 0x0b, 0x2b, 0x93, 0xc6, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d},
    {0x70, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3a, 0xdc, 0xe1, 0x91, 0x7c, 0x73, 0x71, 0x79, 0x96, 0x98, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e},
    {0x72, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5e, 0xa0, 0x58, 0x50, 0x70, 0x68, 0x3a, 0x1a, 0x13, 0x50, 0x51, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b},
    {0x70, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x83, 0x41, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x0a, 0x52, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15},
    {0x6a, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x59, 0x6a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a, 0x4d, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c},
    {0x63, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x80, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x2e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04},
    {0x72, 0x69, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0x61, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x50, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05},
    {0xba, 0xdc, 0x7e, 0x3f, 0x1c, 0x03, 0x00, 0x00, 0x3c, 0x84, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31},
    {0xc1, 0xae, 0xa0, 0x9d, 0x99, 0x84, 0x65, 0x62, 0x9d, 0x6b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x4a, 0x2e, 0x00, 0x00, 0x00, 0x01, 0x42, 0xa3},
    {0x2f, 0x0d, 0x12, 0x27, 0x4b, 0x70, 0x93, 0xd7, 0xe2, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x53, 0x25, 0x05, 0x1b, 0x6a, 0xa6, 0x7e},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x68, 0xd2, 0x7b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x63, 0x55, 0x8c, 0x9e, 0x48, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6f, 0xae, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x78, 0xc5, 0x9d, 0x27, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xa2, 0x7d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x92, 0xc5, 0x36, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6a, 0xc6, 0x61, 0x14, 0x0c, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x4e, 0xcb, 0x9a, 0x09, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5a, 0xe1, 0xd8, 0x7d, 0x31, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x4f, 0x9e, 0xa7, 0x84, 0x2d, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2d, 0x8c, 0x95, 0x9b, 0xb0, 0x8b, 0x40, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1c, 0x67, 0x9c, 0x70, 0x33, 0x44, 0x5f, 0x23, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3a, 0x7e, 0x56, 0x1c, 0x18, 0x48, 0x99, 0xad, 0x7c, 0x57, 0x48, 0x3f, 0x38, 0x39, 0x55, 0x8c, 0xa1, 0x58, 0x0f, 0x00, 0x01, 0x2a, 0x59, 0x3a, 0x03},
    {0x25, 0x00, 0x00, 0x00, 0x00, 0x12, 0x5e, 0x7b, 0x36, 0x00, 0x00, 0x00, 0x00, 0x28, 0x98, 0xdd, 0xb7, 0x91, 0x8c, 0x99, 0xa2, 0xa0, 0xa8, 0x5a, 0x05, 0x00, 0x00, 0x00, 0x00, 0x16, 0x4e, 0x52},
    {0x4e, 0x4b, 0x36, 0x27, 0x4d, 0x84, 0x64, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4e, 0x9f, 0x5b, 0x23, 0x23, 0x38, 0x6e, 0x89, 0x6e, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2b},
    {0x02, 0x2c, 0x66, 0xa5, 0xae, 0x4b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3d, 0x59, 0x00, 0x00, 0x00, 0x00, 0x03, 0x67, 0x6c, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x17, 0x89, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x3d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45, 0x7a, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x01, 0x56, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x7d, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
  },
  {
    {0x00, 0x00, 0x30, 0x4a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x61, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x20, 0x2b, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x36, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3b, 0x81, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x24, 0x32, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x38, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3b, 0xaa, 0x5f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x2e, 0x35, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x36, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0xad, 0xac, 0x3e, 0x27, 0x20, 0x16, 0x16, 0x43, 0x46, 0x46, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x41, 0x4f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x7b, 0x2a, 0x19, 0x20, 0x2b, 0x3a, 0x75, 0xa0, 0x76, 0x84, 0x71, 0x25, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x10, 0x78, 0xa1, 0x45, 0x14, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x57, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2d, 0xa5, 0xce, 0xa2, 0x83, 0x72, 0x5e, 0x40, 0x26, 0x12, 0x02, 0x00},
    {0x47, 0x78, 0xb7, 0xb4, 0x94, 0x89, 0x60, 0x10, 0x00, 0x00, 0x00, 0x00, 0x29, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0xb5, 0x43, 0x1f, 0x36, 0x5b, 0x7c, 0x7a, 0x63, 0x5b, 0x54},
    {0xbe, 0xbc, 0x62, 0x26, 0x2e, 0x4c, 0x6d, 0x6e, 0x40, 0x0e, 0x00, 0x03, 0x5a, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7a, 0x5f, 0x00, 0x00, 0x00, 0x01, 0x20, 0x24, 0x1e, 0x33, 0x72},
    {0x98, 0x72, 0x00, 0x00, 0x00, 0x00, 0x02, 0x2d, 0x56, 0x5e, 0x53, 0x72, 0x7d, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2d, 0x8a, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d},
    {0x5a, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x61, 0xd0, 0xc2, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0x72, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x67, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x86, 0xfd, 0xad, 0x42, 0x22, 0x14, 0x0d, 0x24, 0x88, 0xb8, 0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08},
    {0x81, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0xcd, 0xa5, 0x83, 0x7e, 0x83, 0x8f, 0x94, 0xaa, 0x98, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31},
    {0x7d, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x89, 0x5e, 0x0e, 0x0e, 0x1d, 0x32, 0x54, 0x5a, 0x3b, 0x5f, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x37},
    {0x70, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x53, 0x75, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x0c, 0x21, 0x50, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20},
    {0x67, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x81, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13},
    {0x66, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5e, 0x69, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x4b, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b},
    {0x87, 0x76, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x81, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x48, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14},
    {0xd5, 0xdd, 0x94, 0x63, 0x3e, 0x1a, 0x07, 0x18, 0x7d, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x5e},
    {0x9c, 0x7d, 0x82, 0x90, 0x9c, 0x97, 0x8d, 0xac, 0xaf, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x4b, 0x20, 0x00, 0x00, 0x00, 0x1b, 0x75, 0xb2},
    {0x0b, 0x00, 0x00, 0x0a, 0x27, 0x4e, 0x86, 0xee, 0xcf, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x4e, 0x1e, 0x0d, 0x3e, 0x93, 0x99, 0x49},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5d, 0xcc, 0x67, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45, 0x65, 0x6f, 0xa5, 0x7b, 0x19, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0xab, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x96, 0xca, 0x6c, 0x03, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0xb0, 0x6f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2b, 0xad, 0x99, 0x08, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0xc4, 0x43, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x6d, 0xc7, 0x58, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5d, 0xf2, 0xdb, 0x6b, 0x26, 0x10, 0x18, 0x13, 0x00, 0x00, 0x00, 0x00, 0x03, 0x14, 0x31, 0x69, 0xb0, 0xb7, 0x5b, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x8e, 0xa8, 0xad, 0xbc, 0xb7, 0x8e, 0x5b, 0x3b, 0x29, 0x27, 0x34, 0x46, 0x56, 0x63, 0x77, 0x9f, 0x84, 0x55, 0x67, 0x36, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0x7f, 0x64, 0x24, 0x1a, 0x43, 0x9c, 0xdc, 0xce, 0xa6, 0x9b, 0x99, 0x84, 0x6e, 0x55, 0x3d, 0x44, 0x60, 0x3d, 0x0f, 0x24, 0x59, 0x3f, 0x03, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x56, 0x81, 0x42, 0x01, 0x00, 0x00, 0x00, 0x24, 0xa0, 0xce, 0x7b, 0x62, 0x72, 0x69, 0x33, 0x07, 0x00, 0x11, 0x3d, 0x1b, 0x00, 0x00, 0x14, 0x50, 0x55, 0x21},
    {0x47, 0x20, 0x03, 0x03, 0x38, 0x7b, 0x6f, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x71, 0x59, 0x00, 0x00, 0x00, 0x1e, 0x28, 0x00, 0x00, 0x0a, 0x34, 0x0c, 0x00, 0x00, 0x00, 0x02, 0x34, 0x59},
    {0x39, 0x5e, 0x73, 0x90, 0x99, 0x4d, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x64, 0x23, 0x00, 0x00, 0x00, 0x00, 0x14, 0x14, 0x00, 0x12, 0x34, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c},
    {0x00, 0x12, 0x79, 0xb5, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x61, 0x13, 0x00, 0x00, 0x00, 0x00, 0x01, 0x19, 0x04, 0x1a, 0x33, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x37, 0x62, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x5e, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x17, 0x22, 0x32, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
  },
  {
    {0x00, 0x05, 0x4e, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x59, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x0c, 0x00, 0x00, 0x00, 0x11, 0x30, 0x04, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x09, 0x4b, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5a, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x02, 0x00, 0x00, 0x13, 0x2d, 0x01, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x0c, 0x4c, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x6f, 0x67, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x01, 0x00, 0x00, 0x19, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x0b, 0x49, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x71, 0xb2, 0x3c, 0x03, 0x00, 0x00, 0x00, 0x09, 0x24, 0x00, 0x00, 0x00, 0x45, 0x57, 0x0a, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x1b, 0x57, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5a, 0xb6, 0x6e, 0x2f, 0x28, 0x29, 0x37, 0x5b, 0x44, 0x17, 0x20, 0x4d, 0x86, 0x91, 0x5d, 0x27, 0x0d, 0x00, 0x00},
    {0x30, 0x4c, 0x94, 0x65, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x61, 0x43, 0x04, 0x0a, 0x16, 0x26, 0x5f, 0xa0, 0x82, 0x6d, 0x5e, 0x4d, 0x40, 0x4f, 0x7c, 0x87, 0x6d, 0x5b, 0x45},
    {0x93, 0xb8, 0xca, 0xa0, 0x70, 0x5c, 0x4f, 0x25, 0x00, 0x00, 0x00, 0x00, 0x20, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x75, 0xc1, 0x67, 0x16, 0x00, 0x00, 0x00, 0x2b, 0x38, 0x30, 0x40, 0x61},
    {0x5e, 0xcc, 0x86, 0x3b, 0x3d, 0x57, 0x76, 0x74, 0x43, 0x0e, 0x00, 0x00, 0x4b, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x85, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x13, 0x7a, 0x34, 0x00, 0x00, 0x00, 0x08, 0x2d, 0x51, 0x57, 0x4b, 0x61, 0x78, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5e, 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x12, 0x62, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x5e, 0xbf, 0xd1, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x82, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x1f, 0x63, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x97, 0xee, 0xa9, 0x6a, 0x50, 0x41, 0x35, 0x46, 0xab, 0xa2, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x31, 0x61, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x89, 0x81, 0x4a, 0x4e, 0x5e, 0x67, 0x6a, 0x87, 0xd6, 0xb9, 0x34, 0x00, 0x00, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x4f, 0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x78, 0x12, 0x00, 0x00, 0x00, 0x00, 0x05, 0x13, 0x57, 0xa5, 0x68, 0x0b, 0x00, 0x00, 0x02, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00},
    {0x80, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x79, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x63, 0x75, 0x19, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x02, 0x1c},
    {0x88, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x54, 0x67, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x6d, 0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x4c},
    {0x72, 0x2e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x7a, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x48, 0x52, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a},
    {0x99, 0x81, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x62, 0x5f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x24, 0x3f, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a},
    {0xcb, 0xc0, 0x97, 0x79, 0x5c, 0x39, 0x25, 0x55, 0x8b, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x13, 0x10, 0x3f, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x83},
    {0x5f, 0x46, 0x55, 0x6d, 0x85, 0x92, 0xa9, 0xd6, 0x89, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x0a, 0x00, 0x17, 0x42, 0x11, 0x00, 0x00, 0x05, 0x42, 0x90, 0x95},
    {0x00, 0x00, 0x00, 0x00, 0x08, 0x28, 0x6e, 0xdd, 0xa4, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x06, 0x00, 0x00, 0x2c, 0x43, 0x18, 0x1e, 0x64, 0x98, 0x67, 0x17},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x59, 0xbd, 0x4d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x03, 0x00, 0x00, 0x06, 0x4a, 0x64, 0x84, 0x95, 0x41, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8b, 0x9b, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x3b, 0xaa, 0xab, 0x31, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0xb0, 0x5b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x0c, 0x00, 0x00, 0x00, 0x13, 0x62, 0xba, 0x60, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x86, 0xb3, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x32, 0x22, 0x24, 0x31, 0x40, 0x5c, 0xad, 0xb8, 0x30, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5e, 0xec, 0xc4, 0x51, 0x17, 0x04, 0x01, 0x0c, 0x3b, 0x71, 0x70, 0x68, 0x67, 0x61, 0x53, 0x52, 0x84, 0xaa, 0x41, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x86, 0xb1, 0xb8, 0xb9, 0xa6, 0x7e, 0x74, 0x8a, 0x9c, 0x8c, 0x69, 0x4b, 0x2d, 0x16, 0x06, 0x06, 0x2f, 0x8f, 0x7b, 0x05, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x76, 0x6a, 0x2d, 0x25, 0x5b, 0xae, 0xf4, 0xeb, 0xb2, 0x77, 0x3d, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x61, 0x87, 0x4e, 0x04, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x7b, 0x49, 0x06, 0x00, 0x00, 0x01, 0x4b, 0xb8, 0x8e, 0x39, 0x2f, 0x2b, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x45, 0x4e, 0x56, 0x4f, 0x1b, 0x00},
    {0x10, 0x00, 0x00, 0x00, 0x25, 0x6b, 0x71, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x37, 0x75, 0x17, 0x00, 0x00, 0x0e, 0x1b, 0x05, 0x00, 0x00, 0x00, 0x00, 0x07, 0x3b, 0x27, 0x0c, 0x34, 0x58, 0x3f},
    {0x55, 0x4b, 0x41, 0x65, 0x85, 0x51, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x43, 0x5b, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x14, 0x00, 0x00, 0x00, 0x00, 0x09, 0x35, 0x11, 0x00, 0x00, 0x15, 0x43},
    {0x1a, 0x5f, 0xa8, 0x9e, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x4b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x09, 0x00, 0x00, 0x00, 0x0c, 0x31, 0x08, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x12, 0x75, 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x57, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x0d, 0x00, 0x00, 0x00, 0x0f, 0x30, 0x06, 0x00, 0x00, 0x00, 0x00}
  },
  {
    {0x00, 0x36, 0x4a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x70, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x2f, 0x06, 0x00, 0x00},
    {0x00, 0x30, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x72, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x2d, 0x02, 0x00, 0x00},
    {0x00, 0x33, 0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x76, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x3e, 0x02, 0x00, 0x00},
    {0x00, 0x40, 0x3a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x8c, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x03, 0x00, 0x00, 0x00, 0x02, 0x24, 0x71, 0x87, 0x31, 0x00, 0x00},
    {0x32, 0x61, 0x52, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0xa1, 0xac, 0x2d, 0x0a, 0x00, 0x00, 0x14, 0x2d, 0x04, 0x00, 0x14, 0x3f, 0x5f, 0x73, 0x86, 0x98, 0x94, 0x68, 0x3d},
    {0x94, 0xbc, 0x9c, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x8c, 0x9a, 0x48, 0x2b, 0x30, 0x57, 0x71, 0x45, 0x35, 0x52, 0x68, 0x5e, 0x40, 0x27, 0x21, 0x43, 0x65, 0x64, 0x6f},
    {0x4d, 0xd7, 0xf3, 0x95, 0x55, 0x38, 0x24, 0x10, 0x03, 0x00, 0x00, 0x00, 0x23, 0x5e, 0x10, 0x00, 0x02, 0x11, 0x47, 0x93, 0x94, 0x84, 0x5a, 0x26, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x09},
    {0x00, 0x6c, 0xc0, 0x79, 0x55, 0x5f, 0x6e, 0x74, 0x57, 0x17, 0x00, 0x00, 0x45, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0xb6, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x39, 0x7b, 0x1b, 0x00, 0x04, 0x1c, 0x3a, 0x54, 0x5b, 0x4c, 0x5c, 0x80, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5f, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x38, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x62, 0xc1, 0xe4, 0x7c, 0x2e, 0x10, 0x02, 0x00, 0x30, 0x8d, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x45, 0x5d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a, 0xad, 0xba, 0x82, 0x72, 0x68, 0x60, 0x67, 0xbf, 0xc5, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x57, 0x52, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x82, 0x2b, 0x0c, 0x14, 0x24, 0x33, 0x46, 0x8a, 0xcb, 0x5f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x68, 0x3a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x74, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x71, 0xa4, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x1a, 0x72, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4f, 0x69, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x90, 0x7d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x55, 0x68, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x78, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0xa8, 0x55, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x00},
    {0x9e, 0x5d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x59, 0x5f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0xa9, 0x42, 0x0a, 0x06, 0x0d, 0x2a, 0x27, 0x13, 0x13, 0x43},
    {0xdc, 0xa2, 0x49, 0x17, 0x01, 0x00, 0x00, 0x33, 0x7c, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x93, 0x86, 0x24, 0x08, 0x09, 0x0b, 0x18, 0x26, 0x3c, 0x8a},
    {0xbc, 0xa8, 0x96, 0x89, 0x76, 0x5e, 0x5b, 0x93, 0x69, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x51, 0x78, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x62, 0xab},
    {0x2a, 0x1e, 0x2d, 0x48, 0x66, 0x85, 0xc8, 0xdc, 0x56, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2d, 0x4f, 0x3f, 0x07, 0x00, 0x00, 0x23, 0x6e, 0x91, 0x65},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x59, 0xcc, 0x87, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x20, 0x36, 0x35, 0x17, 0x3b, 0x83, 0x80, 0x2f, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x62, 0xb7, 0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x22, 0x0b, 0x1d, 0x56, 0x76, 0x97, 0x71, 0x13, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x99, 0x8f, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0x37, 0x20, 0x33, 0x83, 0xcf, 0x84, 0x0f, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0xb4, 0x4a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3b, 0x82, 0x77, 0x64, 0x6b, 0xaa, 0xc8, 0x49, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x93, 0xa6, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x2d, 0x73, 0xa7, 0x96, 0x67, 0x45, 0x37, 0x5c, 0x9f, 0x55, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0xec, 0xb1, 0x3f, 0x13, 0x0b, 0x18, 0x3f, 0x77, 0x9c, 0x95, 0x5f, 0x25, 0x09, 0x00, 0x00, 0x10, 0x76, 0x7a, 0x04, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x83, 0xc0, 0xc6, 0xbe, 0xa4, 0x90, 0x97, 0x9b, 0x89, 0x59, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x92, 0x33, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x70, 0x72, 0x37, 0x37, 0x7d, 0xdf, 0xf4, 0xa9, 0x5f, 0x26, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x82, 0x78, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x78, 0x52, 0x0c, 0x00, 0x00, 0x19, 0x8c, 0xb9, 0x57, 0x2b, 0x1d, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4a, 0xa6, 0x4d, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x18, 0x61, 0x75, 0x32, 0x00, 0x00, 0x00, 0x00, 0x04, 0x73, 0x67, 0x03, 0x00, 0x0c, 0x15, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x89, 0x99, 0x40, 0x06},
    {0x33, 0x14, 0x16, 0x4d, 0x7b, 0x5a, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x73, 0x34, 0x00, 0x00, 0x00, 0x04, 0x0f, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x56, 0x62, 0x5a, 0x4f},
    {0x59, 0x7d, 0x97, 0x84, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x71, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x05, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x40, 0x25, 0x07, 0x26},
    {0x09, 0x6e, 0x91, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x6f, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x34, 0x0d, 0x00, 0x00}
  },
  {
    {0x2b, 0x60, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x39, 0x24},
    {0x1b, 0x42, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x62, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x31, 0x13},
    {0x29, 0x48, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x59, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a, 0x4f, 0x20},
    {0x58, 0x65, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x32, 0x55, 0x88, 0x9f, 0x69},
    {0xbc, 0xb4, 0x3d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x92, 0x27, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x0a, 0x38, 0x62, 0x75, 0x75, 0x71, 0x78, 0x93, 0xa6},
    {0x7c, 0xe9, 0xa7, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0xb4, 0x97, 0x2a, 0x13, 0x12, 0x29, 0x24, 0x05, 0x13, 0x46, 0x72, 0x72, 0x54, 0x32, 0x19, 0x14, 0x24, 0x23, 0x30},
    {0x00, 0x71, 0xef, 0xa5, 0x41, 0x18, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x93, 0x70, 0x39, 0x3c, 0x6a, 0x59, 0x35, 0x51, 0x75, 0x6d, 0x3c, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x07, 0xad, 0xce, 0x7a, 0x60, 0x5b, 0x4e, 0x3a, 0x24, 0x0d, 0x12, 0x57, 0x51, 0x00, 0x00, 0x01, 0x33, 0x7a, 0x8e, 0x7e, 0x3f, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x6c, 0x80, 0x24, 0x1d, 0x34, 0x4b, 0x60, 0x6c, 0x5d, 0x67, 0x9e, 0x5a, 0x00, 0x00, 0x00, 0x03, 0x8e, 0x9b, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x5d, 0x58, 0x00, 0x00, 0x00, 0x00, 0x07, 0x31, 0x74, 0xc7, 0xcb, 0x8b, 0x60, 0x40, 0x39, 0x71, 0xb1, 0x52, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x64, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5d, 0x9e, 0x58, 0x3b, 0x44, 0x4d, 0x5e, 0xa2, 0xd9, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x09, 0x72, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x71, 0x44, 0x00, 0x00, 0x00, 0x00, 0x03, 0x21, 0x93, 0xa6, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x21, 0x79, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4c, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0xa4, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x42, 0x6e, 0x02, 0x00, 0x00, 0x00, 0x00, 0x18, 0x74, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45, 0xb1, 0x3a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x6b, 0x4d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x57, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7b, 0xa0, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x4e, 0xa1, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x26, 0x72, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x9f, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0xed, 0xe4, 0x84, 0x44, 0x19, 0x09, 0x1a, 0x72, 0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xa5, 0x49, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x2c, 0x34, 0x77},
    {0xc3, 0x9b, 0x8e, 0x88, 0x80, 0x7f, 0xa2, 0x9d, 0x2c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x69, 0x95, 0x2b, 0x00, 0x00, 0x14, 0x45, 0x93, 0xd1, 0xdd},
    {0x1b, 0x0d, 0x11, 0x21, 0x3c, 0x6c, 0xd0, 0xbd, 0x2c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x86, 0x86, 0x3b, 0x33, 0x51, 0x91, 0xc2, 0xa0, 0x50},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0xba, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x5a, 0x9b, 0x70, 0x57, 0x86, 0x9f, 0x5a, 0x0d, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6e, 0xac, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x73, 0x9b, 0xa8, 0xad, 0x9c, 0x44, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xa3, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x88, 0xa8, 0x8f, 0xaa, 0xd3, 0x6f, 0x04, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0xb0, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a, 0x75, 0xa4, 0x92, 0x55, 0x30, 0x43, 0x94, 0x6e, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x9b, 0x94, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x06, 0x36, 0x79, 0xa6, 0x9c, 0x5e, 0x1c, 0x00, 0x00, 0x05, 0x5e, 0x86, 0x15, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6a, 0xe4, 0x9a, 0x33, 0x1a, 0x29, 0x51, 0x84, 0xa4, 0x99, 0x60, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x8f, 0x4b, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x7d, 0xca, 0xd2, 0xc2, 0xae, 0xa5, 0x9c, 0x83, 0x51, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x70, 0x87, 0x07, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x66, 0x77, 0x44, 0x53, 0xb6, 0xe7, 0x98, 0x48, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x9b, 0x4a, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2e, 0x71, 0x59, 0x13, 0x00, 0x05, 0x53, 0xad, 0x77, 0x29, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x70, 0x91, 0x0d, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x0d, 0x54, 0x75, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x3d, 0x9a, 0x4d, 0x0b, 0x0a, 0x0d, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x9b, 0x65, 0x00},
    {0x02, 0x00, 0x05, 0x39, 0x71, 0x61, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4c, 0x7d, 0x05, 0x00, 0x00, 0x04, 0x09, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5d, 0xac, 0x4f},
    {0x60, 0x53, 0x70, 0x76, 0x3a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0x61, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x8c, 0x99},
    {0x69, 0x9c, 0x74, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x56, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x53, 0x58}
  },
  {
    {0x00, 0x39, 0x6e, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x7a, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x09, 0x00, 0x00},
    {0x00, 0x36, 0x5d, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x7c, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x06, 0x00, 0x00},
    {0x00, 0x47, 0x59, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x7c, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x23, 0x00, 0x00},
    {0x1d, 0x74, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x77, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x2f, 0x70, 0x8c, 0x48, 0x15},
    {0xa1, 0xd8, 0x8c, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x69, 0x5d, 0x00, 0x00, 0x00, 0x00, 0x05, 0x01, 0x00, 0x00, 0x00, 0x01, 0x2c, 0x5a, 0x74, 0x80, 0x88, 0x92, 0x97, 0x92},
    {0x77, 0xdb, 0xd8, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4f, 0x8c, 0x22, 0x00, 0x02, 0x0c, 0x07, 0x00, 0x00, 0x05, 0x39, 0x70, 0x7c, 0x64, 0x41, 0x25, 0x18, 0x27, 0x45, 0x54},
    {0x00, 0x34, 0xca, 0xba, 0x38, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0xa4, 0x73, 0x24, 0x14, 0x10, 0x05, 0x0a, 0x34, 0x6f, 0x7b, 0x4f, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x53, 0xd9, 0xa5, 0x55, 0x37, 0x25, 0x13, 0x06, 0x02, 0x1a, 0x70, 0x9f, 0x6e, 0x49, 0x2d, 0x1d, 0x30, 0x62, 0x7b, 0x53, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x16, 0x99, 0x8b, 0x46, 0x43, 0x50, 0x57, 0x59, 0x60, 0x8a, 0xc1, 0xa4, 0x6e, 0x6b, 0x7c, 0x80, 0x8b, 0x73, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x0e, 0x75, 0x48, 0x00, 0x00, 0x0a, 0x26, 0x54, 0x9d, 0xbd, 0x86, 0x65, 0x64, 0x70, 0xaf, 0xee, 0x99, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x14, 0x6f, 0x27, 0x00, 0x00, 0x00, 0x00, 0x2a, 0x7f, 0x57, 0x11, 0x04, 0x08, 0x17, 0x45, 0xb1, 0xa3, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x27, 0x73, 0x15, 0x00, 0x00, 0x00, 0x00, 0x4a, 0x59, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f, 0xa8, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x43, 0x6d, 0x03, 0x00, 0x00, 0x00, 0x1d, 0x68, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x57, 0xaa, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x63, 0x58, 0x00, 0x00, 0x00, 0x01, 0x5c, 0x4f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x8c, 0x90, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x17, 0x83, 0x40, 0x00, 0x00, 0x07, 0x43, 0x73, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0xac, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x06, 0x80, 0xa7, 0x49, 0x1c, 0x0e, 0x1a, 0x55, 0x54, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x57, 0xad, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x9f, 0xfa, 0xcb, 0x67, 0x29, 0x04, 0x01, 0x2f, 0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x82, 0x90, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x23, 0x41},
    {0xcc, 0xa2, 0x82, 0x6e, 0x3e, 0x06, 0x00, 0x28, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x93, 0x63, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x51, 0xb1, 0xd6},
    {0x2f, 0x0d, 0x0b, 0x21, 0x41, 0x43, 0x25, 0x39, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x93, 0x44, 0x0a, 0x0b, 0x28, 0x7a, 0xc3, 0xc2, 0x7b},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x59, 0x76, 0x5d, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x88, 0xb9, 0x85, 0x7f, 0xb2, 0xc9, 0x87, 0x29, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x89, 0x9b, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x4e, 0xab, 0xe4, 0xe0, 0xfa, 0xd8, 0x62, 0x09, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2d, 0xa9, 0x68, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x75, 0xa3, 0x95, 0x6b, 0x6d, 0xb0, 0x98, 0x21, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x62, 0xa0, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x73, 0xa2, 0x96, 0x59, 0x1e, 0x08, 0x0b, 0x49, 0x86, 0x2d, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x98, 0x7a, 0x08, 0x00, 0x00, 0x00, 0x0f, 0x3f, 0x7d, 0xa5, 0x9b, 0x62, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x13, 0x7a, 0x5d, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6a, 0xcd, 0x7c, 0x33, 0x3b, 0x60, 0x89, 0xa0, 0x90, 0x59, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x53, 0x89, 0x18, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x6f, 0xca, 0xd7, 0xc1, 0xa9, 0x93, 0x72, 0x40, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x8a, 0x5c, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x55, 0x72, 0x54, 0x84, 0xcb, 0x89, 0x33, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5a, 0x95, 0x1c, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x60, 0x59, 0x18, 0x03, 0x30, 0x86, 0x68, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x95, 0x84, 0x04},
    {0x0d, 0x00, 0x00, 0x00, 0x05, 0x41, 0x6a, 0x41, 0x04, 0x00, 0x00, 0x11, 0x83, 0x84, 0x22, 0x06, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x62, 0xa0, 0x61},
    {0x66, 0x32, 0x11, 0x32, 0x62, 0x5e, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x11, 0x89, 0x63, 0x02, 0x01, 0x03, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x38, 0x3b, 0x56},
    {0x3d, 0x89, 0x95, 0x81, 0x43, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x80, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x1d, 0x00, 0x00},
    {0x00, 0x62, 0xac, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x7a, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x0d, 0x00, 0x00}
  },
  {
    {0x00, 0x00, 0x11, 0x6c, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x61, 0x56, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x0a, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x18, 0x6e, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5f, 0x59, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x07, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x1f, 0x6d, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x57, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x0c, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x37, 0x6e, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x70, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x63, 0x61, 0x0b, 0x00, 0x00},
    {0x28, 0x28, 0x71, 0x7e, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x87, 0x2c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0x4f, 0x80, 0xa9, 0xb3, 0x88, 0x5e, 0x40},
    {0x90, 0x9d, 0xda, 0xab, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x82, 0x61, 0x0f, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a, 0x68, 0x81, 0x72, 0x57, 0x3d, 0x41, 0x68, 0x91, 0x9d},
    {0x2c, 0x55, 0xbb, 0xdd, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x6d, 0x56, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x66, 0x83, 0x60, 0x28, 0x00, 0x00, 0x00, 0x00, 0x02, 0x1c, 0x21},
    {0x00, 0x00, 0x1d, 0xa7, 0xbc, 0x4f, 0x16, 0x05, 0x00, 0x00, 0x05, 0x3e, 0x8e, 0x67, 0x21, 0x07, 0x03, 0x04, 0x18, 0x52, 0x7f, 0x65, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x49, 0xb8, 0x8b, 0x55, 0x4e, 0x53, 0x56, 0x69, 0x9e, 0xc8, 0xae, 0x84, 0x64, 0x4a, 0x51, 0x7c, 0x78, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x2f, 0x8e, 0x64, 0x44, 0x58, 0x70, 0x81, 0xa7, 0xa9, 0x78, 0x70, 0x86, 0xa1, 0xc6, 0xcc, 0x83, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x3f, 0x9b, 0x57, 0x1e, 0x14, 0x17, 0x30, 0x63, 0x4d, 0x10, 0x05, 0x0f, 0x30, 0x88, 0xd5, 0x72, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x78, 0x9d, 0x27, 0x00, 0x00, 0x00, 0x07, 0x3a, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x04, 0x72, 0xa3, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x2c, 0xad, 0x5b, 0x00, 0x00, 0x00, 0x00, 0x06, 0x33, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x94, 0x7b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x8b, 0x97, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x33, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x37, 0xac, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x55, 0xb4, 0x3d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x31, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6e, 0xa4, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x37, 0xb2, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x30, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x98, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0xaf, 0x8d, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x30, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0xa5, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x4c},
    {0xc6, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x30, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5f, 0x94, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x72, 0xe2},
    {0x85, 0x59, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x30, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x80, 0x7e, 0x1d, 0x03, 0x04, 0x10, 0x3e, 0x93, 0xca, 0xbd},
    {0x1b, 0x30, 0x43, 0x33, 0x08, 0x00, 0x00, 0x00, 0x00, 0x08, 0x30, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x56, 0xc3, 0xb1, 0x77, 0x70, 0x8f, 0xc1, 0xb5, 0x68, 0x25},
    {0x00, 0x00, 0x09, 0x30, 0x40, 0x1b, 0x00, 0x00, 0x00, 0x06, 0x31, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3a, 0x91, 0xd5, 0xde, 0xc9, 0xe1, 0xfa, 0xab, 0x3a, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x19, 0x42, 0x2d, 0x00, 0x00, 0x05, 0x35, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x6e, 0x9d, 0x96, 0x6a, 0x4e, 0x56, 0x81, 0xb6, 0x69, 0x04, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x3b, 0x39, 0x09, 0x09, 0x3a, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x06, 0x35, 0x76, 0x9f, 0x95, 0x5c, 0x20, 0x06, 0x00, 0x00, 0x11, 0x66, 0x70, 0x07, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0x43, 0x28, 0x46, 0x3d, 0x05, 0x01, 0x1a, 0x4d, 0x83, 0xa0, 0x92, 0x5b, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x7e, 0x31, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3a, 0x61, 0x76, 0x82, 0x65, 0x6e, 0x89, 0x95, 0x7f, 0x4a, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x70, 0x70, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x80, 0xc4, 0xdd, 0xb9, 0x87, 0x5b, 0x2e, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0xa0, 0x51, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x46, 0x77, 0x8b, 0xb9, 0x86, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x85, 0x90, 0x40},
    {0x52, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x11, 0x4f, 0x56, 0x24, 0x28, 0x73, 0x61, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0x3c, 0x31, 0x63},
    {0x4c, 0x65, 0x2c, 0x01, 0x08, 0x33, 0x5b, 0x43, 0x09, 0x00, 0x04, 0x60, 0x72, 0x23, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x2a, 0x08, 0x00, 0x08},
    {0x00, 0x38, 0x7a, 0x71, 0x6f, 0x63, 0x25, 0x00, 0x00, 0x00, 0x00, 0x5d, 0x90, 0x31, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x1c, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x3f, 0xae, 0x87, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x7f, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x13, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x12, 0x7c, 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x61, 0x5e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x0d, 0x00, 0x00, 0x00}
  },
  {
    {0x00, 0x00, 0x00, 0x00, 0x5b, 0x5d, 0x00, 0x00, 0x00, 0x00, 0x2f, 0x7c, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x0b, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x01, 0x66, 0x56, 0x00, 0x00, 0x00, 0x00, 0x2b, 0x74, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x0a, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x08, 0x6e, 0x4a, 0x00, 0x00, 0x00, 0x00, 0x24, 0x78, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x07, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x13, 0x76, 0x3d, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x7e, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4f, 0x34, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x3a, 0x86, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x10, 0x71, 0x5c, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x59, 0xb8, 0xbc, 0x5d, 0x26, 0x07, 0x00},
    {0x60, 0x47, 0x47, 0x7d, 0x8d, 0x37, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x4b, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x5c, 0x85, 0x8d, 0x84, 0x86, 0x92, 0x96, 0x8f, 0x7c},
    {0x87, 0x8f, 0xb5, 0xbc, 0x85, 0x64, 0x28, 0x0f, 0x0a, 0x0b, 0x31, 0x6f, 0x4a, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x5b, 0x89, 0x72, 0x3b, 0x0e, 0x00, 0x00, 0x0a, 0x35, 0x74, 0x8d},
    {0x02, 0x19, 0x5a, 0xab, 0xba, 0xaa, 0x84, 0x6d, 0x69, 0x6c, 0x8b, 0xb8, 0x79, 0x23, 0x01, 0x00, 0x00, 0x00, 0x0b, 0x45, 0x81, 0x76, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x69, 0xd1, 0x90, 0x56, 0x50, 0x56, 0x60, 0x7c, 0xb6, 0xcb, 0x92, 0x54, 0x2e, 0x21, 0x38, 0x6f, 0x80, 0x44, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x6d, 0xa0, 0x2e, 0x02, 0x00, 0x00, 0x04, 0x12, 0x42, 0xa2, 0xc3, 0xa5, 0xa0, 0xa6, 0xa7, 0x7f, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x11, 0x9b, 0x6f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x4e, 0x64, 0x50, 0x71, 0xcd, 0xd1, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x5a, 0xb0, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0x31, 0x00, 0x00, 0x3b, 0xac, 0x72, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x0d, 0xad, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x1e, 0x00, 0x00, 0x00, 0x51, 0xa6, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x77, 0xb4, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2e, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x82, 0x90, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x47, 0xc2, 0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2e, 0x16, 0x00, 0x00, 0x00, 0x00, 0x21, 0xa6, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0xbe, 0x84, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2d, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x53, 0xaa, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x49},
    {0xa1, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2d, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x80, 0x8f, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3b, 0xbf},
    {0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x93, 0x63, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x8b, 0xb1},
    {0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4e, 0x9c, 0x50, 0x0f, 0x02, 0x00, 0x06, 0x1c, 0x7c, 0xbe, 0x60},
    {0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f, 0xae, 0xe3, 0xa2, 0x7c, 0x70, 0x77, 0xa6, 0xe1, 0xad, 0x4b},
    {0x3f, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x87, 0xc1, 0xda, 0xd1, 0xca, 0xd1, 0xf4, 0xf1, 0x97, 0x5c, 0x4e},
    {0x24, 0x3e, 0x31, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x25, 0x00, 0x00, 0x00, 0x03, 0x33, 0x74, 0x9a, 0x92, 0x65, 0x41, 0x40, 0x4d, 0x63, 0x99, 0xac, 0x4f, 0x10, 0x0a},
    {0x00, 0x08, 0x2d, 0x3b, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x43, 0x0e, 0x14, 0x46, 0x80, 0x9e, 0x8d, 0x55, 0x1c, 0x03, 0x00, 0x00, 0x00, 0x01, 0x24, 0x74, 0x5b, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x17, 0x3c, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x85, 0x80, 0x8a, 0x96, 0x81, 0x4a, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x50, 0x8a, 0x2e, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x08, 0x38, 0x33, 0x04, 0x00, 0x00, 0x00, 0x18, 0x8b, 0xd0, 0x9f, 0x67, 0x32, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x42, 0x91, 0x83, 0x28},
    {0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x3f, 0x12, 0x08, 0x1d, 0x79, 0xc4, 0x82, 0x2a, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x48, 0x42, 0x4e, 0x72},
    {0x68, 0x59, 0x0f, 0x00, 0x00, 0x00, 0x06, 0x52, 0x70, 0x6a, 0x96, 0xc7, 0x7c, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x36, 0x00, 0x00, 0x23},
    {0x09, 0x53, 0x6a, 0x2e, 0x00, 0x00, 0x1c, 0x61, 0x88, 0x87, 0xbe, 0xa3, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x33, 0x12, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x38, 0x75, 0x5e, 0x4a, 0x62, 0x4f, 0x22, 0x23, 0x6e, 0x79, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x2f, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x2d, 0x91, 0x9f, 0x49, 0x02, 0x00, 0x00, 0x47, 0x73, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x22, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x65, 0x80, 0x0d, 0x00, 0x00, 0x00, 0x36, 0x80, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x15, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x53, 0x63, 0x00, 0x00, 0x00, 0x00, 0x31, 0x89, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x0d, 0x00, 0x00, 0x00, 0x00}
  },
  {
    {0x00, 0x00, 0x00, 0x00, 0x11, 0x45, 0x2f, 0x39, 0x40, 0x16, 0x00, 0x2f, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2e, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x30, 0x3a, 0x0e, 0x3b, 0x50, 0x0b, 0x00, 0x15, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2e, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x0d, 0x44, 0x13, 0x00, 0x35, 0x54, 0x09, 0x00, 0x01, 0x2a, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2d, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x36, 0x2f, 0x00, 0x00, 0x26, 0x52, 0x0d, 0x00, 0x00, 0x24, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x23, 0x3e, 0x04, 0x00, 0x00, 0x15, 0x52, 0x16, 0x00, 0x00, 0x1b, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x92, 0x98, 0x23, 0x00, 0x00, 0x00, 0x00},
    {0x0d, 0x30, 0x52, 0x15, 0x00, 0x00, 0x00, 0x13, 0x61, 0x3b, 0x00, 0x00, 0x1d, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x49, 0x8c, 0xb8, 0xc5, 0xa2, 0x7b, 0x58, 0x32, 0x15},
    {0xa2, 0x99, 0x46, 0x03, 0x00, 0x00, 0x07, 0x43, 0x9d, 0x9d, 0x3b, 0x16, 0x38, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x47, 0x7e, 0x79, 0x51, 0x27, 0x1e, 0x3d, 0x63, 0x89, 0x9d, 0x9e},
    {0x7c, 0xa2, 0x66, 0x23, 0x18, 0x37, 0x67, 0x86, 0x93, 0x9f, 0x9b, 0x90, 0x93, 0x5e, 0x15, 0x00, 0x00, 0x00, 0x03, 0x33, 0x75, 0x7c, 0x43, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x37, 0x5e},
    {0x00, 0x34, 0x8e, 0x8a, 0x85, 0x83, 0x6a, 0x44, 0x28, 0x2b, 0x50, 0x88, 0xc7, 0xc2, 0x63, 0x21, 0x12, 0x21, 0x5a, 0x7d, 0x53, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x7e, 0xdf, 0x8a, 0x33, 0x09, 0x00, 0x00, 0x00, 0x00, 0x12, 0x4d, 0xaf, 0xd1, 0x9a, 0x71, 0x80, 0x78, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x97, 0xa6, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f, 0xa0, 0xdf, 0xe3, 0xa3, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x39, 0xb6, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x7f, 0xab, 0xa3, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x98, 0x92, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x46, 0x3f, 0x8e, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x63, 0xbb, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x35, 0x0d, 0x40, 0x97, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0xc5, 0x67, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x2d, 0x00, 0x00, 0x6b, 0x8f, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x59},
    {0x98, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x25, 0x00, 0x00, 0x14, 0x90, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3d, 0xce},
    {0x2c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x22, 0x00, 0x00, 0x00, 0x3d, 0x98, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x96, 0xb9},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x22, 0x00, 0x00, 0x00, 0x00, 0x63, 0x85, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5d, 0xb9, 0x57},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x23, 0x00, 0x00, 0x00, 0x00, 0x1d, 0x89, 0x7f, 0x2a, 0x0d, 0x03, 0x01, 0x09, 0x42, 0xac, 0x7c, 0x0e},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x27, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x89, 0xe5, 0xc5, 0x97, 0x7f, 0x71, 0x7f, 0xba, 0xa2, 0x32, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x38, 0x05, 0x00, 0x05, 0x3a, 0x7e, 0xa4, 0xb5, 0xba, 0xbc, 0xc2, 0xc7, 0xea, 0xff, 0x8c, 0x25, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x5a, 0x43, 0x4b, 0x75, 0x8b, 0x79, 0x4e, 0x2d, 0x28, 0x32, 0x43, 0x59, 0x7f, 0xc4, 0xca, 0x64, 0x14},
    {0x2e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4b, 0xb1, 0xab, 0x8c, 0x6c, 0x3a, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x1f, 0x61, 0xcd, 0xd0, 0x7d},
    {0xa8, 0x55, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2e, 0xaf, 0xcd, 0x77, 0x2c, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x4e, 0x6c, 0x9f, 0xcc},
    {0xbd, 0xc3, 0x7a, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0xa4, 0xc1, 0x56, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f, 0x3d, 0x12, 0x1f, 0x69},
    {0x3b, 0x9e, 0xca, 0x9a, 0x3e, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x9d, 0xbd, 0x52, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x41, 0x0e, 0x00, 0x00, 0x00},
    {0x00, 0x1a, 0x79, 0xc2, 0xb3, 0x59, 0x14, 0x00, 0x00, 0x03, 0x2b, 0x96, 0xba, 0x53, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x30, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x05, 0x53, 0xb1, 0xc2, 0x7c, 0x41, 0x31, 0x48, 0x99, 0xba, 0x57, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x3f, 0x11, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x3b, 0xa9, 0xc1, 0x81, 0x72, 0xa4, 0xc7, 0x69, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x51, 0x97, 0x6c, 0x51, 0x8c, 0x98, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2b, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x6e, 0x4b, 0x26, 0x48, 0x64, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2e, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x3b, 0x4f, 0x37, 0x29, 0x28, 0x29, 0x3e, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2d, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00}
  },
  {
    {0x00, 0x00, 0x00, 0x21, 0x3b, 0x02, 0x0a, 0x65, 0x78, 0x0b, 0x00, 0x00, 0x00, 0x17, 0x3c, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x37, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x01, 0x38, 0x17, 0x00, 0x00, 0x53, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x37, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x21, 0x2f, 0x00, 0x00, 0x00, 0x45, 0x61, 0x02, 0x00, 0x00, 0x00, 0x00, 0x29, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x0c, 0x35, 0x08, 0x00, 0x00, 0x00, 0x3a, 0x65, 0x08, 0x00, 0x00, 0x00, 0x00, 0x1d, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3b, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x05, 0x35, 0x14, 0x00, 0x00, 0x00, 0x00, 0x28, 0x68, 0x12, 0x00, 0x00, 0x00, 0x00, 0x16, 0x39, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5d, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x3f, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x6f, 0x2c, 0x00, 0x00, 0x00, 0x00, 0x10, 0x38, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x44, 0xac, 0xd4, 0x88, 0x43, 0x1a, 0x01, 0x00, 0x12},
    {0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x99, 0x86, 0x0c, 0x00, 0x00, 0x00, 0x12, 0x3a, 0x06, 0x00, 0x00, 0x00, 0x00, 0x2e, 0x6a, 0x81, 0x6f, 0x68, 0x7b, 0x89, 0x8c, 0x84, 0x7b, 0x72},
    {0x24, 0x00, 0x00, 0x00, 0x00, 0x07, 0x3b, 0x86, 0xcc, 0xdc, 0x9b, 0x50, 0x1a, 0x07, 0x34, 0x4b, 0x10, 0x00, 0x00, 0x20, 0x5f, 0x77, 0x4e, 0x15, 0x00, 0x00, 0x00, 0x0a, 0x2e, 0x60, 0xad, 0x9c},
    {0x5d, 0x11, 0x00, 0x0e, 0x42, 0x7f, 0x97, 0x8c, 0x70, 0x78, 0x9f, 0xb0, 0xa5, 0x92, 0x9c, 0x79, 0x32, 0x1e, 0x44, 0x71, 0x5a, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x72},
    {0x5c, 0x69, 0x61, 0x8c, 0x9f, 0x80, 0x43, 0x15, 0x06, 0x08, 0x17, 0x3d, 0x73, 0xa7, 0xe1, 0xde, 0x8c, 0x79, 0x75, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c},
    {0x20, 0xb8, 0xd8, 0x8c, 0x3d, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0x60, 0xcc, 0xff, 0xaf, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x39, 0xcb, 0x87, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0xdf, 0xbf, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x95, 0x9a, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x7e, 0xd2, 0x4f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a},
    {0xbe, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0xc6, 0x9c, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x7e},
    {0x7b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x8c, 0xc7, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0xd0},
    {0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x52, 0xa3, 0x8d, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x9f, 0xae},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x5e, 0x87, 0x5c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5f, 0xbe, 0x4f},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2b, 0x37, 0x47, 0x7c, 0x3a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f, 0xaa, 0x7d, 0x0a},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f, 0x33, 0x22, 0x69, 0x92, 0x4e, 0x1f, 0x0e, 0x06, 0x08, 0x26, 0x8f, 0x97, 0x28, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x54, 0x5a, 0x45, 0x7a, 0xca, 0xc9, 0xa4, 0x8f, 0x7d, 0x7a, 0xa7, 0xb5, 0x54, 0x05, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x73, 0x66, 0x7f, 0x8e, 0x82, 0x89, 0x99, 0xa7, 0xb1, 0xc6, 0xfe, 0xc3, 0x46, 0x0a, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x67, 0x45, 0x55, 0x68, 0x37, 0x18, 0x13, 0x19, 0x26, 0x3b, 0x5d, 0x9d, 0xe3, 0xac, 0x40, 0x06},
    {0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x61, 0x32, 0x3b, 0x57, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x51, 0xa4, 0xd9, 0xc9, 0x72},
    {0xb2, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f, 0x62, 0x2f, 0x32, 0x56, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x4d, 0x41, 0x56, 0xb6, 0xed},
    {0xec, 0xe3, 0x6c, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x66, 0x3c, 0x37, 0x58, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x3b, 0x35, 0x02, 0x01, 0x1f, 0x81},
    {0x46, 0xca, 0xfc, 0xa2, 0x35, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x6d, 0x5c, 0x4b, 0x5a, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x48, 0x0a, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x1c, 0x9b, 0xff, 0xd0, 0x7a, 0x50, 0x3c, 0x34, 0x33, 0x4b, 0x85, 0x8e, 0x71, 0x5c, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3b, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x09, 0x84, 0xe2, 0xa5, 0x73, 0x64, 0x63, 0x6a, 0x88, 0xc3, 0xaf, 0x64, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x48, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x17, 0x85, 0x76, 0x25, 0x14, 0x1a, 0x2b, 0x5b, 0xb7, 0x9e, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x57, 0x79, 0x18, 0x00, 0x00, 0x1a, 0x51, 0x5f, 0x69, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x44, 0x69, 0x3e, 0x0e, 0x15, 0x42, 0x24, 0x00, 0x2a, 0x45, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x35, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x09, 0x45, 0x2f, 0x37, 0x4e, 0x5c, 0x34, 0x00, 0x00, 0x00, 0x3a, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
  },
  {
    {0x00, 0x00, 0x28, 0x18, 0x00, 0x00, 0x35, 0x60, 0x6c, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x49, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x10, 0x24, 0x00, 0x00, 0x00, 0x07, 0x7b, 0x7e, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x48, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3b, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x02, 0x23, 0x09, 0x00, 0x00, 0x00, 0x00, 0x60, 0x6e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3b, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x1e, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4c, 0x6c, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x37, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x73, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3b, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x78, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x47, 0x02, 0x00, 0x00, 0x00, 0x03, 0x60, 0xa4, 0x4e, 0x0a, 0x00, 0x00, 0x09, 0x2c, 0x1e},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0x8b, 0x5f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x47, 0x07, 0x00, 0x00, 0x17, 0x56, 0x87, 0x9d, 0x96, 0x7c, 0x6a, 0x5c, 0x53, 0x2c, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x5b, 0xd0, 0xde, 0x68, 0x19, 0x00, 0x00, 0x00, 0x00, 0x32, 0x4d, 0x15, 0x0f, 0x3d, 0x61, 0x53, 0x2a, 0x0e, 0x1c, 0x3d, 0x6c, 0xa4, 0x6e, 0x06, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x0d, 0x45, 0x85, 0xa8, 0xb2, 0xc0, 0xc6, 0xaf, 0x84, 0x4c, 0x21, 0x22, 0x63, 0x65, 0x47, 0x58, 0x52, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x64, 0x26, 0x00},
    {0x00, 0x00, 0x0d, 0x4b, 0x8f, 0xa3, 0x82, 0x4c, 0x27, 0x26, 0x49, 0x7f, 0xaa, 0xb6, 0xaa, 0xb1, 0xc8, 0xa5, 0x7a, 0x3e, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x52, 0x0f},
    {0x2a, 0x4b, 0x8d, 0xa8, 0x84, 0x3c, 0x08, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1a, 0x45, 0x75, 0xaf, 0xff, 0xe0, 0x58, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x43, 0x59},
    {0xb7, 0xaf, 0x86, 0x3c, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x25, 0x91, 0xd4, 0x59, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x8d},
    {0xd7, 0x53, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2e, 0xac, 0x8e, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xb5},
    {0x5a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x74, 0xb3, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x52, 0xbf},
    {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0xac, 0x7a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x9d, 0x8c},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x7f, 0xa9, 0x2e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x62, 0xaa, 0x38},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x51, 0xc0, 0x90, 0x22, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0xa2, 0x6a, 0x03},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x59, 0xbc, 0xc1, 0x79, 0x52, 0x3c, 0x2b, 0x1c, 0x19, 0x33, 0x91, 0x90, 0x21, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x45, 0x66, 0x73, 0x80, 0x63, 0x58, 0x5e, 0x65, 0x64, 0x6a, 0x9f, 0xb5, 0x53, 0x06, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x4f, 0x4f, 0x1f, 0x28, 0x4f, 0x2d, 0x0b, 0x0f, 0x20, 0x2e, 0x42, 0x83, 0x9b, 0x3f, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x56, 0x48, 0x08, 0x00, 0x06, 0x3c, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x7d, 0x5c, 0x07, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x54, 0x40, 0x00, 0x00, 0x00, 0x00, 0x41, 0x45, 0x08, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x7d, 0xa4, 0x41, 0x03},
    {0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x50, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x05, 0x5e, 0x66, 0x34, 0x21, 0x20, 0x20, 0x33, 0x5c, 0x94, 0xde, 0xc6, 0x5d},
    {0x98, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x5e, 0x2e, 0x1b, 0x21, 0x38, 0x5b, 0x76, 0x63, 0x54, 0x74, 0xc6, 0xe3},
    {0xed, 0xd1, 0x63, 0x1e, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x57, 0x03, 0x00, 0x00, 0x00, 0x00, 0x26, 0x5a, 0x20, 0x00, 0x00, 0x00, 0x19, 0x5b, 0x4f, 0x0d, 0x01, 0x08, 0x2f, 0x94},
    {0x5a, 0xe2, 0xf5, 0xa2, 0x6e, 0x52, 0x44, 0x3b, 0x34, 0x37, 0x5d, 0x76, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x25, 0x5c, 0x23, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x4b, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x05},
    {0x00, 0x3e, 0xc9, 0xbd, 0x7d, 0x6e, 0x6a, 0x6d, 0x6c, 0x74, 0x9e, 0x7d, 0x0c, 0x00, 0x00, 0x00, 0x2e, 0x5c, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3a, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x56, 0x8a, 0x37, 0x10, 0x0c, 0x11, 0x16, 0x23, 0x4a, 0x77, 0x5a, 0x1e, 0x1c, 0x4c, 0x5d, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x49, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x15, 0x79, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x65, 0xa1, 0x97, 0x98, 0x6e, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x43, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x69, 0x70, 0x08, 0x00, 0x00, 0x00, 0x00, 0x32, 0x57, 0x42, 0x55, 0x8d, 0x54, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2e, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x07, 0x4f, 0x60, 0x39, 0x03, 0x00, 0x00, 0x27, 0x44, 0x0e, 0x00, 0x00, 0x26, 0x58, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x35, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x18, 0x3b, 0x15, 0x2d, 0x35, 0x11, 0x28, 0x4b, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x42, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x37, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
  },
  {
    {0x08, 0x1a, 0x00, 0x00, 0x00, 0x24, 0x42, 0x1f, 0x42, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x46, 0x00, 0x00, 0x00, 0x38, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x15, 0x07, 0x00, 0x00, 0x00, 0x00, 0x36, 0x77, 0x78, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x56, 0x05, 0x00, 0x00, 0x38, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x88, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x5e, 0x23, 0x00, 0x00, 0x44, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x69, 0x56, 0x0d, 0x16, 0x6a, 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x12},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x7b, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x81, 0x6b, 0x0f, 0x10, 0x53, 0x7c, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x16, 0x16, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2e, 0x81, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x7b, 0x1e, 0x00, 0x00, 0x00, 0x64, 0x65, 0x05, 0x00, 0x0e, 0x21, 0x17, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0x89, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45, 0x51, 0x00, 0x00, 0x00, 0x00, 0x1a, 0xa4, 0x83, 0x43, 0x34, 0x14, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x35, 0xbb, 0xc0, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5b, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x8a, 0xa1, 0x71, 0x1f, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x51, 0x9d, 0xd9, 0xf0, 0xc9, 0x8b, 0x4f, 0x1e, 0x01, 0x00, 0x2f, 0x72, 0x1c, 0x00, 0x00, 0x00, 0x0c, 0x39, 0x22, 0x1b, 0x47, 0x39, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x14, 0x57, 0x95, 0xa3, 0x89, 0x5f, 0x5c, 0x89, 0xb2, 0xc2, 0xb3, 0x8f, 0x76, 0x9d, 0x96, 0x24, 0x00, 0x00, 0x18, 0x26, 0x0d, 0x00, 0x00, 0x02, 0x39, 0x29, 0x00, 0x00, 0x00},
    {0x00, 0x11, 0x56, 0x9b, 0xaa, 0x7c, 0x35, 0x09, 0x00, 0x00, 0x03, 0x1b, 0x4a, 0x7e, 0xa0, 0xba, 0xf6, 0xdb, 0x5f, 0x2e, 0x2a, 0x20, 0x05, 0x00, 0x00, 0x00, 0x00, 0x06, 0x40, 0x1d, 0x00, 0x00},
    {0x4b, 0x92, 0xaa, 0x7e, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x3f, 0x88, 0xe0, 0xbe, 0x59, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x4e, 0x24, 0x1a},
    {0xa5, 0x81, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x7a, 0xbf, 0x51, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4a, 0x83, 0x95},
    {0x46, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a, 0x95, 0x79, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0xc6, 0xac},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x6b, 0xa9, 0x4d, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x79, 0xa2, 0x33},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x50, 0xb4, 0x9c, 0x55, 0x37, 0x24, 0x12, 0x06, 0x00, 0x05, 0x4a, 0x9d, 0x51, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x70, 0xa2, 0x6f, 0x55, 0x57, 0x60, 0x5f, 0x55, 0x4e, 0x67, 0xa9, 0x84, 0x1a, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x4f, 0x7e, 0x7c, 0x3b, 0x0f, 0x0c, 0x1f, 0x33, 0x41, 0x4e, 0x80, 0xb7, 0x66, 0x0b, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x58, 0x51, 0x35, 0x51, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x27, 0x83, 0x6f, 0x0d, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0x63, 0x40, 0x06, 0x00, 0x38, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x56, 0x80, 0x22, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x64, 0x34, 0x00, 0x00, 0x00, 0x20, 0x42, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0x84, 0x4e, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x63, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x12, 0x44, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x70, 0x80, 0x16},
    {0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x61, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x41, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x61, 0xbf, 0x73},
    {0x8f, 0x3a, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x5e, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x3e, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x4f, 0x83, 0xd9, 0xf0},
    {0xf3, 0xb8, 0x81, 0x61, 0x4d, 0x41, 0x35, 0x2c, 0x27, 0x36, 0x71, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x4b, 0x1a, 0x00, 0x00, 0x00, 0x18, 0x4c, 0x64, 0x5a, 0x50, 0x60, 0xb8},
    {0x94, 0xbb, 0x80, 0x6f, 0x6d, 0x73, 0x77, 0x76, 0x76, 0x93, 0xb0, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x77, 0x45, 0x16, 0x16, 0x45, 0x6e, 0x57, 0x26, 0x04, 0x00, 0x00, 0x1e},
    {0x1e, 0x82, 0x55, 0x16, 0x0b, 0x11, 0x1a, 0x20, 0x26, 0x3c, 0x74, 0x59, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x65, 0x6f, 0x45, 0x44, 0x71, 0x82, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x4e, 0x67, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x68, 0x4c, 0x07, 0x00, 0x00, 0x00, 0x14, 0x5e, 0x5b, 0x16, 0x08, 0x2e, 0x73, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x21, 0x80, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0xa6, 0x8e, 0x5f, 0x51, 0x5c, 0x7b, 0x5d, 0x0a, 0x00, 0x00, 0x21, 0x49, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x0e, 0x73, 0x68, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x5e, 0x6e, 0x64, 0x65, 0x86, 0xbf, 0x88, 0x17, 0x00, 0x00, 0x00, 0x2a, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x17, 0x44, 0x43, 0x3f, 0x07, 0x00, 0x00, 0x00, 0x07, 0x48, 0x33, 0x00, 0x00, 0x00, 0x0c, 0x5f, 0x71, 0x0e, 0x00, 0x00, 0x00, 0x33, 0x2c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x20, 0x18, 0x00, 0x27, 0x38, 0x09, 0x00, 0x03, 0x40, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x5c, 0x27, 0x00, 0x00, 0x00, 0x37, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
  },
  {
    {0x03, 0x00, 0x00, 0x00, 0x17, 0x40, 0x16, 0x00, 0x17, 0x4c, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x76, 0x6a, 0x69, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x4c, 0x36, 0x58, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x92, 0xa7, 0x94, 0x6a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2b, 0x89, 0x7c, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2e, 0x95, 0x49, 0x31, 0x77, 0x4e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x09, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x61, 0x59, 0x00, 0x00, 0x13, 0x75, 0x3a, 0x00, 0x00, 0x00, 0x0b, 0x0b, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5a, 0x7f, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x72, 0x14, 0x00, 0x00, 0x00, 0x21, 0x80, 0x33, 0x12, 0x14, 0x0b, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3a, 0x86, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46, 0x5d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5f, 0x9a, 0x42, 0x0b, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x89, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x95, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x9f, 0x91, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x6c, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x75, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x71, 0xda, 0xfb, 0xa6, 0x52, 0x1d, 0x00, 0x00, 0x00, 0x4d, 0x6a, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x72, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x1d, 0x5e, 0x95, 0xa6, 0x97, 0x9a, 0xb7, 0xc2, 0xb4, 0x8f, 0x63, 0x5b, 0x9a, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x81, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x1d, 0x64, 0x9e, 0x9f, 0x6d, 0x31, 0x11, 0x0d, 0x1e, 0x48, 0x7e, 0xa4, 0xb5, 0xce, 0xfd, 0x9c, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x6e, 0x70, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x18, 0x5d, 0x9c, 0xa2, 0x6c, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x38, 0x72, 0xc1, 0xd7, 0x69, 0x0b, 0x00, 0x10, 0x2e, 0x46, 0x2b, 0x25, 0x3d, 0x16, 0x00, 0x00, 0x00, 0x00},
    {0x90, 0x9f, 0x6d, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x33, 0x94, 0xc5, 0x8c, 0x64, 0x59, 0x42, 0x15, 0x00, 0x00, 0x18, 0x39, 0x1c, 0x0b, 0x1a, 0x4f},
    {0x74, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0x97, 0xdf, 0xaf, 0x6f, 0x41, 0x22, 0x0b, 0x00, 0x08, 0x3d, 0x5f, 0x63, 0x86, 0x98},
    {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x68, 0x99, 0x6a, 0x56, 0x57, 0x56, 0x4f, 0x46, 0x44, 0x5e, 0xa6, 0xbe, 0x88, 0x3d},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x78, 0x68, 0x1e, 0x0b, 0x15, 0x29, 0x39, 0x44, 0x4f, 0x70, 0xba, 0x9b, 0x2e, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x5d, 0x92, 0x52, 0x07, 0x00, 0x00, 0x00, 0x00, 0x01, 0x08, 0x26, 0x7c, 0x7b, 0x19, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x5a, 0x63, 0x67, 0x4f, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4a, 0x80, 0x2a, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x65, 0x41, 0x11, 0x2d, 0x50, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x83, 0x52, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x68, 0x30, 0x00, 0x00, 0x08, 0x49, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x71, 0x7b, 0x0f},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x68, 0x25, 0x00, 0x00, 0x00, 0x00, 0x3d, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45, 0x90, 0x38},
    {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0x69, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x86, 0x71},
    {0x53, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x6b, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x62, 0xa8},
    {0xb7, 0x70, 0x53, 0x43, 0x34, 0x27, 0x1c, 0x15, 0x15, 0x2f, 0x78, 0x4e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x56, 0xcb},
    {0x95, 0x67, 0x63, 0x6b, 0x74, 0x78, 0x77, 0x73, 0x79, 0xad, 0x98, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x3d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x47, 0x90, 0xcb},
    {0x72, 0x23, 0x0b, 0x0d, 0x19, 0x26, 0x2f, 0x35, 0x41, 0x73, 0x86, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2e, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x49, 0x5d, 0x55, 0x5b, 0x8d},
    {0x6f, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x4d, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x34, 0x00, 0x00, 0x00, 0x03, 0x3c, 0x65, 0x4f, 0x21, 0x02, 0x00, 0x2c},
    {0x54, 0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x60, 0x4c, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x32, 0x00, 0x00, 0x17, 0x58, 0x5e, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x2e, 0x81, 0x2c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x9b, 0x92, 0x56, 0x36, 0x20, 0x17, 0x3f, 0x78, 0x43, 0x12, 0x2f, 0x68, 0x47, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x1e, 0x62, 0x57, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46, 0x86, 0x8b, 0x7f, 0x81, 0x82, 0x92, 0xb8, 0xa3, 0x62, 0x5d, 0x76, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x1e, 0x1c, 0x22, 0x42, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f, 0x4e, 0x19, 0x00, 0x03, 0x12, 0x29, 0x5d, 0xb4, 0x8d, 0x4c, 0x6e, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x13, 0x00, 0x00, 0x17, 0x40, 0x13, 0x00, 0x00, 0x00, 0x21, 0x46, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x74, 0x38, 0x4d, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06}
  },
  {
    {0x00, 0x00, 0x00, 0x07, 0x3e, 0x24, 0x00, 0x00, 0x00, 0x3c, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x98, 0xf8, 0xd9, 0xa2, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x01, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x09, 0x3f, 0x28, 0x03, 0x31, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x95, 0x6c, 0x34, 0x72, 0x74, 0x0b, 0x00, 0x00, 0x00, 0x02, 0x02, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x4c, 0x53, 0x65, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4e, 0x74, 0x00, 0x00, 0x02, 0x68, 0x6d, 0x0b, 0x03, 0x07, 0x03, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x90, 0x80, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x79, 0x2d, 0x00, 0x00, 0x00, 0x07, 0x86, 0x7d, 0x2e, 0x06, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x74, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3d, 0x72, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xaa, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x8a, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6a, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x84, 0x3a, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x8a, 0x2e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x77, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x75, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x90, 0x6d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x6a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x72, 0x4a, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0xc1, 0xe8, 0x71, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x68, 0x58, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x74, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x66, 0xa0, 0xbc, 0xcd, 0xcb, 0xb3, 0x8f, 0x5e, 0x39, 0x4d, 0x9a, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x75, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x29, 0x6e, 0x9a, 0x92, 0x66, 0x37, 0x2b, 0x4a, 0x7a, 0xa3, 0xb4, 0xb5, 0xd8, 0xeb, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x71, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x27, 0x6d, 0x9d, 0x94, 0x5a, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x35, 0x64, 0xa6, 0xed, 0xb9, 0x32, 0x00, 0x00, 0x00, 0x00, 0x16, 0x78, 0x70, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x63, 0x97, 0x93, 0x58, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x66, 0xc7, 0xb7, 0x70, 0x5b, 0x61, 0x76, 0xae, 0xda, 0x97, 0x44, 0x13, 0x04, 0x02, 0x06, 0x23},
    {0x90, 0x5c, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x69, 0xd5, 0xd1, 0xab, 0xa8, 0xa8, 0xab, 0xaa, 0xa8, 0x99, 0x78, 0x53, 0x45, 0x65, 0x8d},
    {0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x4d, 0x97, 0x75, 0x44, 0x32, 0x29, 0x23, 0x27, 0x40, 0x68, 0x90, 0xb3, 0xbe, 0xa4, 0x6d},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x77, 0x81, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x26, 0x7d, 0xc1, 0x6e, 0x13},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x5b, 0x86, 0x70, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x7f, 0x5e, 0x03},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0x63, 0x4f, 0x39, 0x59, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x66, 0x75, 0x10},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x6a, 0x35, 0x00, 0x00, 0x3f, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45, 0x8b, 0x33},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f, 0x6b, 0x25, 0x00, 0x00, 0x00, 0x21, 0x4c, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x88, 0x63},
    {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x6c, 0x20, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x4d, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x67, 0x8c},
    {0x64, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x6f, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x49, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0xa1},
    {0xbe, 0x68, 0x38, 0x26, 0x18, 0x0e, 0x06, 0x02, 0x05, 0x2c, 0x7b, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x98},
    {0xaf, 0x7d, 0x68, 0x6d, 0x70, 0x6e, 0x69, 0x65, 0x77, 0xaf, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x7e},
    {0x65, 0x1e, 0x13, 0x21, 0x31, 0x3e, 0x46, 0x4d, 0x67, 0xa6, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x73},
    {0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f, 0x6d, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x50, 0x80},
    {0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x49, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x44, 0x56, 0x7e, 0x92},
    {0x59, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x5d, 0x52, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x4f, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x04, 0x39, 0x5c, 0x46, 0x29, 0x3a, 0x7b},
    {0x71, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x8b, 0x8d, 0x4b, 0x26, 0x11, 0x10, 0x50, 0x5b, 0x06, 0x00, 0x00, 0x00, 0x16, 0x52, 0x51, 0x1f, 0x00, 0x00, 0x00, 0x49},
    {0x2f, 0x46, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0x85, 0xaa, 0x95, 0x89, 0x84, 0x94, 0xb9, 0x70, 0x0f, 0x00, 0x00, 0x25, 0x5a, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20},
    {0x00, 0x0c, 0x3c, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x53, 0x39, 0x10, 0x11, 0x24, 0x3e, 0x71, 0xc8, 0xaf, 0x41, 0x16, 0x37, 0x5f, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x0b},
    {0x00, 0x00, 0x06, 0x3d, 0x27, 0x00, 0x00, 0x00, 0x00, 0x07, 0x45, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0xc1, 0xc3, 0x95, 0x83, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00}
  },
  {
    {0x00, 0x00, 0x00, 0x2d, 0x38, 0x01, 0x00, 0x00, 0x00, 0x1f, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0xd2, 0xf3, 0xa4, 0xb2, 0xd1, 0x77, 0x17, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x2f, 0x36, 0x02, 0x00, 0x0d, 0x45, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0xaa, 0x52, 0x1d, 0x34, 0x8b, 0xa6, 0x55, 0x1c, 0x01, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0x3b, 0x15, 0x45, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x5b, 0x00, 0x00, 0x00, 0x00, 0x8f, 0x95, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x6a, 0x6a, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x82, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x33, 0x94, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x86, 0x87, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0x5f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x81, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5b, 0x90, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x7f, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6d, 0x63, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x8b, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3a, 0x75, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x62, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x88, 0x56, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5f, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5e, 0x73, 0x01, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x9c, 0xbc, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x09, 0x7a, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x62, 0x6b, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x2d, 0x79, 0xc1, 0xe5, 0xc2, 0x8b, 0x5a, 0x2e, 0x1a, 0x50, 0x95, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6a, 0x59, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x04, 0x31, 0x6e, 0x90, 0x8a, 0x69, 0x5a, 0x7a, 0x9d, 0xaf, 0xaa, 0xa3, 0xd3, 0xc8, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x78, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x05, 0x32, 0x73, 0x96, 0x83, 0x49, 0x14, 0x00, 0x00, 0x01, 0x12, 0x34, 0x5e, 0x8a, 0xd2, 0xf4, 0x83, 0x12, 0x00, 0x00, 0x00, 0x00, 0x10, 0x70, 0x92, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x2f, 0x6e, 0x94, 0x82, 0x45, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x3d, 0xa1, 0xd9, 0x9b, 0x68, 0x64, 0x71, 0x84, 0xad, 0xea, 0xca, 0x6e, 0x2c, 0x09, 0x02, 0x01, 0x09},
    {0x8c, 0x7f, 0x44, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x3c, 0xb3, 0xe4, 0xb8, 0xb1, 0xb3, 0xb0, 0xaa, 0xa3, 0xa5, 0xa7, 0x93, 0x70, 0x4a, 0x48, 0x6f},
    {0x51, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x8d, 0x86, 0x4f, 0x3b, 0x31, 0x26, 0x1d, 0x1c, 0x29, 0x4b, 0x78, 0xa0, 0xbc, 0xb7, 0x91},
    {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x67, 0x92, 0x4b, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x3b, 0xa1, 0xb4, 0x47},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x5e, 0x68, 0x6d, 0x49, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x43, 0x85, 0x3b},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f, 0x68, 0x41, 0x12, 0x32, 0x53, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x7b, 0x5a},
    {0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x6b, 0x2b, 0x00, 0x00, 0x06, 0x4b, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x66, 0x7f},
    {0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x6c, 0x20, 0x00, 0x00, 0x00, 0x00, 0x3a, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3d, 0x91},
    {0x83, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x6e, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a, 0x4a, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x8e},
    {0xae, 0x73, 0x26, 0x0e, 0x04, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x78, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x4d, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x66},
    {0xa0, 0xb1, 0x71, 0x61, 0x5e, 0x58, 0x51, 0x4f, 0x6d, 0xa0, 0x56, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x4b, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x35},
    {0x82, 0x67, 0x31, 0x34, 0x44, 0x50, 0x57, 0x60, 0x91, 0xae, 0x2e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x49, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2c},
    {0x6e, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x17, 0x6a, 0x4d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x49, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39},
    {0x58, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x64, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x4b, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x51},
    {0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x5e, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x49, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x6f},
    {0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x35, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x63, 0x9a},
    {0x51, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x7c, 0x81, 0x3e, 0x16, 0x07, 0x17, 0x63, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x1e, 0x2e, 0x60, 0x8a, 0x81},
    {0x40, 0x3d, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x6f, 0xb9, 0xa5, 0x89, 0x82, 0xa6, 0xaf, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x4d, 0x52, 0x56, 0x62, 0x32, 0x1c},
    {0x00, 0x2e, 0x3a, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x52, 0x2b, 0x25, 0x37, 0x54, 0x9b, 0xdb, 0x87, 0x1b, 0x00, 0x00, 0x00, 0x22, 0x54, 0x43, 0x1e, 0x22, 0x13, 0x00, 0x00},
    {0x00, 0x00, 0x2b, 0x3a, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2e, 0x3a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x7b, 0xd7, 0xa1, 0x5f, 0x4c, 0x5a, 0x6c, 0x38, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00}
  },
  {
    {0x00, 0x00, 0x15, 0x3f, 0x11, 0x00, 0x00, 0x00, 0x00, 0x07, 0x3c, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x51, 0xf1, 0xdd, 0x9e, 0x9e, 0xb4, 0xcf, 0xb2, 0x5e, 0x0a, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x17, 0x3e, 0x0f, 0x00, 0x00, 0x00, 0x32, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0xa9, 0x42, 0x21, 0x31, 0x6d, 0xdf, 0xa2, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x1a, 0x3d, 0x10, 0x00, 0x1e, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x85, 0x37, 0x00, 0x00, 0x00, 0x00, 0x55, 0x99, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x42, 0x28, 0x4b, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x72, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x85, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x6d, 0x6b, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x79, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0x6d, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6b, 0x89, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a, 0x7c, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4d, 0x81, 0x07, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x8a, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0x61, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x86, 0x14, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x7f, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x85, 0x18, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x7c, 0x87, 0x18, 0x00, 0x00, 0x00, 0x00, 0x20, 0x7f, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x80, 0x15, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x43, 0xa2, 0xce, 0x94, 0x52, 0x29, 0x0f, 0x11, 0x63, 0x89, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4d, 0x72, 0x0b, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x07, 0x30, 0x66, 0x87, 0x8a, 0x87, 0x95, 0xa0, 0x9a, 0x83, 0x76, 0xa3, 0x98, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x69, 0x5f, 0x02, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x09, 0x35, 0x6c, 0x83, 0x6e, 0x3f, 0x18, 0x0e, 0x19, 0x34, 0x5b, 0x7b, 0x96, 0xc2, 0xd0, 0x64, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x5e, 0xa5, 0x59, 0x0d, 0x00, 0x00, 0x00, 0x00},
    {0x0c, 0x35, 0x6c, 0x84, 0x6d, 0x36, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x3c, 0x87, 0xd3, 0xc9, 0x90, 0x70, 0x6f, 0x7a, 0x8c, 0xaa, 0xe2, 0xe3, 0x95, 0x4c, 0x17, 0x04, 0x00, 0x00},
    {0x6d, 0x7f, 0x69, 0x32, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x43, 0x65, 0x9b, 0xcb, 0xbd, 0xb3, 0xb3, 0xad, 0xa2, 0x93, 0x91, 0x9f, 0x9c, 0x84, 0x5d, 0x3d, 0x47},
    {0x77, 0x36, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x2a, 0x20, 0x49, 0x6c, 0x4f, 0x37, 0x2c, 0x21, 0x17, 0x14, 0x19, 0x2e, 0x56, 0x83, 0xa5, 0xb3, 0xa0},
    {0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x2a, 0x0c, 0x0a, 0x3c, 0x40, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x17, 0x53, 0xaf, 0x91},
    {0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x32, 0x1b, 0x09, 0x20, 0x55, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x5d, 0x76},
    {0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x35, 0x30, 0x18, 0x11, 0x1c, 0x55, 0x4d, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x7b},
    {0x65, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x36, 0x3d, 0x1c, 0x0a, 0x00, 0x00, 0x25, 0x53, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x7b},
    {0x8f, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x32, 0x43, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x03, 0x4b, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x66},
    {0x88, 0x7f, 0x22, 0x02, 0x00, 0x00, 0x00, 0x00, 0x09, 0x34, 0x40, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x36},
    {0x5c, 0xb0, 0x83, 0x51, 0x44, 0x3e, 0x39, 0x45, 0x54, 0x42, 0x26, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c},
    {0x4d, 0x91, 0x63, 0x42, 0x49, 0x54, 0x5d, 0x79, 0x7a, 0x4d, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x57, 0x67, 0x11, 0x00, 0x00, 0x03, 0x09, 0x17, 0x4c, 0x79, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x67, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x58, 0x4a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c},
    {0x6f, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x61, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f, 0x3d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26},
    {0x65, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x5f, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x37, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48},
    {0x52, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4e, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00, 0x47, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x75},
    {0x4c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x6b, 0x71, 0x2e, 0x08, 0x00, 0x29, 0x64, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x64, 0xa6},
    {0x5b, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4c, 0xaf, 0xa9, 0x7f, 0x7c, 0xab, 0x8e, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x63, 0x7c, 0x6b},
    {0x26, 0x43, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x56, 0x46, 0x3a, 0x47, 0x68, 0xbb, 0xc7, 0x56, 0x07, 0x00, 0x00, 0x00, 0x00, 0x12, 0x20, 0x4e, 0x6f, 0x51, 0x1a, 0x07},
    {0x00, 0x16, 0x3e, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x40, 0x14, 0x00, 0x00, 0x00, 0x00, 0x23, 0xa5, 0xc9, 0x7e, 0x4d, 0x37, 0x2d, 0x40, 0x6b, 0x7b, 0x64, 0x27, 0x00, 0x00, 0x00}
  },
  {
    {0x00, 0x04, 0x36, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8b, 0xff, 0xc9, 0xa3, 0xa6, 0xb8, 0xb4, 0x94, 0x88, 0x5c, 0x16, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x04, 0x37, 0x22, 0x00, 0x00, 0x00, 0x00, 0x19, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8b, 0xa3, 0x3b, 0x2a, 0x3d, 0x70, 0xdc, 0xe5, 0x65, 0x07, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x06, 0x39, 0x20, 0x00, 0x00, 0x03, 0x38, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x8d, 0x17, 0x00, 0x00, 0x00, 0x00, 0x3b, 0xb2, 0x52, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x08, 0x3b, 0x23, 0x02, 0x30, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x69, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x71, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x46, 0x43, 0x56, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x83, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x88, 0x0f, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x75, 0x58, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4c, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2b, 0x8f, 0x26, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x60, 0x4b, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x74, 0x4a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x8c, 0x36, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x8c, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x88, 0x3b, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x58, 0x8c, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x86, 0x35, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x4e, 0x68, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x7a, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2d, 0x7f, 0x24, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x3b, 0x87, 0xa5, 0x77, 0x4a, 0x2b, 0x14, 0x06, 0x0d, 0x41, 0x80, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x54, 0x78, 0x15, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x0a, 0x32, 0x62, 0x7b, 0x73, 0x64, 0x66, 0x6e, 0x74, 0x70, 0x63, 0x63, 0x82, 0xbf, 0x8e, 0x22, 0x00, 0x00, 0x00, 0x00, 0x10, 0x4e, 0xb3, 0x82, 0x21, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x0d, 0x36, 0x66, 0x77, 0x60, 0x32, 0x0e, 0x02, 0x05, 0x16, 0x3b, 0x6a, 0x95, 0x98, 0x9a, 0xbf, 0xcb, 0xa0, 0x82, 0x7f, 0x87, 0x98, 0xb0, 0xdc, 0xed, 0xb9, 0x72, 0x31, 0x0c, 0x00, 0x00},
    {0x45, 0x67, 0x74, 0x5c, 0x2a, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x37, 0x48, 0x34, 0x32, 0x49, 0x80, 0xaf, 0xc3, 0xc2, 0xb9, 0xb0, 0xa0, 0x8b, 0x7d, 0x8e, 0x9d, 0x94, 0x76, 0x4b, 0x33},
    {0x8d, 0x64, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x23, 0x04, 0x00, 0x00, 0x04, 0x34, 0x72, 0x64, 0x3b, 0x2a, 0x1e, 0x13, 0x0f, 0x10, 0x1b, 0x38, 0x66, 0x8e, 0xa6, 0xa6},
    {0x71, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x4c, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x25, 0x6e, 0xb3},
    {0x61, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x1f, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x49, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x6f},
    {0x7a, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x1e, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x43, 0x52, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x53},
    {0x89, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x1d, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x5a, 0x5a, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3e},
    {0x5e, 0x89, 0x37, 0x01, 0x00, 0x00, 0x12, 0x24, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x35, 0x5a, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x1d},
    {0x27, 0x8b, 0x97, 0x50, 0x3b, 0x45, 0x3f, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x51, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x12, 0x77, 0x92, 0x56, 0x52, 0x6f, 0x52, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x18, 0x72, 0x52, 0x0b, 0x06, 0x1e, 0x49, 0x3c, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x2e, 0x72, 0x25, 0x00, 0x00, 0x00, 0x09, 0x39, 0x3e, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x4d, 0x69, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2d, 0x50, 0x3c, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3b, 0x2c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x69, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0x74, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03},
    {0x76, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x69, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x01, 0x46, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d},
    {0x71, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x5d, 0x05, 0x00, 0x00, 0x00, 0x0f, 0x4c, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47},
    {0x63, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x60, 0x6a, 0x22, 0x00, 0x00, 0x41, 0x59, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x85},
    {0x63, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f, 0x9c, 0xab, 0x7a, 0x7e, 0xa9, 0x68, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x70, 0xac},
    {0x54, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x49, 0x5b, 0x53, 0x5a, 0x84, 0xd2, 0xa5, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x37, 0x71, 0x70, 0x4f},
    {0x0c, 0x3a, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x34, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x51, 0xc3, 0xad, 0x64, 0x42, 0x32, 0x31, 0x31, 0x1c, 0x2d, 0x5e, 0x6f, 0x3f, 0x0b, 0x00}
  },
  {
    {0x00, 0x00, 0x0f, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x1f, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5f, 0xee, 0xe7, 0xb7, 0xbe, 0xc6, 0x90, 0x4e, 0x63, 0x9f, 0x5d, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x32, 0x21, 0x00, 0x00, 0x00, 0x06, 0x21, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x81, 0xa7, 0x51, 0x3a, 0x52, 0x88, 0xd6, 0xf0, 0xbd, 0x7b, 0x13, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x2d, 0x56, 0x07, 0x00, 0x04, 0x23, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x8d, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x34, 0xdb, 0xac, 0x2f, 0x05, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x10, 0x49, 0x45, 0x18, 0x2f, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x7a, 0x4d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0xa1, 0x25, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x04, 0x4c, 0x6d, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x83, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x98, 0x3a, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0x79, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5c, 0x68, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x8e, 0x54, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x5c, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x71, 0x4b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x84, 0x62, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x50, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x77, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x64, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x75, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x81, 0x5f, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3b, 0x4e, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x77, 0x46, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0xa6, 0x61, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x46, 0x71, 0x33, 0x06, 0x00, 0x00, 0x00, 0x00, 0x03, 0x33, 0x9f, 0x90, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x7f, 0xb5, 0x73, 0x1f, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x07, 0x36, 0x7b, 0xa5, 0x89, 0x67, 0x56, 0x52, 0x53, 0x59, 0x6a, 0x8c, 0xbb, 0xc4, 0x88, 0x4a, 0x2b, 0x1d, 0x1f, 0x46, 0x7e, 0x7e, 0x4c, 0x4b, 0x4d, 0x25, 0x00, 0x00},
    {0x02, 0x02, 0x0e, 0x30, 0x5b, 0x72, 0x6d, 0x61, 0x6c, 0x8d, 0xaf, 0xaf, 0xa8, 0xa9, 0xb2, 0x9f, 0x6b, 0x63, 0x7e, 0x8d, 0x96, 0xa1, 0xbb, 0xb5, 0x72, 0x2a, 0x09, 0x06, 0x25, 0x47, 0x3b, 0x14},
    {0x46, 0x47, 0x62, 0x6d, 0x58, 0x2f, 0x10, 0x0c, 0x25, 0x53, 0x5e, 0x4c, 0x41, 0x3f, 0x47, 0x55, 0x41, 0x1c, 0x17, 0x2d, 0x52, 0x87, 0xb8, 0xa0, 0x67, 0x41, 0x22, 0x0f, 0x0b, 0x20, 0x47, 0x57},
    {0x8b, 0x90, 0x5f, 0x26, 0x02, 0x00, 0x00, 0x00, 0x25, 0x1e, 0x01, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x31, 0x29, 0x08, 0x0e, 0x31, 0x4c, 0x47, 0x42, 0x48, 0x4d, 0x4e, 0x4b, 0x43, 0x3a, 0x3b, 0x5b},
    {0x9c, 0x6b, 0x11, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x3c, 0x41, 0x3f, 0x34, 0x11, 0x00, 0x00, 0x00, 0x09, 0x1a, 0x2d, 0x3c, 0x47, 0x51, 0x6c},
    {0x95, 0x5c, 0x02, 0x00, 0x00, 0x01, 0x1a, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x72, 0x4f, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x21, 0x52},
    {0x77, 0x7e, 0x22, 0x00, 0x05, 0x1d, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x65, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29},
    {0x3a, 0x82, 0x68, 0x2b, 0x2c, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x5f, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x17},
    {0x0a, 0x59, 0xa9, 0x84, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3d, 0x67, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00},
    {0x00, 0x3c, 0xa3, 0x98, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4d, 0x6b, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x44, 0x81, 0x56, 0x51, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x5e, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x5a, 0x70, 0x1c, 0x18, 0x47, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x4e, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x0c, 0x72, 0x56, 0x00, 0x00, 0x0f, 0x46, 0x3d, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x45, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x28, 0x81, 0x31, 0x00, 0x00, 0x00, 0x06, 0x3d, 0x49, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x46, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x4b, 0x7b, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2e, 0x4f, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x49, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x6c, 0x5f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x50, 0x48, 0x16, 0x00, 0x00, 0x00, 0x00, 0x18, 0x49, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x87, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x5c, 0x73, 0x1b, 0x00, 0x00, 0x00, 0x33, 0x4b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d},
    {0xab, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x73, 0x93, 0x4a, 0x1f, 0x28, 0x73, 0x4f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f},
    {0x9c, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x8a, 0x80, 0x72, 0x97, 0xc3, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x6b, 0xb2},
    {0x1b, 0x48, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x34, 0x15, 0x0e, 0x20, 0x4a, 0x9a, 0xa1, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x7b, 0x74, 0x28},
    {0x00, 0x02, 0x3a, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x21, 0x02, 0x00, 0x00, 0x00, 0x00, 0x25, 0x97, 0xad, 0x68, 0x42, 0x39, 0x38, 0x13, 0x00, 0x00, 0x2f, 0x86, 0x56, 0x00, 0x00}
  },
  {
    {0x00, 0x00, 0x00, 0x27, 0x30, 0x00, 0x00, 0x00, 0x1c, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0xa5, 0xe8, 0xb8, 0xb7, 0x9d, 0x55, 0x26, 0x46, 0x89, 0x3a, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x06, 0x3e, 0x10, 0x00, 0x1a, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x9c, 0x74, 0x49, 0x5c, 0x8b, 0xb8, 0xc2, 0xc0, 0x68, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x36, 0x3e, 0x26, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x70, 0x51, 0x00, 0x00, 0x00, 0x00, 0x38, 0xdd, 0xf6, 0x3a, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x35, 0x76, 0x45, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3a, 0x6e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45, 0xd4, 0x63, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x01, 0x55, 0x9a, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x6a, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9a, 0x94, 0x14, 0x00, 0x00, 0x00},
    {0x01, 0x02, 0x03, 0x11, 0x4f, 0x7c, 0x2e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0x71, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x71, 0x91, 0x31, 0x08, 0x03, 0x01},
    {0x04, 0x04, 0x03, 0x05, 0x1d, 0x5e, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3a, 0x63, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5f, 0x86, 0x1f, 0x05, 0x04, 0x05},
    {0x00, 0x00, 0x00, 0x00, 0x06, 0x4e, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x56, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6e, 0x98, 0x1e, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x03, 0x53, 0x51, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4d, 0x56, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x94, 0x9a, 0x4a, 0x07, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x24, 0x6c, 0x79, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x68, 0x84, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x6f, 0x6a, 0x3c, 0x49, 0x41, 0x15, 0x00},
    {0x08, 0x00, 0x0c, 0x2a, 0x3d, 0x35, 0x3f, 0x56, 0x42, 0x16, 0x00, 0x00, 0x00, 0x03, 0x19, 0x51, 0xa2, 0xc8, 0x7c, 0x28, 0x08, 0x00, 0x0b, 0x3c, 0x6d, 0x4e, 0x0c, 0x00, 0x04, 0x29, 0x46, 0x2b},
    {0x48, 0x3b, 0x42, 0x2b, 0x09, 0x00, 0x00, 0x15, 0x4e, 0x66, 0x57, 0x4c, 0x53, 0x67, 0x7c, 0x82, 0x79, 0x78, 0x8b, 0x88, 0x7c, 0x81, 0x86, 0x76, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x45},
    {0x51, 0x6d, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x81, 0xb8, 0xae, 0xb2, 0xa3, 0x6a, 0x35, 0x14, 0x0a, 0x16, 0x33, 0x57, 0x8a, 0x88, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b},
    {0x24, 0x4d, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x51, 0x74, 0x67, 0x53, 0x53, 0x5b, 0x42, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x47, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x12, 0x3d, 0x0e, 0x00, 0x00, 0x01, 0x29, 0x57, 0x53, 0x23, 0x03, 0x00, 0x00, 0x0a, 0x2e, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2d, 0x60, 0x38, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x1f, 0x4e, 0x2a, 0x06, 0x1c, 0x48, 0x58, 0x36, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x29, 0x23, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x42, 0x61, 0x5f, 0x52, 0x45, 0x34, 0x20, 0x0d, 0x00, 0x00},
    {0x5b, 0x85, 0x6d, 0x54, 0x55, 0x3f, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x2d, 0x22, 0x00, 0x00, 0x15, 0x36, 0x2c, 0x14, 0x14, 0x25, 0x39, 0x4b, 0x52, 0x4f, 0x45, 0x42},
    {0x75, 0xaf, 0xb7, 0x6b, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x3d, 0x36, 0x33, 0x38, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x1c, 0x38, 0x54, 0x63},
    {0x27, 0x71, 0xa8, 0x4f, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x71, 0x59, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x18, 0x16},
    {0x00, 0x34, 0x8f, 0x5b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x6a, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x1e, 0x91, 0x81, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x5d, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x22, 0x89, 0x90, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x5d, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x33, 0x7c, 0x5a, 0x4e, 0x2c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x5d, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x4b, 0x71, 0x21, 0x17, 0x42, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3a, 0x52, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x03, 0x65, 0x59, 0x01, 0x00, 0x0b, 0x3f, 0x3f, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x19, 0x75, 0x34, 0x00, 0x00, 0x00, 0x02, 0x34, 0x48, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x3b, 0x73, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0x4c, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x71, 0x6b, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x52, 0x48, 0x1e, 0x0d, 0x0e, 0x0b, 0x12, 0x57, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0xbf, 0x72, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x6e, 0x86, 0x6c, 0x69, 0x6d, 0x86, 0x9b, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5d},
    {0x71, 0x68, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x4b, 0x48, 0x3b, 0x3e, 0x4e, 0x7b, 0xb0, 0x74, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x6b, 0x89},
    {0x00, 0x11, 0x47, 0x1d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x28, 0x12, 0x00, 0x00, 0x00, 0x00, 0x07, 0x4a, 0x97, 0x59, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2b, 0x7c, 0x5d, 0x12},
    {0x00, 0x00, 0x0c, 0x40, 0x0a, 0x00, 0x00, 0x00, 0x01, 0x1e, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x57, 0xa3, 0x73, 0x42, 0x3e, 0x20, 0x00, 0x00, 0x00, 0x32, 0x7f, 0x3f, 0x00, 0x00}
  },
  {
    {0x00, 0x00, 0x00, 0x00, 0x53, 0x3e, 0x25, 0x2c, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x6a, 0xf4, 0xf3, 0xc5, 0x8c, 0x4c, 0x45, 0x64, 0x86, 0x41, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x37, 0x89, 0x58, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x7f, 0xae, 0x77, 0x7e, 0xba, 0xdd, 0x9b, 0x5d, 0x42, 0x02, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x25, 0xaa, 0x4b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x83, 0x17, 0x00, 0x00, 0x14, 0x8f, 0xa9, 0x4e, 0x2c, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x2c, 0x9d, 0x2c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x76, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0x7f, 0x34, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x43, 0x90, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3a, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x9d, 0x5b, 0x06, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x70, 0x83, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x69, 0x4f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0xa2, 0xa6, 0x3b, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x33, 0xb4, 0x78, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x86, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xac, 0xd8, 0xa8, 0x56, 0x07, 0x00},
    {0x2e, 0x2b, 0x42, 0x92, 0xd5, 0x98, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x61, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45, 0x94, 0x71, 0x84, 0xab, 0x8d, 0x4c},
    {0x77, 0x67, 0x7f, 0x7d, 0x6c, 0x75, 0x72, 0x3c, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x04, 0x20, 0x2e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x77, 0x33, 0x04, 0x08, 0x32, 0x7d, 0x99},
    {0x87, 0x89, 0x54, 0x14, 0x00, 0x05, 0x30, 0x62, 0x5d, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x10, 0x26, 0x03, 0x26, 0x77, 0x34, 0x00, 0x00, 0x0d, 0x4a, 0x6d, 0x26, 0x00, 0x00, 0x00, 0x00, 0x04, 0x47},
    {0x5f, 0x73, 0x07, 0x00, 0x00, 0x00, 0x00, 0x09, 0x48, 0x6e, 0x45, 0x16, 0x12, 0x34, 0x7b, 0x80, 0x5e, 0x76, 0xb5, 0xb4, 0x80, 0x74, 0x7b, 0x64, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x3b, 0x4d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x37, 0x81, 0x8f, 0x91, 0x9d, 0x9a, 0x7b, 0x5e, 0x53, 0x4e, 0x66, 0x99, 0xb9, 0x6b, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x35, 0x4b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6a, 0xe0, 0xb8, 0x65, 0x2e, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x50, 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x30, 0x4f, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x8d, 0xa4, 0x72, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x46, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x27, 0x4f, 0x05, 0x00, 0x00, 0x00, 0x00, 0x07, 0x4a, 0x7b, 0x58, 0x29, 0x38, 0x3d, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x1f, 0x53, 0x10, 0x00, 0x00, 0x00, 0x2e, 0x6b, 0x6f, 0x30, 0x00, 0x00, 0x00, 0x24, 0x38, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x54, 0x3e, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x33, 0x6e, 0x39, 0x08, 0x20, 0x58, 0x74, 0x4e, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x34, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x5b, 0x8f, 0x63, 0x3e, 0x26, 0x11, 0x04, 0x00, 0x02},
    {0x80, 0xb9, 0x92, 0x6b, 0x6f, 0x59, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x2c, 0x00, 0x00, 0x00, 0x00, 0x02, 0x36, 0x50, 0x4a, 0x53, 0x64, 0x6f, 0x73, 0x6e, 0x5f, 0x5e},
    {0x7f, 0xca, 0xea, 0x96, 0x3b, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f, 0x2a, 0x00, 0x00, 0x0f, 0x35, 0x2f, 0x0a, 0x00, 0x00, 0x0f, 0x37, 0x5f, 0x68, 0x63, 0x66},
    {0x18, 0x69, 0xcb, 0x75, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x45, 0x3e, 0x34, 0x44, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x09, 0x00, 0x01, 0x06},
    {0x00, 0x29, 0xa3, 0x7c, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x81, 0x75, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x15, 0xa0, 0xa5, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x77, 0x4f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x1a, 0x99, 0xb3, 0x4b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x68, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x2c, 0x8e, 0x73, 0x5d, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x68, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x48, 0x84, 0x2e, 0x19, 0x49, 0x3f, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x64, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x65, 0x6a, 0x04, 0x00, 0x08, 0x43, 0x4c, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x59, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x11, 0x80, 0x46, 0x00, 0x00, 0x00, 0x00, 0x33, 0x55, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x57, 0x4f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x65, 0xa9, 0x2e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0x66, 0x55, 0x2e, 0x1c, 0x14, 0x0f, 0x0f, 0x32, 0x84, 0x51, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0xc4, 0xc0, 0x4a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3b, 0x9b, 0x94, 0x7a, 0x76, 0x73, 0x74, 0x9b, 0xcf, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x5f},
    {0x4d, 0x46, 0x6a, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x4c, 0x31, 0x2b, 0x31, 0x38, 0x41, 0x60, 0xa9, 0xab, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x84, 0x87},
    {0x00, 0x00, 0x1b, 0x56, 0x12, 0x00, 0x00, 0x00, 0x1c, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x8d, 0x90, 0x25, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45, 0x97, 0x5b, 0x06},
    {0x00, 0x00, 0x00, 0x29, 0x4d, 0x02, 0x00, 0x17, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x9f, 0xa0, 0x64, 0x3e, 0x08, 0x00, 0x00, 0x0c, 0x58, 0x9a, 0x3e, 0x00, 0x00}
  },
  {
    {0x00, 0x00, 0x00, 0x00, 0x34, 0xaf, 0x4e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0xb9, 0xff, 0xbb, 0x7e, 0x5c, 0x30, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x23, 0xa0, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0xa3, 0xae, 0xbc, 0xb6, 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x25, 0x91, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x6a, 0x57, 0x00, 0x0d, 0x6d, 0x73, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x32, 0x8d, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x6b, 0x00, 0x00, 0x00, 0x03, 0x71, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x4d, 0x89, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x8d, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x31, 0x7d, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x92, 0x8c, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0e, 0x2f, 0x69, 0x23, 0x00, 0x00, 0x00, 0x00, 0x03, 0x98, 0x8b, 0x25, 0x0d, 0x17, 0x03, 0x00},
    {0x00, 0x00, 0x18, 0x80, 0xf2, 0xb1, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x0f, 0x0a, 0x00, 0x13, 0x18, 0x00, 0x00, 0x00, 0x00, 0x16, 0xa0, 0xa4, 0x7a, 0x6e, 0x68, 0x53, 0x1f},
    {0x88, 0x8a, 0xa7, 0xad, 0x93, 0x8c, 0x82, 0x41, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x08, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x06, 0x61, 0x5b, 0x27, 0x31, 0x51, 0x75, 0x93, 0x9f},
    {0xd5, 0xd9, 0x72, 0x1d, 0x01, 0x0e, 0x3d, 0x70, 0x67, 0x22, 0x00, 0x00, 0x00, 0x04, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x2e, 0x18, 0x00, 0x1d, 0x52, 0x41, 0x02, 0x00, 0x00, 0x00, 0x0a, 0x32, 0x7b},
    {0x7e, 0x77, 0x01, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x51, 0x78, 0x45, 0x0c, 0x09, 0x37, 0x1a, 0x00, 0x00, 0x00, 0x08, 0x8a, 0xb3, 0x6f, 0x5e, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d},
    {0x4e, 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x7f, 0x7a, 0x77, 0xa2, 0x72, 0x39, 0x32, 0x43, 0x73, 0xae, 0xdc, 0x99, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x4b, 0x4b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2d, 0xac, 0xdf, 0xb3, 0x88, 0x70, 0x61, 0x56, 0x47, 0x34, 0x59, 0x6a, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x4b, 0x51, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x8f, 0x9d, 0x38, 0x16, 0x0e, 0x05, 0x00, 0x00, 0x00, 0x05, 0x4c, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x45, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x9d, 0x4a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x50, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x3d, 0x58, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2d, 0x99, 0xa3, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x2f, 0x59, 0x04, 0x00, 0x00, 0x00, 0x00, 0x09, 0x50, 0x88, 0x69, 0x45, 0x4f, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x4f, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x28, 0x62, 0x16, 0x00, 0x00, 0x00, 0x36, 0x76, 0x7a, 0x36, 0x04, 0x00, 0x16, 0x3c, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x50, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x47, 0x86, 0x4b, 0x10, 0x29, 0x63, 0x7f, 0x54, 0x10, 0x00, 0x00, 0x00, 0x00, 0x11, 0x39, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x6f, 0x79, 0x36, 0x1c, 0x15, 0x0f, 0x0b, 0x11},
    {0x93, 0xd4, 0xb1, 0x7d, 0x7b, 0x5e, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x37, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x71, 0x8d, 0x8c, 0x8b, 0x7f, 0x77, 0x70, 0x72},
    {0x6f, 0xbb, 0xfa, 0xaa, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0x34, 0x01, 0x00, 0x00, 0x00, 0x00, 0x27, 0x47, 0x33, 0x2f, 0x4d, 0x4d, 0x45, 0x49, 0x4e, 0x54},
    {0x08, 0x4e, 0xc3, 0x86, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x32, 0x00, 0x00, 0x0a, 0x33, 0x33, 0x0c, 0x00, 0x09, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x1c, 0x93, 0x84, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x40, 0x2e, 0x44, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x13, 0x80, 0x94, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x77, 0x78, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x14, 0x7e, 0xa9, 0x56, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x70, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x1d, 0x89, 0x82, 0x62, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x63, 0x2b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x47, 0x90, 0x41, 0x22, 0x47, 0x3e, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x61, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x18, 0x7a, 0x81, 0x23, 0x00, 0x07, 0x3e, 0x4e, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x64, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x4e, 0x4f, 0x5b, 0x27, 0x00, 0x00, 0x00, 0x3c, 0x6a, 0x58, 0x3b, 0x2e, 0x26, 0x1f, 0x1a, 0x16, 0x23, 0x71, 0x75, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26},
    {0x23, 0x15, 0x4f, 0x46, 0x00, 0x00, 0x00, 0x03, 0x65, 0x9a, 0x80, 0x77, 0x77, 0x77, 0x76, 0x73, 0x84, 0xc3, 0xa2, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x42, 0x4b},
    {0x05, 0x0c, 0x37, 0x6e, 0x23, 0x00, 0x00, 0x19, 0x44, 0x2a, 0x18, 0x18, 0x1d, 0x26, 0x2d, 0x33, 0x44, 0x7b, 0xb2, 0x6a, 0x0f, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x2f, 0x51, 0x40, 0x15},
    {0x02, 0x00, 0x00, 0x3c, 0x65, 0x15, 0x16, 0x30, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x55, 0x9f, 0x60, 0x28, 0x02, 0x00, 0x00, 0x00, 0x20, 0x42, 0x41, 0x1d, 0x05, 0x03},
    {0x00, 0x00, 0x00, 0x00, 0x5e, 0x74, 0x4e, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6a, 0xb5, 0x7f, 0x31, 0x12, 0x23, 0x3d, 0x3d, 0x1d, 0x00, 0x00, 0x00, 0x00}
  }
};
#define CAUSTICS_BR                     (100U)                // яркость бликов в процентах (от чистого белого света)
void poolRoutine()
{
  if (loadingFlag) {
    loadingFlag = false;
    hue = modes[currentMode].Scale * 2.55;
    fillAll(CHSV(hue, 255U, 255U));
    deltaHue = 0U;
    deltaHue2 = 0U;
  }
  if (modes[currentMode].Speed != 255U) // если регулятор скорости на максимуме, то будет работать старый эффект "цвет" (без анимации бликов воды)
  {
    if (step > 24U) // количество кадров в анимации -1 (отсчёт с нуля)
      step = 0U;
    if (step > 0U && step < 3U) // пару раз за цикл анимации двигаем текстуру по радиусу лампы. а может и не двигаем. как повезёт
    {
      if (random(2U) == 0U)
      {
        deltaHue++;
        if (deltaHue > 31U) deltaHue = 0U;
      }
    }
    if (step > 11U && step < 14U) // пару раз за цикл анимации двигаем текстуру по вертикали. а может и не двигаем. как повезёт
    {
      if (random(2U) == 0U)
      {
        deltaHue2++;
        if (deltaHue2 > 31U) deltaHue2 = 0U;
      }
    }
    for (uint8_t x = 0U; x < pWIDTH ; x++) {
      for (uint8_t y = 0U; y < pHEIGHT; y++) {
        leds[XY(x, y)] = CHSV(hue, 255U - pgm_read_byte(&aquariumGIF[step][(y + deltaHue2) % 32U][(x + deltaHue) % 32U]) * CAUSTICS_BR / 100U, 255U);
      }
    }
    step++;
  }
}

// ------------------------------ ЭФФЕКТ ДЫМ ----------------------
// (c) SottNick
uint8_t smoke_type = 0;
void smokeRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    smoke_type = (specialTextEffectParam >= 0) ? specialTextEffectParam : getEffectScaleParamValue2(MC_SMOKE);
    // Если авто - генерировать один из типов - Дым, Цветной дым
    if (smoke_type == 0 || smoke_type > 2) {
      smoke_type = random8(1, 3);
    }
    FastLED.clear();  // очистить
  }
  switch (smoke_type) {
    case 1:  MultipleStreamSmoke(true); break;
    default: MultipleStreamSmoke(false); break;
  }
}
void MultipleStreamSmoke(bool isColored) {
  if (loadingFlag)
  {
    loadingFlag = false;
    hue2 = 0U;
  }
  dimAll(254U);//(255U - modes[currentMode].Scale * 2);
  deltaHue++;
  CRGB color;
  if (isColored)
  {
    if (hue2 == modes[currentMode].Scale)
    {
      hue2 = 0U;
      hue = random8();
    }
    if (deltaHue & 0x01) // какой-то умножитель охота подключить к задержке смены цвета, но хз какой...
      hue2++;
    hsv2rgb_spectrum(CHSV(hue, 255U, 127U), color);
  }
  else {
    hsv2rgb_spectrum(CHSV((modes[currentMode].Scale - 1U) * 2.6, (modes[currentMode].Scale > 98U) ? 0U : 255U, 127U), color);
  }
  if (random8(pWIDTH) != 0U) // встречная спираль движется не всегда синхронно основной
    deltaHue2--;
  for (uint8_t y = 0; y < pHEIGHT; y++) {
    leds[XY((deltaHue  + y + 1U) % pWIDTH, pHEIGHT - 1U - y)] += color;
    leds[XY((deltaHue  + y     ) % pWIDTH, pHEIGHT - 1U - y)] += color; //color2
    leds[XY((deltaHue2 + y     ) % pWIDTH,               y)] += color;
    leds[XY((deltaHue2 + y + 1U) % pWIDTH,               y)] += color; //color2
  }
  // Noise
  // скорость движения по массиву noise
  noise32_x[0] += 1500;//1000;
  noise32_y[0] += 1500;//1000;
  noise32_z[0] += 1500;//1000;
  scale32_x[0] = 4000;
  scale32_y[0] = 4000;
  FillNoise(0);
  // допустимый отлёт зажжённого пикселя от изначально присвоенного местоположения (от 0 до указанного значения. дробное)
  MoveFractionalNoiseX(3);//4
  MoveFractionalNoiseY(3);//4
  blurScreen(20); // без размытия как-то пиксельно, наверное...
}

// =============- новый огонь / водопад -===============
// COOLING: How much does the air cool as it rises?
// Less cooling = taller flames.  More cooling = shorter flames.
// Default 55, suggested range 20-100
#define COOLINGNEW 32
// 8  практически сплошной поток красивой подсвеченной воды ровным потоком сверху донизу. будто бы на столе стоит маленький "родничок"
// 20 ровный водопад с верщиной на свету, где потоки летящей воды наверху разбиваются ветром в белую пену
// 32 уже не ровный водопад, у которого струи воды долетают до земли неравномерно
// чем больше параметр, тем больше тени снизу
// 55 такое, как на видео
// SPARKING: What chance (out of 255) is there that a new spark will be lit?
// Higher chance = more roaring fire.  Lower chance = more flickery fire.
// Default 120, suggested range 50-200.
#define SPARKINGNEW 80 // 30 // 120 // 90 // 60
// 80 почти все белые струи сверху будут долетать до низа - хорошо при выбранном ползунке Масштаб = 100 (белая вода без подкрашивания)
// 50 чуть больше половины будет долетать. для цветных вариантов жидкости так более эффектно
uint8_t waterfall_type = 0;
void waterfallRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    waterfall_type = (specialTextEffectParam >= 0) ? specialTextEffectParam : getEffectScaleParamValue2(MC_WATERFALL);
    // Если авто - генерировать один из типов - Водопад Водопад 4в1
    if (waterfall_type == 0 || waterfall_type > 2) {
      waterfall_type = random8(1, 3);
    }
  }
  switch (waterfall_type) {
    case 1:  fire2012WithPalette4in1(); break;
    default: fire2012WithPalette(); break;
  }
}
void fire2012WithPalette() {
  for (uint8_t x = 0; x < pWIDTH; x++) {
    // Step 1.  Cool down every cell a little
    for (uint8_t i = 0; i < pHEIGHT; i++) {
      noise_3d[0][x][i] = qsub8(noise_3d[0][x][i], random8(0, ((COOLINGNEW * 10) / pHEIGHT) + 2));
    }
    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for (uint8_t k = pHEIGHT - 1; k >= 2; k--) {
      noise_3d[0][x][k] = (noise_3d[0][x][k - 1] + noise_3d[0][x][k - 2] + noise_3d[0][x][k - 2]) / 3;
    }
    // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
    if (random8() < SPARKINGNEW) {
      uint8_t y = random8(2);
      noise_3d[0][x][y] = qadd8(noise_3d[0][x][y], random8(160, 255));
    }
    // Step 4.  Map from heat cells to LED colors
    for (uint8_t j = 0; j < pHEIGHT; j++) {
      // Scale the heat value from 0-255 down to 0-240
      // for best results with color palettes.
      byte colorindex = scale8(noise_3d[0][x][j], 240);
      if (modes[currentMode].Scale == 100)
        leds[XY(x, (pHEIGHT - 1) - j)] = ColorFromPalette(WaterfallColors_p, colorindex);
      else
        leds[XY(x, (pHEIGHT - 1) - j)] = ColorFromPalette(CRGBPalette16( CRGB::Black, CHSV(modes[currentMode].Scale * 2.57, 255U, 255U) , CHSV(modes[currentMode].Scale * 2.57, 128U, 255U) , CRGB::White), colorindex);// 2.57 вместо 2.55, потому что 100 для белого цвета
    }
  }
}

// ============= водо/огне/лава/радуга/хренопад ===============
// SPARKING: What chance (out of 255) is there that a new spark will be lit?
// Higher chance = more roaring fire.  Lower chance = more flickery fire.
// Default 120, suggested range 50-200.
extern const TProgmemRGBPalette16 WaterfallColors4in1_p FL_PROGMEM = {
  CRGB::Black,
  CRGB::DarkSlateGray,
  CRGB::DimGray,
  CRGB::LightSlateGray,
  CRGB::DimGray,
  CRGB::DarkSlateGray,
  CRGB::Silver,
  CRGB::DarkCyan,
  CRGB::Lavender,
  CRGB::Silver,
  CRGB::Azure,
  CRGB::LightGrey,
  CRGB::GhostWhite,
  CRGB::Silver,
  CRGB::White,
  CRGB::RoyalBlue
};
void fire2012WithPalette4in1() {
  uint8_t rCOOLINGNEW = constrain((uint16_t)(modes[currentMode].Scale % 16) * 32 / pHEIGHT + 16, 1, 255) ;
  // Array of temperature readings at each simulation cell
  //static byte heat[pWIDTH][pHEIGHT]; будет noise_3d[0][pWIDTH][pHEIGHT]
  for (uint8_t x = 0; x < pWIDTH; x++) {
    // Step 1.  Cool down every cell a little
    for (uint8_t i = 0; i < pHEIGHT; i++) {
      noise_3d[0][x][i] = qsub8(noise_3d[0][x][i], random8(0, rCOOLINGNEW));
    }
    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for (uint8_t k = pHEIGHT - 1; k >= 2; k--) {
      noise_3d[0][x][k] = (noise_3d[0][x][k - 1] + noise_3d[0][x][k - 2] + noise_3d[0][x][k - 2]) / 3;
    }
    // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
    if (random8() < SPARKINGNEW) {
      uint8_t y = random8(2);
      noise_3d[0][x][y] = qadd8(noise_3d[0][x][y], random8(160, 255));
    }
    // Step 4.  Map from heat cells to LED colors
    for (uint8_t j = 0; j < pHEIGHT; j++) {
      // Scale the heat value from 0-255 down to 0-240
      // for best results with color palettes.
      byte colorindex = scale8(noise_3d[0][x][j], 240);
      if  (modes[currentMode].Scale < 16) {            // Lavafall
        leds[XY(x, (pHEIGHT - 1) - j)] = ColorFromPalette(LavaColors_p, colorindex);
      } else if (modes[currentMode].Scale < 32) {      // Firefall
        leds[XY(x, (pHEIGHT - 1) - j)] = ColorFromPalette(HeatColors_p, colorindex);
      } else if (modes[currentMode].Scale < 48) {      // Waterfall
        leds[XY(x, (pHEIGHT - 1) - j)] = ColorFromPalette(WaterfallColors4in1_p, colorindex);
      } else if (modes[currentMode].Scale < 64) {      // Skyfall
        leds[XY(x, (pHEIGHT - 1) - j)] = ColorFromPalette(CloudColors_p, colorindex);
      } else if (modes[currentMode].Scale < 80) {      // Forestfall
        leds[XY(x, (pHEIGHT - 1) - j)] = ColorFromPalette(ForestColors_p, colorindex);
      } else if (modes[currentMode].Scale < 96) {      // Rainbowfall
        leds[XY(x, (pHEIGHT - 1) - j)] = ColorFromPalette(RainbowColors_p, colorindex);
      } else {                      // Aurora
        leds[XY(x, (pHEIGHT - 1) - j)] = ColorFromPalette(RainbowStripeColors_p, colorindex);
      }
    }
  }
}

//================================кометы======================
uint8_t comet_type = 0;
void comet() {
  if (loadingFlag)
  {
    loadingFlag = false;
    comet_type = (specialTextEffectParam >= 0) ? specialTextEffectParam : getEffectScaleParamValue2(MC_COMET);
    // Если авто - генерировать один из типов
    if (comet_type == 0 || comet_type > 5) {
      comet_type = random8(1, 6);
    }
  }
  switch (comet_type) {
    case 1: RainbowCometRoutine(); break;
    case 2:  ColorCometRoutine(); break;
    case 3:  MultipleStream(); break;
    case 4:  MultipleStream2(); break;
    default: starwarsRoutine(); break;
  }
}

// Кометы обычные
void RainbowCometRoutine() {
  dimAll(254U); // < -- затухание эффекта для последующего кадра
  CRGB _eNs_color = CHSV(millis() / modes[currentMode].Scale * 2, 255, 255);
  leds[XY(CENTER_X_MINOR, CENTER_Y_MINOR)] += _eNs_color;
  leds[XY(CENTER_X_MINOR + 1, CENTER_Y_MINOR)] += _eNs_color;
  leds[XY(CENTER_X_MINOR, CENTER_Y_MINOR + 1)] += _eNs_color;
  leds[XY(CENTER_X_MINOR + 1, CENTER_Y_MINOR + 1)] += _eNs_color;
  // Noise
  noise32_x[0] += 1500;
  noise32_y[0] += 1500;
  noise32_z[0] += 1500;
  scale32_x[0] = 8000;
  scale32_y[0] = 8000;
  FillNoise(0);
  MoveFractionalNoiseX(pWIDTH / 2U - 1U);
  MoveFractionalNoiseY(pHEIGHT / 2U - 1U);
}
// Кометы белые и одноцветные
void ColorCometRoutine() {      // <- ******* для оригинальной прошивки Gunner47 ******* (раскомментить/закоментить)
  dimAll(254U); // < -- затухание эффекта для последующего кадра
  CRGB _eNs_color = CRGB::White;
  if (modes[currentMode].Scale < 100) _eNs_color = CHSV((modes[currentMode].Scale) * 2.57, 255, 255); // 2.57 вместо 2.55, потому что при 100 будет белый цвет
  leds[XY(CENTER_X_MINOR, CENTER_Y_MINOR)] += _eNs_color;
  leds[XY(CENTER_X_MINOR + 1, CENTER_Y_MINOR)] += _eNs_color;
  leds[XY(CENTER_X_MINOR, CENTER_Y_MINOR + 1)] += _eNs_color;
  leds[XY(CENTER_X_MINOR + 1, CENTER_Y_MINOR + 1)] += _eNs_color;
  // Noise
  noise32_x[0] += 1500;
  noise32_y[0] += 1500;
  noise32_z[0] += 1500;
  scale32_x[0] = 8000;
  scale32_y[0] = 8000;
  FillNoise(0);
  MoveFractionalNoiseX(pWIDTH / 2U - 1U);
  MoveFractionalNoiseY(pHEIGHT / 2U - 1U);
}
// NoiseSmearing(by StefanPetrick) Effect mod for GyverLamp by PalPalych
void MultipleStream() { // 2 comets
  trackingObjectState[0] = pWIDTH / 8;
  trackingObjectState[1] = pHEIGHT / 8;
  trackingObjectShift[0] = 255. / (pWIDTH - 1. - trackingObjectState[0] - trackingObjectState[0]);
  trackingObjectShift[1] = 255. / (pHEIGHT - 1. - trackingObjectState[1] - trackingObjectState[1]);
  trackingObjectState[2] = pWIDTH / 4;
  trackingObjectState[3] = pHEIGHT / 4;
  trackingObjectShift[2] = 255. / (pWIDTH - 1. - trackingObjectState[2] - trackingObjectState[2]); // ((pWIDTH>10)?9.:5.));
  trackingObjectShift[3] = 255. / (pHEIGHT - 1. - trackingObjectState[3] - trackingObjectState[3]); //- ((pHEIGHT>10)?9.:5.));
  // }
  dimAll(255U - modes[currentMode].Scale * 2);
  // gelb im Kreis
  byte xx = trackingObjectState[0] + sin8( millis() / 10) / trackingObjectShift[0];// / 22;
  byte yy = trackingObjectState[1] + cos8( millis() / 10) / trackingObjectShift[1];// / 22;
  if (xx < pWIDTH && yy < pHEIGHT)
    leds[XY( xx, yy)] = CHSV(hue2 , 255, 255);//0xFFFF00;
  // rot in einer Acht
  xx = trackingObjectState[2] + sin8( millis() / 46) / trackingObjectShift[2];// / 32;
  yy = trackingObjectState[3] + cos8( millis() / 15) / trackingObjectShift[3];// / 32;
  if (xx < pWIDTH && yy < pHEIGHT)
    leds[XY( xx, yy)] = CHSV(hue , 255, 255);//0xFF0000;
  // Noise
  noise32_x[0] += 3000;
  noise32_y[0] += 3000;
  noise32_z[0] += 3000;
  scale32_x[0] = 8000;
  scale32_y[0] = 8000;
  FillNoise(0);
  MoveFractionalNoiseX(3, 0.33);
  MoveFractionalNoiseY(3);
}

void MultipleStream2() { // 3 comets
  trackingObjectState[0] = pWIDTH / 8;
  trackingObjectState[1] = pHEIGHT / 8;
  trackingObjectShift[0] = 255. / (pWIDTH - 1. - trackingObjectState[0] - trackingObjectState[0]);
  trackingObjectShift[1] = 255. / (pHEIGHT - 1. - trackingObjectState[1] - trackingObjectState[1]);
  trackingObjectState[2] = pWIDTH / 4;
  trackingObjectState[3] = pHEIGHT / 4;
  trackingObjectShift[2] = 255. / (pWIDTH - 1. - trackingObjectState[2] - trackingObjectState[2]); // ((pWIDTH>10)?9.:5.));
  trackingObjectShift[3] = 255. / (pHEIGHT - 1. - trackingObjectState[3] - trackingObjectState[3]); //- ((pHEIGHT>10)?9.:5.));
  dimAll(255U - modes[currentMode].Scale * 2);
  byte xx = trackingObjectState[0] + sin8( millis() / 10) / trackingObjectShift[0];// / 22;
  byte yy = trackingObjectState[1] + cos8( millis() / 9) / trackingObjectShift[1];// / 22;
  if (xx < pWIDTH && yy < pHEIGHT)
    leds[XY( xx, yy)] += CHSV(deltaHue , 255, 255);//0x0000FF;
  xx = trackingObjectState[2] + sin8( millis() / 10) / trackingObjectShift[2];// / 32;
  yy = trackingObjectState[3] + cos8( millis() / 7) / trackingObjectShift[3];// / 32;
  if (xx < pWIDTH && yy < pHEIGHT)
    leds[XY( xx, yy)] += CHSV(hue , 255, 255);//0xFF0000;
  leds[XY( CENTER_X_MINOR, CENTER_Y_MINOR)] += CHSV(hue2 , 255, 255);//0xFFFF00;
  noise32_x[0] += 3000;
  noise32_y[0] += 3000;
  noise32_z[0] += 3000;
  scale32_x[0] = 8000;
  scale32_y[0] = 8000;
  FillNoise(0);
  MoveFractionalNoiseX(2);
  MoveFractionalNoiseY(2, 0.33);
}

// ============= ЭФФЕКТ ЗВЁЗДНЫЕ ВОЙНЫ ===============
// (c) SottNick
void starwarsEmit(uint8_t i) //particlesEmit(Particle_Abstract *particle, ParticleSysConfig *g)
{
  if (deltaHue++ & 0x01)
    if (hue++ & 0x01)
      hue2++;//counter++;
  trackingObjectPosX[i] = boids[1].location.x;
  trackingObjectPosY[i] = boids[1].location.y;

  float dx = boids[0].location.x - boids[1].location.x;
  float dy = boids[0].location.y - boids[1].location.y;
  float dxy = dx * dx + dy * dy;
  if (dxy != 0) {
    dxy = SQRT_VARIANT(dxy) / 0.25; // 0.25 пикселя - расстояние, пролетаемое снарядом за 1 цикл
    trackingObjectSpeedX[i] = dx / dxy;
    trackingObjectSpeedY[i] = dy / dxy;
    trackingObjectState[i] = 60;
    trackingObjectHue[i] = hue2;
    trackingObjectIsShift[i] = true; // particle->isAlive

    if (!trackingObjectIsShift[0U] && pcnt) {
      trackingObjectPosX[0] = boids[0].location.x;
      trackingObjectPosY[0] = boids[0].location.y;
      trackingObjectSpeedX[0U] = (-4) * trackingObjectSpeedX[i];
      trackingObjectSpeedY[0U] = (-4) * trackingObjectSpeedY[i];
      trackingObjectState[0U] = 255;
      trackingObjectHue[0U] = hue;
      trackingObjectIsShift[0U] = true;
      pcnt--;
    }
  }
}

void starwarsRoutine() {
  deltaValue = 1; // количество зарождающихся частиц за 1 цикл //perCycle = 1;
  enlargedObjectNUM = (modes[currentMode].Scale - 1U) / 99.0 * (trackingOBJECT_MAX_COUNT - 1U) + 1U;
  if (enlargedObjectNUM > trackingOBJECT_MAX_COUNT) enlargedObjectNUM = trackingOBJECT_MAX_COUNT;
  for (int i = 0; i < enlargedObjectNUM; i++)
    trackingObjectIsShift[i] = false; // particle->isAlive
  boids[0].colorIndex = random8();
  boids[1].colorIndex = boids[0].colorIndex + 127U;
  trackingObjectShift[4] = pWIDTH / 8;
  trackingObjectShift[5] = pHEIGHT / 8;
  trackingObjectShift[0] = 255. / (pWIDTH - 1. - trackingObjectShift[4] - trackingObjectShift[4]);
  trackingObjectShift[1] = 255. / (pHEIGHT - 1. - trackingObjectShift[5] - trackingObjectShift[5]);
  trackingObjectShift[6] = pWIDTH / 4;
  trackingObjectShift[7] = pHEIGHT / 4;
  trackingObjectShift[2] = 255. / (pWIDTH - 1. - trackingObjectShift[6] - trackingObjectShift[6]); // ((pWIDTH>10)?9.:5.));
  trackingObjectShift[3] = 255. / (pHEIGHT - 1. - trackingObjectShift[7] - trackingObjectShift[7]); //- ((pHEIGHT>10)?9.:5.));
  boids[0].location.x = trackingObjectShift[4] + sin8( millis() / 10) / trackingObjectShift[0];// / 22;
  boids[0].location.y = trackingObjectShift[5] + cos8( millis() / 10) / trackingObjectShift[1];// / 22;
  boids[1].location.x = trackingObjectShift[6] + sin8( millis() / 46) / trackingObjectShift[2];// / 32;
  boids[1].location.y = trackingObjectShift[7] + cos8( millis() / 15) / trackingObjectShift[3];// / 32;
  step = random(2U);

  pcnt = 1U;
  if (modes[currentMode].Speed & 0x01)
    dimAll(127);
  else FastLED.clear();

  //go over particles and update matrix cells on the way
  for (int i = 0; i < enlargedObjectNUM; i++) {
    if (i > 0U && !trackingObjectIsShift[i] && step) {
      //emitter->emit(&particles[i], this->g);
      starwarsEmit(i);
      step--;
    }
    if (trackingObjectIsShift[i]) { // particle->isAlive
      particlesUpdate2(i);

      //generate RGB values for particle
      CRGB baseRGB = CHSV(trackingObjectHue[i], 255, 255); // particles[i].hue
      baseRGB.nscale8(trackingObjectState[i]);//эквивалент
      drawPixelXYF(trackingObjectPosX[i], trackingObjectPosY[i], baseRGB);
    }
  }
  drawPixelXYF(boids[0].location.x, boids[0].location.y, CHSV(boids[0].colorIndex, 160U, 255U));
  drawPixelXYF(boids[1].location.x, boids[1].location.y, CHSV(boids[1].colorIndex, 160U, 255U));
}

//==================================Радужный змей================================
void MultipleStream8() { // Windows ))
  if (loadingFlag) {
    loadingFlag = false;
    if (modes[currentMode].Scale > 1U)
      hue = (modes[currentMode].Scale - 2U) * 2.6;
    else
      hue = random8();
  }
  if (modes[currentMode].Scale <= 1U)
    hue++;
  dimAll(96); // < -- затухание эффекта для последующего кадра на 96/255*100=37%
  for (uint8_t y = 2; y < pHEIGHT - 1; y += 5) {
    for (uint8_t x = 2; x < pWIDTH - 1; x += 5) {
      leds[XY(x, y)]  += CHSV(x * y + hue, 255, 255);
      leds[XY(x + 1, y)] += CHSV((x + 4) * y + hue, 255, 255);
      leds[XY(x, y + 1)] += CHSV(x * (y + 4) + hue, 255, 255);
      leds[XY(x + 1, y + 1)] += CHSV((x + 4) * (y + 4) + hue, 255, 255);
    }
  }
  // Noise
  noise32_x[0] += 3000;
  noise32_y[0] += 3000;
  noise32_z[0] += 3000;
  scale32_x[0] = 8000;
  scale32_y[0] = 8000;
  FillNoise(0);
  MoveFractionalNoiseX(3);
  MoveFractionalNoiseY(3);
}

// ============= ЭФФЕКТ ИСТОЧНИКИ ===============
// (c) SottNick
void fountainsDrift(uint8_t j) {
  boids[j].location.x += boids[j].velocity.x;
  boids[j].location.y += boids[j].velocity.y;
  if (boids[j].location.x + boids[j].velocity.x < 0) {
    boids[j].location.x = -boids[j].location.x;
    boids[j].velocity.x = -boids[j].velocity.x;
  }
  if (boids[j].location.x > pWIDTH - 1) {
    boids[j].location.x = pWIDTH + pWIDTH - 2 - boids[j].location.x;
    boids[j].velocity.x = -boids[j].velocity.x;
  }
  if (boids[j].location.y < 0) {
    boids[j].location.y = -boids[j].location.y;
    boids[j].velocity.y = -boids[j].velocity.y;
  }
  if (boids[j].location.y > pHEIGHT - 1) {
    boids[j].location.y = pHEIGHT + pHEIGHT - 2 - boids[j].location.y;
    boids[j].velocity.y = -boids[j].velocity.y;
  }
}

void fountainsEmit(uint8_t i) {
  if (hue++ & 0x01)
    hue2++;
  uint8_t j = random8(enlargedObjectNUM);
  fountainsDrift(j);
  trackingObjectPosX[i] = boids[j].location.x;
  trackingObjectPosY[i] = boids[j].location.y;
  trackingObjectSpeedX[i] = ((float)random8() - 127.) / 512.; // random(_hVar)-_constVel; // particle->vx
  trackingObjectSpeedY[i] = SQRT_VARIANT(0.0626 - trackingObjectSpeedX[i] * trackingObjectSpeedX[i]);
  if (random8(2U)) {
    trackingObjectSpeedY[i] = -trackingObjectSpeedY[i];
  }
  trackingObjectState[i] = random8(50, 250); // random8(minLife, maxLife);// particle->ttl
  if (modes[currentMode].Speed & 0x01)
    trackingObjectHue[i] = hue2;// (counter/2)%255; // particle->hue
  else
    trackingObjectHue[i] = boids[j].colorIndex;//random8();
  trackingObjectIsShift[i] = true; // particle->isAlive
}

void fountainsRoutine() {
  if (loadingFlag)
  {
    loadingFlag = false;
    enlargedObjectNUM = (modes[currentMode].Scale - 1U) / 99.0 * (AVAILABLE_BOID_COUNT - 1U) + 1U;
    if (enlargedObjectNUM > AVAILABLE_BOID_COUNT) enlargedObjectNUM = AVAILABLE_BOID_COUNT;
    deltaValue = trackingOBJECT_MAX_COUNT / (SQRT_VARIANT(CENTER_X_MAJOR * CENTER_X_MAJOR + CENTER_Y_MAJOR * CENTER_Y_MAJOR) * 4U) + 1U; // 4 - это потому что за 1 цикл частица пролетает ровно четверть расстояния между 2мя соседними пикселями
    for (int i = 0; i < trackingOBJECT_MAX_COUNT; i++)
      trackingObjectIsShift[i] = false;
    for (int j = 0; j < enlargedObjectNUM; j++) {
      boids[j] = Boid(random8(pWIDTH), random8(pHEIGHT));
      boids[j].velocity.x = ((float)random8() - 127.) / 512.;
      boids[j].velocity.y = SQRT_VARIANT(0.0626 - boids[j].velocity.x * boids[j].velocity.x) /  8.; // скорость источников в восемь раз ниже, чем скорость частиц
      boids[j].velocity.x                                                        /= 8.; // скорость источников в восемь раз ниже, чем скорость частиц
      if (random8(2U))
        boids[j].velocity.y = -boids[j].velocity.y;
      boids[j].colorIndex = random8();
    }
  }
  step = deltaValue; //счётчик количества частиц в очереди на зарождение в этом цикле
  dimAll(127);

  //go over particles and update matrix cells on the way
  for (int i = 0; i < trackingOBJECT_MAX_COUNT; i++) {
    if (!trackingObjectIsShift[i] && step) {
      fountainsEmit(i);
      step--;
    }
    if (trackingObjectIsShift[i]) { // particle->isAlive
      particlesUpdate2(i);
      //generate RGB values for particle
      CRGB baseRGB = CHSV(trackingObjectHue[i], 255, 255); // particles[i].hue
      baseRGB.nscale8(trackingObjectState[i]);//эквивалент
      drawPixelXYF(trackingObjectPosX[i], trackingObjectPosY[i], baseRGB);
    }
  }
}

// --------- Эффект "Северное Сияние"
// (c) kostyamat 05.02.2021
// идеи подсмотрены тут https://www.reddit.com/r/FastLED/comments/jyly1e/challenge_fastled_sketch_that_fits_entirely_in_a/
// особая благодарность https://www.reddit.com/user/ldirko/ Yaroslaw Turbin aka ldirko
// вместо набора палитр в оригинальном эффекте сделан генератор палитр
#define AURORA_COLOR_RANGE 10 // (+/-10 единиц оттенка) диапазон, в котором плавает цвет сияния относительно выбранного оттенка 
#define AURORA_COLOR_PERIOD 2 // (2 раза в минуту) частота, с которой происходит колебание выбранного оттенка в разрешённом диапазоне

// генератор палитр для Северного сияния (c) SottNick
static const uint8_t MBAuroraColors_arr[5][4] PROGMEM = // палитра в формате CHSV
{ //№, цвет, насыщенность, яркость
  {0  , 0 , 255,   0},// black
  {80 , 0 , 255, 255},
  {130, 25, 220, 255},
  {180, 25, 185, 255},
  {255, 25, 155, 255} //245
};

CRGBPalette16 myPal;

void fillMyPal16_2(uint8_t hue, bool isInvert = false) {
  // я бы, конечно, вместо копии функции генерации палитры "_2"
  // лучше бы сделал её параметром указатель на массив с базовой палитрой,
  // но я пониятия не имею, как это делается с грёбаным PROGMEM

  int8_t lastSlotUsed = -1;
  uint8_t istart8, iend8;
  CRGB rgbstart, rgbend;

  // начинаем с нуля
  if (isInvert)
    //с неявным преобразованием оттенков цвета получаются, как в фотошопе, но для данного эффекта не красиво выглядят
    hsv2rgb_spectrum(CHSV(256 + hue - pgm_read_byte(&MBAuroraColors_arr[0][1]), pgm_read_byte(&MBAuroraColors_arr[0][2]), pgm_read_byte(&MBAuroraColors_arr[0][3])), rgbstart);
  else
    hsv2rgb_spectrum(CHSV(hue + pgm_read_byte(&MBAuroraColors_arr[0][1]), pgm_read_byte(&MBAuroraColors_arr[0][2]), pgm_read_byte(&MBAuroraColors_arr[0][3])), rgbstart);
  int indexstart = 0; // начальный индекс палитры
  for (uint8_t i = 1U; i < 5U; i++) { // в палитре @obliterator всего 5 строчек
    int indexend = pgm_read_byte(&MBAuroraColors_arr[i][0]);
    if (isInvert)
      hsv2rgb_spectrum(CHSV(hue + pgm_read_byte(&MBAuroraColors_arr[i][1]), pgm_read_byte(&MBAuroraColors_arr[i][2]), pgm_read_byte(&MBAuroraColors_arr[i][3])), rgbend);
    else
      hsv2rgb_spectrum(CHSV(256 + hue - pgm_read_byte(&MBAuroraColors_arr[i][1]), pgm_read_byte(&MBAuroraColors_arr[i][2]), pgm_read_byte(&MBAuroraColors_arr[i][3])), rgbend);
    istart8 = indexstart / 16;
    iend8   = indexend   / 16;
    if ((istart8 <= lastSlotUsed) && (lastSlotUsed < 15)) {
      istart8 = lastSlotUsed + 1;
      if (iend8 < istart8)
        iend8 = istart8;
    }
    lastSlotUsed = iend8;
    fill_gradient_RGB( myPal, istart8, rgbstart, iend8, rgbend);
    indexstart = indexend;
    rgbstart = rgbend;
  }
}

unsigned long polarTimer;

void auroraRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    emitterX = 400. / pHEIGHT; // а это - максимум без яркой засветки крайних рядов матрицы (сверху и снизу)
    ff_y = map(pWIDTH, 8, 64, 310, 63);
    ff_z = ff_y;
    speedfactor = map(modes[currentMode].Speed, 1, 255, 128, 16);
  }
  if (modes[currentMode].Scale == 100) {
    if (hue2++ & 0x01 && deltaHue++ & 0x01 && deltaHue2++ & 0x01) hue++; // это ж бред, но я хз. как с 60ю кадрами в секунду можно эффективно скорость замедлять...
    fillMyPal16_2((uint8_t)((modes[currentMode].Scale - 1U) * 2.55) + hue, modes[currentMode].Scale & 0x01);
  }
  else
    fillMyPal16_2((uint8_t)((modes[currentMode].Scale - 1U) * 2.55) + AURORA_COLOR_RANGE - beatsin8(AURORA_COLOR_PERIOD, 0U, AURORA_COLOR_RANGE + AURORA_COLOR_RANGE), modes[currentMode].Scale & 0x01);
  for (byte x = 0; x < pWIDTH; x++) {
    for (byte y = 0; y < pHEIGHT; y++) {
      polarTimer++;
      leds[XY(x, y)] = ColorFromPalette(myPal, qsub8(inoise8(polarTimer % 2 + x * ff_z, y * 16 + polarTimer % 16, polarTimer / speedfactor), fabs((float)pHEIGHT / 2 - (float)y) * emitterX));
    }
  }
}

//===================== Часы с циферблатом ======================================
// Эффект часов для gyver лампы
// вызов:
// ClockRoutine1(pWIDTH,pHEIGHT,true);  - цвет меток циферблата - радужный, цвет метки возле минутной стрелки стремится к цвету минутной стрелки
// фон черный.
// ClockRoutine1(pWIDTH,pHEIGHT,false); - цвет всех элементов - белый

uint8_t clock_type = 0;
uint8_t clock_height;
uint8_t clock_width;
void clocks() {
  if (loadingFlag)
  {
    loadingFlag = false;
    clock_type = (specialTextEffectParam >= 0) ? specialTextEffectParam : getEffectScaleParamValue2(MC_CLOCKS);

    //теперь часы всегда занимают квадратную область с размерами, соответствующими короткой стороне матрицы
    //не то чтобы эффект до этого не работал, даже наоборот, выводился вполне корректно, но часы, растянутые на всю ширину прямоугольной матрицы, выглядели стремно и тупо
    if (pWIDTH > pHEIGHT)
    {
      clock_width = pHEIGHT;
      clock_height = pHEIGHT;
    }
    if (pWIDTH < pHEIGHT)
    {
      clock_width = pWIDTH;
      clock_height = pWIDTH;
    }
    if (pWIDTH == pHEIGHT)
    {
      clock_width = pWIDTH;
      clock_height = pHEIGHT;
    }
    // Если авто - генерировать один из типов
    if (clock_type == 0 || clock_type > 2) {
      clock_type = random8(1, 3);
    }
  }

  switch (clock_type) {
    case 1: ClockRoutine1(clock_width, clock_height, false); break;
    default: ClockRoutine1(clock_width, clock_height, true); break;
  }
}

void ClockRoutine1(uint8_t cl_w, uint8_t cl_h, bool fon_clear)
{
  float r_clock;
  int c_x1;
  int c_y1;
  int c_x2;
  int c_y2;
  float dx = (cl_w % 2 == 1) ? 0.49 : 0;
  float dy = (cl_h % 2 == 1) ? 0.49 : 0;
  int c_cx = (pWIDTH - cl_w) / 2;
  int c_cy = (pHEIGHT - cl_h) / 2;
  time_t currentLocalTime = now();
  uint32_t thisTime = hour(currentLocalTime) * 60 + minute(currentLocalTime);
  CHSV color = CHSV(0, (modes[currentMode].Speed > 150) ? 0 : 255, (modes[currentMode].Speed < 150) ? 0 : 255);
  FastLED.clear();

  //Циферблат
  // метки 1,2,4,5, 7,8, 10,11 (цвет метка сопровождает цвет минутной стрелки
  for (int i = 0; i < 12; i++)
  {
    r_clock = PI * i / 6;
    c_x1 = sin(r_clock) * (cl_w / 2 + dx - 0.1) + cl_w / 2 + dx + c_cx;
    c_y1 = cos(r_clock) * (cl_h / 2 + dy - 0.1) + cl_h / 2 + dy + c_cy;

    if (fon_clear) color = CHSV(i * 21 - map(thisTime % 60, 0, 59, 0, 255) + map(modes[currentMode].Scale, 1, 100, 0, 255), 255, 255);
    drawPixelXY(c_x1, c_y1, color);
  }
  // метки 3, 9,
  if (cl_h % 2 == 0) {
    if (fon_clear) color = CHSV(192 - map(thisTime % 60, 0, 59, 0, 255) + map(modes[currentMode].Scale, 1, 100, 0, 255), 255, 255);
    DrawLine(c_cx, cl_h / 2 - 1 + c_cy,  c_cx, cl_h / 2 + c_cy, color);
    if (fon_clear) color = CHSV(64 - map(thisTime % 60, 0, 59, 0, 255) + map(modes[currentMode].Scale, 1, 100, 0, 255), 255, 255);
    DrawLine(c_cx + cl_w - 1, cl_h / 2 - 1 + c_cy, c_cx + cl_w - 1, cl_h / 2 + c_cy, color);
  }
  // метки 12, 6,
  if (cl_w % 2 == 0) {
    if (fon_clear) color = CHSV(128 - map(thisTime % 60, 0, 59, 0, 255) + map(modes[currentMode].Scale, 1, 100, 0, 255), 255, 255);
    DrawLine(cl_w / 2 - 1 + c_cx, c_cy,    cl_w / 2 + c_cx,    c_cy, color);
    if (fon_clear) color = CHSV(-map(thisTime % 60, 0, 59, 0, 255) + map(modes[currentMode].Scale, 1, 100, 0, 255), 255, 255);
    DrawLine(cl_w / 2 - 1 + c_cx, cl_h - 1 + c_cy, cl_w / 2 + c_cx, cl_h - 1 + c_cy, color);
  }

  // Минутная стрелка
  r_clock = PI * (thisTime % 60) / 30;
  c_x1 = sin(r_clock) + cl_w / 2 + dx + c_cx;
  c_y1 = cos(r_clock) + cl_h / 2 + dy + c_cy;
  c_x2 = sin(r_clock) * (cl_w / 2.5) + cl_w / 2 + dx + c_cx;
  c_y2 = cos(r_clock) * (cl_h / 2.5) + cl_h / 2 + dy + c_cy;
  if (fon_clear) color = CHSV(map(modes[currentMode].Scale, 1, 100, 0, 255), 255, 255);
  DrawLine(c_x1, c_y1, c_x2, c_y2, color);

  // Часовая стрелка
  r_clock = PI * (thisTime % 720) / 360;
  c_x1 = sin(r_clock) + cl_w / 2 + dx + c_cx;
  c_y1 = cos(r_clock) + cl_h / 2 + dy + c_cy;
  c_x2 = sin(r_clock) * (cl_w / 3) + cl_w / 2 + dx + c_cx;
  c_y2 = cos(r_clock) * (cl_h / 3) + cl_h / 2 + dy + c_cy;
  if (fon_clear) color = CHSV(modes[currentMode].Speed, 255, 255) ;
  DrawLine(c_x1, c_y1, c_x2, c_y2, color);
}

//Дополнительная функция построения линий
void DrawLine(int x1, int y1, int x2, int y2, CHSV color)
{
  int tmp;
  int x, y;
  int dx, dy;
  int err;
  int ystep;
  uint8_t swapxy = 0;
  if ( x1 > x2 ) dx = x1 - x2; else dx = x2 - x1;
  if ( y1 > y2 ) dy = y1 - y2; else dy = y2 - y1;
  if ( dy > dx )
  {
    swapxy = 1;
    tmp = dx; dx = dy; dy = tmp;
    tmp = x1; x1 = y1; y1 = tmp;
    tmp = x2; x2 = y2; y2 = tmp;
  }
  if ( x1 > x2 )
  {
    tmp = x1; x1 = x2; x2 = tmp;
    tmp = y1; y1 = y2; y2 = tmp;
  }
  err = dx >> 1;
  if ( y2 > y1 ) ystep = 1; else ystep = -1;
  y = y1;

  for ( x = x1; x <= x2; x++ )
  {
    if ( swapxy == 0 ) drawPixelXY(x, y, color);
    else drawPixelXY(y, x, color);
    err -= (uint8_t)dy;
    if ( err < 0 )
    {
      y += ystep;
      err += dx;
    }
  }
}

// ============== Swirl ================
//    © SlingMaster | by Alex Dovby  работает
//              EFF_SWIRL
//--------------------------------------
void Swirl() {
  uint8_t divide;
  uint8_t lastHue;
  uint8_t swirltype;
  static const uint32_t colors[5][6] PROGMEM = {
    {CRGB::Blue, CRGB::DarkRed, CRGB::Aqua, CRGB::Magenta, CRGB::Gold, CRGB::Green },
    {CRGB::Yellow, CRGB::LemonChiffon, CRGB::LightYellow, CRGB::Gold, CRGB::Chocolate, CRGB::Goldenrod},
    {CRGB::Green, CRGB::DarkGreen, CRGB::LawnGreen, CRGB::SpringGreen, CRGB::Cyan, CRGB::Black },
    {CRGB::Blue, CRGB::DarkBlue, CRGB::MidnightBlue, CRGB::MediumSeaGreen, CRGB::MediumBlue, CRGB:: DeepSkyBlue },
    {CRGB::Magenta, CRGB::Red, CRGB::DarkMagenta, CRGB::IndianRed, CRGB::Gold, CRGB::MediumVioletRed }
  };
  uint32_t color;
  if (loadingFlag) {
    loadingFlag = false;
    FastLED.clear();
    deltaValue = 255U - modes[currentMode].Speed + 1U;
    step = deltaValue;                      // чтообы при старте эффекта сразу покрасить лампу
    deltaHue2 = 0U;                         // count для замедления смены цвета
    deltaHue = 0U;                          // direction | 0 hue-- | 1 hue++ |
    hue2 = 0U;                              // x
    swirltype = random8(2);                 //у нас 2 вида эффекта, выбираем какой-то из них произвольно
  }
  if (step >= deltaValue) {
    step = 0U;
  }
  divide = floor((modes[currentMode].Scale - 1) / 20); // маштаб задает смену палитры
  // задаем цвет и рисуем завиток --------
  color = colors[divide][hue];
  drawPixelXY(hue2, deltaHue2, color);
  // -------------------------------------
  hue2++;                     // x
  // два варианта
  if (swirltype == 0) {
    deltaHue2++;
  }
  else {
    if (hue2 % 2 == 0)
      deltaHue2++;
  }
  // -------------------------------------
  if  (hue2 > pWIDTH) {
    hue2 = 0U;
  }
  if (deltaHue2 >= pHEIGHT) {
    deltaHue2 = 0U;
    // new swirl ------------
    hue2 = random8(pWIDTH - 2);
    // select new color -----
    hue = random8(6);
    if (lastHue == hue) {
      hue = hue + 1;
      if (hue >= 6) {
        hue = 0;
      }
    }
    lastHue = hue;
  }
  blurScreen(4U + random8(8));
  step++;
}

// =========== FeatherCandle ============
//         адаптация © SottNick
//    github.com/mnemocron/FeatherCandle
//      modify & design © SlingMaster
//           EFF_FEATHER_CANDLE
//                Свеча
//---------------------------------------
#include "data7x15flip.h"                       // FeatherCandle animation data
const uint8_t  level = 160;
const uint8_t  low_level = 110;
const uint8_t *ptr  = anim;                     // Current pointer into animation data
const uint8_t  wdth    = 7;                     // image width
const uint8_t  hght    = 13;                    // image height было 15
uint8_t        img[wdth * hght];                // Buffer for rendering image
uint8_t        last_brightness;

void FeatherCandleRoutine() {
  if (loadingFlag) {
    FastLED.clear();
    hue = 0;
    trackingObjectState[0] = low_level;
    trackingObjectState[1] = low_level;
    trackingObjectState[2] = low_level;
    trackingObjectState[4] = floor(pWIDTH * 0.5);
    loadingFlag = false;
  }
  uint8_t a = pgm_read_byte(ptr++);     // New frame X1/Y1
  if (a >= 0x90) {                      // EOD marker? (valid X1 never exceeds 8)
    ptr = anim;                         // Reset animation data pointer to start
    a   = pgm_read_byte(ptr++);         // and take first value
  }
  uint8_t x1 = a >> 4;                  // X1 = high 4 bits
  uint8_t y1 = a & 0x0F;                // Y1 = low 4 bits
  a  = pgm_read_byte(ptr++);            // New frame X2/Y2
  uint8_t x2 = a >> 4;                  // X2 = high 4 bits
  uint8_t y2 = a & 0x0F;                // Y2 = low 4 bits
  // Read rectangle of data from anim[] into portion of img[] buffer
  for (uint8_t y = y1; y <= y2; y++)
    for (uint8_t x = x1; x <= x2; x++) {
      img[y * wdth + x] = pgm_read_byte(ptr++);
    }
  int i = 0;
  uint8_t color = 40;  //красивое желто-оранжевое пламя с красными частицами, а не зеленое нечто

  // рисуем статичное мерцающее пламя для маленьких матриц высотой менее 13 пикселей (если матрица крупнее, выводится анимированное пламя)
  for (uint8_t y = 1; y < hght; y++) {
    if (pHEIGHT < 13) {
      // for small matrix -----
      if (y % 3 == 0) {
        //рисуем пламя по строкам снизу вверх
        leds[XY(floor(pWIDTH * 0.5) - 1, 3)] = CHSV(color - 20U , 255U, 50  + random8(50));
        leds[XY(floor(pWIDTH * 0.5) - 2, 4)] = CHSV(color - 20U , 255U, 50  + random8(50));
        leds[XY(floor(pWIDTH * 0.5) - 1, 4)] = CHSV(color       , 255U, 180 + random8(70));
        leds[XY(floor(pWIDTH * 0.5),     4)] = CHSV(color - 20U , 255U, 50  + random8(50));
        leds[XY(floor(pWIDTH * 0.5) - 2, 5)] = CHSV(color - 20U , 255U, 205 + random8(50));
        leds[XY(floor(pWIDTH * 0.5) - 1, 5)] = CHSV(color,        255U, 205 + random8(50));
        leds[XY(floor(pWIDTH * 0.5)    , 5)] = CHSV(color - 20U , 255U, 205 + random8(50));
        leds[XY(floor(pWIDTH * 0.5) - 2, 6)] = CHSV(color - 20U , 255U, 50  + random8(50));
        leds[XY(floor(pWIDTH * 0.5) - 1, 6)] = CHSV(color,        255U, 155 + random8(100));
        leds[XY(floor(pWIDTH * 0.5)    , 6)] = CHSV(color - 20U , 255U, 50  + random8(50));
        leds[XY(floor(pWIDTH * 0.5) - 1, 7)] = CHSV(color,        255U, 55  + random8(200));
      }
    } else {
      for (uint8_t x = 0; x < wdth; x++) {
        uint8_t brightness = img[i];
        leds[XY(delta_X + x, y)] = CHSV(brightness > 240 ? color : color - 10U , 255U, brightness); //255
        i++;
      }
    }
    // draw body FeatherCandle ------
    if (y <= 4) {
      if (y % 2 == 0) {
      }
    }
    //Рисуем свечу попиксельно слева направо и построчно сверху вниз
    //Да, это сделано в лоб и топорно, но при оригинальном выводе в цикле у второго и последнего столбца получаются значения цветов, которые на малых яркостях не видны
    //Если же сделать так, выглядит гораздо лучше, хотя используется меньшее количество цветов
    for (uint8_t y = 0; y < 3; y++) {
      leds[XY(floor(pWIDTH * 0.5) - 5, y)] = CHSV(48, 160U + abs(floor(pWIDTH * 0.5) - 5 - floor(pWIDTH * 0.5)) * 30, 255 - abs(floor(pWIDTH * 0.5) - 5 - floor(pWIDTH * 0.5)) * 60);
      leds[XY(floor(pWIDTH * 0.5) - 4, y)] = CHSV(48, 160U + abs(floor(pWIDTH * 0.5) - 2 - floor(pWIDTH * 0.5)) * 30, 255 - abs(floor(pWIDTH * 0.5) - 2 - floor(pWIDTH * 0.5)) * 60);
      leds[XY(floor(pWIDTH * 0.5) - 3, y)] = CHSV(48, 160U + abs(floor(pWIDTH * 0.5) - 3 - floor(pWIDTH * 0.5)) * 30, 255 - abs(floor(pWIDTH * 0.5) - 3 - floor(pWIDTH * 0.5)) * 60);
      leds[XY(floor(pWIDTH * 0.5) - 2, y)] = CHSV(48, 160U + abs(floor(pWIDTH * 0.5) - 2 - floor(pWIDTH * 0.5)) * 30, 255 - abs(floor(pWIDTH * 0.5) - 2 - floor(pWIDTH * 0.5)) * 60);
      leds[XY(floor(pWIDTH * 0.5) - 1, y)] = CHSV(48, 160U + abs(floor(pWIDTH * 0.5) - 1 - floor(pWIDTH * 0.5)) * 30, 255 - abs(floor(pWIDTH * 0.5) - 1 - floor(pWIDTH * 0.5)) * 60);
      leds[XY(floor(pWIDTH * 0.5)    , y)] = CHSV(48, 160U + abs(floor(pWIDTH * 0.5)     - floor(pWIDTH * 0.5)) * 30, 255 - abs(floor(pWIDTH * 0.5)     - floor(pWIDTH * 0.5)) * 60);
      leds[XY(floor(pWIDTH * 0.5) + 1, y)] = CHSV(48, 160U + abs(floor(pWIDTH * 0.5) + 1 - floor(pWIDTH * 0.5)) * 30, 255 - abs(floor(pWIDTH * 0.5) + 1 - floor(pWIDTH * 0.5)) * 60);
      leds[XY(floor(pWIDTH * 0.5) + 2, y)] = CHSV(48, 160U + abs(floor(pWIDTH * 0.5) + 2 - floor(pWIDTH * 0.5)) * 30, 255 - abs(floor(pWIDTH * 0.5) + 2 - floor(pWIDTH * 0.5)) * 60);
      leds[XY(floor(pWIDTH * 0.5) + 3, y)] = CHSV(48, 160U + abs(floor(pWIDTH * 0.5) + 3 - floor(pWIDTH * 0.5)) * 30, 255 - abs(floor(pWIDTH * 0.5) + 3 - floor(pWIDTH * 0.5)) * 60);
      leds[XY(floor(pWIDTH * 0.5) + 4, y)] = CHSV(48, 160U + abs(floor(pWIDTH * 0.5) - 2 - floor(pWIDTH * 0.5)) * 30, 255 - abs(floor(pWIDTH * 0.5) - 2 - floor(pWIDTH * 0.5)) * 60);
    }
    // drops of wax move -------------
    switch (hue ) {
      case 0:
        if (trackingObjectState[0] < level) {
          trackingObjectState[0]++;
        }
        break;
      case 1:
        if (trackingObjectState[0] > low_level) {
          trackingObjectState[0] --;
        }
        if (trackingObjectState[1] < level) {
          trackingObjectState[1] ++;
        }
        break;
      case 2:
        if (trackingObjectState[1] > low_level) {
          trackingObjectState[1] --;
        }
        if (trackingObjectState[2] < level) {
          trackingObjectState[2] ++;
        }
        break;
      case 3:
        if (trackingObjectState[2] > low_level) {
          trackingObjectState[2] --;
        } else {
          hue++;
          // set random position drop of wax
          trackingObjectState[4] = floor(pWIDTH * 0.5) - 3 + random8(6);
        }
        break;
    }
    if (hue > 3) {
      hue++;
    } else {
      if (hue < 2) {
        leds[XY(trackingObjectState[4], 2)] = CHSV(50U, 20U, trackingObjectState[0]);
      }
      if ((hue == 1) || (hue == 2)) {
        leds[XY(trackingObjectState[4], 1)] = CHSV(50U, 15U, trackingObjectState[1]); // - 10;
      }
      if (hue > 1) {
        leds[XY(trackingObjectState[4], 0)] = CHSV(50U, 5U, trackingObjectState[2]); // - 20;
      }
    }
  }
  // next -----------------
  if ((trackingObjectState[0] == level) || (trackingObjectState[1] == level) || (trackingObjectState[2] == level)) {
    hue++;
  }
}

// ***************************** КУБИК РУБИКА *****************************
uint8_t* cube_h   = NULL;  // Цвет плашек поля эффекта
uint8_t* order_h  = NULL;  // Порядок вывода плашек на поле
int16_t* order_mt = NULL;  // Для варианта "Спираль" - массив задержек движения полос
int16_t  cube_idx;         // Индекс выводимой плашки в фазе начального вывода плашек на матрицу
int16_t  cube_black_idx;   // Индекс черной плашки в варианте "Пятнашки"
int16_t  cube_new_idx;     // Индекс плашки в варианте "Пятнашки", куда будет перемещаться черная
int8_t   cube_last_mv;     // Прошлое направление движение цветной плашки на место черной в "Пятнашках" ;
uint8_t  cube_phase;       // Фаза формирования изображения: 0 - начальное размещение плашек на матрице;
uint8_t  cube_variant;     // Вариант анимации; 0 - случайный выбор; 1 - сдвиг по одной плашке; 2 - сдвиг всей полосы; 3 - вращение полос; 4 - пятнашки
uint16_t cube_size;        // Количество плашек на поле
uint8_t  cube_vh;          // 0 - вертикальное движение; 1 - горизонтальное
uint8_t  cube_rl;          // верт: 0 - вниз, 1 - вверх; гориз: 0 - влево; 1 - вправо
uint8_t  cube_move_cnt;    // На сколько линий в координатах матрицы (не плашек!) выполнять смещение
uint8_t  RUBIK_BLOCK_SIZE; // Размер квадратика палитры
uint8_t  py, px, ppx, ppy;
CHSV     cubeColorFrom;
CHSV     cubeColorTo;
void rubikRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    uint8_t old_RBS = RUBIK_BLOCK_SIZE;
    uint8_t max_block_size;
    set_EffectContrast(thisMode, 255);
    //определяем максимальный размер плитки как половину короткой стороны матрицы
    if (pWIDTH < pHEIGHT)
      max_block_size = round(pWIDTH / 2);
    else
      max_block_size = round(pHEIGHT / 2);
    RUBIK_BLOCK_SIZE = map8(getEffectScaleParamValue(MC_RUBICK), 2, max_block_size); //размер плитки от 2 до половины короткой стороны матрицы через ползунок варианта
    num_x = pWIDTH / RUBIK_BLOCK_SIZE;
    num_y = pHEIGHT / RUBIK_BLOCK_SIZE;
    off_x = (pWIDTH - RUBIK_BLOCK_SIZE * num_x) / 2;
    off_y = (pHEIGHT - RUBIK_BLOCK_SIZE * num_y) / 2;
    cube_last_mv = -1;
    hue = 0;
    cube_size = num_x * num_y;
    uint8_t step = 256 / (cube_size + 1);
    if (step < 10) step = 10;
    if (old_RBS != RUBIK_BLOCK_SIZE || cube_h == NULL) {
      if (cube_h != NULL) {
        delete [] cube_h;
        delete [] order_h;
        delete [] order_mt;
      }
      cube_h   = new uint8_t[cube_size]; for (uint8_t i = 0; i < cube_size; i++) {
        cube_h[i]  = hue;
        hue += step;
      }
      order_h  = new uint8_t[cube_size]; for (uint8_t i = 0; i < cube_size; i++) {
        order_h[i] = i;
      }
      order_mt = new int16_t[max(num_x, num_y)];
    }
    // Перемешать плашки и их порядок появления на матрице
    for (uint16_t i = 0; i < cube_size; i++) {
      uint16_t idx1 = random16(0, cube_size - 1);
      uint16_t idx2 = random16(0, cube_size - 1);
      hue = cube_h[idx1];
      cube_h[idx1] = cube_h[idx2];
      cube_h[idx2] = hue;
      idx1 = random16(0, cube_size - 1);
      idx2 = random16(0, cube_size - 1);
      hue = order_h[idx1];
      order_h[idx1] = order_h[idx2];
      order_h[idx2] = hue;
    }
    // Если в настройках выбран вариант "Случайный" - выбрать любой другой из доступных
    //   cube_variant = getEffectScaleParamValue2(MC_RUBIK); //nj же самое, зависит от ползунка вариант
    cube_variant = 4;
    if (cube_variant == 0 || cube_variant > 4) cube_variant = random8(1, 4);
    cube_idx = 0;
    cube_black_idx = random16(0, cube_size);
    cube_phase = 0;    // Фаза 0 - размещение плашек на матрице
    FastLED.clear();
  }
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  if (cube_phase == 0) {
    // Взять из массива порядка размещения плашек очередную позицию, из массива цветов - цвет  и вывести на матрицу очередную плашку
    uint16_t idx = order_h[cube_idx];
    CHSV color = cube_variant == 4 && idx == cube_black_idx
                 ? CHSV(0, 0, 0)
                 : CHSV(cube_h[idx], 255, effectBrightness);
    py = idx / num_x;
    px = idx % num_x;
    ppx = off_x + px * RUBIK_BLOCK_SIZE;
    ppy = off_y + py * RUBIK_BLOCK_SIZE;
    for (uint8_t y = 0; y < RUBIK_BLOCK_SIZE; y++) {
      for (uint8_t x = 0; x < RUBIK_BLOCK_SIZE; x++) {
        drawPixelXY(ppx + x, ppy + y, color);
      }
    }
    cube_idx++;
    if (cube_idx >= cube_size) {
      // Все плашки выведены? - перейти к следующей фазе.
      cube_idx = 0;
      cube_phase++;
    }
    return;
  }
  // Фаза "Настройка движения". Определяем что и в какую сторону нужно сдвигать
  if (cube_phase == 1) {
    cube_idx = 0;
    if (cube_variant == 4) {
      // Определяем текущие координаты черной плашки - px,py
      // Определяем какой из квадратиков вокруг черного будет перемещаться на его место:
      py = cube_black_idx / num_x;          // X,Y координаты черной плашки в сетке размера RUBIK_BLOCK_SIZE
      px = cube_black_idx % num_x;
      ppx = off_x + px * RUBIK_BLOCK_SIZE;  // левый верхний угол черной плашки в координатах матрицы
      ppy = off_y + py * RUBIK_BLOCK_SIZE;
      bool ok = false;
      uint8_t mv = 0;
      // Движение "черной плашки": цветная плашка перемещается на место черной - определить с какой строны плашка будет еремещаться на место черной
      while (!ok) {
        mv = random8(0, 4);  // 0 - сверху, 1 - справа, 2 - снизу, 3 - слева
        // Проверить, что новое направление не противоположно предыдущему, чтобы черная плашка не ерзала туда-сюда
        ok = mv < 4 && ((mv == 0 && cube_last_mv != 2) || (mv == 2 && cube_last_mv != 0) || (mv == 1 && cube_last_mv != 3) || (mv == 3 && cube_last_mv != 1));
        // Проверить, что выбранная плашка не выходит за границы поля. Например - черная в нулевой строке, а выбор пал на плашку выше - там ее нет - поле кончилось
        ok &= (mv == 0 && py > 0) || (mv == 2 && py < num_y - 1) || (mv == 1 && px < num_x - 1) || (mv  == 3 && px > 0);
      }
      switch (mv) {
        case 0:
          // Плашка сверху от черной
          cube_new_idx = (py - 1) * num_x + px;
          break;
        case 1:
          // Плашка справа от черной
          cube_new_idx = py * num_x + px + 1;
          break;
        case 2:
          // Плашка снизу от черной
          cube_new_idx = (py + 1) * num_x + px;
          break;
        case 3:
          // плашка слева от черной
          cube_new_idx = py * num_x + px - 1;
          break;
      }
      cube_last_mv = mv;
      cubeColorFrom = CHSV(cube_h[cube_new_idx], 255, effectBrightness);  // Цветная плашка
      cubeColorTo = CHSV(0, 0, 0);                                        // Черная плашка
    } else if (cube_variant == 3) {
      // Определяем какая полоса будет двигаться (индекс кубика), вертикально/горизонтально и в каком направдении вверх/вниз / вправо/влево
      cube_vh = random16(0, cube_size - 1) % 2;           // как - вертикально или горизонтально: 0 - вертикальное движение; 1 - горизонтальное - чередовать
      cube_rl = random16(0, cube_size - 1) % 2;           // куда - верт: 0 - вниз, 1 - вверх; гориз: 0 - влево; 1 - вправо
      uint8_t cube_mt = random16(0, cube_size - 1) % 2;   // начало: верт: 0 с левой до правой, 1 - с правой до левой; гориз: 0 - с верхней до нижней; 1 - с нижней до верхней
      // Сдвиг на целую плашку занимает RUBIK_BLOCK_SIZE шагов;
      // Начало движения каждой следующей полосы задерживается на RUBIK_BLOCK_SIZE / 2 шагов
      // Сдвиг полной полосы по горизонтали занимает num_x * RUBIK_BLOCK_SIZE шагов
      // Сдвиг полной полосы по вертикали занимает num_y * RUBIK_BLOCK_SIZE шагов
      // Массив с задержками для плашки размером 4x4 пикселя, в поле 4x4 плашки с учетом задержек определяется массивом [-6, -4, -2, 0]
      // При каждом проходе увеличиваем на 1 элемент массива. Пока элемент массива меньше нуля - сдвига нет; Если 0 или больше - выполняем сдвиг
      // Когда значение элемента массива достигает num_x * RUBIK_BLOCK_SIZE для гориз или num_y * RUBIK_BLOCK_SIZE по верикали - перестаем прокручивать полосу (движение завершено)
      // Когда ВСЕ элементы массива достигают верхнего предела - весь цикл завершен, меняем cube_phase на 1 - возврат к началу формирования цикла эффекта
      uint8_t stp = RUBIK_BLOCK_SIZE + RUBIK_BLOCK_SIZE / 2;
      uint8_t mcnt = cube_vh == 0 ? num_x : num_y;
      int8_t low = -1 * stp * (mcnt - 1);
      for (uint8_t i = 0; i < mcnt; i++) order_mt[i] = low + (cube_mt == 0 ? i : mcnt - i - 1) * stp;
    } else
    {
      // cube_variant == 2, cube_variant == 1
      // Определяем какая полоса будет двигаться (индекс кубика), вертикально/горизонтально и в каком направлении вверх/вниз | вправо/влево
      cube_vh = cube_vh == 0 ? 1 : 0;  // 0 - вертикальное движение; 1 - горизонтальное - чередовать
      cube_rl = random16(0, cube_size - 1) % 2;  // верт: 0 - вниз, 1 - вверх; гориз: 0 - влево; 1 - вправо
      // Для вертикального смещения - номер колонки, которая будет двигаться, для горизонтального - номер строки
      if (cube_vh == 0) {
        px = random8(0, num_x);          // Случайная колонка, которая  будет смещаться вниз или вверх
      } else {
        py = random8(0, num_y);          // Случайная строка, которая будет смешаться влево или вправо
      }
      if (px >= num_x) px = num_x - 1;
      if (py >= num_y) py = num_y - 1;

      // смещение на один блок для режима 1 или на всю ширину / высоту для режима 2 и 3
      cube_move_cnt = cube_variant == 1 ? RUBIK_BLOCK_SIZE : (cube_vh == 0 ? num_y : num_x) * RUBIK_BLOCK_SIZE;
    }
    cube_phase++;
  }
  // Отображение следующей фазы зависит от выбранного варианта
  // Перемещение плашек (cube_phase == 2)
  switch (cube_variant) {
    // 1 - Сдвиг на одину плашку всего ряда/колонки
    // 2 - Сдвиг всей полосы (ряд / колонка) на несколько плашек
    case 1:
    case 2: {
        cube_idx++;
        bool isEdge = cube_idx >= RUBIK_BLOCK_SIZE;
        if (isEdge) cube_idx = 0;
        rubikMoveLane(px, py, isEdge, effectBrightness);
        // Если все шаги по перемещению полосы завершены - вернуться к файзе формирования направления движения следующей полосы
        cube_move_cnt--;
        if (cube_move_cnt == 0) {
          cube_phase = 1;
        }
      }
      break;
    case 3: {
        // Вращение полос
        uint8_t mcnt = cube_vh == 0 ? num_x : num_y; // Количество полос - в зависимости от верт/гориз - это либо ширина, либо высота матрицы
        uint8_t maxx = mcnt * RUBIK_BLOCK_SIZE;      // Максимальное уол-во шагов сдвига полосы - по количеству светодиодов
        bool processed = false;
        for (uint8_t i = 0; i < mcnt; i++) {
          int8_t cnt = order_mt[i] + 1;
          order_mt[i] = cnt;
          if (cnt > 0 && cnt <= maxx) {
            processed = true;
            // сдвиг ровно на одну плашку?
            bool isEdge = cnt > 0 && cnt % RUBIK_BLOCK_SIZE == 0;
            rubikMoveLane(i, i, isEdge, effectBrightness);
          }
        }
        if (!processed) {
          cube_phase = 1;
        }
      }
      break;
    case 4: {
        // Пятнашки
        // Имитируем движение цветной плашки (cube_new_idx) на место черной (cube_black_idx)
        if (cube_new_idx - cube_black_idx > 1) {
          // Вертикально. Цветная вверх на место черной; cube_new_idx - индекс цветной плашки, которая движется на место черной
          for (uint8_t x = 0; x < RUBIK_BLOCK_SIZE; x++) {
            drawPixelXY(ppx + x, ppy + RUBIK_BLOCK_SIZE - cube_idx - 1, cubeColorFrom);
            drawPixelXY(ppx + x, ppy + 2 * RUBIK_BLOCK_SIZE - cube_idx - 1, cubeColorTo);
          }
        } else if (cube_new_idx - cube_black_idx < -1) {
          // Вертикально. Цветная вниз на место черной; newIdx - индекс цветной плашки, которая движется на место черной
          for (uint8_t x = 0; x < RUBIK_BLOCK_SIZE; x++) {
            drawPixelXY(ppx + x, ppy + cube_idx, cubeColorFrom);
            drawPixelXY(ppx + x, ppy - RUBIK_BLOCK_SIZE + cube_idx, cubeColorTo);
          }
        } else if (cube_new_idx - cube_black_idx == -1) {
          // Горизонтально. Цветная вправо на место черной; newIdx - индекс цветной плашки, которая движется на место черной
          for (uint8_t y = 0; y < RUBIK_BLOCK_SIZE; y++) {
            drawPixelXY(ppx + cube_idx, ppy + y, cubeColorFrom);
            drawPixelXY(ppx - RUBIK_BLOCK_SIZE + cube_idx, ppy + y, cubeColorTo);
          }
        } else if (cube_new_idx - cube_black_idx == 1) {
          // Горизонтально. Цветная влево на место черной; newIdx - индекс цветной плашки, которая движется на место черной
          for (uint8_t y = 0; y < RUBIK_BLOCK_SIZE; y++) {
            drawPixelXY(ppx + RUBIK_BLOCK_SIZE - cube_idx - 1, ppy + y, cubeColorFrom);
            drawPixelXY(ppx + 2 * RUBIK_BLOCK_SIZE - cube_idx - 1, ppy + y, cubeColorTo);
          }
        }
        cube_idx++;
        if (cube_idx >= RUBIK_BLOCK_SIZE) {
          cube_h[cube_black_idx] = cube_h[cube_new_idx];
          cube_black_idx = cube_new_idx;
          // Если перемещение блока завершено - фаза движения закончена, перейти к фазе настройки следующего движения
          cube_phase = 1;
        }
      }
      break;
  }
}

void rubikMoveLane(uint8_t px, uint8_t py, bool isEdge, uint8_t effectBrightness) {
  CHSV cubeColorFrom;
  uint8_t  hue;
  uint16_t idx, idx1, idx2;
  uint32_t color;
  if (cube_vh == 0 && cube_rl == 0) {
    // Вертикально. Сдвиг колонки вниз - перерисовать строки с 1 до высоты матрицы сдвинуть вниз? нулевую заполнить цветом нижней в колонке плашки
    ppx = off_x + px * RUBIK_BLOCK_SIZE;
    for (uint8_t i = num_y  * RUBIK_BLOCK_SIZE + off_y - 1; i > off_y; i--) {
      color = getPixColorXY(ppx, i - 1);
      for (uint8_t x = 0; x < RUBIK_BLOCK_SIZE; x++) {
        drawPixelXY(ppx + x, i, color);
      }
    }
    // Поскольку сдвиг вниз -  заполнить освободившуюся верхнюю строку цветом нижней плашки
    idx = (num_y - 1) * num_x + px;
    cubeColorFrom = CHSV(cube_h[idx], 255, effectBrightness);
    for (uint8_t x = 0; x < RUBIK_BLOCK_SIZE; x++) {
      drawPixelXY(ppx + x, off_y, cubeColorFrom);
    }
  } else if (cube_vh == 0 && cube_rl == 1) {
    // Вертикально. Сдвиг колонки вверх
    ppx = off_x + px * RUBIK_BLOCK_SIZE;
    for (uint8_t i = off_y; i < num_y * RUBIK_BLOCK_SIZE + off_y - 1; i++) {
      color = getPixColorXY(ppx, i + 1);
      for (uint8_t x = 0; x < RUBIK_BLOCK_SIZE; x++) {
        drawPixelXY(ppx + x, i, color);
      }
    }
    // Поскольку сдвиг вверх - заполнить освободившуюся нижнюю строку цветом верхней плашки
    idx = px;
    cubeColorFrom = CHSV(cube_h[idx], 255, effectBrightness);
    for (uint8_t x = 0; x < RUBIK_BLOCK_SIZE; x++) {
      drawPixelXY(ppx + x, num_y * RUBIK_BLOCK_SIZE + off_y - 1, cubeColorFrom);
    }
  } else if (cube_vh == 1 && cube_rl == 0) {
    // Горизонтально. Сдвиг строки влево
    ppy = off_y + py * RUBIK_BLOCK_SIZE;
    for (uint8_t i = off_x; i < num_x * RUBIK_BLOCK_SIZE + off_x - 1; i++) {
      color = getPixColorXY(i + 1, ppy);
      for (uint8_t y = 0; y < RUBIK_BLOCK_SIZE; y++) {
        drawPixelXY(i, ppy + y, color);
      }
    }
    // Поскольку сдвиг влeво -  заполнить освободившуюся правую колонку цветом левой плашки
    idx = py *  num_x;
    cubeColorFrom = CHSV(cube_h[idx], 255, effectBrightness);
    for (uint8_t y = 0; y < RUBIK_BLOCK_SIZE; y++) {
      drawPixelXY(num_x * RUBIK_BLOCK_SIZE + off_x - 1 , ppy + y, cubeColorFrom);
    }
  } else if (cube_vh == 1 && cube_rl == 1) {
    // Горизонтально. Сдвиг строки вправо
    ppy = off_y + py * RUBIK_BLOCK_SIZE;
    for (uint8_t i = num_x  * RUBIK_BLOCK_SIZE + off_x - 1; i > off_x; i--) {
      uint32_t color = getPixColorXY(i - 1, ppy);
      for (uint8_t y = 0; y < RUBIK_BLOCK_SIZE; y++) {
        drawPixelXY(i, ppy + y, color);
      }
    }
    // Поскольку сдвиг вправо -  заполнить освободившуюся левую колонку цветом правой плашки
    idx = py * num_x + num_x - 1;
    cubeColorFrom = CHSV(cube_h[idx], 255, effectBrightness);
    for (uint8_t y = 0; y < RUBIK_BLOCK_SIZE; y++) {
      drawPixelXY(off_x, ppy + y, cubeColorFrom);
    }
  }
  // Перемещение на одну плашку закончено?
  if (isEdge) {
    // Сместить индексы цветов в таблице текущих цветов плашек
    if (cube_vh == 0 && cube_rl == 0) {
      // Вертикально. Сдвиг колонки вниз
      idx = (num_y - 1) * num_x + px; // цвет последней строки
      hue = cube_h[idx];
      for (uint8_t i = num_y - 1; i > 0; i--) {
        idx1 = (i - 1) * num_x + px;
        idx2 = i * num_x + px;
        cube_h[idx2] = cube_h[idx1];
      }
      idx = px; // Индекс первой строки - поместить туда цвет, который был вытеснен из последней строки при сдвиге плашек вниз
      cube_h[idx] = hue;
    } else if (cube_vh == 0 && cube_rl == 1) {
      // Вертикально. Сдвиг колонки вверх
      idx = px; // цвет первой строки
      hue = cube_h[idx];
      for (uint8_t i = 0; i < num_y; i++) {
        idx1 = (i + 1) * num_x + px;
        idx2 = i * num_x + px;
        cube_h[idx2] = cube_h[idx1];
      }
      idx = (num_y - 1) * num_x + px; // Индекс последней строки - поместить туда цвет, который был вытеснен из первой строки при сдвиге плашек вверх
      cube_h[idx] = hue;
    } else if (cube_vh == 1 && cube_rl == 0) {
      // Горизонтально. Сдвиг строки влево
      idx = py * num_x; // цвет первой колонки в строке
      hue = cube_h[idx];
      for (uint8_t i = 0; i < num_x; i++) {
        idx1 = py * num_x + i + 1;
        idx2 = py * num_x + i;
        cube_h[idx2] = cube_h[idx1];
      }
      idx = py * num_x + num_x - 1; // Индекс последней колонки в строке - поместить туда цвет, который был вытеснен из первой колонки при сдвиге плашек влево
      cube_h[idx] = hue;
    } else if (cube_vh == 1 && cube_rl == 1) {
      // Горизонтально. Сдвиг строки вправо
      idx = py * num_x + num_x - 1; // цвет последней колонки в строке
      hue = cube_h[idx];
      for (uint8_t i = num_x - 1; i > 0; i--) {
        idx1 = py * num_x + i - 1 ;
        idx2 = py * num_x + i;
        cube_h[idx2] = cube_h[idx1];
      }
      idx = py * num_x; // Индекс первой колонки в строкн - поместить туда цвет, который был вытеснен из последней колонки при сдвиге плашек вправр
      cube_h[idx] = hue;
    }
  }
}

void rubikRoutineRelease() {
  if (cube_h   != NULL) {
    delete [] cube_h;
    cube_h   = NULL;
  }
  if (order_h  != NULL) {
    delete [] order_h;
    order_h  = NULL;
  }
  if (order_mt != NULL) {
    delete [] order_mt;
    order_mt = NULL;
  }
}

// =====================================
//            Цветные кудри
//           Color Frizzles
//             © Stepko
//       адаптация © SlingMaster
// =====================================
#define DYNAMIC                (0U)   // динамическая задержка для кадров ( будет использоваться бегунок Скорость )
#define LOW_DELAY             (15U)   // низкая фиксированная задержка для смены кадров
uint8_t FPSdelay = DYNAMIC;

void ColorFrizzles() {
  if (loadingFlag) {
    loadingFlag = false;
    FPSdelay = 10U;
    deltaValue = 0;
  }
  if (modes[currentMode].Scale > 50) {
    if (FPSdelay > 48) deltaValue = 0;
    if (FPSdelay < 5) deltaValue = 1;
    if (deltaValue == 1) {
      FPSdelay++;
    } else {
      FPSdelay--;
    }
    blur2d(leds, pWIDTH, pHEIGHT, 16);
  } else {
    FPSdelay = 20;
    dimAll(240U);
  }
  for (byte i = 8; i--;) {
    leds[XY(beatsin8(12 + i, 0, pWIDTH - 1), beatsin8(15 - i, 0, pHEIGHT - 1))] = CHSV(beatsin8(12, 0, 255), 255, (255 - FPSdelay * 2));
  }
}

// ============ Lotus Flower ============
//             © SlingMaster
//               EFF_LOTUS
//             Цветок Лотоса пусть будет
//---------------------------------------

uint8_t validMinMax(float val, uint8_t minV, uint32_t maxV) {
  uint8_t result;
  if (val <= minV) {
    result = minV;
  } else if (val >= maxV) {
    result = maxV;
  } else {
    result = ceil(val);
  }
  return result;
}

void  Flower() {
  uint8_t br;
  if (step < 128) {
    br = 255 - step;  // 255 >> 128
  } else {
    br = step;        // 128 >> 255
  }
  if (modes[currentMode].Scale > 10) {
    dimAll(90);
    hue = floor(modes[currentMode].Scale * 1.9) + floor(br / 4);
  } else {
    FastLED.clear();
    hue = step;
  }
  if (step > 190) {
    hue2 = validMinMax(hue - 64 + floor(br / 4), 190, 250);
  } else {
    hue2 = hue + 64 - floor(br / 4);
  }
  for (uint8_t x = 0U ; x < pWIDTH ; x++) {
    if (x % 6 == 0) {
      gradientVertical( x - deltaValue, 2U, x + 1 - deltaValue, pHEIGHT - floor((255 - br) / 24) - random8(2), hue, hue2, 255, floor(br * 0.5), 255U);
      gradientVertical( x + 3U - deltaValue, 0U, x + 4U - deltaValue, pHEIGHT - floor(br / 24) + random8(3), hue, hue2, 255, floor((255 - br * 0.5)), 255U);
      drawPixelXY(x - deltaValue, 0, 0x005F00);
    }
  }
}

void LotusFlower() {
  if (loadingFlag) {
    loadingFlag = false;
    deltaValue = 0;
    step = deltaValue;
    deltaValue = 0;
    hue = 120;
    hue2 = 0;
    deltaHue = 0;
    deltaHue2 = 0;
    FastLED.clear();
  }
  Flower();
  if (deltaHue == 0) {               // rotation
    deltaValue--;
    if (deltaValue <= 0) {
      deltaValue = 3;
    }
  } else {
    deltaValue++;
    if (deltaValue >= 3) {
      deltaValue = 0;
    }
  }
  deltaHue2++;
  if (deltaHue2 >= 18) {           // swap direction rotate
    deltaHue2 = 0;
    deltaHue = (deltaHue == 0) ? 1 : 0;
  }
  step++;
}

// =========== Christmas Tree ===========
//             © SlingMaster
//           EFF_CHRISTMAS_TREE
//            Новогодняя Елка
//---------------------------------------
void clearNoiseArr() {
  for (uint8_t x = 0U; x < pWIDTH; x++) {
    for (uint8_t y = 0U; y < pHEIGHT; y++) {
      noise_3d[0][x][y] = 0;
      noise_3d[1][x][y] = 0;
    }
  }
}

//---------------------------------------
void VirtualSnow(byte snow_type) {
  uint8_t posX = random8(pWIDTH - 1);
  const uint8_t maxX = pWIDTH - 1;
  static int deltaPos;
  byte delta = (snow_type == 3) ? 0 : 1;
  for (uint8_t x = delta; x < pWIDTH - delta; x++) {
    // заполняем случайно верхнюю строку
    if ((noise_3d[0][x][pHEIGHT - 2] == 0U) &&  (posX == x) && (random8(0, 2) == 0U)) {
      noise_3d[0][x][pHEIGHT] = 1;
    } else {
      noise_3d[0][x][pHEIGHT] = 0;
    }
    for (uint8_t y = 0U; y < pHEIGHT; y++) {
      switch (snow_type) {
        case 0:
          noise_3d[0][x][y] = noise_3d[0][x][y + 1];
          deltaPos = 0;
          break;
        case 1:
        case 2:
          noise_3d[0][x][y] = noise_3d[0][x][y + 1];
          deltaPos = 1 - random8(2);
          break;
        default:
          deltaPos = -1;
          if ((x == 0 ) & (y == 0 ) & (random8(2) == 0U)) {
            noise_3d[0][pWIDTH - 1][random8(CENTER_Y_MAJOR / 2, pHEIGHT - CENTER_Y_MAJOR / 4)] = 1;
          }
          if (x > pWIDTH - 2) {
            noise_3d[0][pWIDTH - 1][y] = 0;
          }
          if (x < 1)  {
            noise_3d[0][x][y] = noise_3d[0][x][y + 1];
          } else {
            noise_3d[0][x - 1][y] = noise_3d[0][x][y + 1];
          }
          break;
      }
      if (noise_3d[0][x][y] > 0) {
        if (snow_type < 3) {
          if (y % 2 == 0U) {
            leds[XY(x - ((x > 0) ? deltaPos : 0), y)] = CHSV(160, 5U, random8(200U, 240U));
          } else {
            leds[XY(x + deltaPos, y)] = CHSV(160, 5U,  random8(200U, 240U));
          }
        } else {
          leds[XY(x, y)] = CHSV(160, 5U,  random8(200U, 240U));
        }
      }
    }
  }
}

//---------------------------------------
void GreenTree(uint8_t tree_h) {
  hue = floor(step / 32) * 32U;
  for (uint8_t x = 0U; x < pWIDTH + 1 ; x++) {
    if (x % 8 == 0) {
      if (modes[currentMode].Scale < 60) {
        // nature -----
        DrawLine(x - 1U - deltaValue, floor(tree_h * 0.70), x + 1U - deltaValue, floor(tree_h * 0.70), 0x002F00);
        DrawLine(x - 1U - deltaValue, floor(tree_h * 0.55), x + 1U - deltaValue, floor(tree_h * 0.55), 0x004F00);
        DrawLine(x - 2U - deltaValue, floor(tree_h * 0.35), x + 2U - deltaValue, floor(tree_h * 0.35), 0x005F00);
        DrawLine(x - 2U - deltaValue, floor(tree_h * 0.15), x + 2U - deltaValue, floor(tree_h * 0.15), 0x007F00);
        drawPixelXY(x - 3U - deltaValue, floor(tree_h * 0.15), 0x001F00);
        drawPixelXY(x + 3U - deltaValue, floor(tree_h * 0.15), 0x001F00);
        if ((x - deltaValue ) >= 0) {
          gradientVertical( x - deltaValue, 0U, x - deltaValue, tree_h, 90U, 90U, 190U, 64U, 255U);
        }
      } else {
        // holiday -----
        drawPixelXY(x - 1 - deltaValue, floor(tree_h * 0.6), CHSV(step, 255U, 128 + random8(128)));
        drawPixelXY(x + 1 - deltaValue, floor(tree_h * 0.6), CHSV(step, 255U, 128 + random8(128)));
        drawPixelXY(x - deltaValue, floor(tree_h * 0.4), CHSV(step, 255U, 200U));
        drawPixelXY(x - deltaValue, floor(tree_h * 0.2), CHSV(step, 255U, 190 + random8(65)));
        drawPixelXY(x - 2 - deltaValue, floor(tree_h * 0.25), CHSV(step, 255U, 96 + random8(128)));
        drawPixelXY(x + 2 - deltaValue, floor(tree_h * 0.25), CHSV(step, 255U, 96 + random8(128)));
        drawPixelXY(x - 2 - deltaValue, 1U, CHSV(step, 255U, 200U));
        drawPixelXY(x - deltaValue, 0U, CHSV(step, 255U, 250U));
        drawPixelXY(x + 2 - deltaValue, 1U, CHSV(step, 255U, 200U));
        gradientVertical( x - deltaValue, floor(tree_h * 0.75), x - deltaValue, tree_h,  hue, hue, 250U, 0U, 128U);
      }
    }
  }
}

//---------------------------------------
uint8_t treemove;
uint8_t snowspeed;
void ChristmasTree() {
  static uint8_t tree_h = pHEIGHT;
  if (loadingFlag) {
    loadingFlag = false;
    treemove = getEffectScaleParamValue2(MC_TREE); //передаем значение чекбокса
    snowspeed = map8(getEffectScaleParamValue(MC_TREE), 0, 4); //0 - снег падает строго вертикально, 1 - раскачиваясь из стороны в сторону, далее начинается метель
    clearNoiseArr();
    deltaValue = 0;
    step = deltaValue;
    FastLED.clear();
    if (pHEIGHT > 16) tree_h = 16;
  }
  if (pHEIGHT > 16) {
    if (modes[currentMode].Scale < 60) {
      gradientVertical(0, 0, pWIDTH, pHEIGHT, 160, 160, 64, 128, 255U);
    } else {
      FastLED.clear();
    }
  } else {
    FastLED.clear();
  }
  GreenTree(tree_h);
  if (treemove == 1)  deltaValue++; //если чекбокс включен, елки будут двигаться
  VirtualSnow(snowspeed);
  if (deltaValue >= 8) deltaValue = 0;
  step++;
}

// =====================================
//               Фейерверк
//                Firework
//             © SlingMaster
// =====================================

void gradientVertical(uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY, uint8_t start_color, uint8_t end_color, uint8_t start_br, uint8_t end_br, uint8_t saturate) {
  float step_color = 0;
  float step_br = 0;
  if (startX == endX) {
    endX++;
  }
  if (startY == endY) {
    endY++;
  }
  step_color = (end_color - start_color) / abs(startY - endY);
  if (start_color >  end_color) {
    step_color -= 1.2;
  } else {
    step_color += 1.2;
  }
  step_br = (end_br - start_br) / abs(startY - endY);
  if (start_br >  end_color) {
    step_br -= 1.2;
  } else {
    step_br += 1.2;
  }
  for (uint8_t y = startY; y < endY; y++) {
    CHSV thisColor = CHSV( (uint8_t) validMinMax((start_color + (y - startY) * step_color), 0, 255), saturate,
                           (uint8_t) validMinMax((start_br + (y - startY) * step_br), 0, 255) );
    for (uint8_t x = startX; x < endX; x++) {
      drawPixelXY(x, y, thisColor);
    }
  }
}

void gradientDownTop( uint8_t bottom, CHSV bottom_color, uint8_t top, CHSV top_color ) {
  //  FORWARD_HUES: hue always goes clockwise
  //  BACKWARD_HUES: hue always goes counter-clockwise
  //  SHORTEST_HUES: hue goes whichever way is shortest
  //  LONGEST_HUES: hue goes whichever way is longest
  if (STRIP_DIRECTION < 2) {
    // STRIP_DIRECTION to UP ========
    fill_gradient(leds, top * pWIDTH, top_color, bottom * pWIDTH, bottom_color, SHORTEST_HUES);
  } else {
    // STRIP_DIRECTION to DOWN ======
    fill_gradient(leds, NUM_LEDS - bottom * pWIDTH - 1, bottom_color, NUM_LEDS - top * pWIDTH, top_color, SHORTEST_HUES);
  }
}

void VirtualExplosion(uint8_t f_type, int8_t timeline) {
  const uint8_t DELAY_SECOND_EXPLOSION = pHEIGHT * 0.25;
  uint8_t horizont = 1U;
  const int8_t STEP = 255 / pHEIGHT;
  uint8_t firstColor = random8(255);
  uint8_t secondColor = 0;
  uint8_t saturation = 255U;
  switch (f_type) {
    case 0:
      secondColor =  random(50U, 255U);
      saturation = random(245U, 255U);
      break;
    case 1: /* сакура */
      firstColor = random(210U, 230U);
      secondColor = random(65U, 85U);
      saturation = 255U;
      break;
    case 2: /* день Независимости */
      firstColor = random(160U, 170U);
      secondColor = random(25U, 50U);
      saturation = 255U;
      break;
    default: /* фризантемы */
      firstColor = random(30U, 40U);
      secondColor = random(25U, 50U);
      saturation = random(128U, 255U);
      break;
  }
  if ((timeline > pHEIGHT - 1 ) & (timeline < pHEIGHT * 1.75)) {
    for (uint8_t x = 0U; x < pWIDTH; x++) {
      for (uint8_t y =  horizont; y < pHEIGHT - 1; y++) {
        noise_3d[0][x][y] = noise_3d[0][x][y + 1];
        uint8_t bri = y * STEP;
        if (noise_3d[0][x][y] > 0) {
          if (timeline > (pHEIGHT + DELAY_SECOND_EXPLOSION) ) {
            /* second explosion */
            drawPixelXY((x - 2 + random8(4)), y - 1, CHSV(secondColor + random8(16), saturation, bri));
          }
          if (timeline < ((pHEIGHT - DELAY_SECOND_EXPLOSION) * 1.75) ) {
            /* first explosion */
            drawPixelXY(x, y, CHSV(firstColor, 255U, bri));
          }
        } else {
        }
      }
    }
    uint8_t posX = random8(pWIDTH);
    for (uint8_t x = 0U; x < pWIDTH; x++) {
      // заполняем случайно верхнюю строку
      if (posX == x) {
        if (step % 2 == 0) {
          noise_3d[0][x][pHEIGHT - 1U] = 1;
        } else {
          noise_3d[0][x][pHEIGHT - 1U]  = 0;
        }
      } else {
        noise_3d[0][x][pHEIGHT - 1U]  = 0;
      }
    }
  }
}

// --------------------------------------
uint8_t sizeH;
void Firework() {
  const uint8_t MAX_BRIGHTNESS = 40U;            // sky brightness
  const uint8_t DELTA = 1U;                      // центровка по вертикали
  const uint8_t STEP = 3U;
  const uint8_t skyColor = 156U;

  if (FPSdelay > 128U) {
    /* вечерело */
    FPSdelay--;
    sizeH = (FPSdelay - 128U) * stepH;
    dimAll(200);
    if (STRIP_DIRECTION % 2 == 0) {
      gradientDownTop( 0, CHSV(skyColor, 255U, floor(FPSdelay / 2.2)), sizeH, CHSV(skyColor, 255U, 2U));
    } else {
      gradientVertical(0, 0, pWIDTH, sizeH, skyColor, skyColor, floor(FPSdelay / 2.2), 2U, 255U);
    }
    if (sizeH > HORIZONT) return;
    if (sizeH == HORIZONT )  FPSdelay = FPS_DELAY;
  }
  if (step > DOT_EXPLOSION ) {
    blurScreen(beatsin8(3, 64, 80));
  }
  if (step == DOT_EXPLOSION - 1) {
    /* включаем фазу затухания */
    FPSdelay = 70;
  }
  if (step > CENTER_Y_MAJOR) {
    dimAll(140);
  } else {
    dimAll(100);
  }

  /* ============ draw sky =========== */
  if (modes[currentMode].Speed < 180U) {
    if (STRIP_DIRECTION % 2 == 0) {
      gradientDownTop( 0, CHSV(skyColor, 255U, hue ), HORIZONT, CHSV(skyColor, 255U, 0U ));
    } else {
      gradientVertical(0, 0, pWIDTH, HORIZONT, skyColor, skyColor, hue + 1, 0U, 255U);
    }
  }
  VirtualExplosion(deltaHue2, step);
  if ((step > DOT_EXPLOSION ) & (step < pHEIGHT * 1.5)) {
    /* фаза взрыва */
    FPSdelay += 5U;
  }
  const uint8_t rows = (pHEIGHT + 1) / 3U;
  deltaHue = floor(modes[currentMode].Speed / 64) * 64;
  if (step > CENTER_Y_MAJOR) {
    bool dir = false;
    for (uint8_t y = 0; y < rows; y++) {
      /* сдвигаем слои / эмитация разлета */
      for (uint8_t x = 0U ; x < pWIDTH; x++) {
        if (dir) {  // <==
          drawPixelXY(x - 1, y * 3 + DELTA, getPixColorXY(x, y * 3 + DELTA));
        } else {    // ==>
          drawPixelXY(pWIDTH - x, y * 3 + DELTA, getPixColorXY(pWIDTH - x - 1, y * 3 + DELTA));
        }
      }
      dir = !dir;
    }
  }

  /* ========== фаза полета ========== */
  if (step < DOT_EXPLOSION ) {
    FPSdelay ++;
    if (pHEIGHT < 20) {
      FPSdelay ++;
    }
    /* закоментируйте следующие две строки если плоская лампа
      подсветка заднего фона */
#if (DEVICE_TYPE == 0)
    {
      DrawLine(0U, 0U, 0U, pHEIGHT - step, CHSV(skyColor, 255U, 32U));
      DrawLine(pWIDTH - 1, 0U, pWIDTH - 1U, pHEIGHT - step, CHSV(skyColor, 255U, 32U));
    }
#endif
    /* ------------------------------------------------------ */
    uint8_t saturation = (step > (DOT_EXPLOSION - 2U)) ? 192U : 20U;
    drawPixelXY(CENTER_X_MINOR + deltaHue2, step,  CHSV(50U, saturation, 80U));                 // first
    drawPixelXY(CENTER_X_MAJOR - deltaHue2, step - HORIZONT,  CHSV(50U, saturation, 80U));  // second
    /* sky brightness */
    if (hue > 2U) {
      hue -= 1U;
    }
  }
  if (step > pHEIGHT * 1.25) {
    /* sky brightness */
    if (hue < MAX_BRIGHTNESS) {
      hue += 2U;
    }
  }
  if (step >= (pHEIGHT * 2.5)) {
    step = 0U;
    FPSdelay = FPS_DELAY;
    if (modes[currentMode].Scale <= 1) {
      deltaHue2++;
    }
    if (deltaHue2 >= 4U) deltaHue2 = 0U;  // next Firework type
  }
  step ++;
}

// ================================
//Салют
//Функция построения окружности на цилиндре
void DrawCircleCylinder(int xc, int yc, int r, CRGB color)
{
  int x = 0;
  int y = r;
  int d = 3 - 2 * y;;
  while (x <= y)
  {
    sim(x, y, xc, yc, color);
    if (d < 0) d = d + 4 * x + 6;
    else d = d + 4 * (x - y--) + 10;
    x++;
  }
  if (x == y) sim(x, y, xc, yc, color);
}

//Функция точки на цилиндре
void drawPixelXYCylinder(int x, int y, CRGB color)
{
  if (y >= 0 && y < pHEIGHT) {
    while (x < 0) x += pWIDTH;
    drawPixelXY(x % pWIDTH, y, color);
  }
}

void pixel4(int x, int y, int xc, int yc, CRGB color)
{
  drawPixelXYCylinder(x + xc, y + yc, color);
  drawPixelXYCylinder(x + xc, -y + yc, color);
  drawPixelXYCylinder(-x + xc, -y + yc, color);
  drawPixelXYCylinder(-x + xc, y + yc, color);
}
//8 точек Требуется для окружности
void sim(int x, int y, int xc, int yc, CRGB color)
{
  pixel4(x, y, xc, yc, color);
  pixel4(y, x, xc, yc, color);
}

uint8_t SaluteStep;
void SaluteRoutine()
{
  switch (SaluteStep) {
    case 0: SaluteStart();          break;
    case 1: SaluteDrawLine();       break;
    case 2: SaluteExplosion();      break;
    case 3: SaluteDecay();          break;
  }
  SaluteFadeAll(32);
}

uint8_t SaluteY2;
int SaluteX, SaluteY;
CRGB SaluteColor;
uint8_t SaluteR;

void SaluteStart()
{
  SaluteStep = 1;
  SaluteColor = CHSV(random(0, 9) * 28, 255U, 255U);
  SaluteY = 0;
  SaluteY2 = random(pHEIGHT * 2 / 3, pHEIGHT);
  SaluteX = random(0, pWIDTH);;
}

void SaluteFadeAll(uint8_t val)
{
  for (uint8_t x = 0; x < pWIDTH; x++)
    for (uint8_t y = 0; y < pHEIGHT; y++)
      leds[getPixelNumber(x, y)] -= CHSV(0, 0, val);
}

void SaluteDrawLine()
{
  if (SaluteY < SaluteY2)
  {
    drawPixelXY(SaluteX, SaluteY, SaluteColor);
    SaluteY++;
  } else {
    SaluteStep = 2;
    SaluteColor = CHSV(random(0, 9) * 28, 128U, 255U);
    SaluteR = 1;
  }
}

void SaluteExplosion()
{
  if (SaluteR < map(modes[currentMode].Scale, 1, 100, 2, pWIDTH * 0.7))
  {
    // Окружность на цилиндре
#if (DEVICE_TYPE == 0)
    DrawCircleCylinder(SaluteX, SaluteY, SaluteR, SaluteColor - CHSV(0, 0, SaluteR * 64) + CHSV(0, SaluteR * 32, 0)); //Использовать для цилиндрических ламп
#endif
    // Окружность на развертке
#if (DEVICE_TYPE == 1)
    drawCircle(SaluteX, SaluteY, SaluteR, SaluteColor - CHSV(0, 0, SaluteR * 64) + CHSV(0, SaluteR * 32, 0)); //Использовать для плоских матриц
#endif
    SaluteR++;
  } else {
    // Окружность на цилиндре
#if (DEVICE_TYPE == 0)
    DrawCircleCylinder(SaluteX, SaluteY, SaluteR, SaluteColor - CHSV(0, 0, SaluteR * 64 + 128) + CHSV(0, SaluteR * 32 + 128, 0)); //Использовать для цилиндрических ламп
#endif
    // Окружность на развертке
#if (DEVICE_TYPE == 1)
    drawCircle(SaluteX, SaluteY, SaluteR, SaluteColor - CHSV(0, 0, SaluteR * 64) + CHSV(0, SaluteR * 32, 0)); //Использовать для плоских матриц
#endif
    SaluteStep = 3;
  }
}

void SaluteDecay()
{
  if (SaluteR > 0) SaluteR--;
  else SaluteStep = 0;
}


//---------- Эффект "Фейерверк" Салют ---
//адаптация и переписал - kostyamat
//https://gist.github.com/jasoncoon/0cccc5ba7ab108c0a373
//https://github.com/marcmerlin/FastLED_NeoMatrix_SmartMatrix_LEDMatrix_GFX_Demos/blob/master/FastLED/FireWorks2/FireWorks2.ino

uint16_t launchcountdown[SPARK];
Dot gDot[SPARK];

void sparkGen() {
  for (byte c = 0; c < enlargedObjectNUM; c++) { // modes[currentMode].Scale / хз
    if ( gDot[c].show == 0 ) {
      if ( launchcountdown[c] == 0) {
        gDot[c].GroundLaunch();
        gDot[c].theType = 1;
        launchcountdown[c] = random16(1200 - modes[currentMode].Speed * 4) + 1;
      } else {
        launchcountdown[c] --;
      }
    }
    if ( store[c].gSkyburst) {
      store[c].gBurstcolor = CHSV(random8(), 200, 100);
      store[c].gSkyburst = false;
      byte nsparks = random8( num_sparks / 2, num_sparks + 1);
      for ( byte b = 0; b < nsparks; b++) {
        gSparks[b].Skyburst( store[c].gBurstx, store[c].gBursty, store[c].gBurstyv, store[c].gBurstcolor, pcnt);
      }
    }
  }
}
void fireworksRoutine()  {
  pcnt = beatsin8(100, 20, 100);
  if (hue++ % 10 == 0U) {
    deltaValue = random8(25, 50);
  }
  fadeToBlackBy(leds, NUM_LEDS, deltaValue);
  sparkGen();
  for (byte a = 0; a < enlargedObjectNUM; a++) { //modes[currentMode].Scale / хз
    gDot[a].Move(a, false);//flashing);
    gDot[a].Draw();
  }
  for ( byte b = 0; b < num_sparks; b++) {
    gSparks[b].Move(0, false);//flashing);
    gSparks[b].Draw();
  }
}

uint8_t salute_type = 0;
void salute() {
  if (loadingFlag) {
    loadingFlag = false;
    salute_type = (specialTextEffectParam >= 0) ? specialTextEffectParam : getEffectScaleParamValue2(MC_FIREWORKS);
    // Если авто - генерировать один из типов
    if (salute_type == 0 || salute_type > 3)
      salute_type = random8(1, 4);
    if (salute_type == 1)
      SaluteStep = 0;
    if (salute_type == 2)  {
      enlargedObjectNUM = (modes[currentMode].Scale - 1U) / 99.0 * (SPARK - 1U) + 1U;
      if (enlargedObjectNUM > SPARK) enlargedObjectNUM = SPARK;
      for (byte c = 0; c < SPARK; c++)
        launchcountdown[c] = 0;
    }
    if (salute_type == 3)  {
      deltaHue2 = 0;
      FPSdelay = 255U;
      FastLED.clear();
      step = 0U;
      deltaHue2 = floor(modes[currentMode].Scale / 26);
      hue = 48U;            // skyBright
      if (modes[currentMode].Speed > 85U) {
        sizeH = HORIZONT;
        FPSdelay = FPS_DELAY;
      }
    }
  }
  switch (salute_type) {
    case 1: SaluteRoutine(); break;
    case 2: fireworksRoutine(); break;
    default: Firework(); break;
  }
}

//===========================================
//объединяем акварель и масляные краски
//===========================================
uint8_t paint_type;
uint8_t wave;
uint16_t max_val;
void paint() {
  if (loadingFlag)
  {
    loadingFlag = false;
    paint_type = (specialTextEffectParam >= 0) ? specialTextEffectParam : getEffectScaleParamValue2(MC_PAINT);
    // Если авто - генерировать один из типов
    if (paint_type == 0 || paint_type > 3) {
      paint_type = random8(1, 4);
    }
    if (paint_type == 1 || 2)
    {
      FastLED.clear();
      deltaValue = 255U - modes[currentMode].Speed + 1U;
      step = deltaValue;                    // чтообы при старте эффекта сразу покрасить лампу
      hue = floor(21.25 * (random8(11) + 1)); // next color
      deltaHue = hue - 22;                  // last color
      deltaHue2 = 80;                       // min bright
      wave = floor(pHEIGHT * 0.5);           // position swap color
      max_val = pow(2, pWIDTH);
    }
    if (paint_type == 3)
    {
      FastLED.clear();
      deltaValue = 255U - modes[currentMode].Speed + 1U;
      step = deltaValue;                    // чтообы при старте эффекта сразу покрасить лампу
      hue = 0;
      deltaHue = 255;                       // last color
      trackingObjectHue[1] = floor(pWIDTH * 0.25);
      trackingObjectHue[3] = floor(pHEIGHT * 0.25);
    }
  }
  switch (paint_type) {
    case 1:  OilPaints(false); break;
    case 2:  OilPaints(true);  break;
    default: Watercolor();     break;
  }
}

// ============ Watercolor ==============
//      © SlingMaster | by Alex Dovby
//            EFF_WATERCOLOR
//               Акварель
//---------------------------------------
void SmearPaint(uint8_t obj[trackingOBJECT_MAX_COUNT]) {
  uint8_t divide;
  int temp;
  static const uint32_t colors[6][8] PROGMEM = {
    {0x2F0000,  0xFF4040, 0x6F0000, 0xAF0000, 0xff5f00, CRGB::Red, 0x480000, 0xFF0030},
    {0x002F00, CRGB::LawnGreen, 0x006F00, 0x00AF00, CRGB::DarkMagenta, 0x00FF00, 0x004800, 0x00FF30},
    {0x002F1F, CRGB::DarkCyan, 0x00FF7F, 0x007FFF, 0x20FF5F, CRGB::Cyan, 0x004848, 0x7FCFCF },
    {0x00002F, 0x5030FF, 0x00006F, 0x0000AF, CRGB::DarkCyan, 0x0000FF, 0x000048, 0x5F5FFF},
    {0x2F002F, 0xFF4040, 0x6F004A, 0xFF0030, CRGB::DarkMagenta, CRGB::Magenta, 0x480048, 0x3F00FF},
    {CRGB::Blue, CRGB::Red, CRGB::Gold, CRGB::Green, CRGB::DarkCyan, CRGB::DarkMagenta, 0x000000, 0xFF7F00 }
  };
  if (trackingObjectHue[5] == 1) {  // direction >>>
    obj[1]++;
    if (obj[1] >= obj[2]) {
      trackingObjectHue[5] = 0;     // swap direction
      obj[3]--;                     // new line
      if (step % 2 == 0) {
        obj[1]++;
      } else {
        obj[1]--;
      }
      obj[0]--;
    }
  } else {                          // direction <<<
    obj[1]--;
    if (obj[1] <= (obj[2] - obj[0])) {
      trackingObjectHue[5] = 1;     // swap direction
      obj[3]--;                     // new line
      if (obj[0] >= 1) {
        temp = obj[0] - 1;
        if (temp < 0) {
          temp = 0;
        }
        obj[0] = temp;
        obj[1]++;
      }
    }
  }
  if (obj[3] == 255) {
    deltaHue = 255;
  }
  divide = floor((modes[currentMode].Scale - 1) / 16.7);
  if ( (obj[1] >= pWIDTH) || (obj[3] == obj[4]) ) {
    deltaHue = 255;
  }
  drawPixelXY(obj[1], obj[3], colors[divide][hue]);
}

void Watercolor() {
  uint8_t divide;
  if (loadingFlag) {
    loadingFlag = false;
    FastLED.clear();
    deltaValue = 255U - modes[currentMode].Speed + 1U;
    step = deltaValue;                    // чтообы при старте эффекта сразу покрасить лампу
    hue = 0;
    deltaHue = 255;                       // last color
    trackingObjectHue[1] = floor(pWIDTH * 0.25);
    trackingObjectHue[3] = floor(pHEIGHT * 0.25);
  }
  if (step >= deltaValue) {
    step = 0U;
  }

  // ******************************
  // set random parameter for smear
  // ******************************
  if (deltaHue == 255) {
    trackingObjectHue[0] = 4 + random8(floor(pWIDTH * 0.25));                // width
    trackingObjectHue[1] = random8(pWIDTH - trackingObjectHue[0]);           // x
    int temp =  trackingObjectHue[1] + trackingObjectHue[0];
    if (temp >= (pWIDTH - 1)) {
      temp = pWIDTH - 1;
      if (trackingObjectHue[1] > 1) {
        trackingObjectHue[1]--;
      } else {
        trackingObjectHue[1]++;
      }
    }
    trackingObjectHue[2] = temp;                                            // x end
    trackingObjectHue[3] = 3 + random8(pHEIGHT - 4);                         // y
    temp = trackingObjectHue[3] - random8(3) - 3;
    if (temp <= 0) {
      temp = 0;
    }
    trackingObjectHue[4] = temp;                                            // y end
    trackingObjectHue[5] = 1;
    divide = floor((modes[currentMode].Scale - 1) / 16.7);                 // маштаб задает смену палитры
    hue = random8(8);
    hue2 = 255;
    deltaHue = 0;
  }
  SmearPaint(trackingObjectHue);
  if (step % 2 == 0) {
    blurScreen(beatsin8(1U, 1U, 6U));
  }
  step++;
}

// ============ Oil Paints ==============
//      © SlingMaster | by Alex Dovby
//              EFF_PAINT
//           Масляные Краски
//---------------------------------------
void OilPaints(bool custeff) {
  uint8_t divide;
  uint8_t entry_point;
  uint16_t value;
  //"побочный эффект", вызывается как OilPaints(true)
  if (custeff == true) {
    if (step % wave - 1 == 0) {
      drawPixelXY(random8(pWIDTH), 1U + random8(4), CHSV(hue + 180, 255U, 255U));
      drawPixelXY(random8(pWIDTH), 1U + random8(4), CHSV(hue + 90, 255U, 255U));
    }
    blurScreen(32U);
  }

  if (step >= deltaValue) {
    step = 0U;
  }

  // Create Oil Paints --------------
  // выбираем краски  ---------------
  if (step % wave == 0) {
    divide = floor((modes[currentMode].Scale - 1) / 10);             // маштаб задает диапазон изменения цвета
    deltaHue = hue;                                                   // set last color
    hue += 6 * divide;                                               // new color
    hue2 = 255;                                                       // restore brightness
    deltaHue2 = 80 - floor(log(modes[currentMode].Brightness) * 6);   // min bright
    entry_point = random8(pWIDTH);                                     // start X position
    trackingObjectHue[entry_point] = hue;                             // set start position
    drawPixelXY(entry_point,  pHEIGHT - 2, CHSV(hue, 255U, 255U));
    // !!! ********
    //    if (custeff == true) {
    drawPixelXY(entry_point + 1,  pHEIGHT - 3, CHSV(hue + 30, 255U, 255U));
    //    }
  }

  // формируем форму краски, плавно расширяя струю ----
  if (random8(3) == 1) {
    for (uint8_t x = 1U; x < pWIDTH; x++) {
      if (trackingObjectHue[x] == hue) {
        trackingObjectHue[x - 1] = hue;
        break;
      }
    }
  } else {
    for (uint8_t x = pWIDTH - 1; x > 0U ; x--) {
      if (trackingObjectHue[x] == hue) {
        trackingObjectHue[x + 1] = hue;
        break;
      }
    }
  }
  for (uint8_t x = 0U; x < pWIDTH; x++) {
    //                                                                                set color  next |    last  |
    drawPixelXY(x,  pHEIGHT - 1, CHSV(trackingObjectHue[x], 255U, (trackingObjectHue[x] == hue) ? hue2 : deltaHue2));
  }
  // уменьшаем яркость для следующих строк
  if ( hue2 > (deltaHue2 + 16)) {
    hue2 -= 16U;
  }
  // сдвигаем неравномерно поток вниз ---
  value = random16(max_val);
  //LOG.printf_P(PSTR("value = %06d | "), value);
  for (uint8_t x = 0U; x < pWIDTH; x++) {
    if ( bitRead(value, x ) == 0) {
      //LOG.print (" X");
      for (uint8_t y = 0U; y < pHEIGHT - 1; y++) {
        drawPixelXY(x, y, getPixColorXY(x, y + 1U));
      }
    }
  }
  step++;
}

//Короче, тут будут находиться все эффекты асо сменой палитр
//=================================================================
//                           NoiseEffects
//Объединяем все шумовые эффекты в один для экономии места в списке
//                      И добавляем еще палитры
//=================================================================
//эти переменные нужны для выбора палитр
CRGBPalette16 currentPalette( PartyColors_p );  
uint8_t palette_number;
uint32_t color_timer; //таймер для автоперебора
uint8_t colorLoop = 1;
uint8_t noiseefftype;
uint8_t startnum;
void noiseEffectsRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    noiseefftype = getEffectScaleParamValue2(MC_NOISE_EFFECTS);
    if (noiseefftype == 0 || noiseefftype == 60) startnum = random8(1, 60); //Если Случайный выбор или Авто, задать произвольный вариант (в Авто от него начинается отсчет)
    else if (noiseefftype > 0 || noiseefftype < 60) startnum = noiseefftype;  //Если что-то из вариантов 1-59, берем только это значение
    FastLED.clear();  // очистить
  }
  if (noiseefftype == 60) {  //автоперебор вариантов, если выбран вариант Авто
    if (millis() - color_timer > 10000) {
      color_timer = millis();
      startnum++;
      if (startnum > 59) startnum = 1;
    }
  }
  switch (startnum) {
    case 1: {
        createNoise();
        madnessNoise();
      }  break;
    case 2: {
        createNoise();
        currentPalette = CloudColors_p;
        colorLoop = 0;
        cloudNoise();
      }  break;
    case 3: {
        createNoise();
        currentPalette = LavaColors_p;
        colorLoop = 0;
        lavaNoise();
      }  break;
    case 4: {
        createNoise();
        currentPalette = PartyColors_p;
        colorLoop = 1;
        plasmaNoise();
      }  break;
    case 5: {
        createNoise();
        currentPalette = RainbowColors_p;
        colorLoop = 1;
        rainbowNoise();
      }  break;
    case 6: {
        createNoise();
        currentPalette = RainbowStripeColors_p;
        colorLoop = 1;
        rainbowStripeNoise();
      }  break;
    case 7: {
        // 'black out' all 16 palette entries...
        createNoise();
        fill_solid( currentPalette, 16, CRGB::Black);
        // and set every fourth one to white.
        currentPalette[0] = CRGB::White;
        currentPalette[4] = CRGB::White;
        currentPalette[8] = CRGB::White;
        currentPalette[12] = CRGB::White;
        colorLoop = 1;
        zebraNoise();
      }  break;
    case 8: {
        createNoise();
        currentPalette = ForestColors_p;
        colorLoop = 0;
        forestNoise();
      }  break;
    case 9: {
        createNoise();
        currentPalette = AlcoholFireColors_p;
        colorLoop = 0;
        alcoholFireNoise();
      }  break;
    case 10: {
        createNoise();
        currentPalette = HeatColors_p;
        colorLoop = 0;
        heatColorsNoise();
      }  break;
    case 11: {
        createNoise();
        currentPalette = WaterfallColors4in1_p;
        colorLoop = 0;
        waterfallColors4in1Noise();
      }  break;
    case 12: {
        createNoise();
        currentPalette = WoodFireColors_p;
        colorLoop = 0;
        woodFireColorsNoise();
      }  break;
    case 13: {
        createNoise();
        currentPalette = NormalFire_p;
        colorLoop = 0;
        normalFireNoise();
      }  break;
    case 14: {
        createNoise();
        currentPalette = NormalFire2_p;
        colorLoop = 0;
        normalFire2Noise();
      }  break;
    case 15: {
        createNoise();
        currentPalette = LithiumFireColors_p;
        colorLoop = 0;
        lithiumFireColorsNoise();
      }  break;
    case 16: {
        createNoise();
        currentPalette = SodiumFireColors_p;
        colorLoop = 0;
        sodiumFireColorsNoise();
      }  break;
    case 17: {
        createNoise();
        currentPalette = CopperFireColors_p;
        colorLoop = 0;
        copperFireColorsNoise();
      }  break;
    case 18: {
        createNoise();
        currentPalette = RubidiumFireColors_p;
        colorLoop = 0;
        rubidiumFireColorsNoise();
      }  break;
    case 19: {
        createNoise();
        currentPalette = PotassiumFireColors_p;
        colorLoop = 0;
        potassiumFireColorsNoise();
      }  break;
    case 20: {
        createNoise();
        currentPalette = OceanColors_p;
        colorLoop = 0;
        oceanNoise();
      }  break;
    case 21: {
        createNoise();
        currentPalette = Sunset_Real_gp;
        colorLoop = 0;
        SunsetRealNoise();
      }  break;
    case 22: {
        createNoise();
        currentPalette = dkbluered_gp;
        colorLoop = 0;
        dkblueredNoise();
      }  break;
    case 23: {
        createNoise();
        currentPalette = Optimus_Prime_gp;
        colorLoop = 0;
        OptimusPrimeNoise();
      }  break;
    case 24: {
        createNoise();
        currentPalette = warmGrad_gp;
        colorLoop = 0;
        warmGradNoise();
      }  break;
    case 25: {
        createNoise();
        currentPalette = coldGrad_gp;
        colorLoop = 0;
        coldGradNoise();
      }  break;
    case 26: {
        createNoise();
        currentPalette = hotGrad_gp;
        colorLoop = 0;
        hotGradNoise();
      }  break;
    case 27: {
        createNoise();
        currentPalette = pinkGrad_gp;
        colorLoop = 0;
        pinkGradNoise();
      }  break;
    case 28: {
        createNoise();
        currentPalette = comfy_gp;
        colorLoop = 0;
        comfyNoise();
      }  break;
    case 29: {
        createNoise();
        currentPalette = cyperpunk_gp;
        colorLoop = 0;
        cyperpunkNoise();
      }  break;
    case 30: {
        createNoise();
        currentPalette = girl_gp;
        colorLoop = 0;
        girlNoise();
      }  break;
    case 31: {
        createNoise();
        currentPalette = xmas_gp;
        colorLoop = 0;
        xmasNoise();
      }  break;
    case 32: {
        createNoise();
        currentPalette = acid_gp;
        colorLoop = 0;
        acidNoise();
      }  break;
    case 33: {
        createNoise();
        currentPalette = blueSmoke_gp;
        colorLoop = 0;
        blueSmokeNoise();
      }  break;
    case 34: {
        createNoise();
        currentPalette = gummy_gp;
        colorLoop = 0;
        gummyNoise();
      }  break;
    case 35: {
        createNoise();
        currentPalette = leo_gp;
        colorLoop = 0;
        leoNoise();
      }  break;
    case 36: {
        createNoise();
        currentPalette = aurora_gp;
        colorLoop = 0;
        auroraNoise();
      }  break;
    case 37: {
        createNoise();
        currentPalette = rainClouds_p;
        colorLoop = 0;
        rainCloudsNoise();
      }  break;
    case 38: {
        createNoise();
        currentPalette = redwhite_gp;
        colorLoop = 0;
        redwhiteNoise();
      }  break;
    case 39: {
        createNoise();
        currentPalette = ib_jul01_gp;
        colorLoop = 0;
        ib_jul01Noise();
      }  break;
    case 40: {
        createNoise();
        currentPalette = rgi_15_gp;
        colorLoop = 0;
        rgi_15Noise();
      }  break;
    case 41: {
        createNoise();
        currentPalette = retro2_16_gp;
        colorLoop = 0;
        retro2_16Noise();
      }  break;
    case 42: {
        createNoise();
        currentPalette = Analogous_1_gp;
        colorLoop = 0;
        Analogous_1Noise();
      }  break;
    case 43: {
        createNoise();
        currentPalette = pinksplash_08_gp;
        colorLoop = 0;
        pinksplash_08Noise();
      }  break;
    case 44: {
        createNoise();
        currentPalette = pinksplash_07_gp;
        colorLoop = 0;
        pinksplash_07Noise();
      }  break;
    case 45: {
        createNoise();
        currentPalette = Coral_reef_gp;
        colorLoop = 0;
        Coral_reefNoise();
      }  break;
    case 46: {
        createNoise();
        currentPalette = ocean_breeze_gp;
        colorLoop = 0;
        ocean_breezeNoise();
      }  break;
    case 47: {
        createNoise();
        currentPalette = landscape_64_gp;
        colorLoop = 0;
        landscape_64Noise();
      }  break;
    case 48: {
        createNoise();
        currentPalette = landscape_33_gp;
        colorLoop = 0;
        landscape_33Noise();
      }  break;
    case 49: {
        createNoise();
        currentPalette = rainbowsherbet_gp;
        colorLoop = 0;
        rainbowsherbetNoise();
      }  break;
    case 50: {
        createNoise();
        currentPalette = gr65_hult_gp;
        colorLoop = 0;
        gr65_hultNoise();
      }  break;
    case 51: {
        createNoise();
        currentPalette = GMT_drywet_gp;
        colorLoop = 0;
        GMT_drywetNoise();
      }  break;
    case 52: {
        createNoise();
        currentPalette = emerald_dragon_gp;
        colorLoop = 0;
        emerald_dragonNoise();
      }  break;
    case 53: {
        createNoise();
        currentPalette = Colorfull_gp;
        colorLoop = 0;
        ColorfullNoise();
      }  break;
    case 54: {
        createNoise();
        currentPalette = Pink_Purple_gp;
        colorLoop = 0;
        Pink_PurpleNoise();
      }  break;
    case 55: {
        createNoise();
        currentPalette = autumn_19_gp;
        colorLoop = 0;
        autumn_19Noise();
      }  break;
    case 56: {
        createNoise();
        currentPalette = daybreak_gp;
        colorLoop = 0;
        daybreakNoise();
      }  break;
    case 57: {
        createNoise();
        currentPalette = Blue_Cyan_Yellow_gp;
        colorLoop = 0;
        Blue_Cyan_YellowNoise();
      }  break;
    case 58: {
        createNoise();
        currentPalette = bhw1_28_gp;
        colorLoop = 0;
        bhw1_28Noise();
      }  break;
    case 59: {
        createNoise();
        currentPalette = rbw_gp;
        colorLoop = 0;
        rbwNoise();
      }  break;
  }
}

// ============ Colored Python ============
//      base code WavingCell from © Stepko
//       Adaptation & modefed © alvikskor
// --------------------------------------
uint8_t thickness;
void Colored_Python() {
  uint8_t efspeed = getEffectSpeedValue(MC_PYTHON);
  if (loadingFlag) {
    loadingFlag = false;
    thickness = map8(getEffectScaleParamValue(MC_PYTHON), 1, 2 * pWIDTH); //ползунок толщины 1-2*pWIDTH
    palette_number = getEffectScaleParamValue2(MC_PYTHON);
    if (palette_number == 0 || palette_number == 61) startnum = random8(1, 61); //Если Случайный выбор или Авто, задать произвольный вариант (в Авто от него начинается отсчет)
    else if (palette_number > 0 || palette_number < 61) startnum = palette_number;  //Если что-то из вариантов 1-60, берем только это значение
  }
  uint16_t  t = millis() / (128 - (efspeed / 2));
  if (palette_number == 61) {  //автоперебор вариантов, если выбран вариант Авто, дается 10 сек на эффект
    if (millis() - color_timer > 10000) {
      color_timer = millis();
      startnum++;
      if (startnum > 60) startnum = 1;
    }
  }
  switch (startnum) {
    case 1:  currentPalette = CloudColors_p;         break;
    case 2:  currentPalette = LavaColors_p;          break;
    case 3:  currentPalette = PartyColors_p;         break;
    case 4:  currentPalette = RainbowColors_p;       break;
    case 5:  currentPalette = RainbowStripeColors_p; break;
    case 6:  currentPalette = ForestColors_p;        break;
    case 7:  currentPalette = AlcoholFireColors_p;   break;
    case 8:  currentPalette = HeatColors_p;          break;
    case 9:  currentPalette = WaterfallColors4in1_p; break;
    case 10: currentPalette = WoodFireColors_p;      break;
    case 11: currentPalette = NormalFire_p;          break;
    case 12: currentPalette = NormalFire2_p;         break;
    case 13: currentPalette = LithiumFireColors_p;   break;
    case 14: currentPalette = SodiumFireColors_p;    break;
    case 15: currentPalette = CopperFireColors_p;    break;
    case 16: currentPalette = RubidiumFireColors_p;  break;
    case 17: currentPalette = PotassiumFireColors_p; break;
    case 18: currentPalette = OceanColors_p;         break;
    case 19: currentPalette = Sunset_Real_gp;        break;
    case 20: currentPalette = dkbluered_gp;          break;
    case 21: currentPalette = Optimus_Prime_gp;      break;
    case 22: currentPalette = warmGrad_gp;           break;
    case 23: currentPalette = coldGrad_gp;           break;
    case 24: currentPalette = hotGrad_gp;            break;
    case 25: currentPalette = pinkGrad_gp;           break;
    case 26: currentPalette = comfy_gp;              break;
    case 27: currentPalette = cyperpunk_gp;          break;
    case 28: currentPalette = girl_gp;               break;
    case 29: currentPalette = xmas_gp;               break;
    case 30: currentPalette = acid_gp;               break;
    case 31: currentPalette = blueSmoke_gp;          break;
    case 32: currentPalette = gummy_gp;              break;
    case 33: currentPalette = leo_gp;                break;
    case 34: currentPalette = aurora_gp;             break;
    case 35: currentPalette = rainClouds_p;          break;
    case 36: currentPalette = pacifica_palette_1;    break;
    case 37: currentPalette = pacifica_palette_2;    break;
    case 38: currentPalette = pacifica_palette_3;    break;
    case 39: currentPalette = redwhite_gp;           break;
    case 40: currentPalette = ib_jul01_gp;           break;
    case 41: currentPalette = rgi_15_gp;             break;
    case 42: currentPalette = retro2_16_gp;          break;
    case 43: currentPalette = Analogous_1_gp;        break;
    case 44: currentPalette = pinksplash_08_gp;      break;
    case 45: currentPalette = pinksplash_07_gp;      break;
    case 46: currentPalette = Coral_reef_gp;         break;
    case 47: currentPalette = ocean_breeze_gp;       break;
    case 48: currentPalette = landscape_64_gp;       break;
    case 49: currentPalette = landscape_33_gp;       break;
    case 50: currentPalette = rainbowsherbet_gp;     break;
    case 51: currentPalette = gr65_hult_gp;          break;
    case 52: currentPalette = GMT_drywet_gp;         break;
    case 53: currentPalette = emerald_dragon_gp;     break;
    case 54: currentPalette = Colorfull_gp;          break;
    case 55: currentPalette = Pink_Purple_gp;        break;
    case 56: currentPalette = autumn_19_gp;          break;
    case 57: currentPalette = daybreak_gp;           break;
    case 58: currentPalette = Blue_Cyan_Yellow_gp;   break;
    case 59: currentPalette = bhw1_28_gp;            break;
    case 60: currentPalette = rbw_gp;                break;
  }
  for (byte x = 0; x < pWIDTH; x++) {
    for (byte y = 0; y < pHEIGHT; y++) {
      leds[XY(x, y)] = ColorFromPalette(currentPalette, ((sin8((x * thickness) + sin8(y * 5 + t * 5)) + cos8(y * 10)) + 1) + t * (modes[currentMode].Speed % 10)); //HeatColors_p -палитра, t*scale/10 -меняет скорость движения вверх, sin8(x*20) -меняет ширину рисунка
    }
  }
}

// ****************************** СИНУСЫ *****************************
uint8_t prizmata_type = 0;
uint8_t direct = 0;
void prizmataRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    prizmata_type = (specialTextEffectParam >= 0) ? specialTextEffectParam : getEffectScaleParamValue2(MC_PRIZMATA);
    if (prizmata_type == 0 || prizmata_type > 2) prizmata_type = random8(1, 3); // Если Случайный выбор - генерировать один из типов - 1 вариант, 2 вариант
    palette_number = map8(getEffectScaleParamValue(MC_PRIZMATA), 0, 56); //выбор палитры от 1 до 55; если 0 - случайный выбор, если 56 - запускаем автоперебор
    if (palette_number == 0 || palette_number == 56) startnum = random8(1, 56); //Если случайный выбор или автоперебор, задать произвольный вариант (в авто от него начинается отсчет)
    else if (palette_number > 0 || palette_number < 56) startnum = palette_number;  //Если что-то из вариантов 1-55, берем только это значение
    if (pWIDTH > pHEIGHT)   //если матрица широкая - врубаем эффект горизонтально
      direct = 0;
    if (pWIDTH < pHEIGHT)   //если матрица высокая - врубаем эффект вертикально
      direct = 1;
    if (pWIDTH == pHEIGHT)  //если матрица квадратная - на все воля Великого Рандома, эффект может запуститься и так, и так
      direct = random8(2);
  }
  if (palette_number == 56) {  //автоперебор вариантов, если выбран вариант Авто, дается 10 сек на эффект
    if (millis() - color_timer > 10000) {
      color_timer = millis();
      startnum++;
      if (startnum > 55) startnum = 1;
    }
  }
  switch (startnum) {
    case 1:  currentPalette = CloudColors_p;         break;
    case 2:  currentPalette = LavaColors_p;          break;
    case 3:  currentPalette = PartyColors_p;         break;
    case 4:  currentPalette = RainbowColors_p;       break;
    case 5:  currentPalette = RainbowStripeColors_p; break;
    case 6:  currentPalette = ForestColors_p;        break;
    case 7:  currentPalette = AlcoholFireColors_p;   break;
    case 8:  currentPalette = HeatColors_p;          break;
    case 9:  currentPalette = WaterfallColors4in1_p; break;
    case 10: currentPalette = WoodFireColors_p;      break;
    case 11: currentPalette = NormalFire_p;          break;
    case 12: currentPalette = NormalFire2_p;         break;
    case 13: currentPalette = LithiumFireColors_p;   break;
    case 14: currentPalette = SodiumFireColors_p;    break;
    case 15: currentPalette = CopperFireColors_p;    break;
    case 16: currentPalette = RubidiumFireColors_p;  break;
    case 17: currentPalette = PotassiumFireColors_p; break;
    case 18: currentPalette = OceanColors_p;         break;
    case 19: currentPalette = Sunset_Real_gp;        break;
    case 20: currentPalette = dkbluered_gp;          break;
    case 21: currentPalette = Optimus_Prime_gp;      break;
    case 22: currentPalette = warmGrad_gp;           break;
    case 23: currentPalette = coldGrad_gp;           break;
    case 24: currentPalette = hotGrad_gp;            break;
    case 25: currentPalette = pinkGrad_gp;           break;
    case 26: currentPalette = comfy_gp;              break;
    case 27: currentPalette = cyperpunk_gp;          break;
    case 28: currentPalette = girl_gp;               break;
    case 29: currentPalette = xmas_gp;               break;
    case 30: currentPalette = acid_gp;               break;
    case 31: currentPalette = blueSmoke_gp;          break;
    case 32: currentPalette = gummy_gp;              break;
    case 33: currentPalette = aurora_gp;             break;
    case 34: currentPalette = redwhite_gp;           break;
    case 35: currentPalette = ib_jul01_gp;           break;
    case 36: currentPalette = rgi_15_gp;             break;
    case 37: currentPalette = retro2_16_gp;          break;
    case 38: currentPalette = Analogous_1_gp;        break;
    case 39: currentPalette = pinksplash_08_gp;      break;
    case 40: currentPalette = pinksplash_07_gp;      break;
    case 41: currentPalette = Coral_reef_gp;         break;
    case 42: currentPalette = ocean_breeze_gp;       break;
    case 43: currentPalette = landscape_64_gp;       break;
    case 44: currentPalette = landscape_33_gp;       break;
    case 45: currentPalette = rainbowsherbet_gp;     break;
    case 46: currentPalette = gr65_hult_gp;          break;
    case 47: currentPalette = GMT_drywet_gp;         break;
    case 48: currentPalette = emerald_dragon_gp;     break;
    case 49: currentPalette = Colorfull_gp;          break;
    case 50: currentPalette = Pink_Purple_gp;        break;
    case 51: currentPalette = autumn_19_gp;          break;
    case 52: currentPalette = daybreak_gp;           break;
    case 53: currentPalette = Blue_Cyan_Yellow_gp;   break;
    case 54: currentPalette = bhw1_28_gp;            break;
    case 55: currentPalette = rbw_gp;                break;
  }
  switch (prizmata_type) {
    case 1: prizmata(currentPalette);  break;
    case 2: prismata(currentPalette);  break;
  }
}

void prizmata(CRGBPalette16 pal) {
  delay(5);  // Если совсем задержки нет - матрица мерцает от постоянного обновления
  EVERY_N_MILLIS(33) {
    hue++;
  }
  FastLED.clear();
  // Отрисовка режима происходит на максимальной скорости. Знеачение effectSpeed влияет на параметр BPM функции beatsin8
  uint8_t spd = map8(255 - getEffectSpeedValue(MC_PRIZMATA), 12, 64);
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  if (direct == 0) {
    for (uint8_t x = 0; x < pWIDTH; x++) {
      uint8_t y = beatsin8(spd + x, 0, pHEIGHT - 1);
      drawPixelXY(x, y, ColorFromPalette(pal, x * 7 + hue, effectBrightness));
    }
  } else {
    for (uint8_t y = 0; y < pHEIGHT; y++) {
      uint8_t x = beatsin8(spd + y, 0, pWIDTH - 1);
      drawPixelXY(x, y, ColorFromPalette(pal, x * 7 + hue, effectBrightness));
    }
  }
}

void prismata(CRGBPalette16 pal) {
  uint8_t spd = map8(255 - getEffectSpeedValue(MC_PRIZMATA), 12, 64);
  uint8_t beat;
  uint8_t x;
  uint8_t y;
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  EVERY_N_MILLIS(33) {
    hue++;
  }
  blurScreen(20); // @Palpalych посоветовал делать размытие
  dimAll(255U - (modes[currentMode].Scale - 1U) % 11U * 3U);
  if (direct == 0) {   //добавили направление
    for (uint8_t x = 0; x < pWIDTH; x++) {
      beat = (GET_MILLIS() * (accum88(x + 1)) * spd / 1000); //и чуть замедлили эффект
      y = scale8(sin8(beat), pHEIGHT - 1);
      drawPixelXY(x, y, ColorFromPalette(pal, x * 7 + hue, effectBrightness));
    }
  } else {
    for (uint8_t y = 0; y < pHEIGHT; y++) {
      beat = (GET_MILLIS() * (accum88(y + 1)) * spd / 1000); //и чуть замедлили эффект
      x = scale8(sin8(beat), pWIDTH - 1);
      drawPixelXY(x, y, ColorFromPalette(pal, y * 7 + hue, effectBrightness));
    }
  }
}

// --------------------------- эффект спирали ----------------------
/*
   Aurora: https://github.com/pixelmatix/aurora
   https://github.com/pixelmatix/aurora/blob/sm3.0-64x64/PatternSpiro.h
   Copyright (c) 2014 Jason Coon
   Неполная адаптация SottNick
*/

uint8_t mapsin8(uint8_t theta, uint8_t lowest = 0, uint8_t highest = 255) {
  uint8_t beatsin = sin8(theta);
  uint8_t rangewidth = highest - lowest;
  uint8_t scaledbeat = scale8(beatsin, rangewidth);
  uint8_t result = lowest + scaledbeat;
  return result;
}

uint8_t mapcos8(uint8_t theta, uint8_t lowest = 0, uint8_t highest = 255) {
  uint8_t beatcos = cos8(theta);
  uint8_t rangewidth = highest - lowest;
  uint8_t scaledbeat = scale8(beatcos, rangewidth);
  uint8_t result = lowest + scaledbeat;
  return result;
}

void spiroRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    palette_number = getEffectScaleParamValue2(MC_SPIRO);
    if (palette_number == 0 || palette_number == 55) startnum = random8(1, 55); //Если Случайный выбор или Авто, задать произвольный вариант (в Авто от него начинается отсчет)
    else if (palette_number > 0 || palette_number < 55) startnum = palette_number;  //Если что-то из вариантов 1-54, берем только это значение
  }
  if (palette_number == 55) {  //автоперебор вариантов, если выбран вариант Авто, дается 10 сек на эффект
    if (millis() - color_timer > 10000) {
      color_timer = millis();
      startnum++;
      if (startnum > 54) startnum = 1;
    }
  }
  switch (startnum) {
    case 1:  currentPalette = CloudColors_p;         break;
    case 2:  currentPalette = LavaColors_p;          break;
    case 3:  currentPalette = PartyColors_p;         break;
    case 4:  currentPalette = RainbowColors_p;       break;
    case 5:  currentPalette = RainbowStripeColors_p; break;
    case 6:  currentPalette = ForestColors_p;        break;
    case 7:  currentPalette = AlcoholFireColors_p;   break;
    case 8:  currentPalette = HeatColors_p;          break;
    case 9:  currentPalette = WaterfallColors4in1_p; break;
    case 10: currentPalette = WoodFireColors_p;      break;
    case 11: currentPalette = NormalFire_p;          break;
    case 12: currentPalette = NormalFire2_p;         break;
    case 13: currentPalette = LithiumFireColors_p;   break;
    case 14: currentPalette = SodiumFireColors_p;    break;
    case 15: currentPalette = CopperFireColors_p;    break;
    case 16: currentPalette = RubidiumFireColors_p;  break;
    case 17: currentPalette = PotassiumFireColors_p; break;
    case 18: currentPalette = OceanColors_p;         break;
    case 19: currentPalette = Sunset_Real_gp;        break;
    case 20: currentPalette = dkbluered_gp;          break;
    case 21: currentPalette = Optimus_Prime_gp;      break;
    case 22: currentPalette = warmGrad_gp;           break;
    case 23: currentPalette = coldGrad_gp;           break;
    case 24: currentPalette = hotGrad_gp;            break;
    case 25: currentPalette = pinkGrad_gp;           break;
    case 26: currentPalette = comfy_gp;              break;
    case 27: currentPalette = cyperpunk_gp;          break;
    case 28: currentPalette = girl_gp;               break;
    case 29: currentPalette = xmas_gp;               break;
    case 30: currentPalette = acid_gp;               break;
    case 31: currentPalette = gummy_gp;              break;
    case 32: currentPalette = aurora_gp;             break;
    case 33: currentPalette = redwhite_gp;           break;
    case 34: currentPalette = ib_jul01_gp;           break;
    case 35: currentPalette = rgi_15_gp;             break;
    case 36: currentPalette = retro2_16_gp;          break;
    case 37: currentPalette = Analogous_1_gp;        break;
    case 38: currentPalette = pinksplash_08_gp;      break;
    case 39: currentPalette = pinksplash_07_gp;      break;
    case 40: currentPalette = Coral_reef_gp;         break;
    case 41: currentPalette = ocean_breeze_gp;       break;
    case 42: currentPalette = landscape_64_gp;       break;
    case 43: currentPalette = landscape_33_gp;       break;
    case 44: currentPalette = rainbowsherbet_gp;     break;
    case 45: currentPalette = gr65_hult_gp;          break;
    case 46: currentPalette = GMT_drywet_gp;         break;
    case 47: currentPalette = emerald_dragon_gp;     break;
    case 48: currentPalette = Colorfull_gp;          break;
    case 49: currentPalette = Pink_Purple_gp;        break;
    case 50: currentPalette = autumn_19_gp;          break;
    case 51: currentPalette = daybreak_gp;           break;
    case 52: currentPalette = Blue_Cyan_Yellow_gp;   break;
    case 53: currentPalette = bhw1_28_gp;            break;
    case 54: currentPalette = rbw_gp;                break;
  }
  blurScreen(20); // @Palpalych советует делать размытие
  dimAll(255U - modes[currentMode].Speed / 10);
  boolean change = false;
  for (uint8_t i = 0; i < spirocount; i++) {
    uint8_t x = mapsin8(spirotheta1 + i * spirooffset, spirominx, spiromaxx);
    uint8_t y = mapcos8(spirotheta1 + i * spirooffset, spirominy, spiromaxy);
    uint8_t x2 = mapsin8(spirotheta2 + i * spirooffset, x - spiroradiusx, x + spiroradiusx);
    uint8_t y2 = mapcos8(spirotheta2 + i * spirooffset, y - spiroradiusy, y + spiroradiusy);
    if (x2 < pWIDTH && y2 < pHEIGHT) // добавил проверки. не знаю, почему эффект подвисает без них
      leds[XY(x2, y2)] += (CRGB)ColorFromPalette(currentPalette, hue + i * spirooffset);
    if ((x2 == spirocenterX && y2 == spirocenterY) || (x2 == spirocenterX && y2 == spirocenterY)) change = true;
  }
  spirotheta2 += 2;
  spirotheta1 += 1;
  EVERY_N_MILLIS(75) {
    if (change && !spirohandledChange) {
      spirohandledChange = true;
      if (spirocount >= pWIDTH || spirocount == 1) spiroincrement = !spiroincrement;
      if (spiroincrement) {
        if (spirocount >= 4) spirocount *= 2;
        else spirocount += 1;
      }
      else {
        if (spirocount > 4) spirocount /= 2;
        else spirocount -= 1;
      }
      spirooffset = 256 / spirocount;
    }
    if (!change) spirohandledChange = false;
  }
  hue += 1;
}

//-------- Эффект Дымовые шашки ----------- aka "Детские сны"
// (c) Stepko
// https://editor.soulmatelights.com/gallery/505
// https://github.com/DmytroKorniienko/FireLamp_JeeUI/blob/master/src/effects.cpp

void smokeballsRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    palette_number = getEffectScaleParamValue2(MC_SMOKEBALLS);
    if (palette_number == 0 || palette_number == 56) startnum = random8(1, 56); //Если Случайный выбор или Авто, задать произвольный вариант (в Авто от него начинается отсчет)
    else if (palette_number > 0 || palette_number < 56) startnum = palette_number;  //Если что-то из вариантов 1-55, берем только это значение
    enlargedObjectNUM = enlargedObjectNUM = (modes[currentMode].Scale - 1U) % 11U + 1U;
    speedfactor = fmap(modes[currentMode].Speed, 1., 255., .02, .1); // попробовал разные способы управления скоростью. Этот максимально приемлемый, хотя и сильно тупой.
    for (byte j = 0; j < enlargedObjectNUM; j++) {
      trackingObjectShift[j] =  random((pWIDTH * 10) - ((pWIDTH / 3) * 20)); // сумма trackingObjectState + trackingObjectShift не должна выскакивать за макс.Х
      trackingObjectSpeedX[j] = (float)random(25, 80 * pWIDTH) / 5.;
      trackingObjectState[j] = random((pWIDTH / 2) * 10, (pWIDTH / 3) * 20);
      trackingObjectHue[j] = random8();//(9) * 28;
      trackingObjectPosX[j] = trackingObjectShift[j];
    }
  }
  if (palette_number == 56) {  //автоперебор вариантов, если выбран вариант Авто, дается 10 сек на эффект
    if (millis() - color_timer > 10000) {
      color_timer = millis();
      startnum++;
      if (startnum > 55) startnum = 1;
    }
  }
  switch (startnum) {
    case 1:  currentPalette = CloudColors_p;         break;
    case 2:  currentPalette = LavaColors_p;          break;
    case 3:  currentPalette = PartyColors_p;         break;
    case 4:  currentPalette = RainbowColors_p;       break;
    case 5:  currentPalette = RainbowStripeColors_p; break;
    case 6:  currentPalette = ForestColors_p;        break;
    case 7:  currentPalette = AlcoholFireColors_p;   break;
    case 8:  currentPalette = HeatColors_p;          break;
    case 9:  currentPalette = WaterfallColors4in1_p; break;
    case 10: currentPalette = WoodFireColors_p;      break;
    case 11: currentPalette = NormalFire_p;          break;
    case 12: currentPalette = NormalFire2_p;         break;
    case 13: currentPalette = LithiumFireColors_p;   break;
    case 14: currentPalette = SodiumFireColors_p;    break;
    case 15: currentPalette = CopperFireColors_p;    break;
    case 16: currentPalette = RubidiumFireColors_p;  break;
    case 17: currentPalette = PotassiumFireColors_p; break;
    case 18: currentPalette = OceanColors_p;         break;
    case 19: currentPalette = Sunset_Real_gp;        break;
    case 20: currentPalette = dkbluered_gp;          break;
    case 21: currentPalette = Optimus_Prime_gp;      break;
    case 22: currentPalette = warmGrad_gp;           break;
    case 23: currentPalette = coldGrad_gp;           break;
    case 24: currentPalette = hotGrad_gp;            break;
    case 25: currentPalette = pinkGrad_gp;           break;
    case 26: currentPalette = comfy_gp;              break;
    case 27: currentPalette = cyperpunk_gp;          break;
    case 28: currentPalette = girl_gp;               break;
    case 29: currentPalette = xmas_gp;               break;
    case 30: currentPalette = acid_gp;               break;
    case 31: currentPalette = blueSmoke_gp;          break;
    case 32: currentPalette = gummy_gp;              break;
    case 33: currentPalette = aurora_gp;             break;
    case 34: currentPalette = redwhite_gp;           break;
    case 35: currentPalette = ib_jul01_gp;           break;
    case 36: currentPalette = rgi_15_gp;             break;
    case 37: currentPalette = retro2_16_gp;          break;
    case 38: currentPalette = Analogous_1_gp;        break;
    case 39: currentPalette = pinksplash_08_gp;      break;
    case 40: currentPalette = pinksplash_07_gp;      break;
    case 41: currentPalette = Coral_reef_gp;         break;
    case 42: currentPalette = ocean_breeze_gp;       break;
    case 43: currentPalette = landscape_64_gp;       break;
    case 44: currentPalette = landscape_33_gp;       break;
    case 45: currentPalette = rainbowsherbet_gp;     break;
    case 46: currentPalette = gr65_hult_gp;          break;
    case 47: currentPalette = GMT_drywet_gp;         break;
    case 48: currentPalette = emerald_dragon_gp;     break;
    case 49: currentPalette = Colorfull_gp;          break;
    case 50: currentPalette = Pink_Purple_gp;        break;
    case 51: currentPalette = autumn_19_gp;          break;
    case 52: currentPalette = daybreak_gp;           break;
    case 53: currentPalette = Blue_Cyan_Yellow_gp;   break;
    case 54: currentPalette = bhw1_28_gp;            break;
    case 55: currentPalette = rbw_gp;                break;
  }
  for (byte x = 0; x < pWIDTH; x++) {
    for (float y = (float)pHEIGHT; y > 0.; y -= speedfactor) {
      drawPixelXY(x, y, getPixColorXY(x, y - 1));
    }
  }
  fadeToBlackBy(leds, NUM_LEDS, 128U / pHEIGHT);
  if (modes[currentMode].Speed & 0x01) blurScreen(20);
  for (byte j = 0; j < enlargedObjectNUM; j++) {
    trackingObjectPosX[j] = beatsin16((uint8_t)(trackingObjectSpeedX[j] * (speedfactor * 5.)), trackingObjectShift[j], trackingObjectState[j] + trackingObjectShift[j], trackingObjectHue[j] * 256, trackingObjectHue[j] * 8);
    drawPixelXYF(trackingObjectPosX[j] / 10., 0.05, ColorFromPalette(currentPalette, trackingObjectHue[j]));
  }
  EVERY_N_SECONDS(20) {
    for (byte j = 0; j < enlargedObjectNUM; j++) {
      trackingObjectShift[j] += random(-20, 20);
      trackingObjectHue[j] += 28;
    }
  }
  loadingFlag = random8() > 253U;
}

//====================================================================================================================
// ----------- Эффект "Попкорн"
// (C) Aaron Gotwalt (Soulmate)
// https://editor.soulmatelights.com/gallery/117
// переосмысление (c) SottNick
void popcornRestart_rocket(uint8_t r) {
  trackingObjectSpeedX[r] = (float)(random(-(pWIDTH * pHEIGHT + (pWIDTH * 2)), pWIDTH * pHEIGHT + (pWIDTH * 2))) / 256.0; // * (deltaHue ? 1 : -1); // Наклон. "Мальчики" налево, "девочки" направо. :)
  if ((trackingObjectPosX[r] < 0 && trackingObjectSpeedX[r] < 0) || (trackingObjectPosX[r] > (pWIDTH - 1) && trackingObjectSpeedX[r] > 0)) { // меняем направление только после выхода за пределы экрана
    // leap towards the centre of the screen
    trackingObjectSpeedX[r] = -trackingObjectSpeedX[r];
  }
  // controls the leap pHEIGHT
  trackingObjectSpeedY[r] = (float)(random8() * 8 + pHEIGHT * 10) / 256.0;
  trackingObjectHue[r] = random8();
  trackingObjectPosX[r] = random8(pWIDTH);
}

uint8_t popcorncount;
void popcornRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    speedfactor = fmap((float)modes[currentMode].Speed, 1., 255., 0.25, 1.0);
    popcorncount = map8(getEffectScaleParamValue(MC_POPCORN), round(pWIDTH / 2), round(1.5 * pWIDTH)); //количество частиц, которое можно задавать от pWIDTH/2 до 1.5*pWIDTH через ползунок варианта
    palette_number = getEffectScaleParamValue2(MC_POPCORN);
    if (palette_number == 0 || palette_number == 56) startnum = random8(1, 56); //Если Случайный выбор или Авто, задать произвольный вариант (в Авто от него начинается отсчет)
    else if (palette_number > 0 || palette_number < 56) startnum = palette_number;  //Если что-то из вариантов 1-55, берем только это значение
    enlargedObjectNUM = (modes[currentMode].Scale - 1U) % 11U / 10.0 * (popcorncount - 1U) + 1U;
    if (enlargedObjectNUM > popcorncount) enlargedObjectNUM = popcorncount;
    for (uint8_t r = 0; r < enlargedObjectNUM; r++) {
      trackingObjectPosX[r] = random8(pWIDTH);
      trackingObjectPosY[r] = random8(pHEIGHT);
      trackingObjectSpeedX[r] = 0;
      trackingObjectSpeedY[r] = -1;
      trackingObjectHue[r] = random8();
    }
  }
  if (palette_number == 56) {  //автоперебор вариантов, если выбран вариант Авто, дается 10 сек на эффект
    if (millis() - color_timer > 10000) {
      color_timer = millis();
      startnum++;
      if (startnum > 55) startnum = 1;
    }
  }
  switch (startnum) {
    case 1:  currentPalette = CloudColors_p;         break;
    case 2:  currentPalette = LavaColors_p;          break;
    case 3:  currentPalette = PartyColors_p;         break;
    case 4:  currentPalette = RainbowColors_p;       break;
    case 5:  currentPalette = RainbowStripeColors_p; break;
    case 6:  currentPalette = ForestColors_p;        break;
    case 7:  currentPalette = AlcoholFireColors_p;   break;
    case 8:  currentPalette = HeatColors_p;          break;
    case 9:  currentPalette = WaterfallColors4in1_p; break;
    case 10: currentPalette = WoodFireColors_p;      break;
    case 11: currentPalette = NormalFire_p;          break;
    case 12: currentPalette = NormalFire2_p;         break;
    case 13: currentPalette = LithiumFireColors_p;   break;
    case 14: currentPalette = SodiumFireColors_p;    break;
    case 15: currentPalette = CopperFireColors_p;    break;
    case 16: currentPalette = RubidiumFireColors_p;  break;
    case 17: currentPalette = PotassiumFireColors_p; break;
    case 18: currentPalette = OceanColors_p;         break;
    case 19: currentPalette = Sunset_Real_gp;        break;
    case 20: currentPalette = dkbluered_gp;          break;
    case 21: currentPalette = Optimus_Prime_gp;      break;
    case 22: currentPalette = warmGrad_gp;           break;
    case 23: currentPalette = coldGrad_gp;           break;
    case 24: currentPalette = hotGrad_gp;            break;
    case 25: currentPalette = pinkGrad_gp;           break;
    case 26: currentPalette = comfy_gp;              break;
    case 27: currentPalette = cyperpunk_gp;          break;
    case 28: currentPalette = girl_gp;               break;
    case 29: currentPalette = xmas_gp;               break;
    case 30: currentPalette = acid_gp;               break;
    case 31: currentPalette = blueSmoke_gp;          break;
    case 32: currentPalette = gummy_gp;              break;
    case 33: currentPalette = aurora_gp;             break;
    case 34: currentPalette = redwhite_gp;           break;
    case 35: currentPalette = ib_jul01_gp;           break;
    case 36: currentPalette = rgi_15_gp;             break;
    case 37: currentPalette = retro2_16_gp;          break;
    case 38: currentPalette = Analogous_1_gp;        break;
    case 39: currentPalette = pinksplash_08_gp;      break;
    case 40: currentPalette = pinksplash_07_gp;      break;
    case 41: currentPalette = Coral_reef_gp;         break;
    case 42: currentPalette = ocean_breeze_gp;       break;
    case 43: currentPalette = landscape_64_gp;       break;
    case 44: currentPalette = landscape_33_gp;       break;
    case 45: currentPalette = rainbowsherbet_gp;     break;
    case 46: currentPalette = gr65_hult_gp;          break;
    case 47: currentPalette = GMT_drywet_gp;         break;
    case 48: currentPalette = emerald_dragon_gp;     break;
    case 49: currentPalette = Colorfull_gp;          break;
    case 50: currentPalette = Pink_Purple_gp;        break;
    case 51: currentPalette = autumn_19_gp;          break;
    case 52: currentPalette = daybreak_gp;           break;
    case 53: currentPalette = Blue_Cyan_Yellow_gp;   break;
    case 54: currentPalette = bhw1_28_gp;            break;
    case 55: currentPalette = rbw_gp;                break;
  }
  float popcornGravity = 0.1 * speedfactor;
  fadeToBlackBy(leds, NUM_LEDS, 60);
  for (uint8_t r = 0; r < enlargedObjectNUM; r++) {
    trackingObjectPosX[r] += trackingObjectSpeedX[r] ;
    if (trackingObjectPosX[r] > pWIDTH - 1) trackingObjectPosX[r] = trackingObjectPosX[r] - (pWIDTH - 1);
    if (trackingObjectPosX[r] < 0) trackingObjectPosX[r] = pWIDTH - 1 + trackingObjectPosX[r];
    trackingObjectPosY[r] += trackingObjectSpeedY[r] * speedfactor;
    if (trackingObjectPosY[r] > pHEIGHT - 1) {
      trackingObjectPosY[r] = pHEIGHT + pHEIGHT - 2 - trackingObjectPosY[r];
      trackingObjectSpeedY[r] = -trackingObjectSpeedY[r];
    }
    // bounce off the floor?
    if (trackingObjectPosY[r] < 0 && trackingObjectSpeedY[r] < -0.7) { // 0.7 вычислено в экселе. скорость свободного падения ниже этой не падает. если ниже, значит ещё есть ускорение
      trackingObjectSpeedY[r] = (-trackingObjectSpeedY[r]) * 0.9375;//* 240) >> 8;
      trackingObjectPosY[r] = -trackingObjectPosY[r];
    }
    // settled on the floor?
    if (trackingObjectPosY[r] <= -1)
      popcornRestart_rocket(r);
    // bounce off the sides of the screen?
    // popcornGravity
    trackingObjectSpeedY[r] -= popcornGravity;
    // viscosity
    trackingObjectSpeedX[r] *= 0.875;
    trackingObjectSpeedY[r] *= 0.875;
    // make the acme gray, because why not
    if (-0.004 > trackingObjectSpeedY[r] and trackingObjectSpeedY[r] < 0.004)
      drawPixelXYF(trackingObjectPosX[r], trackingObjectPosY[r], (modes[currentMode].Speed & 0x01) ?
                   ColorFromPalette(currentPalette, trackingObjectHue[r])
                   : CRGB::Pink);
    else
      drawPixelXYF(trackingObjectPosX[r], trackingObjectPosY[r], (modes[currentMode].Speed & 0x01) ?
                   CRGB::Gray
                   : ColorFromPalette(currentPalette, trackingObjectHue[r]));
  }
}

// ============= BOUNCE / ПРЫЖКИ / МЯЧИКИ БЕЗ ГРАНИЦ ===============
// Aurora : https://github.com/pixelmatix/aurora/blob/master/PatternBounce.h
// Copyright(c) 2014 Jason Coon
// v1.0 - Updating for GuverLamp v1.7 by Palpalych 14.04.2020
uint8_t bounce_type = 0;
uint8_t leapcount;
void bounceRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    leapcount = map8(getEffectScaleParamValue(MC_BALLS_BOUNCE), round(pWIDTH / 2), 2 * pWIDTH); //количество частиц, которое можно задавать от pWIDTH/2 до 2*pWIDTH через ползунок варианта
    enlargedObjectNUM = (modes[currentMode].Scale - 1U) % 11U / 10.0 * (leapcount - 1U) + 1U;
    if (enlargedObjectNUM > leapcount) enlargedObjectNUM = leapcount;
    bounce_type = (specialTextEffectParam >= 0) ? specialTextEffectParam : getEffectScaleParamValue2(MC_BALLS_BOUNCE); // Если авто - генерировать один из типов - Вариант 1, Вариант 2
    startnum = random8(1, 56); //задаем произвольно палитру из списка
    if (bounce_type == 0 || bounce_type > 2) bounce_type = random8(1, 3);
    if (bounce_type == 1) {
      for (uint8_t i = 0 ; i < enlargedObjectNUM ; i++) {
        trackingObjectPosX[i] = random8(pWIDTH);
        trackingObjectPosY[i] = random8(pHEIGHT);
        trackingObjectHue[i] = random8();
      }
    }
    if (bounce_type == 2) {
      uint8_t colorWidth = 256U / enlargedObjectNUM;
      for (uint8_t i = 0; i < enlargedObjectNUM; i++) {
        Boid boid = Boid(i % pWIDTH, 0);
        boid.velocity.x = 0;
        boid.velocity.y = i * -0.01;
        boid.colorIndex = colorWidth * i;
        boid.maxforce = 10;
        boid.maxspeed = 10;
        boids[i] = boid;
      }
    }
  }
  switch (startnum) {
    case 1:  currentPalette = CloudColors_p;         break;
    case 2:  currentPalette = LavaColors_p;          break;
    case 3:  currentPalette = PartyColors_p;         break;
    case 4:  currentPalette = RainbowColors_p;       break;
    case 5:  currentPalette = RainbowStripeColors_p; break;
    case 6:  currentPalette = ForestColors_p;        break;
    case 7:  currentPalette = AlcoholFireColors_p;   break;
    case 8:  currentPalette = HeatColors_p;          break;
    case 9:  currentPalette = WaterfallColors4in1_p; break;
    case 10: currentPalette = WoodFireColors_p;      break;
    case 11: currentPalette = NormalFire_p;          break;
    case 12: currentPalette = NormalFire2_p;         break;
    case 13: currentPalette = LithiumFireColors_p;   break;
    case 14: currentPalette = SodiumFireColors_p;    break;
    case 15: currentPalette = CopperFireColors_p;    break;
    case 16: currentPalette = RubidiumFireColors_p;  break;
    case 17: currentPalette = PotassiumFireColors_p; break;
    case 18: currentPalette = OceanColors_p;         break;
    case 19: currentPalette = Sunset_Real_gp;        break;
    case 20: currentPalette = dkbluered_gp;          break;
    case 21: currentPalette = Optimus_Prime_gp;      break;
    case 22: currentPalette = warmGrad_gp;           break;
    case 23: currentPalette = coldGrad_gp;           break;
    case 24: currentPalette = hotGrad_gp;            break;
    case 25: currentPalette = pinkGrad_gp;           break;
    case 26: currentPalette = comfy_gp;              break;
    case 27: currentPalette = cyperpunk_gp;          break;
    case 28: currentPalette = girl_gp;               break;
    case 29: currentPalette = xmas_gp;               break;
    case 30: currentPalette = acid_gp;               break;
    case 31: currentPalette = blueSmoke_gp;          break;
    case 32: currentPalette = gummy_gp;              break;
    case 33: currentPalette = aurora_gp;             break;
    case 34: currentPalette = redwhite_gp;           break;
    case 35: currentPalette = ib_jul01_gp;           break;
    case 36: currentPalette = rgi_15_gp;             break;
    case 37: currentPalette = retro2_16_gp;          break;
    case 38: currentPalette = Analogous_1_gp;        break;
    case 39: currentPalette = pinksplash_08_gp;      break;
    case 40: currentPalette = pinksplash_07_gp;      break;
    case 41: currentPalette = Coral_reef_gp;         break;
    case 42: currentPalette = ocean_breeze_gp;       break;
    case 43: currentPalette = landscape_64_gp;       break;
    case 44: currentPalette = landscape_33_gp;       break;
    case 45: currentPalette = rainbowsherbet_gp;     break;
    case 46: currentPalette = gr65_hult_gp;          break;
    case 47: currentPalette = GMT_drywet_gp;         break;
    case 48: currentPalette = emerald_dragon_gp;     break;
    case 49: currentPalette = Colorfull_gp;          break;
    case 50: currentPalette = Pink_Purple_gp;        break;
    case 51: currentPalette = autumn_19_gp;          break;
    case 52: currentPalette = daybreak_gp;           break;
    case 53: currentPalette = Blue_Cyan_Yellow_gp;   break;
    case 54: currentPalette = bhw1_28_gp;            break;
    case 55: currentPalette = rbw_gp;                break;
  }
  switch (bounce_type) {
    case 1: Leapers_Routine(currentPalette); break;
    case 2: bounce_Routine(currentPalette); break;
  }
}

#define e_bnc_SIDEJUMP (true)
PVector gravity = PVector(0, -0.0125);
void bounce_Routine(CRGBPalette16 pal) {
  blurScreen(beatsin8(5U, 1U, 5U));
  dimAll(255U - modes[currentMode].Speed); // dimAll(hue2);
  for (uint8_t i = 0; i < enlargedObjectNUM; i++) {
    Boid boid = boids[i];
    boid.applyForce(gravity);
    boid.update();
    if (boid.location.x >= pWIDTH) boid.location.x = boid.location.x - pWIDTH; // это только
    else if (boid.location.x < 0) boid.location.x = boid.location.x + pWIDTH; // для субпиксельной версии
    CRGB color = ColorFromPalette(pal, boid.colorIndex); // boid.colorIndex + hue
    drawPixelXYF(boid.location.x, boid.location.y, color);
    if (boid.location.y <= 0) {
      boid.location.y = 0;
      boid.velocity.y = -boid.velocity.y;
      boid.velocity.x *= 0.9;
      if (!random8() || boid.velocity.y < 0.01) {
#if e_bnc_SIDEJUMP
        boid.applyForce(PVector((float)random(127) / 255 - 0.25, (float)random(255) / 255));
#else
        boid.applyForce(PVector(0, (float)random(255) / 255));
#endif
      }
    }
    boids[i] = boid;
  }
}
void LeapersRestart_leaper(uint8_t l) {
  // leap up and to the side with some random component
  trackingObjectSpeedX[l] = (1 * (float)random8(1, 100) / 100);
  trackingObjectSpeedY[l] = (2 * (float)random8(1, 100) / 100);
  // for variety, sometimes go 50% faster
  if (random8() < 12) {
    trackingObjectSpeedX[l] += trackingObjectSpeedX[l] * 0.5;
    trackingObjectSpeedY[l] += trackingObjectSpeedY[l] * 0.5;
  }
  // leap towards the centre of the screen
  if (trackingObjectPosX[l] > (pWIDTH / 2)) {
    trackingObjectSpeedX[l] = -trackingObjectSpeedX[l];
  }
}
void LeapersMove_leaper(uint8_t l) {
#define GRAVITY            0.06
#define SETTLED_THRESHOLD  0.1
#define WALL_FRICTION      0.95
#define WIND               0.95    // wind resistance
  trackingObjectPosX[l] += trackingObjectSpeedX[l];
  trackingObjectPosY[l] += trackingObjectSpeedY[l];
  // bounce off the floor and ceiling?
  if (trackingObjectPosY[l] < 0 || trackingObjectPosY[l] > pHEIGHT - 1) {
    trackingObjectSpeedY[l] = (-trackingObjectSpeedY[l] * WALL_FRICTION);
    trackingObjectSpeedX[l] = ( trackingObjectSpeedX[l] * WALL_FRICTION);
    trackingObjectPosY[l] += trackingObjectSpeedY[l];
    if (trackingObjectPosY[l] < 0)
      trackingObjectPosY[l] = 0; // settled on the floor?
    if (trackingObjectPosY[l] <= SETTLED_THRESHOLD && fabs(trackingObjectSpeedY[l]) <= SETTLED_THRESHOLD) {
      LeapersRestart_leaper(l);
    }
  }
  // bounce off the sides of the screen?
  if (trackingObjectPosX[l] <= 0 || trackingObjectPosX[l] >= pWIDTH - 1) {
    trackingObjectSpeedX[l] = (-trackingObjectSpeedX[l] * WALL_FRICTION);
    if (trackingObjectPosX[l] <= 0) {
      trackingObjectPosX[l] = -trackingObjectPosX[l];
    } else {
      trackingObjectPosX[l] = pWIDTH + pWIDTH - 2 - trackingObjectPosX[l];
    }
  }
  trackingObjectSpeedY[l] -= GRAVITY;
  trackingObjectSpeedX[l] *= WIND;
  trackingObjectSpeedY[l] *= WIND;
}

void Leapers_Routine(CRGBPalette16 pal) {
  FastLED.clear();
  for (uint8_t i = 0; i < enlargedObjectNUM; i++) {
    LeapersMove_leaper(i);
    drawPixelXYF(trackingObjectPosX[i], trackingObjectPosY[i], ColorFromPalette(pal, trackingObjectHue[i]));
  };
  blurScreen(20);
}

// ============= ЭФФЕКТ ПРИТЯЖЕНИЕ ===============
// https://github.com/pixelmatix/aurora/blob/master/PatternAttract.h
// Адаптация (c) SottNick
// используются переменные эффекта Стая. Без него работать не будет.
void attractRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    palette_number = getEffectScaleParamValue2(MC_ATTRACT);
    if (palette_number == 0 || palette_number == 56) startnum = random8(1, 56); //Если Случайный выбор или Авто, задать произвольный вариант (в Авто от него начинается отсчет)
    else if (palette_number > 0 || palette_number < 56) startnum = palette_number;  //Если что-то из вариантов 1-55, берем только это значение
    enlargedObjectNUM = (modes[currentMode].Scale - 1U) % 11U + 1U;//(modes[currentMode].Scale - 1U) / 99.0 * (AVAILABLE_BOID_COUNT - 1U) + 1U;
    for (uint8_t i = 0; i < enlargedObjectNUM; i++) {
      boids[i] = Boid(random8(pWIDTH), random8(pHEIGHT));//pWIDTH - 1, pHEIGHT - i);
      boids[i].mass = ((float)random8(33U, 134U)) / 100.; // random(0.1, 2); // сюда можно поставить регулятор разлёта. чем меньше число, тем дальше от центра будет вылет
      boids[i].velocity.x = ((float) random8(46U, 100U)) / 500.0;
      if (random8(2U)) boids[i].velocity.x = -boids[i].velocity.x;
      boids[i].velocity.y = 0;
      boids[i].colorIndex = random8();//i * 32;
    }
  }
  if (palette_number == 56) {  //автоперебор вариантов, если выбран вариант Авто, дается 10 сек на эффект
    if (millis() - color_timer > 10000) {
      color_timer = millis();
      startnum++;
      if (startnum > 55) startnum = 1;
    }
  }
  switch (startnum) {
    case 1:  currentPalette = CloudColors_p;         break;
    case 2:  currentPalette = LavaColors_p;          break;
    case 3:  currentPalette = PartyColors_p;         break;
    case 4:  currentPalette = RainbowColors_p;       break;
    case 5:  currentPalette = RainbowStripeColors_p; break;
    case 6:  currentPalette = ForestColors_p;        break;
    case 7:  currentPalette = AlcoholFireColors_p;   break;
    case 8:  currentPalette = HeatColors_p;          break;
    case 9:  currentPalette = WaterfallColors4in1_p; break;
    case 10: currentPalette = WoodFireColors_p;      break;
    case 11: currentPalette = NormalFire_p;          break;
    case 12: currentPalette = NormalFire2_p;         break;
    case 13: currentPalette = LithiumFireColors_p;   break;
    case 14: currentPalette = SodiumFireColors_p;    break;
    case 15: currentPalette = CopperFireColors_p;    break;
    case 16: currentPalette = RubidiumFireColors_p;  break;
    case 17: currentPalette = PotassiumFireColors_p; break;
    case 18: currentPalette = OceanColors_p;         break;
    case 19: currentPalette = Sunset_Real_gp;        break;
    case 20: currentPalette = dkbluered_gp;          break;
    case 21: currentPalette = Optimus_Prime_gp;      break;
    case 22: currentPalette = warmGrad_gp;           break;
    case 23: currentPalette = coldGrad_gp;           break;
    case 24: currentPalette = hotGrad_gp;            break;
    case 25: currentPalette = pinkGrad_gp;           break;
    case 26: currentPalette = comfy_gp;              break;
    case 27: currentPalette = cyperpunk_gp;          break;
    case 28: currentPalette = girl_gp;               break;
    case 29: currentPalette = xmas_gp;               break;
    case 30: currentPalette = acid_gp;               break;
    case 31: currentPalette = blueSmoke_gp;          break;
    case 32: currentPalette = gummy_gp;              break;
    case 33: currentPalette = aurora_gp;             break;
    case 34: currentPalette = redwhite_gp;           break;
    case 35: currentPalette = ib_jul01_gp;           break;
    case 36: currentPalette = rgi_15_gp;             break;
    case 37: currentPalette = retro2_16_gp;          break;
    case 38: currentPalette = Analogous_1_gp;        break;
    case 39: currentPalette = pinksplash_08_gp;      break;
    case 40: currentPalette = pinksplash_07_gp;      break;
    case 41: currentPalette = Coral_reef_gp;         break;
    case 42: currentPalette = ocean_breeze_gp;       break;
    case 43: currentPalette = landscape_64_gp;       break;
    case 44: currentPalette = landscape_33_gp;       break;
    case 45: currentPalette = rainbowsherbet_gp;     break;
    case 46: currentPalette = gr65_hult_gp;          break;
    case 47: currentPalette = GMT_drywet_gp;         break;
    case 48: currentPalette = emerald_dragon_gp;     break;
    case 49: currentPalette = Colorfull_gp;          break;
    case 50: currentPalette = Pink_Purple_gp;        break;
    case 51: currentPalette = autumn_19_gp;          break;
    case 52: currentPalette = daybreak_gp;           break;
    case 53: currentPalette = Blue_Cyan_Yellow_gp;   break;
    case 54: currentPalette = bhw1_28_gp;            break;
    case 55: currentPalette = rbw_gp;                break;
  }
  dimAll(220);
  PVector attractLocation = PVector(pWIDTH * 0.5, pHEIGHT * 0.5);
  for (uint8_t i = 0; i < enlargedObjectNUM; i++) {
    Boid boid = boids[i];
    PVector force = attractLocation - boid.location;    // Calculate direction of force // и вкорячиваем сюда регулировку скорости
    float d = force.mag();                              // Distance between objects
    d = constrain(d, 5.0f, pHEIGHT * 2.);               // Limiting the distance to eliminate "extreme" results for very close or very far objects
    force.normalize();                                  // Normalize vector (distance doesn't matter here, we just want this vector for direction)
    float strength = (5. * boid.mass) / (d * d);        // Calculate gravitional force magnitude 5.=attractG*attractMass
    force *= strength;                                  // Get force vector --> magnitude * direction
    boid.applyForce(force);
    boid.update();
    drawPixelXYF(boid.location.x, boid.location.y, ColorFromPalette(currentPalette, boid.colorIndex + hue));
    boids[i] = boid;
  }
  EVERY_N_MILLIS(200) {
    hue++;
  }
}

// ============= ЭФФЕКТ ВИХРИ ===============
// https://github.com/pixelmatix/aurora/blob/master/PatternFlowField.h
// Адаптация (c) SottNick
// используются переменные эффекта Стая. Без него работать не будет.
static const uint8_t ff_speed = 1; // чем выше этот параметр, тем короче переходы (градиенты) между цветами. 1 - это самое красивое
static const uint8_t ff_scale = 26; // чем больше этот параметр, тем больше "языков пламени" или как-то так. 26 - это норм
uint8_t whirl_type = 0;
void whirl() {
  if (loadingFlag) {
    loadingFlag = false;
    palette_number = getEffectScaleParamValue2(MC_WHIRL);
    if (palette_number == 0 || palette_number == 56) startnum = random8(1, 56); //Если Случайный выбор или Авто, задать произвольный вариант (в Авто от него начинается отсчет)
    else if (palette_number > 0 || palette_number < 56) startnum = palette_number;  //Если что-то из вариантов 1-55, берем только это значение
    ff_x = random16();
    ff_y = random16();
    ff_z = random16();
    for (uint8_t i = 0; i < AVAILABLE_BOID_COUNT; i++) {
      boids[i] = Boid(random8(pWIDTH), 0);
    }
  }
  if (palette_number == 56) {  //автоперебор вариантов, если выбран вариант Авто, дается 10 сек на эффект
    if (millis() - color_timer > 10000) {
      color_timer = millis();
      startnum++;
      if (startnum > 35) startnum = 1;
    }
  }
  switch (startnum) {
    case 1:  currentPalette = CloudColors_p;         break;
    case 2:  currentPalette = LavaColors_p;          break;
    case 3:  currentPalette = PartyColors_p;         break;
    case 4:  currentPalette = RainbowColors_p;       break;
    case 5:  currentPalette = RainbowStripeColors_p; break;
    case 6:  currentPalette = ForestColors_p;        break;
    case 7:  currentPalette = AlcoholFireColors_p;   break;
    case 8:  currentPalette = HeatColors_p;          break;
    case 9:  currentPalette = WaterfallColors4in1_p; break;
    case 10: currentPalette = WoodFireColors_p;      break;
    case 11: currentPalette = NormalFire_p;          break;
    case 12: currentPalette = NormalFire2_p;         break;
    case 13: currentPalette = LithiumFireColors_p;   break;
    case 14: currentPalette = SodiumFireColors_p;    break;
    case 15: currentPalette = CopperFireColors_p;    break;
    case 16: currentPalette = RubidiumFireColors_p;  break;
    case 17: currentPalette = PotassiumFireColors_p; break;
    case 18: currentPalette = OceanColors_p;         break;
    case 19: currentPalette = Sunset_Real_gp;        break;
    case 20: currentPalette = dkbluered_gp;          break;
    case 21: currentPalette = Optimus_Prime_gp;      break;
    case 22: currentPalette = warmGrad_gp;           break;
    case 23: currentPalette = coldGrad_gp;           break;
    case 24: currentPalette = hotGrad_gp;            break;
    case 25: currentPalette = pinkGrad_gp;           break;
    case 26: currentPalette = comfy_gp;              break;
    case 27: currentPalette = cyperpunk_gp;          break;
    case 28: currentPalette = girl_gp;               break;
    case 29: currentPalette = xmas_gp;               break;
    case 30: currentPalette = acid_gp;               break;
    case 31: currentPalette = blueSmoke_gp;          break;
    case 32: currentPalette = gummy_gp;              break;
    case 33: currentPalette = aurora_gp;             break;
    case 34: currentPalette = redwhite_gp;           break;
    case 35: currentPalette = ib_jul01_gp;           break;
    case 36: currentPalette = rgi_15_gp;             break;
    case 37: currentPalette = retro2_16_gp;          break;
    case 38: currentPalette = Analogous_1_gp;        break;
    case 39: currentPalette = pinksplash_08_gp;      break;
    case 40: currentPalette = pinksplash_07_gp;      break;
    case 41: currentPalette = Coral_reef_gp;         break;
    case 42: currentPalette = ocean_breeze_gp;       break;
    case 43: currentPalette = landscape_64_gp;       break;
    case 44: currentPalette = landscape_33_gp;       break;
    case 45: currentPalette = rainbowsherbet_gp;     break;
    case 46: currentPalette = gr65_hult_gp;          break;
    case 47: currentPalette = GMT_drywet_gp;         break;
    case 48: currentPalette = emerald_dragon_gp;     break;
    case 49: currentPalette = Colorfull_gp;          break;
    case 50: currentPalette = Pink_Purple_gp;        break;
    case 51: currentPalette = autumn_19_gp;          break;
    case 52: currentPalette = daybreak_gp;           break;
    case 53: currentPalette = Blue_Cyan_Yellow_gp;   break;
    case 54: currentPalette = bhw1_28_gp;            break;
    case 55: currentPalette = rbw_gp;                break;
  }
  dimAll(240);
  for (uint8_t i = 0; i < AVAILABLE_BOID_COUNT; i++) {
    Boid * boid = &boids[i];
    int ioffset = ff_scale * boid->location.x;
    int joffset = ff_scale * boid->location.y;
    byte angle = inoise8(ff_x + ioffset, ff_y + joffset, ff_z);
    boid->velocity.x = (float) sin8(angle) * 0.0078125 - 1.0;
    boid->velocity.y = -((float)cos8(angle) * 0.0078125 - 1.0);
    boid->update();
    drawPixelXYF(boid->location.x, boid->location.y, ColorFromPalette(currentPalette, angle + hue)); // + hue постепенно сдвигает палитру по кругу
    if (boid->location.x < 0 || boid->location.x >= pWIDTH || boid->location.y < 0 || boid->location.y >= pHEIGHT) {
      boid->location.x = random(pWIDTH);
      boid->location.y = 0;
    }
  }
  EVERY_N_MILLIS(200) {
    hue++;
  }
  ff_x += ff_speed;
  ff_y += ff_speed;
  ff_z += ff_speed;
}

// ============= Эффект Плазменная лампа ===============
// эффект Паук (c) stepko
// плюс выбор палитры и багфикс (c) SottNick
void spiderRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    palette_number = getEffectScaleParamValue2(MC_PLASMALAMP);
    if (palette_number == 0 || palette_number == 56) startnum = random8(1, 56); //Если Случайный выбор или Авто, задать произвольный вариант (в Авто от него начинается отсчет)
    else if (palette_number > 0 || palette_number < 56) startnum = palette_number;  //Если что-то из вариантов 1-55, берем только это значение
    pcnt = 4;  //Количество линий; на мой взгляд, 4 - самое то
    speedfactor = fmap(modes[currentMode].Speed, 1, 255, 20., 2.);
  }
  if (palette_number == 56) {  //автоперебор вариантов, если выбран вариант Авто, дается 10 сек на эффект
    if (millis() - color_timer > 10000) {
      color_timer = millis();
      startnum++;
      if (startnum > 55) startnum = 1;
    }
  }
  switch (startnum) {
    case 1:  currentPalette = CloudColors_p;         break;
    case 2:  currentPalette = LavaColors_p;          break;
    case 3:  currentPalette = PartyColors_p;         break;
    case 4:  currentPalette = RainbowColors_p;       break;
    case 5:  currentPalette = RainbowStripeColors_p; break;
    case 6:  currentPalette = ForestColors_p;        break;
    case 7:  currentPalette = AlcoholFireColors_p;   break;
    case 8:  currentPalette = HeatColors_p;          break;
    case 9:  currentPalette = WaterfallColors4in1_p; break;
    case 10: currentPalette = WoodFireColors_p;      break;
    case 11: currentPalette = NormalFire_p;          break;
    case 12: currentPalette = NormalFire2_p;         break;
    case 13: currentPalette = LithiumFireColors_p;   break;
    case 14: currentPalette = SodiumFireColors_p;    break;
    case 15: currentPalette = CopperFireColors_p;    break;
    case 16: currentPalette = RubidiumFireColors_p;  break;
    case 17: currentPalette = PotassiumFireColors_p; break;
    case 18: currentPalette = OceanColors_p;         break;
    case 19: currentPalette = Sunset_Real_gp;        break;
    case 20: currentPalette = dkbluered_gp;          break;
    case 21: currentPalette = Optimus_Prime_gp;      break;
    case 22: currentPalette = warmGrad_gp;           break;
    case 23: currentPalette = coldGrad_gp;           break;
    case 24: currentPalette = hotGrad_gp;            break;
    case 25: currentPalette = pinkGrad_gp;           break;
    case 26: currentPalette = comfy_gp;              break;
    case 27: currentPalette = cyperpunk_gp;          break;
    case 28: currentPalette = girl_gp;               break;
    case 29: currentPalette = xmas_gp;               break;
    case 30: currentPalette = acid_gp;               break;
    case 31: currentPalette = blueSmoke_gp;          break;
    case 32: currentPalette = gummy_gp;              break;
    case 33: currentPalette = aurora_gp;             break;
    case 34: currentPalette = redwhite_gp;           break;
    case 35: currentPalette = ib_jul01_gp;           break;
    case 36: currentPalette = rgi_15_gp;             break;
    case 37: currentPalette = retro2_16_gp;          break;
    case 38: currentPalette = Analogous_1_gp;        break;
    case 39: currentPalette = pinksplash_08_gp;      break;
    case 40: currentPalette = pinksplash_07_gp;      break;
    case 41: currentPalette = Coral_reef_gp;         break;
    case 42: currentPalette = ocean_breeze_gp;       break;
    case 43: currentPalette = landscape_64_gp;       break;
    case 44: currentPalette = landscape_33_gp;       break;
    case 45: currentPalette = rainbowsherbet_gp;     break;
    case 46: currentPalette = gr65_hult_gp;          break;
    case 47: currentPalette = GMT_drywet_gp;         break;
    case 48: currentPalette = emerald_dragon_gp;     break;
    case 49: currentPalette = Colorfull_gp;          break;
    case 50: currentPalette = Pink_Purple_gp;        break;
    case 51: currentPalette = autumn_19_gp;          break;
    case 52: currentPalette = daybreak_gp;           break;
    case 53: currentPalette = Blue_Cyan_Yellow_gp;   break;
    case 54: currentPalette = bhw1_28_gp;            break;
    case 55: currentPalette = rbw_gp;                break;
  }
  if (hue2++ & 0x01 && deltaHue++ & 0x01 && deltaHue2++ & 0x01) hue++; // хз. как с 60ю кадрами в секунду скорость замедлять...
  dimAll(205);
  float time_shift = millis() & 0x7FFFFF; // overflow protection proper by SottNick
  time_shift /= speedfactor;
  for (uint8_t c = 0; c < pcnt; c++) {
    float xx = 2. + sin8(time_shift + 6000 * c) / 12.;
    float yy = 2. + cos8(time_shift + 9000 * c) / 12.;
    DrawLineF(xx, yy, (float)pWIDTH - xx - 1, (float)pHEIGHT - yy - 1, ColorFromPalette(currentPalette, hue + c * (255 / pcnt)));
  }
}

// =============== Эффект Lumenjer ================
// (c) SottNick
#define DIMSPEED (254U - 500U / pWIDTH / pHEIGHT)
void lumenjerRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    FastLED.clear();
    palette_number = getEffectScaleParamValue2(MC_LUMENJER);
    if (palette_number == 0 || palette_number == 56) startnum = random8(1, 56); //Если Случайный выбор или Авто, задать произвольный вариант (в Авто от него начинается отсчет)
    else if (palette_number > 0 || palette_number < 56) startnum = palette_number;  //Если что-то из вариантов 1-55, берем только это значение
    deltaHue = -1;
    deltaHue2 = -1;
    dimAll(245U);
  }
  if (palette_number == 56) {  //автоперебор вариантов, если выбран вариант Авто, дается 10 сек на эффект
    if (millis() - color_timer > 10000) {
      color_timer = millis();
      startnum++;
      if (startnum > 55) startnum = 1;
    }
  }
  switch (startnum) {
    case 1:  currentPalette = CloudColors_p;         break;
    case 2:  currentPalette = LavaColors_p;          break;
    case 3:  currentPalette = PartyColors_p;         break;
    case 4:  currentPalette = RainbowColors_p;       break;
    case 5:  currentPalette = RainbowStripeColors_p; break;
    case 6:  currentPalette = ForestColors_p;        break;
    case 7:  currentPalette = AlcoholFireColors_p;   break;
    case 8:  currentPalette = HeatColors_p;          break;
    case 9:  currentPalette = WaterfallColors4in1_p; break;
    case 10: currentPalette = WoodFireColors_p;      break;
    case 11: currentPalette = NormalFire_p;          break;
    case 12: currentPalette = NormalFire2_p;         break;
    case 13: currentPalette = LithiumFireColors_p;   break;
    case 14: currentPalette = SodiumFireColors_p;    break;
    case 15: currentPalette = CopperFireColors_p;    break;
    case 16: currentPalette = RubidiumFireColors_p;  break;
    case 17: currentPalette = PotassiumFireColors_p; break;
    case 18: currentPalette = OceanColors_p;         break;
    case 19: currentPalette = Sunset_Real_gp;        break;
    case 20: currentPalette = dkbluered_gp;          break;
    case 21: currentPalette = Optimus_Prime_gp;      break;
    case 22: currentPalette = warmGrad_gp;           break;
    case 23: currentPalette = coldGrad_gp;           break;
    case 24: currentPalette = hotGrad_gp;            break;
    case 25: currentPalette = pinkGrad_gp;           break;
    case 26: currentPalette = comfy_gp;              break;
    case 27: currentPalette = cyperpunk_gp;          break;
    case 28: currentPalette = girl_gp;               break;
    case 29: currentPalette = xmas_gp;               break;
    case 30: currentPalette = acid_gp;               break;
    case 31: currentPalette = blueSmoke_gp;          break;
    case 32: currentPalette = gummy_gp;              break;
    case 33: currentPalette = aurora_gp;             break;
    case 34: currentPalette = redwhite_gp;           break;
    case 35: currentPalette = ib_jul01_gp;           break;
    case 36: currentPalette = rgi_15_gp;             break;
    case 37: currentPalette = retro2_16_gp;          break;
    case 38: currentPalette = Analogous_1_gp;        break;
    case 39: currentPalette = pinksplash_08_gp;      break;
    case 40: currentPalette = pinksplash_07_gp;      break;
    case 41: currentPalette = Coral_reef_gp;         break;
    case 42: currentPalette = ocean_breeze_gp;       break;
    case 43: currentPalette = landscape_64_gp;       break;
    case 44: currentPalette = landscape_33_gp;       break;
    case 45: currentPalette = rainbowsherbet_gp;     break;
    case 46: currentPalette = gr65_hult_gp;          break;
    case 47: currentPalette = GMT_drywet_gp;         break;
    case 48: currentPalette = emerald_dragon_gp;     break;
    case 49: currentPalette = Colorfull_gp;          break;
    case 50: currentPalette = Pink_Purple_gp;        break;
    case 51: currentPalette = autumn_19_gp;          break;
    case 52: currentPalette = daybreak_gp;           break;
    case 53: currentPalette = Blue_Cyan_Yellow_gp;   break;
    case 54: currentPalette = bhw1_28_gp;            break;
    case 55: currentPalette = rbw_gp;                break;
  }
  dimAll(DIMSPEED);
  deltaHue = random8(3) ? deltaHue : -deltaHue;
  deltaHue2 = random8(3) ? deltaHue2 : -deltaHue2;
#if (pWIDTH % 2 == 0 && pHEIGHT % 2 == 0)
  hue = (pWIDTH + hue + deltaHue * (bool)random8(64)) % pWIDTH;
#else
  hue = (pWIDTH + hue + deltaHue) % pWIDTH;
#endif
  hue2 = (pHEIGHT + hue2 + deltaHue2) % pHEIGHT;
  if (modes[currentMode].Scale == 100U)
    leds[XY(hue, hue2)] += CHSV(random8(), 255U, 255U);
  else
    leds[XY(hue, hue2)] += ColorFromPalette(currentPalette, step++);
}

// --------------------------- эффект МетаБолз ----------------------
// https://gist.github.com/StefanPetrick/170fbf141390fafb9c0c76b8a0d34e54
// Stefan Petrick's MetaBalls Effect mod by PalPalych for GyverLamp
void MetaBallsRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    palette_number = getEffectScaleParamValue2(MC_METABALLS);
    if (palette_number == 0 || palette_number == 56) startnum = random8(1, 56); //Если Случайный выбор или Авто, задать произвольный вариант (в Авто от него начинается отсчет)
    else if (palette_number > 0 || palette_number < 56) startnum = palette_number;  //Если что-то из вариантов 1-55, берем только это значение
    speedfactor = 1;  //modes[currentMode].Speed / 127.0;
  }
  if (palette_number == 56) {  //автоперебор вариантов, если выбран вариант Авто, дается 10 сек на эффект
    if (millis() - color_timer > 10000) {
      color_timer = millis();
      startnum++;
      if (startnum > 55) startnum = 1;
    }
  }
  switch (startnum) {
    case 1:  currentPalette = CloudColors_p;         break;
    case 2:  currentPalette = LavaColors_p;          break;
    case 3:  currentPalette = PartyColors_p;         break;
    case 4:  currentPalette = RainbowColors_p;       break;
    case 5:  currentPalette = RainbowStripeColors_p; break;
    case 6:  currentPalette = ForestColors_p;        break;
    case 7:  currentPalette = AlcoholFireColors_p;   break;
    case 8:  currentPalette = HeatColors_p;          break;
    case 9:  currentPalette = WaterfallColors4in1_p; break;
    case 10: currentPalette = WoodFireColors_p;      break;
    case 11: currentPalette = NormalFire_p;          break;
    case 12: currentPalette = NormalFire2_p;         break;
    case 13: currentPalette = LithiumFireColors_p;   break;
    case 14: currentPalette = SodiumFireColors_p;    break;
    case 15: currentPalette = CopperFireColors_p;    break;
    case 16: currentPalette = RubidiumFireColors_p;  break;
    case 17: currentPalette = PotassiumFireColors_p; break;
    case 18: currentPalette = OceanColors_p;         break;
    case 19: currentPalette = Sunset_Real_gp;        break;
    case 20: currentPalette = dkbluered_gp;          break;
    case 21: currentPalette = Optimus_Prime_gp;      break;
    case 22: currentPalette = warmGrad_gp;           break;
    case 23: currentPalette = coldGrad_gp;           break;
    case 24: currentPalette = hotGrad_gp;            break;
    case 25: currentPalette = pinkGrad_gp;           break;
    case 26: currentPalette = comfy_gp;              break;
    case 27: currentPalette = cyperpunk_gp;          break;
    case 28: currentPalette = girl_gp;               break;
    case 29: currentPalette = xmas_gp;               break;
    case 30: currentPalette = acid_gp;               break;
    case 31: currentPalette = blueSmoke_gp;          break;
    case 32: currentPalette = gummy_gp;              break;
    case 33: currentPalette = aurora_gp;             break;
    case 34: currentPalette = redwhite_gp;           break;
    case 35: currentPalette = ib_jul01_gp;           break;
    case 36: currentPalette = rgi_15_gp;             break;
    case 37: currentPalette = retro2_16_gp;          break;
    case 38: currentPalette = Analogous_1_gp;        break;
    case 39: currentPalette = pinksplash_08_gp;      break;
    case 40: currentPalette = pinksplash_07_gp;      break;
    case 41: currentPalette = Coral_reef_gp;         break;
    case 42: currentPalette = ocean_breeze_gp;       break;
    case 43: currentPalette = landscape_64_gp;       break;
    case 44: currentPalette = landscape_33_gp;       break;
    case 45: currentPalette = rainbowsherbet_gp;     break;
    case 46: currentPalette = gr65_hult_gp;          break;
    case 47: currentPalette = GMT_drywet_gp;         break;
    case 48: currentPalette = emerald_dragon_gp;     break;
    case 49: currentPalette = Colorfull_gp;          break;
    case 50: currentPalette = Pink_Purple_gp;        break;
    case 51: currentPalette = autumn_19_gp;          break;
    case 52: currentPalette = daybreak_gp;           break;
    case 53: currentPalette = Blue_Cyan_Yellow_gp;   break;
    case 54: currentPalette = bhw1_28_gp;            break;
    case 55: currentPalette = rbw_gp;                break;
  }
  // get some 2 random moving points
  uint16_t param1 = millis() * speedfactor;
  uint8_t x2 = inoise8(param1, 25355, 685 ) / pWIDTH;
  uint8_t y2 = inoise8(param1, 355, 11685 ) / pHEIGHT;
  uint8_t x3 = inoise8(param1, 55355, 6685 ) / pWIDTH;
  uint8_t y3 = inoise8(param1, 25355, 22685 ) / pHEIGHT;
  // and one Lissajou function
  uint8_t x1 = beatsin8(23 * speedfactor, 0, pWIDTH - 1U);
  uint8_t y1 = beatsin8(28 * speedfactor, 0, pHEIGHT - 1U);
  for (uint8_t y = 0; y < pHEIGHT; y++) {
    for (uint8_t x = 0; x < pWIDTH; x++) {
      // calculate distances of the 3 points from actual pixel
      // and add them together with weightening
      uint8_t  dx =  abs(x - x1);
      uint8_t  dy =  abs(y - y1);
      uint8_t dist = 2 * SQRT_VARIANT((dx * dx) + (dy * dy));
      dx =  abs(x - x2);
      dy =  abs(y - y2);
      dist += SQRT_VARIANT((dx * dx) + (dy * dy));
      dx =  abs(x - x3);
      dy =  abs(y - y3);
      dist += SQRT_VARIANT((dx * dx) + (dy * dy));
      // inverse result
      byte color = (dist == 0) ? 255U : 1000U / dist;
      // map color between thresholds
      if (color > 0 && color < 60) {
        if (modes[currentMode].Scale == 100U)
          drawPixelXY(x, y, CHSV(color * 9, 255, 255));// это оригинальный цвет эффекта
        else
          drawPixelXY(x, y, ColorFromPalette(currentPalette, color * 9));
      } else {
        if (modes[currentMode].Scale == 100U)
          drawPixelXY(x, y, CHSV(0, 255, 255)); // в оригинале центральный глаз почему-то красный
        else
          drawPixelXY(x, y, ColorFromPalette(currentPalette, 0U));
      }
      // show the 3 points, too
      drawPixelXY(x1, y1, CRGB(255, 255, 255));
      drawPixelXY(x2, y2, CRGB(255, 255, 255));
      drawPixelXY(x3, y3, CRGB(255, 255, 255));
    }
  }
}

// ============= Эффект Кипение ===============
// (c) SottNick
//по мотивам LDIRKO Ленд - эффект номер 10
//...ldir... Yaroslaw Turbin, 18.11.2020
//https://vk.com/ldirko
//https://www.reddit.com/user/ldirko/
void LLandRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    palette_number = getEffectScaleParamValue2(MC_LLAND);
    if (palette_number == 0 || palette_number == 56) startnum = random8(1, 56); //Если Случайный выбор или Авто, задать произвольный вариант (в Авто от него начинается отсчет)
    else if (palette_number > 0 || palette_number < 56) startnum = palette_number;  //Если что-то из вариантов 1-55, берем только это значение
    deltaValue = 10U * ((modes[currentMode].Scale - 1U) % 11U + 1U);// значения от 1 до 11
  }
  if (palette_number == 56) {  //автоперебор вариантов, если выбран вариант Авто, дается 10 сек на эффект
    if (millis() - color_timer > 10000) {
      color_timer = millis();
      startnum++;
      if (startnum > 55) startnum = 1;
    }
  }
  switch (startnum) {
    case 1:  currentPalette = CloudColors_p;         break;
    case 2:  currentPalette = LavaColors_p;          break;
    case 3:  currentPalette = PartyColors_p;         break;
    case 4:  currentPalette = RainbowColors_p;       break;
    case 5:  currentPalette = RainbowStripeColors_p; break;
    case 6:  currentPalette = ForestColors_p;        break;
    case 7:  currentPalette = AlcoholFireColors_p;   break;
    case 8:  currentPalette = HeatColors_p;          break;
    case 9:  currentPalette = WaterfallColors4in1_p; break;
    case 10: currentPalette = WoodFireColors_p;      break;
    case 11: currentPalette = NormalFire_p;          break;
    case 12: currentPalette = NormalFire2_p;         break;
    case 13: currentPalette = LithiumFireColors_p;   break;
    case 14: currentPalette = SodiumFireColors_p;    break;
    case 15: currentPalette = CopperFireColors_p;    break;
    case 16: currentPalette = RubidiumFireColors_p;  break;
    case 17: currentPalette = PotassiumFireColors_p; break;
    case 18: currentPalette = OceanColors_p;         break;
    case 19: currentPalette = Sunset_Real_gp;        break;
    case 20: currentPalette = dkbluered_gp;          break;
    case 21: currentPalette = Optimus_Prime_gp;      break;
    case 22: currentPalette = warmGrad_gp;           break;
    case 23: currentPalette = coldGrad_gp;           break;
    case 24: currentPalette = hotGrad_gp;            break;
    case 25: currentPalette = pinkGrad_gp;           break;
    case 26: currentPalette = comfy_gp;              break;
    case 27: currentPalette = cyperpunk_gp;          break;
    case 28: currentPalette = girl_gp;               break;
    case 29: currentPalette = xmas_gp;               break;
    case 30: currentPalette = acid_gp;               break;
    case 31: currentPalette = blueSmoke_gp;          break;
    case 32: currentPalette = gummy_gp;              break;
    case 33: currentPalette = aurora_gp;             break;
    case 34: currentPalette = redwhite_gp;           break;
    case 35: currentPalette = ib_jul01_gp;           break;
    case 36: currentPalette = rgi_15_gp;             break;
    case 37: currentPalette = retro2_16_gp;          break;
    case 38: currentPalette = Analogous_1_gp;        break;
    case 39: currentPalette = pinksplash_08_gp;      break;
    case 40: currentPalette = pinksplash_07_gp;      break;
    case 41: currentPalette = Coral_reef_gp;         break;
    case 42: currentPalette = ocean_breeze_gp;       break;
    case 43: currentPalette = landscape_64_gp;       break;
    case 44: currentPalette = landscape_33_gp;       break;
    case 45: currentPalette = rainbowsherbet_gp;     break;
    case 46: currentPalette = gr65_hult_gp;          break;
    case 47: currentPalette = GMT_drywet_gp;         break;
    case 48: currentPalette = emerald_dragon_gp;     break;
    case 49: currentPalette = Colorfull_gp;          break;
    case 50: currentPalette = Pink_Purple_gp;        break;
    case 51: currentPalette = autumn_19_gp;          break;
    case 52: currentPalette = daybreak_gp;           break;
    case 53: currentPalette = Blue_Cyan_Yellow_gp;   break;
    case 54: currentPalette = bhw1_28_gp;            break;
    case 55: currentPalette = rbw_gp;                break;
  }
  hue2 += 32U;
  if (hue2 < 32U)
    hue++;
  ff_y += 16U;
  for (uint8_t y = 0; y < pHEIGHT; y++)
    for (uint16_t x = 0; x < pWIDTH; x++)
      drawPixelXY(x, y, ColorFromPalette (currentPalette, map(inoise8(x * deltaValue, y * deltaValue - ff_y, ff_z) - y * 255 / (pHEIGHT - 1), 0, 255, 205, 255) + hue, 255));
  ff_z++;
}

// ============= Эффект Реакция Белоусова-Жаботинского (Осциллятор) ===============
// по наводке https://www.wikiwand.com/ru/%D0%9A%D0%BB%D0%B5%D1%82%D0%BE%D1%87%D0%BD%D1%8B%D0%B9_%D0%B0%D0%B2%D1%82%D0%BE%D0%BC%D0%B0%D1%82
// (c) SottNick
void drawPixelXYFseamless(float x, float y, CRGB color) {
  uint8_t xx = (x - (int)x) * 255, yy = (y - (int)y) * 255, ix = 255 - xx, iy = 255 - yy;
  // calculate the intensities for each affected pixel
#define WU_WEIGHT(a,b) ((uint8_t) (((a)*(b)+(a)+(b))>>8))
  uint8_t wu[4] = {WU_WEIGHT(ix, iy), WU_WEIGHT(xx, iy),
                   WU_WEIGHT(ix, yy), WU_WEIGHT(xx, yy)
                  };
  // multiply the intensities by the colour, and saturating-add them to the pixels
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t xn = (int8_t)(x + (i & 1)) % pWIDTH;
    uint8_t yn = (int8_t)(y + ((i >> 1) & 1)) % pHEIGHT;
    CRGB clr = getPixColorXY(xn, yn);
    clr.r = qadd8(clr.r, (color.r * wu[i]) >> 8);
    clr.g = qadd8(clr.g, (color.g * wu[i]) >> 8);
    clr.b = qadd8(clr.b, (color.b * wu[i]) >> 8);
    drawPixelXY(xn, yn, clr);
  }
}

uint8_t calcNeighbours(uint8_t x, uint8_t y, uint8_t n) {
  return (noise_3d[0][(x + 1) % pWIDTH][y] == n) +
         (noise_3d[0][x][(y + 1) % pHEIGHT] == n) +
         (noise_3d[0][(x + pWIDTH - 1) % pWIDTH][y] == n) +
         (noise_3d[0][x][(y + pHEIGHT - 1) % pHEIGHT] == n) +
         (noise_3d[0][(x + 1) % pWIDTH][(y + 1) % pHEIGHT] == n) +
         (noise_3d[0][(x + pWIDTH - 1) % pWIDTH][(y + 1) % pHEIGHT] == n) +
         (noise_3d[0][(x + pWIDTH - 1) % pWIDTH][(y + pHEIGHT - 1) % pHEIGHT] == n) +
         (noise_3d[0][(x + 1) % pWIDTH][(y + pHEIGHT - 1) % pHEIGHT] == n);
}
void oscillatingRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    step = 0U;
    if (modes[currentMode].Scale > 100U) modes[currentMode].Scale = 100U; // чтобы не было проблем при прошивке без очистки памяти
    palette_number = getEffectScaleParamValue2(MC_OSCILLATING);
    if (palette_number == 0 || palette_number == 56) startnum = random8(1, 56); //Если Случайный выбор или Авто, задать произвольный вариант (в Авто от него начинается отсчет)
    else if (palette_number > 0 || palette_number < 56) startnum = palette_number;  //Если что-то из вариантов 1-55, берем только это значение
    //случайное заполнение
    for (uint8_t i = 0; i < pWIDTH; i++) {
      for (uint8_t j = 0; j < pHEIGHT; j++) {
        noise_3d[1][i][j] = random8(3);
        noise_3d[0][i][j] = noise_3d[1][i][j];
      }
    }
  }
  if (palette_number == 56) {  //автоперебор вариантов, если выбран вариант Авто, дается 10 сек на эффект
    if (millis() - color_timer > 10000) {
      color_timer = millis();
      startnum++;
      if (startnum > 55) startnum = 1;
    }
  }
  switch (startnum) {
    case 1:  currentPalette = CloudColors_p;         break;
    case 2:  currentPalette = LavaColors_p;          break;
    case 3:  currentPalette = PartyColors_p;         break;
    case 4:  currentPalette = RainbowColors_p;       break;
    case 5:  currentPalette = RainbowStripeColors_p; break;
    case 6:  currentPalette = ForestColors_p;        break;
    case 7:  currentPalette = AlcoholFireColors_p;   break;
    case 8:  currentPalette = HeatColors_p;          break;
    case 9:  currentPalette = WaterfallColors4in1_p; break;
    case 10: currentPalette = WoodFireColors_p;      break;
    case 11: currentPalette = NormalFire_p;          break;
    case 12: currentPalette = NormalFire2_p;         break;
    case 13: currentPalette = LithiumFireColors_p;   break;
    case 14: currentPalette = SodiumFireColors_p;    break;
    case 15: currentPalette = CopperFireColors_p;    break;
    case 16: currentPalette = RubidiumFireColors_p;  break;
    case 17: currentPalette = PotassiumFireColors_p; break;
    case 18: currentPalette = OceanColors_p;         break;
    case 19: currentPalette = Sunset_Real_gp;        break;
    case 20: currentPalette = dkbluered_gp;          break;
    case 21: currentPalette = Optimus_Prime_gp;      break;
    case 22: currentPalette = warmGrad_gp;           break;
    case 23: currentPalette = coldGrad_gp;           break;
    case 24: currentPalette = hotGrad_gp;            break;
    case 25: currentPalette = pinkGrad_gp;           break;
    case 26: currentPalette = comfy_gp;              break;
    case 27: currentPalette = cyperpunk_gp;          break;
    case 28: currentPalette = girl_gp;               break;
    case 29: currentPalette = xmas_gp;               break;
    case 30: currentPalette = acid_gp;               break;
    case 31: currentPalette = blueSmoke_gp;          break;
    case 32: currentPalette = gummy_gp;              break;
    case 33: currentPalette = aurora_gp;             break;
    case 34: currentPalette = redwhite_gp;           break;
    case 35: currentPalette = ib_jul01_gp;           break;
    case 36: currentPalette = rgi_15_gp;             break;
    case 37: currentPalette = retro2_16_gp;          break;
    case 38: currentPalette = Analogous_1_gp;        break;
    case 39: currentPalette = pinksplash_08_gp;      break;
    case 40: currentPalette = pinksplash_07_gp;      break;
    case 41: currentPalette = Coral_reef_gp;         break;
    case 42: currentPalette = ocean_breeze_gp;       break;
    case 43: currentPalette = landscape_64_gp;       break;
    case 44: currentPalette = landscape_33_gp;       break;
    case 45: currentPalette = rainbowsherbet_gp;     break;
    case 46: currentPalette = gr65_hult_gp;          break;
    case 47: currentPalette = GMT_drywet_gp;         break;
    case 48: currentPalette = emerald_dragon_gp;     break;
    case 49: currentPalette = Colorfull_gp;          break;
    case 50: currentPalette = Pink_Purple_gp;        break;
    case 51: currentPalette = autumn_19_gp;          break;
    case 52: currentPalette = daybreak_gp;           break;
    case 53: currentPalette = Blue_Cyan_Yellow_gp;   break;
    case 54: currentPalette = bhw1_28_gp;            break;
    case 55: currentPalette = rbw_gp;                break;
  }
  hue++;
  CRGB currColors[3];
  if (modes[currentMode].Scale == 100U) {
    currColors[0U] = CHSV(hue, 255U, 255U);
    currColors[1U] = CHSV(hue, 128U, 255U);
    currColors[2U] = CHSV(hue, 255U, 128U);
  }
  else if (modes[currentMode].Scale > 50U) {
    currColors[0U] = CHSV((modes[currentMode].Scale - 50U) * 5.1, 255U, 255U);
    currColors[1U] = CHSV((modes[currentMode].Scale - 50U) * 5.1, 128U, 255U);
    currColors[2U] = CHSV((modes[currentMode].Scale - 50U) * 5.1, 255U, 128U);
  }
  else
    for (uint8_t c = 0; c < 3; c++) currColors[c] = ColorFromPalette(currentPalette, c * 85U + hue);
  FastLED.clear();
  // расчёт химической реакции и отрисовка мира
  uint16_t colorCount[3] = {0U, 0U, 0U};
  for (uint8_t x = 0; x < pWIDTH; x++) {
    for (uint8_t y = 0; y < pHEIGHT; y++) {
      if (noise_3d[0][x][y] == 0U) {
        colorCount[0U]++;
        if (calcNeighbours(x, y, 1U) > 2U) noise_3d[1][x][y] = 1U;
      }
      else if (noise_3d[0][x][y] == 1U) {
        colorCount[1U]++;
        if (calcNeighbours(x, y, 2U) > 2U) noise_3d[1][x][y] = 2U;
      }
      else {                       //if (noise_3d[0][x][y] == 2U){
        colorCount[2U]++;
        if (calcNeighbours(x, y, 0U) > 2U) noise_3d[1][x][y] = 0U;
      }
      drawPixelXYFseamless((float)x + 0.5, (float)y + 0.5, currColors[noise_3d[1][x][y]]);
    }
  }
  // проверка зацикливания
  if (colorCount[0] == deltaHue && colorCount[1] == deltaHue2 && colorCount[2] == deltaValue) {
    step++;
    if (step > 10U) {
      if (colorCount[0] < colorCount[1])
        step = 0;
      else
        step = 1;
      if (colorCount[2] < colorCount[step])
        step = 2;
      colorCount[step] = 0U;
      step = 0U;
    }
  }
  else step = 0U;
  // вброс хаоса
  if (hue == hue2) { // чтобы не каждый ход
    hue2 += random8(220U) + 36U;
    uint8_t tx = random8(pWIDTH);
    deltaHue = noise_3d[1][tx][0U] + 1U;
    if (deltaHue > 2U) deltaHue = 0U;
    noise_3d[1][tx][0U] = deltaHue;
    noise_3d[1][(tx + 1U) % pWIDTH][0U] = deltaHue;
    noise_3d[1][(tx + 2U) % pWIDTH][0U] = deltaHue;
  }
  deltaHue = colorCount[0];
  deltaHue2 = colorCount[1];
  deltaValue = colorCount[2];
  // вброс исчезнувшего цвета
  for (uint8_t c = 0; c < 3; c++) {
    if (colorCount[c] < 6U) {
      uint8_t tx = random8(pWIDTH);
      uint8_t ty = random8(pHEIGHT);
      if (random8(2U)) {
        noise_3d[1][tx][ty] = c;
        noise_3d[1][(tx + 1U) % pWIDTH][ty] = c;
        noise_3d[1][(tx + 2U) % pWIDTH][ty] = c;
      }
      else {
        noise_3d[1][tx][ty] = c;
        noise_3d[1][tx][(ty + 1U) % pHEIGHT] = c;
        noise_3d[1][tx][(ty + 2U) % pHEIGHT] = c;
      }
    }
  }
  // перенос на следующий цикл
  for (uint8_t x = 0; x < pWIDTH; x++) {
    for (uint8_t y = 0; y < pHEIGHT; y++) {
      noise_3d[0][x][y] = noise_3d[1][x][y];
    }
  }
}

// ----------- Эффект "Шары"
// (c) stepko and kostyamat https://wokwi.com/arduino/projects/289839434049782281
// 07.02.2021
float randomf(float min, float max) {
  return fmap((float)random16(4095), 0.0, 4095.0, min, max);
}
void ballsfill_circle(float cx, float cy, float radius, CRGB col) {
  radius -= 0.5;
  for (int y = -radius; y <= radius; y++) {
    for (int x = -radius; x <= radius; x++) {
      if (x * x + y * y <= radius * radius)
        drawPixelXYF(cx + x, cy + y, col);
    }
  }
}

void spheresRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    palette_number = getEffectScaleParamValue2(MC_SPHERES);
    if (palette_number == 0 || palette_number == 56) startnum = random8(1, 56); //Если Случайный выбор или Авто, задать произвольный вариант (в Авто от него начинается отсчет)
    else if (palette_number > 0 || palette_number < 56) startnum = palette_number;  //Если что-то из вариантов 1-55, берем только это значение
    speedfactor = fmap(modes[currentMode].Speed, 1, 255, 0.15, 0.5);
    enlargedObjectNUM = (modes[currentMode].Scale - 1U) % 11U + 1U;
    emitterY = .5 + pHEIGHT / 4. / (2. - 1. / enlargedObjectNUM); // radiusMax
    for (uint8_t i = 0; i < enlargedObjectNUM; i++) {
      trackingObjectShift[i] = randomf(0.5, emitterY); // radius[i] = randomf(0.5, radiusMax);
      trackingObjectSpeedX[i] = randomf(0.5, 1.1) * speedfactor; // ball[i][2] =
      trackingObjectSpeedY[i] = randomf(0.5, 1.1) * speedfactor; // ball[i][3] =
      trackingObjectPosX[i] = random8(pWIDTH);  // ball[i][0] = random(0, pWIDTH);
      trackingObjectPosY[i] = random8(pHEIGHT); // ball[i][1] = random(0, pHEIGHT);
      trackingObjectHue[i] = random8();        // color[i] = random(0, 255);
    }
  }
  if (palette_number == 56) {  //автоперебор вариантов, если выбран вариант Авто, дается 10 сек на эффект
    if (millis() - color_timer > 10000) {
      color_timer = millis();
      startnum++;
      if (startnum > 55) startnum = 1;
    }
  }
  switch (startnum) {
    case 1:  currentPalette = CloudColors_p;         break;
    case 2:  currentPalette = LavaColors_p;          break;
    case 3:  currentPalette = PartyColors_p;         break;
    case 4:  currentPalette = RainbowColors_p;       break;
    case 5:  currentPalette = RainbowStripeColors_p; break;
    case 6:  currentPalette = ForestColors_p;        break;
    case 7:  currentPalette = AlcoholFireColors_p;   break;
    case 8:  currentPalette = HeatColors_p;          break;
    case 9:  currentPalette = WaterfallColors4in1_p; break;
    case 10: currentPalette = WoodFireColors_p;      break;
    case 11: currentPalette = NormalFire_p;          break;
    case 12: currentPalette = NormalFire2_p;         break;
    case 13: currentPalette = LithiumFireColors_p;   break;
    case 14: currentPalette = SodiumFireColors_p;    break;
    case 15: currentPalette = CopperFireColors_p;    break;
    case 16: currentPalette = RubidiumFireColors_p;  break;
    case 17: currentPalette = PotassiumFireColors_p; break;
    case 18: currentPalette = OceanColors_p;         break;
    case 19: currentPalette = Sunset_Real_gp;        break;
    case 20: currentPalette = dkbluered_gp;          break;
    case 21: currentPalette = Optimus_Prime_gp;      break;
    case 22: currentPalette = warmGrad_gp;           break;
    case 23: currentPalette = coldGrad_gp;           break;
    case 24: currentPalette = hotGrad_gp;            break;
    case 25: currentPalette = pinkGrad_gp;           break;
    case 26: currentPalette = comfy_gp;              break;
    case 27: currentPalette = cyperpunk_gp;          break;
    case 28: currentPalette = girl_gp;               break;
    case 29: currentPalette = xmas_gp;               break;
    case 30: currentPalette = acid_gp;               break;
    case 31: currentPalette = blueSmoke_gp;          break;
    case 32: currentPalette = gummy_gp;              break;
    case 33: currentPalette = aurora_gp;             break;
    case 34: currentPalette = redwhite_gp;           break;
    case 35: currentPalette = ib_jul01_gp;           break;
    case 36: currentPalette = rgi_15_gp;             break;
    case 37: currentPalette = retro2_16_gp;          break;
    case 38: currentPalette = Analogous_1_gp;        break;
    case 39: currentPalette = pinksplash_08_gp;      break;
    case 40: currentPalette = pinksplash_07_gp;      break;
    case 41: currentPalette = Coral_reef_gp;         break;
    case 42: currentPalette = ocean_breeze_gp;       break;
    case 43: currentPalette = landscape_64_gp;       break;
    case 44: currentPalette = landscape_33_gp;       break;
    case 45: currentPalette = rainbowsherbet_gp;     break;
    case 46: currentPalette = gr65_hult_gp;          break;
    case 47: currentPalette = GMT_drywet_gp;         break;
    case 48: currentPalette = emerald_dragon_gp;     break;
    case 49: currentPalette = Colorfull_gp;          break;
    case 50: currentPalette = Pink_Purple_gp;        break;
    case 51: currentPalette = autumn_19_gp;          break;
    case 52: currentPalette = daybreak_gp;           break;
    case 53: currentPalette = Blue_Cyan_Yellow_gp;   break;
    case 54: currentPalette = bhw1_28_gp;            break;
    case 55: currentPalette = rbw_gp;                break;
  }
  dimAll(255 - map(modes[currentMode].Speed, 1, 255, 5, 20)); //fadeToBlackBy(leds, NUM_LEDS, map(speed, 1, 255, 5, 20));
  for (byte i = 0; i < enlargedObjectNUM; i++) {
    if (trackingObjectIsShift[i]) {  // тут у нас шарики надуваются\сдуваются по ходу движения
      trackingObjectShift[i] += (fabs(trackingObjectSpeedX[i]) > fabs(trackingObjectSpeedY[i]) ? fabs(trackingObjectSpeedX[i]) : fabs(trackingObjectSpeedY[i])) * 0.1 * speedfactor;
      if (trackingObjectShift[i] >= emitterY) {
        trackingObjectIsShift[i] = false;
      }
    }
    else {
      trackingObjectShift[i] -= (fabs(trackingObjectSpeedX[i]) > fabs(trackingObjectSpeedY[i]) ? fabs(trackingObjectSpeedX[i]) : fabs(trackingObjectSpeedY[i])) * 0.1 * speedfactor;
      if (trackingObjectShift[i] < 1.) {
        trackingObjectIsShift[i] = true;
        trackingObjectHue[i] = random(0, 255);
      }
    }
    if (trackingObjectShift[i] > 1)
      ballsfill_circle(trackingObjectPosY[i], trackingObjectPosX[i], trackingObjectShift[i], ColorFromPalette(currentPalette, trackingObjectHue[i]));
    else
      drawPixelXYF(trackingObjectPosY[i], trackingObjectPosX[i], ColorFromPalette(currentPalette, trackingObjectHue[i]));
    if (trackingObjectPosX[i] + trackingObjectShift[i] >= pHEIGHT - 1)
      trackingObjectPosX[i] += (trackingObjectSpeedX[i] * ((pHEIGHT - 1 - trackingObjectPosX[i]) / trackingObjectShift[i] + 0.005));
    else if (trackingObjectPosX[i] - trackingObjectShift[i] <= 0)
      trackingObjectPosX[i] += (trackingObjectSpeedX[i] * (trackingObjectPosX[i] / trackingObjectShift[i] + 0.005));
    else
      trackingObjectPosX[i] += trackingObjectSpeedX[i];
    //-----------------------
    if (trackingObjectPosY[i] + trackingObjectShift[i] >= pWIDTH - 1)
      trackingObjectPosY[i] += (trackingObjectSpeedY[i] * ((pWIDTH - 1 - trackingObjectPosY[i]) / trackingObjectShift[i] + 0.005));
    else if (trackingObjectPosY[i] - trackingObjectShift[i] <= 0)
      trackingObjectPosY[i] += (trackingObjectSpeedY[i] * (trackingObjectPosY[i] / trackingObjectShift[i] + 0.005));
    else
      trackingObjectPosY[i] += trackingObjectSpeedY[i];
    //------------------------
    if (trackingObjectPosX[i] < 0.01) {
      trackingObjectSpeedX[i] = randomf(0.5, 1.1) * speedfactor;
      trackingObjectPosX[i] = 0.01;
    }
    else if (trackingObjectPosX[i] > pHEIGHT - 1.01) {
      trackingObjectSpeedX[i] = randomf(0.5, 1.1) * speedfactor;
      trackingObjectSpeedX[i] = -trackingObjectSpeedX[i];
      trackingObjectPosX[i] = pHEIGHT - 1.01;
    }
    //----------------------
    if (trackingObjectPosY[i] < 0.01) {
      trackingObjectSpeedY[i] = randomf(0.5, 1.1) * speedfactor;
      trackingObjectPosY[i] = 0.01;
    }
    else if (trackingObjectPosY[i] > pWIDTH - 1.01) {
      trackingObjectSpeedY[i] = randomf(0.5, 1.1) * speedfactor;
      trackingObjectSpeedY[i] = -trackingObjectSpeedY[i];
      trackingObjectPosY[i] = pWIDTH - 1.01;
    }
  }
  blurScreen(48);
}

// ------------------------------ ЭФФЕКТ КУБИК РУБИКА 2D ----------------------
// (c) SottNick
#define PAUSE_MAX 7 // пропустить 7 кадров после завершения анимации сдвига ячеек
uint8_t razmerX, razmerY; // размеры ячеек по горизонтали / вертикали
uint8_t shtukX, shtukY; // количество ячеек по горизонтали / вертикали
uint8_t poleX, poleY; // размер всего поля по горизонтали / вертикали (в том числе 1 дополнительная пустая дорожка-разделитель с какой-то из сторон)
int8_t globalShiftX, globalShiftY; // нужно ли сдвинуть всё поле по окончаии цикла и в каком из направлений (-1, 0, +1)
bool seamlessX; // получилось ли сделать поле по Х бесшовным
bool krutimVertikalno; // направление вращения в данный момент
void cube2dRoutine() {
  uint8_t x, y;
  uint8_t anim0; // будем считать тут начальный пиксель для анимации сдвига строки/колонки
  int8_t shift, kudaVse; // какое-то расчётное направление сдвига (-1, 0, +1)
  CRGB color, color2;
  if (loadingFlag) {
    loadingFlag = false;
    FastLED.clear();
    startnum = random8(1, 56); //Задаем произвольно палитру при запуске эффекта
    switch (startnum) {
      case 1:  currentPalette = CloudColors_p;         break;
      case 2:  currentPalette = LavaColors_p;          break;
      case 3:  currentPalette = PartyColors_p;         break;
      case 4:  currentPalette = RainbowColors_p;       break;
      case 5:  currentPalette = RainbowStripeColors_p; break;
      case 6:  currentPalette = ForestColors_p;        break;
      case 7:  currentPalette = AlcoholFireColors_p;   break;
      case 8:  currentPalette = HeatColors_p;          break;
      case 9:  currentPalette = WaterfallColors4in1_p; break;
      case 10: currentPalette = WoodFireColors_p;      break;
      case 11: currentPalette = NormalFire_p;          break;
      case 12: currentPalette = NormalFire2_p;         break;
      case 13: currentPalette = LithiumFireColors_p;   break;
      case 14: currentPalette = SodiumFireColors_p;    break;
      case 15: currentPalette = CopperFireColors_p;    break;
      case 16: currentPalette = RubidiumFireColors_p;  break;
      case 17: currentPalette = PotassiumFireColors_p; break;
      case 18: currentPalette = OceanColors_p;         break;
      case 19: currentPalette = Sunset_Real_gp;        break;
      case 20: currentPalette = dkbluered_gp;          break;
      case 21: currentPalette = Optimus_Prime_gp;      break;
      case 22: currentPalette = warmGrad_gp;           break;
      case 23: currentPalette = coldGrad_gp;           break;
      case 24: currentPalette = hotGrad_gp;            break;
      case 25: currentPalette = pinkGrad_gp;           break;
      case 26: currentPalette = comfy_gp;              break;
      case 27: currentPalette = cyperpunk_gp;          break;
      case 28: currentPalette = girl_gp;               break;
      case 29: currentPalette = xmas_gp;               break;
      case 30: currentPalette = acid_gp;               break;
      case 31: currentPalette = blueSmoke_gp;          break;
      case 32: currentPalette = gummy_gp;              break;
      case 33: currentPalette = aurora_gp;             break;
      case 34: currentPalette = redwhite_gp;           break;
      case 35: currentPalette = ib_jul01_gp;           break;
      case 36: currentPalette = rgi_15_gp;             break;
      case 37: currentPalette = retro2_16_gp;          break;
      case 38: currentPalette = Analogous_1_gp;        break;
      case 39: currentPalette = pinksplash_08_gp;      break;
      case 40: currentPalette = pinksplash_07_gp;      break;
      case 41: currentPalette = Coral_reef_gp;         break;
      case 42: currentPalette = ocean_breeze_gp;       break;
      case 43: currentPalette = landscape_64_gp;       break;
      case 44: currentPalette = landscape_33_gp;       break;
      case 45: currentPalette = rainbowsherbet_gp;     break;
      case 46: currentPalette = gr65_hult_gp;          break;
      case 47: currentPalette = GMT_drywet_gp;         break;
      case 48: currentPalette = emerald_dragon_gp;     break;
      case 49: currentPalette = Colorfull_gp;          break;
      case 50: currentPalette = Pink_Purple_gp;        break;
      case 51: currentPalette = autumn_19_gp;          break;
      case 52: currentPalette = daybreak_gp;           break;
      case 53: currentPalette = Blue_Cyan_Yellow_gp;   break;
      case 54: currentPalette = bhw1_28_gp;            break;
      case 55: currentPalette = rbw_gp;                break;
    }
    razmerX = (modes[currentMode].Scale - 1U) % 11U + 1U; // размер ячейки от 1 до 11 пикселей для каждой из 9 палитр
    razmerY = razmerX;
    if (modes[currentMode].Speed & 0x01) razmerY = (razmerY << 1U) + 1U;// по идее, ячейки не обязательно должны быть квадратными, поэтому можно тут поизвращаться
    shtukY = pHEIGHT / (razmerY + 1U);
    if (shtukY < 2U) shtukY = 2U;
    y = pHEIGHT / shtukY - 1U;
    if (razmerY > y) razmerY = y;
    poleY = (razmerY + 1U) * shtukY;
    shtukX = pWIDTH / (razmerX + 1U);
    if (shtukX < 2U) shtukX = 2U;
    x = pWIDTH / shtukX - 1U;
    if (razmerX > x)  razmerX = x;
    poleX = (razmerX + 1U) * shtukX;
    seamlessX = (poleX == pWIDTH);
    deltaHue = 0U;
    deltaHue2 = 0U;
    globalShiftX = 0;
    globalShiftY = 0;
    for (uint8_t j = 0U; j < shtukY; j++) {
      y = j * (razmerY + 1U); // + deltaHue2 т.к. оно =0U
      for (uint8_t i = 0U; i < shtukX; i++) {
        x = i * (razmerX + 1U); // + deltaHue т.к. оно =0U
        if (modes[currentMode].Scale == 100U) color = CHSV(45U, 0U, 128U + random8(128U));
        else color = ColorFromPalette(currentPalette, random8());
        for (uint8_t k = 0U; k < razmerY; k++)
          for (uint8_t m = 0U; m < razmerX; m++)
            leds[XY(x + m, y + k)] = color;
      }
    }
    step = 4U; // текущий шаг сдвига первоначально с перебором (от 0 до deltaValue-1)
    deltaValue = 4U; // всего шагов сдвига (от razmer? до (razmer?+1) * shtuk?)
    hue2 = 0U; // осталось шагов паузы
  }
  //двигаем, что получилось...
  if (hue2 == 0 && step < deltaValue) {  // если пауза закончилась, а цикл вращения ещё не завершён
    step++;
    if (krutimVertikalno) {
      for (uint8_t i = 0U; i < shtukX; i++) {
        x = (deltaHue + i * (razmerX + 1U)) % pWIDTH;
        if (noise_3d[0][i][0] > 0) {  // в нулевой ячейке храним оставшееся количество ходов прокрутки
          noise_3d[0][i][0]--;
          shift = noise_3d[0][i][1] - 1; // в первой ячейке храним направление прокрутки
          if (globalShiftY == 0) anim0 = (deltaHue2 == 0U) ? 0U : deltaHue2 - 1U;
          else if (globalShiftY > 0) anim0 = deltaHue2;
          else anim0 = deltaHue2 - 1U;
          if (shift < 0) {  // если крутим столбец вниз
            color = leds[XY(x, anim0)];                                   // берём цвет от нижней строчки
            for (uint8_t k = anim0; k < anim0 + poleY - 1; k++) {
              color2 = leds[XY(x, k + 1)];                                // берём цвет от строчки над нашей
              for (uint8_t m = x; m < x + razmerX; m++)
                leds[XY(m % pWIDTH, k)] = color2;                          // копируем его на всю нашу строку
            }
            for (uint8_t m = x; m < x + razmerX; m++)
              leds[XY(m % pWIDTH, anim0 + poleY - 1)] = color;            // цвет нижней строчки копируем на всю верхнюю
          }
          else if (shift > 0) {                                           // если крутим столбец вверх
            color = leds[XY(x, anim0 + poleY - 1)];                       // берём цвет от верхней строчки
            for (uint8_t k = anim0 + poleY - 1; k > anim0 ; k--) {
              color2 = leds[XY(x, k - 1)];                                // берём цвет от строчки под нашей
              for (uint8_t m = x; m < x + razmerX; m++)
                leds[XY(m % pWIDTH, k)] = color2;                          // копируем его на всю нашу строку
            }
            for   (uint8_t m = x; m < x + razmerX; m++)
              leds[XY(m % pWIDTH, anim0)] = color;                         // цвет верхней строчки копируем на всю нижнюю
          }
        }
      }
    }
    else {
      for (uint8_t j = 0U; j < shtukY; j++) {
        y = deltaHue2 + j * (razmerY + 1U);
        if (noise_3d[0][0][j] > 0) {                               // в нулевой ячейке храним оставшееся количество ходов прокрутки
          noise_3d[0][0][j]--;
          shift = noise_3d[0][1][j] - 1;                           // в первой ячейке храним направление прокрутки
          if (seamlessX) anim0 = 0U;
          else if (globalShiftX == 0) anim0 = (deltaHue == 0U) ? 0U : deltaHue - 1U;
          else if (globalShiftX > 0) anim0 = deltaHue;
          else anim0 = deltaHue - 1U;
          if (shift < 0) {                                         // если крутим строку влево
            color = leds[XY(anim0, y)];                            // берём цвет от левой колонки (левого пикселя)
            for (uint8_t k = anim0; k < anim0 + poleX - 1; k++) {
              color2 = leds[XY(k + 1, y)];                         // берём цвет от колонки (пикселя) правее
              for (uint8_t m = y; m < y + razmerY; m++)
                leds[XY(k, m)] = color2;                           // копируем его на всю нашу колонку
            }
            for (uint8_t m = y; m < y + razmerY; m++)
              leds[XY(anim0 + poleX - 1, m)] = color;              // цвет левой колонки копируем на всю правую
          }
          else if (shift > 0) {                                    // если крутим столбец вверх
            color = leds[XY(anim0 + poleX - 1, y)];                // берём цвет от правой колонки
            for (uint8_t k = anim0 + poleX - 1; k > anim0 ; k--) {
              color2 = leds[XY(k - 1, y)];                         // берём цвет от колонки левее
              for (uint8_t m = y; m < y + razmerY; m++)
                leds[XY(k, m)] = color2;                           // копируем его на всю нашу колонку
            }
            for (uint8_t m = y; m < y + razmerY; m++)
              leds[XY(anim0, m)] = color;                          // цвет правой колонки копируем на всю левую
          }
        }
      }
    }
  }
  else if (hue2 != 0U) hue2--;                                     // пропускаем кадры после прокрутки кубика (делаем паузу)
  if (step >= deltaValue) {                                        // если цикл вращения завершён, меняем местами соответствующие ячейки (цвет в них) и точку первой ячейки
    step = 0U;
    hue2 = PAUSE_MAX;
    //если часть ячеек двигалась на 1 пиксель, пододвигаем глобальные координаты начала
    deltaHue2 = deltaHue2 + globalShiftY; //+= globalShiftY;
    globalShiftY = 0;
    deltaHue = (pWIDTH + deltaHue + globalShiftX) % pWIDTH;
    globalShiftX = 0;
    //пришла пора выбрать следующие параметры вращения
    kudaVse = 0;
    krutimVertikalno = random8(2U);
    if (krutimVertikalno) {                                        // идём по горизонтали, крутим по вертикали (столбцы двигаются)
      for (uint8_t i = 0U; i < shtukX; i++) {
        noise_3d[0][i][1] = random8(3);
        shift = noise_3d[0][i][1] - 1;                             // в первой ячейке храним направление прокрутки
        if (kudaVse == 0) kudaVse = shift;
        else if (shift != 0 && kudaVse != shift) kudaVse = 50;
      }
      deltaValue = razmerY + ((deltaHue2 - kudaVse >= 0 && deltaHue2 - kudaVse + poleY < (int)pHEIGHT) ? random8(2U) : 1U);
      if (deltaValue == razmerY) {                                 // значит полюбому kudaVse было = (-1, 0, +1) - и для нуля в том числе мы двигаем весь куб на 1 пиксель
        globalShiftY = 1 - kudaVse;                                // временно на единичку больше, чем надо
        for (uint8_t i = 0U; i < shtukX; i++)
          if (noise_3d[0][i][1] == 1U) {                           // если ячейка никуда не планировала двигаться
            noise_3d[0][i][1] = globalShiftY;
            noise_3d[0][i][0] = 1U;                                // в нулевой ячейке храним количество ходов сдвига
          }
          else
            noise_3d[0][i][0] = deltaValue;                        // в нулевой ячейке храним количество ходов сдвига
        globalShiftY--;
      }
      else {
        x = 0;
        for (uint8_t i = 0U; i < shtukX; i++)
          if (noise_3d[0][i][1] != 1U) {
            y = random8(shtukY);
            if (y > x) x = y;
            noise_3d[0][i][0] = deltaValue * (x + 1U);             // в нулевой ячейке храним количество ходов сдвига
          }
        deltaValue = deltaValue * (x + 1U);
      }
    }
    else {                                                         // идём по вертикали, крутим по горизонтали (строки двигаются)
      for (uint8_t j = 0U; j < shtukY; j++) {
        noise_3d[0][1][j] = random8(3);
        shift = noise_3d[0][1][j] - 1;                             // в первой ячейке храним направление прокрутки
        if (kudaVse == 0) kudaVse = shift;
        else if (shift != 0 && kudaVse != shift) kudaVse = 50;
      }
      if (seamlessX) deltaValue = razmerX + ((kudaVse < 50) ? random8(2U) : 1U);
      else deltaValue = razmerX + ((deltaHue - kudaVse >= 0 && deltaHue - kudaVse + poleX < (int)pWIDTH) ? random8(2U) : 1U);
      if (deltaValue == razmerX) {                                 // значит полюбому kudaVse было = (-1, 0, +1) - и для нуля в том числе мы двигаем весь куб на 1 пиксель
        globalShiftX = 1 - kudaVse;                                // временно на единичку больше, чем надо
        for (uint8_t j = 0U; j < shtukY; j++)
          if (noise_3d[0][1][j] == 1U) {                           // если ячейка никуда не планировала двигаться
            noise_3d[0][1][j] = globalShiftX;
            noise_3d[0][0][j] = 1U;                                // в нулевой ячейке храним количество ходов сдвига
          }
          else noise_3d[0][0][j] = deltaValue;                     // в нулевой ячейке храним количество ходов сдвига
        globalShiftX--;
      }
      else {
        y = 0;
        for (uint8_t j = 0U; j < shtukY; j++)
          if (noise_3d[0][1][j] != 1U) {
            x = random8(shtukX);
            if (x > y) y = x;
            noise_3d[0][0][j] = deltaValue * (x + 1U);             // в нулевой ячейке храним количество ходов сдвига
          }
        deltaValue = deltaValue * (y + 1U);
      }
    }
  }
}

// ============= ЭФФЕКТ ВОЛНЫ ===============
// https://github.com/pixelmatix/aurora/blob/master/PatternWave.h
// Адаптация от (c) SottNick
uint32_t wavetimer;
uint8_t wavenum = 0;
void WaveRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    palette_number = map8(getEffectScaleParamValue(MC_WAVES), 0, 56); //выбор палитры; если 0 - случайный выбор, если 56 - запускаем автоперебор
    if (palette_number == 0 || palette_number == 56) startnum = random8(1, 56); //Если Случайный выбор или Авто, задать произвольный вариант (в Авто от него начинается отсчет)
    else if (palette_number > 0 || palette_number < 56) startnum = palette_number;  //Если что-то из вариантов 1-55, берем только это значение
    waveRotation = getEffectScaleParamValue2(MC_WAVES);
    if (waveRotation == 0) waveRotation = random8(1, 5);
    else if (waveRotation > 4) wavenum = random8(1, 5);
    FastLED.clear();  // очистить
    waveCount = 1;    // Самое оно. Только так понятно, что варианты эффектов все же разные. Если больше, все сливается.
  }
  if (palette_number == 56) {  //автоперебор вариантов, если выбран вариант Авто, дается 10 сек на эффект
    if (millis() - color_timer > 10000) {
      color_timer = millis();
      startnum++;
      if (startnum > 55) startnum = 1;
    }
  }
  switch (startnum) {
    case 1:  currentPalette = CloudColors_p;         break;
    case 2:  currentPalette = LavaColors_p;          break;
    case 3:  currentPalette = PartyColors_p;         break;
    case 4:  currentPalette = RainbowColors_p;       break;
    case 5:  currentPalette = RainbowStripeColors_p; break;
    case 6:  currentPalette = ForestColors_p;        break;
    case 7:  currentPalette = AlcoholFireColors_p;   break;
    case 8:  currentPalette = HeatColors_p;          break;
    case 9:  currentPalette = WaterfallColors4in1_p; break;
    case 10: currentPalette = WoodFireColors_p;      break;
    case 11: currentPalette = NormalFire_p;          break;
    case 12: currentPalette = NormalFire2_p;         break;
    case 13: currentPalette = LithiumFireColors_p;   break;
    case 14: currentPalette = SodiumFireColors_p;    break;
    case 15: currentPalette = CopperFireColors_p;    break;
    case 16: currentPalette = RubidiumFireColors_p;  break;
    case 17: currentPalette = PotassiumFireColors_p; break;
    case 18: currentPalette = OceanColors_p;         break;
    case 19: currentPalette = Sunset_Real_gp;        break;
    case 20: currentPalette = dkbluered_gp;          break;
    case 21: currentPalette = Optimus_Prime_gp;      break;
    case 22: currentPalette = warmGrad_gp;           break;
    case 23: currentPalette = coldGrad_gp;           break;
    case 24: currentPalette = hotGrad_gp;            break;
    case 25: currentPalette = pinkGrad_gp;           break;
    case 26: currentPalette = comfy_gp;              break;
    case 27: currentPalette = cyperpunk_gp;          break;
    case 28: currentPalette = girl_gp;               break;
    case 29: currentPalette = xmas_gp;               break;
    case 30: currentPalette = acid_gp;               break;
    case 31: currentPalette = blueSmoke_gp;          break;
    case 32: currentPalette = gummy_gp;              break;
    case 33: currentPalette = aurora_gp;             break;
    case 34: currentPalette = redwhite_gp;           break;
    case 35: currentPalette = ib_jul01_gp;           break;
    case 36: currentPalette = rgi_15_gp;             break;
    case 37: currentPalette = retro2_16_gp;          break;
    case 38: currentPalette = Analogous_1_gp;        break;
    case 39: currentPalette = pinksplash_08_gp;      break;
    case 40: currentPalette = pinksplash_07_gp;      break;
    case 41: currentPalette = Coral_reef_gp;         break;
    case 42: currentPalette = ocean_breeze_gp;       break;
    case 43: currentPalette = landscape_64_gp;       break;
    case 44: currentPalette = landscape_33_gp;       break;
    case 45: currentPalette = rainbowsherbet_gp;     break;
    case 46: currentPalette = gr65_hult_gp;          break;
    case 47: currentPalette = GMT_drywet_gp;         break;
    case 48: currentPalette = emerald_dragon_gp;     break;
    case 49: currentPalette = Colorfull_gp;          break;
    case 50: currentPalette = Pink_Purple_gp;        break;
    case 51: currentPalette = autumn_19_gp;          break;
    case 52: currentPalette = daybreak_gp;           break;
    case 53: currentPalette = Blue_Cyan_Yellow_gp;   break;
    case 54: currentPalette = bhw1_28_gp;            break;
    case 55: currentPalette = rbw_gp;                break;
  }
  dimAll(254);
  int n = 0;
  switch (waveRotation) {
    case 1: waveRotationI(n, currentPalette); break;
    case 2: waveRotationII(n, currentPalette); break;
    case 3: waveRotationIII(n, currentPalette); break;
    case 4: waveRotationIV(n, currentPalette); break;
    case 5: {
        if (millis() - wavetimer > 10000) { //каждые 10 секунд меняем вариант эффекта
          wavetimer = millis();
          wavenum++;
          if (wavenum > 4) wavenum = 1;
        }
        switch (wavenum) {
          case 1:  waveRotationI(n, currentPalette); break;
          case 2:  waveRotationII(n, currentPalette); break;
          case 3:  waveRotationIII(n, currentPalette); break;
          case 4:  waveRotationIV(n, currentPalette); break;
        }
      } break;
  }
  if (waveThetaUpdate >= waveThetaUpdateFrequency) {
    waveThetaUpdate = 0;
    waveTheta++;
  }
  else waveThetaUpdate++;
  if (hueUpdate >= hueUpdateFrequency) {
    hueUpdate = 0;
    hue++;
  }
  else hueUpdate++;
  blurScreen(20); // @Palpalych советует делать размытие. вот в этом эффекте его явно не хватает...
}

//здесь тоже распихал код по функциям
void waveRotationI(int n, CRGBPalette16 pal) {
  for (uint8_t x = 0; x < pWIDTH; x++) {
    n = quadwave8(x * 2 + waveTheta) / waveScale;
    drawPixelXY(x, n, ColorFromPalette(pal, hue + x));
    if (waveCount != 1)
      drawPixelXY(x, pHEIGHT - 1 - n, ColorFromPalette(pal, hue + x));
  }
}

void waveRotationII(int n, CRGBPalette16 pal) {
  for (uint8_t y = 0; y < pHEIGHT; y++) {
    n = quadwave8(y * 2 + waveTheta) / waveScale;
    drawPixelXY(n, y, ColorFromPalette(pal, hue + y));
    if (waveCount != 1) drawPixelXY(pWIDTH - 1 - n, y, ColorFromPalette(pal, hue + y));
  }
}

void waveRotationIII(int n, CRGBPalette16 pal) {
  for (uint8_t x = 0; x < pWIDTH; x++) {
    n = quadwave8(x * 2 - waveTheta) / waveScale;
    drawPixelXY(x, n, ColorFromPalette(pal, hue + x));
    if (waveCount != 1) drawPixelXY(x, pHEIGHT - 1 - n, ColorFromPalette(pal, hue + x));
  }
}

void waveRotationIV(int n, CRGBPalette16 pal) {
  for (uint8_t y = 0; y < pHEIGHT; y++) {
    n = quadwave8(y * 2 - waveTheta) / waveScale;
    drawPixelXY(n, y, ColorFromPalette(pal, hue + y));
    if (waveCount != 1) drawPixelXY(pWIDTH - 1 - n, y, ColorFromPalette(pal, hue + y));
  }
}

// ------------------------------ ЭФФЕКТ КОЛЬЦА / КОДОВЫЙ ЗАМОК ----------------------
// (c) SottNick
// из-за повторного использоваия переменных от других эффектов теперь в этом коде невозможно что-то понять.
// поэтому для понимания придётся сперва заменить названия переменных на человеческие. но всё равно это песец, конечно.
void ringsRoutine() {
  uint8_t h, x, y;
  if (loadingFlag) {
    loadingFlag = false;
    palette_number = getEffectScaleParamValue2(MC_RINGS);
    if (palette_number == 0 || palette_number == 56) startnum = random8(1, 56); //Если Случайный выбор или Авто, задать произвольный вариант (в Авто от него начинается отсчет)
    else if (palette_number > 0 || palette_number < 56) startnum = palette_number;  //Если что-то из вариантов 1-55, берем только это значение
    deltaHue2 = (modes[currentMode].Scale - 1U) % 11U + 1U; // толщина кольца от 1 до 11 для каждой из палитр
    deltaHue = pHEIGHT / deltaHue2 + ((pHEIGHT % deltaHue2 == 0U) ? 0U : 1U); // количество колец
    hue2 = deltaHue2 - (deltaHue2 * deltaHue - pHEIGHT) / 2U; // толщина верхнего кольца. может быть меньше нижнего
    hue = pHEIGHT - hue2 - (deltaHue - 2U) * deltaHue2; // толщина нижнего кольца = всё оставшееся
    for (uint8_t i = 0; i < deltaHue; i++) {
      noise_3d[0][0][i] = random8(257U - pWIDTH / 2U); // начальный оттенок кольца из палитры 0-255 за минусом длины кольца, делённой пополам
      shiftHue[i] = random8();
      shiftValue[i] = 0U; //random8(pWIDTH); само прокрутится постепенно
      step = 0U;
      deltaValue = random8(deltaHue);
    }
  }
  if (palette_number == 56) {  //автоперебор вариантов, если выбран вариант Авто, дается 10 сек на эффект
    if (millis() - color_timer > 10000) {
      color_timer = millis();
      startnum++;
      if (startnum > 55) startnum = 1;
    }
  }
  switch (startnum) {
    case 1:  currentPalette = CloudColors_p;         break;
    case 2:  currentPalette = LavaColors_p;          break;
    case 3:  currentPalette = PartyColors_p;         break;
    case 4:  currentPalette = RainbowColors_p;       break;
    case 5:  currentPalette = RainbowStripeColors_p; break;
    case 6:  currentPalette = ForestColors_p;        break;
    case 7:  currentPalette = AlcoholFireColors_p;   break;
    case 8:  currentPalette = HeatColors_p;          break;
    case 9:  currentPalette = WaterfallColors4in1_p; break;
    case 10: currentPalette = WoodFireColors_p;      break;
    case 11: currentPalette = NormalFire_p;          break;
    case 12: currentPalette = NormalFire2_p;         break;
    case 13: currentPalette = LithiumFireColors_p;   break;
    case 14: currentPalette = SodiumFireColors_p;    break;
    case 15: currentPalette = CopperFireColors_p;    break;
    case 16: currentPalette = RubidiumFireColors_p;  break;
    case 17: currentPalette = PotassiumFireColors_p; break;
    case 18: currentPalette = OceanColors_p;         break;
    case 19: currentPalette = Sunset_Real_gp;        break;
    case 20: currentPalette = dkbluered_gp;          break;
    case 21: currentPalette = Optimus_Prime_gp;      break;
    case 22: currentPalette = warmGrad_gp;           break;
    case 23: currentPalette = coldGrad_gp;           break;
    case 24: currentPalette = hotGrad_gp;            break;
    case 25: currentPalette = pinkGrad_gp;           break;
    case 26: currentPalette = comfy_gp;              break;
    case 27: currentPalette = cyperpunk_gp;          break;
    case 28: currentPalette = girl_gp;               break;
    case 29: currentPalette = xmas_gp;               break;
    case 30: currentPalette = acid_gp;               break;
    case 31: currentPalette = blueSmoke_gp;          break;
    case 32: currentPalette = gummy_gp;              break;
    case 33: currentPalette = aurora_gp;             break;
    case 34: currentPalette = redwhite_gp;           break;
    case 35: currentPalette = ib_jul01_gp;           break;
    case 36: currentPalette = rgi_15_gp;             break;
    case 37: currentPalette = retro2_16_gp;          break;
    case 38: currentPalette = Analogous_1_gp;        break;
    case 39: currentPalette = pinksplash_08_gp;      break;
    case 40: currentPalette = pinksplash_07_gp;      break;
    case 41: currentPalette = Coral_reef_gp;         break;
    case 42: currentPalette = ocean_breeze_gp;       break;
    case 43: currentPalette = landscape_64_gp;       break;
    case 44: currentPalette = landscape_33_gp;       break;
    case 45: currentPalette = rainbowsherbet_gp;     break;
    case 46: currentPalette = gr65_hult_gp;          break;
    case 47: currentPalette = GMT_drywet_gp;         break;
    case 48: currentPalette = emerald_dragon_gp;     break;
    case 49: currentPalette = Colorfull_gp;          break;
    case 50: currentPalette = Pink_Purple_gp;        break;
    case 51: currentPalette = autumn_19_gp;          break;
    case 52: currentPalette = daybreak_gp;           break;
    case 53: currentPalette = Blue_Cyan_Yellow_gp;   break;
    case 54: currentPalette = bhw1_28_gp;            break;
    case 55: currentPalette = rbw_gp;                break;
  }
  for (uint8_t i = 0; i < deltaHue; i++) {
    if (i != deltaValue) { // если это не активное кольцо
      h = shiftHue[i] & 0x0F; // сдвигаем оттенок внутри кольца
      if (h > 8U) noise_3d[0][0][i]--;
      else noise_3d[0][0][i]++;
    }
    else {
      if (step == 0) {  // если сдвиг активного кольца завершён, выбираем следующее
        deltaValue = random8(deltaHue);
        do {
          step = pWIDTH - 3U - random8((pWIDTH - 3U) * 2U); // проворот кольца от хз до хз
        }
        while (step < pWIDTH / 5U || step > 255U - pWIDTH / 5U);
      }
      else {
        if (step > 127U) {
          step++;
          shiftValue[i] = (shiftValue[i] + 1U) % pWIDTH;
        }
        else {
          step--;
          shiftValue[i] = (shiftValue[i] - 1U + pWIDTH) % pWIDTH;
        }
      }
    }
    // отрисовываем кольца
    h = (shiftHue[i] >> 4) & 0x0F; // берём шаг для градиента вутри кольца
    if (h > 8U) h = 7U - h;
    for (uint8_t j = 0U; j < ((i == 0U) ? hue : ((i == deltaHue - 1U) ? hue2 : deltaHue2)); j++) { // от 0 до (толщина кольца - 1)
      y = i * deltaHue2 + j - ((i == 0U) ? 0U : deltaHue2 - hue);
      for (uint8_t k = 0; k < pWIDTH / 2U; k++) { // полукольцо
        x = (shiftValue[i] + k) % pWIDTH; // первая половина кольца
        leds[XY(x, y)] = ColorFromPalette(currentPalette, noise_3d[0][0][i] + k * h);
        x = (pWIDTH - 1 + shiftValue[i] - k) % pWIDTH; // вторая половина кольца (зеркальная первой)
        leds[XY(x, y)] = ColorFromPalette(currentPalette, noise_3d[0][0][i] + k * h);
      }
      if (pWIDTH & 0x01) { // если число пикселей по ширине матрицы нечётное, тогда не забываем и про среднее значение
        x = (shiftValue[i] + pWIDTH / 2U) % pWIDTH;
        leds[XY(x, y)] = ColorFromPalette(currentPalette, noise_3d[0][0][i] + pWIDTH / 2U * h);
      }
    }
  }
}

// =====================================
//                Stars
//     © SottNick and  © Stepko
//      Adaptation © SlingMaster
//                Звезды
// =====================================

void drawStar(float xlocl, float ylocl, float biggy, float little, int16_t points, float dangle, uint8_t koler, CRGBPalette16 pal) { // random multipoint star
  float radius2 = 255.0 / points;
  for (int i = 0; i < points; i++) {
    DrawLine(xlocl + ((little * (sin8(i * radius2 + radius2 / 2 - dangle) - 128.0)) / 128), ylocl + ((little * (cos8(i * radius2 + radius2 / 2 - dangle) - 128.0)) / 128), xlocl + ((biggy * (sin8(i * radius2 - dangle) - 128.0)) / 128), ylocl + ((biggy * (cos8(i * radius2 - dangle) - 128.0)) / 128), ColorFromPalette(pal, koler));
    DrawLine(xlocl + ((little * (sin8(i * radius2 - radius2 / 2 - dangle) - 128.0)) / 128), ylocl + ((little * (cos8(i * radius2 - radius2 / 2 - dangle) - 128.0)) / 128), xlocl + ((biggy * (sin8(i * radius2 - dangle) - 128.0)) / 128), ylocl + ((biggy * (cos8(i * radius2 - dangle) - 128.0)) / 128), ColorFromPalette(pal, koler));
  }
}

void EffectStars() {
#define STARS_NUM (8U)
#define STAR_BLENDER (255U)
#define CENTER_DRIFT_SPEED (6U)
  static uint8_t spd;
  static uint8_t points[STARS_NUM];
  static float color[STARS_NUM] ;
  static int delay_arr[STARS_NUM];
  static float counter;
  static float driftx;
  static float  drifty;
  static float cangle;
  static float  sangle;
  static uint8_t stars_count;
  static uint8_t blur;
  uint8_t anglecount;
  if (loadingFlag) {
    loadingFlag = false;
    anglecount = map8(getEffectScaleParamValue(MC_EFFECTSTARS), 3, 9); //количество углов у звезды, которое можно задавать от 3 до 8 через ползунок варианта. 9 - рандом от 3 до 8
    palette_number = getEffectScaleParamValue2(MC_EFFECTSTARS);
    spd = getEffectSpeedValue(MC_EFFECTSTARS);
    if (palette_number == 0 || palette_number == 56) startnum = random8(1, 56); //Если Случайный выбор или Авто, задать произвольный вариант (в Авто от него начинается отсчет)
    else if (palette_number > 0 || palette_number < 56) startnum = palette_number;  //Если что-то из вариантов 1-55, берем только это значение
    counter = 0.0;
    // стартуем с центра
    driftx = (float)pWIDTH / 2.0;
    drifty = (float)pHEIGHT / 2.0;
    cangle = (float)(sin8(random8(25, 220)) - 128.0f) / 128.0f; //angle of movement for the center of animation gives a float value between -1 and 1
    sangle = (float)(sin8(random8(25, 220)) - 128.0f) / 128.0f; //angle of moveme for the center of animation in the y direction gives a float value between -1 and 1
    stars_count = pWIDTH / 2U;
    if (stars_count > STARS_NUM) stars_count = STARS_NUM;
    for (uint8_t num = 0; num < stars_count; num++) {
      if (anglecount == 9)                  // количество углов в звезде
        points[num] = random8(3, 8);
      else
        points[num] = anglecount;
      delay_arr[num] = spd / 2 + (num << 2) + 2U;               //spd / 5 задержка следующего пуска звезды
      color[num] = random8();
    }
  }
  if (palette_number == 56) {  //автоперебор вариантов, если выбран вариант Авто
    if (millis() - color_timer > 10000) {
      color_timer = millis();
      startnum++;
      if (startnum > 55) startnum = 1;
    }
  }
  switch (startnum) {
    case 1:  currentPalette = CloudColors_p;         break;
    case 2:  currentPalette = LavaColors_p;          break;
    case 3:  currentPalette = PartyColors_p;         break;
    case 4:  currentPalette = RainbowColors_p;       break;
    case 5:  currentPalette = RainbowStripeColors_p; break;
    case 6:  currentPalette = ForestColors_p;        break;
    case 7:  currentPalette = AlcoholFireColors_p;   break;
    case 8:  currentPalette = HeatColors_p;          break;
    case 9:  currentPalette = WaterfallColors4in1_p; break;
    case 10: currentPalette = WoodFireColors_p;      break;
    case 11: currentPalette = NormalFire_p;          break;
    case 12: currentPalette = NormalFire2_p;         break;
    case 13: currentPalette = LithiumFireColors_p;   break;
    case 14: currentPalette = SodiumFireColors_p;    break;
    case 15: currentPalette = CopperFireColors_p;    break;
    case 16: currentPalette = RubidiumFireColors_p;  break;
    case 17: currentPalette = PotassiumFireColors_p; break;
    case 18: currentPalette = OceanColors_p;         break;
    case 19: currentPalette = Sunset_Real_gp;        break;
    case 20: currentPalette = dkbluered_gp;          break;
    case 21: currentPalette = Optimus_Prime_gp;      break;
    case 22: currentPalette = warmGrad_gp;           break;
    case 23: currentPalette = coldGrad_gp;           break;
    case 24: currentPalette = hotGrad_gp;            break;
    case 25: currentPalette = pinkGrad_gp;           break;
    case 26: currentPalette = comfy_gp;              break;
    case 27: currentPalette = cyperpunk_gp;          break;
    case 28: currentPalette = girl_gp;               break;
    case 29: currentPalette = xmas_gp;               break;
    case 30: currentPalette = acid_gp;               break;
    case 31: currentPalette = blueSmoke_gp;          break;
    case 32: currentPalette = gummy_gp;              break;
    case 33: currentPalette = aurora_gp;             break;
    case 34: currentPalette = redwhite_gp;           break;
    case 35: currentPalette = ib_jul01_gp;           break;
    case 36: currentPalette = rgi_15_gp;             break;
    case 37: currentPalette = retro2_16_gp;          break;
    case 38: currentPalette = Analogous_1_gp;        break;
    case 39: currentPalette = pinksplash_08_gp;      break;
    case 40: currentPalette = pinksplash_07_gp;      break;
    case 41: currentPalette = Coral_reef_gp;         break;
    case 42: currentPalette = ocean_breeze_gp;       break;
    case 43: currentPalette = landscape_64_gp;       break;
    case 44: currentPalette = landscape_33_gp;       break;
    case 45: currentPalette = rainbowsherbet_gp;     break;
    case 46: currentPalette = gr65_hult_gp;          break;
    case 47: currentPalette = GMT_drywet_gp;         break;
    case 48: currentPalette = emerald_dragon_gp;     break;
    case 49: currentPalette = Colorfull_gp;          break;
    case 50: currentPalette = Pink_Purple_gp;        break;
    case 51: currentPalette = autumn_19_gp;          break;
    case 52: currentPalette = daybreak_gp;           break;
    case 53: currentPalette = Blue_Cyan_Yellow_gp;   break;
    case 54: currentPalette = bhw1_28_gp;            break;
    case 55: currentPalette = rbw_gp;                break;
  }
  fadeToBlackBy(leds, NUM_LEDS, 165);
  float speedFactor = ((float)spd / 380.0 + 0.05);
  counter += speedFactor;                                                   // определяет то, с какой скоростью будет приближаться звезда
  if (driftx > (pWIDTH - spirocenterX / 2U)) cangle = 0 - fabs(cangle);      //change directin of drift if you get near the right 1/4 of the screen
  if (driftx < spirocenterX / 2U) cangle = fabs(cangle);                    //change directin of drift if you get near the right 1/4 of the screen
  if ((uint16_t)counter % CENTER_DRIFT_SPEED == 0) driftx = driftx + (cangle * speedFactor); //move the x center every so often
  if (drifty > ( pHEIGHT - spirocenterY / 2U)) sangle = 0 - fabs(sangle);    // if y gets too big, reverse
  if (drifty < spirocenterY / 2U) sangle = fabs(sangle);                    // if y gets too small reverse
  if ((uint16_t)counter % CENTER_DRIFT_SPEED == 0) drifty = drifty + (sangle * speedFactor); //move the y center every so often
  for (uint8_t num = 0; num < stars_count; num++) {
    if (counter >= delay_arr[num]) {              //(counter >= ringdelay)
      if (counter - delay_arr[num] <= pWIDTH + 5) {
        drawStar(driftx, drifty, 2 * (counter - delay_arr[num]), (counter - delay_arr[num]), points[num], STAR_BLENDER + color[num], color[num], currentPalette);
        color[num] += speedFactor;                // в зависимости от знака - направление вращения
      } else {
        delay_arr[num] = counter + (stars_count << 1) + 1U; // задержка следующего пуска звезды
      }
    }
  }
  blur2d(leds, pWIDTH, pHEIGHT, blur);
}

//================================================================================================
// ----------------------------- Лавовая лампа / Пузыри в одном эффекте ---------------------
// ----------- Эффект "Лавовая лампа" (c) obliterator
// https://github.com/DmytroKorniienko/FireLamp_JeeUI/commit/9bad25adc2c917fbf3dfa97f4c498769aaf76ebe
// с генератором палитр by SottNick

float mapcurve(const float x, const float in_min, const float in_max, const float out_min, const float out_max, float (*curve)(float, float, float, float)) {
  if (x <= in_min) return out_min;
  if (x >= in_max) return out_max;
  return curve((x - in_min), out_min, (out_max - out_min), (in_max - in_min));
}
float InQuad(float t, float b, float c, float d) {
  t /= d;
  return c * t * t + b;
}
float OutQuart(float t, float b, float c, float d) {
  t = t / d - 1;
  return -c * (t * t * t * t - 1) + b;
}
float InOutQuad(float t, float b, float c, float d) {
  t /= d / 2;
  if (t < 1) return c / 2 * t * t + b;
  --t;
  return -c / 2 * (t * (t - 2) - 1) + b;
}

unsigned MASS_MIN = 10;
unsigned MASS_MAX = 50;

void LiquidLampPosition() {
  //bool physic_on = modes[currentMode].Speed & 0x01;
  for (uint8_t i = 0; i < enlargedObjectNUM; i++) {
    liquidLampHot[i] += mapcurve(trackingObjectPosY[i], 0, pHEIGHT - 1, 5, -5, InOutQuad) * speedfactor;
    float heat = (liquidLampHot[i] / trackingObjectState[i]) - 1;
    if (heat > 0 && trackingObjectPosY[i] < pHEIGHT - 1) {
      trackingObjectSpeedY[i] += heat * liquidLampSpf[i];
    }
    if (trackingObjectPosY[i] > 0) {
      trackingObjectSpeedY[i] -= 0.07;
    }
    if (trackingObjectSpeedY[i]) trackingObjectSpeedY[i] *= 0.85;
    trackingObjectPosY[i] += trackingObjectSpeedY[i] * speedfactor;
    //if (physic_on) {
    if (trackingObjectSpeedX[i]) trackingObjectSpeedX[i] *= 0.7;
    trackingObjectPosX[i] += trackingObjectSpeedX[i] * speedfactor;
    //}
    if (trackingObjectPosX[i] > pWIDTH - 1) trackingObjectPosX[i] -= pWIDTH - 1;
    if (trackingObjectPosX[i] < 0) trackingObjectPosX[i] += pWIDTH - 1;
    if (trackingObjectPosY[i] > pHEIGHT - 1) trackingObjectPosY[i] = pHEIGHT - 1;
    if (trackingObjectPosY[i] < 0) trackingObjectPosY[i] = 0;
  };
}

void LiquidLampPhysic() {
  for (uint8_t i = 0; i < enlargedObjectNUM; i++) {
    // отключаем физику на границах, чтобы не слипались шары
    if (trackingObjectPosY[i] < 3 || trackingObjectPosY[i] > pHEIGHT - 1) continue;
    for (uint8_t j = 0; j < enlargedObjectNUM; j++) {
      if (trackingObjectPosY[j] < 3 || trackingObjectPosY[j] > pHEIGHT - 1) continue;
      float radius = 3;//(trackingObjectShift[i] + trackingObjectShift[j]);
      if (trackingObjectPosX[i] + radius > trackingObjectPosX[j]
          && trackingObjectPosX[i] < radius + trackingObjectPosX[j]
          && trackingObjectPosY[i] + radius > trackingObjectPosY[j]
          && trackingObjectPosY[i] < radius + trackingObjectPosY[j]
         ) {
        float dx =  min((float)fabs(trackingObjectPosX[i] - trackingObjectPosX[j]), (float)pWIDTH + trackingObjectPosX[i] - trackingObjectPosX[j]); //по идее бесшовный икс
        float dy =  fabs(trackingObjectPosY[i] - trackingObjectPosY[j]);
        float dist = SQRT_VARIANT((dx * dx) + (dy * dy));
        if (dist <= radius) {
          float nx = (trackingObjectPosX[j] - trackingObjectPosX[i]) / dist;
          float ny = (trackingObjectPosY[j] - trackingObjectPosY[i]) / dist;
          float p = 2 * (trackingObjectSpeedX[i] * nx + trackingObjectSpeedY[i] * ny - trackingObjectSpeedX[j] * nx - trackingObjectSpeedY[j] * ny) / (trackingObjectState[i] + trackingObjectState[j]);
          float pnx = p * nx, pny = p * ny;
          trackingObjectSpeedX[i] = trackingObjectSpeedX[i] - pnx * trackingObjectState[i];
          trackingObjectSpeedY[i] = trackingObjectSpeedY[i] - pny * trackingObjectState[i];
          trackingObjectSpeedX[j] = trackingObjectSpeedX[j] + pnx * trackingObjectState[j];
          trackingObjectSpeedY[j] = trackingObjectSpeedY[j] + pny * trackingObjectState[j];
        }
      }
    }
  }
}

// генератор палитр для Жидкой лампы (c) SottNick
static const uint8_t MBVioletColors_arr[5][4] PROGMEM = // та же палитра, но в формате CHSV
{
  {0  , 0  , 255, 255}, //  0, 255,   0,   0, // red
  {1  , 155, 209, 255}, //  1,  46, 124, 255, // сделал поярче цвет воды
  {80 , 170, 255, 140}, // 80,   0,   0, 139, // DarkBlue
  {150, 213, 255, 128}, //150, 128,   0, 128, // purple
  {255, 0  , 255, 255}  //255, 255,   0,   0  // red again
};

void fillMyPal16(uint8_t hue, bool isInvert = false) {
  int8_t lastSlotUsed = -1;
  uint8_t istart8, iend8;
  CRGB rgbstart, rgbend;
  // начинаем с нуля
  if (isInvert)
    hsv2rgb_spectrum(CHSV(256 + hue - pgm_read_byte(&MBVioletColors_arr[0][1]), pgm_read_byte(&MBVioletColors_arr[0][2]), pgm_read_byte(&MBVioletColors_arr[0][3])), rgbstart);
  else
    hsv2rgb_spectrum(CHSV(hue + pgm_read_byte(&MBVioletColors_arr[0][1]), pgm_read_byte(&MBVioletColors_arr[0][2]), pgm_read_byte(&MBVioletColors_arr[0][3])), rgbstart);
  int indexstart = 0; // начальный индекс палитры
  for (uint8_t i = 1U; i < 5U; i++) { // в палитре @obliterator всего 5 строчек
    int indexend = pgm_read_byte(&MBVioletColors_arr[i][0]);
    if (isInvert)
      hsv2rgb_spectrum(CHSV(256 + hue - pgm_read_byte(&MBVioletColors_arr[i][1]), pgm_read_byte(&MBVioletColors_arr[i][2]), pgm_read_byte(&MBVioletColors_arr[i][3])), rgbend);
    else
      hsv2rgb_spectrum(CHSV(hue + pgm_read_byte(&MBVioletColors_arr[i][1]), pgm_read_byte(&MBVioletColors_arr[i][2]), pgm_read_byte(&MBVioletColors_arr[i][3])), rgbend);
    istart8 = indexstart / 16;
    iend8   = indexend   / 16;
    if ((istart8 <= lastSlotUsed) && (lastSlotUsed < 15)) {
      istart8 = lastSlotUsed + 1;
      if (iend8 < istart8)
        iend8 = istart8;
    }
    lastSlotUsed = iend8;
    fill_gradient_RGB( myPal, istart8, rgbstart, iend8, rgbend);
    indexstart = indexend;
    rgbstart = rgbend;
  }
}

void LiquidLampRoutine(CRGBPalette16 pal) {
  LiquidLampPosition();
  LiquidLampPhysic;
  hue2++;
  if (hue2 % 0x10 == 0U) {
    hue++;
    fillMyPal16(hue, deltaHue);
  }
  for (uint8_t x = 0; x < pWIDTH; x++) {
    for (uint8_t y = 0; y < pHEIGHT; y++) {
      float sum = 0;
      for (uint8_t i = 0; i < enlargedObjectNUM; i++) {
        if (abs(x - trackingObjectPosX[i]) > liquidLampTR[i] || abs(y - trackingObjectPosY[i]) > liquidLampTR[i]) continue;
        float dx =  min((float)fabs(trackingObjectPosX[i] - (float)x), (float)pWIDTH + trackingObjectPosX[i] - (float)x); //по идее бесшовный икс
        float dy =  fabs(trackingObjectPosY[i] - (float)y);
        float d = SQRT_VARIANT((dx * dx) + (dy * dy));
        if (d < trackingObjectShift[i]) {
          sum += mapcurve(d, 0, trackingObjectShift[i], 255, liquidLampMX[i], InQuad);
        }
        else if (d < liquidLampSC[i]) {
          sum += mapcurve(d, trackingObjectShift[i], liquidLampSC[i], liquidLampMX[i], 0, OutQuart);
        }
        if (sum >= 255) {
          sum = 255;
          break;
        }
      }
      if (sum < 16) sum = 16;// отрезаем смазанный кусок палитры из-за отсутствия параметра NOBLEND
      CRGB color = ColorFromPalette(pal, sum); // ,255, NOBLEND
      drawPixelXY(x, y, color);
    }
  }
}

void LavaLampGetspeed(uint8_t l) {
  trackingObjectSpeedY[l] = (float)random8(5, 11) / (257U - modes[currentMode].Speed) / 4.0; // если скорость кадров фиксированная
}
void drawBlob(uint8_t l, CRGB color) { //раз круги нарисовать не получается, будем попиксельно вырисовывать 2 варианта пузырей
  if (trackingObjectShift[l] == 2) {
    for (int8_t x = -2; x < 3; x++)
      for (int8_t y = -2; y < 3; y++)
        if (abs(x) + abs(y) < 4)
          drawPixelXYF(fmod(trackingObjectPosX[l] + x + pWIDTH, pWIDTH), trackingObjectPosY[l] + y, color);
  }
  else {
    for (int8_t x = -1; x < 3; x++)
      for (int8_t y = -1; y < 3; y++)
        if (!(x == -1 && (y == -1 || y == 2) || x == 2 && (y == -1 || y == 2)))
          drawPixelXYF(fmod(trackingObjectPosX[l] + x + pWIDTH, pWIDTH), trackingObjectPosY[l] + y, color);
  }
}

void LavaLampRoutine() {
  hue++;
  CRGB color = CHSV(hue, (modes[currentMode].Scale < 100U) ? 255U : 0U, 255U);
  FastLED.clear();
  for (uint8_t i = 0; i < enlargedObjectNUM; i++) { //двигаем по аналогии с https://jiwonk.im/lavalamp/
    if (trackingObjectPosY[i] + trackingObjectShift[i] >= pHEIGHT - 1)
      trackingObjectPosY[i] += (trackingObjectSpeedY[i] * ((pHEIGHT - 1 - trackingObjectPosY[i]) / trackingObjectShift[i] + 0.005));
    else if (trackingObjectPosY[i] - trackingObjectShift[i] <= 0)
      trackingObjectPosY[i] += (trackingObjectSpeedY[i] * (trackingObjectPosY[i] / trackingObjectShift[i] + 0.005));
    else
      trackingObjectPosY[i] += trackingObjectSpeedY[i];
    // bounce off the floor and ceiling?
    if (trackingObjectPosY[i] < 0.01) {                  // почему-то при нуле появляется мерцание (один кадр, еле заметно)
      LavaLampGetspeed(i);
      trackingObjectPosY[i] = 0.01;
    }
    else if (trackingObjectPosY[i] > pHEIGHT - 1.01) {    // тоже на всякий пожарный
      LavaLampGetspeed(i);
      trackingObjectSpeedY[i] = -trackingObjectSpeedY[i];
      trackingObjectPosY[i] = pHEIGHT - 1.01;
    }
    drawBlob(i, color); // раз круги выглядят убого, рисуем попиксельно 2 размера пузырей
  };
  blurScreen(20);
}

//основная функция для запуска эффектов
uint8_t lamp_type = 0;
void LiquidLamp() {
  if (loadingFlag) {
    loadingFlag = false;
    lamp_type = (specialTextEffectParam >= 0) ? specialTextEffectParam : getEffectScaleParamValue2(MC_LIQUIDLAMP);
    // Если авто - генерировать один из типов
    if (lamp_type == 0 || lamp_type > 2)
      lamp_type = random8(1, 3);
    if (lamp_type == 1) {
      startnum = random8(1, 35);  //Задаем произвольно начальную палитру
      speedfactor = modes[currentMode].Speed / 127 + 0.1; // 127 БЫЛО   32
      enlargedObjectNUM = (modes[currentMode].Scale - 1U) / 99.0 * (enlargedOBJECT_MAX_COUNT - 1U) + 1U;
      hue = random8();
      deltaHue = random8(2U);
      fillMyPal16(hue, deltaHue);
      if (enlargedObjectNUM > enlargedOBJECT_MAX_COUNT) enlargedObjectNUM = enlargedOBJECT_MAX_COUNT;
      else if (enlargedObjectNUM < 2U) enlargedObjectNUM = 2U;
      double minSpeed = 0.2, maxSpeed = 0.8;
      for (uint8_t i = 0 ; i < enlargedObjectNUM ; i++) {
        trackingObjectPosX[i] = random8(pWIDTH);
        trackingObjectPosY[i] = 0; //random8(pHEIGHT);
        trackingObjectState[i] = random(MASS_MIN, MASS_MAX);
        liquidLampSpf[i] = fmap(trackingObjectState[i], MASS_MIN, MASS_MAX, 0.0015, 0.0005);
        trackingObjectShift[i] = fmap(trackingObjectState[i], MASS_MIN, MASS_MAX, 2, 3);
        liquidLampMX[i] = map(trackingObjectState[i], MASS_MIN, MASS_MAX, 60, 80); // сила возмущения
        liquidLampSC[i] = map(trackingObjectState[i], MASS_MIN, MASS_MAX, 6, 10); // радиус возмущения
        liquidLampTR[i] = liquidLampSC[i]  * 2 / 3; // отсечка расчетов (оптимизация скорости)
      }
    }
    if (lamp_type == 2) {
      enlargedObjectNUM = (pWIDTH / 2) -  ((pWIDTH - 1) & 0x01);
      uint8_t shift = random8(2);
      for (uint8_t i = 0; i < enlargedObjectNUM; i++) {
        trackingObjectPosY[i] = 0;  //random8(pHEIGHT);
        trackingObjectPosX[i] = i * 2U + shift;
        LavaLampGetspeed(i);
        trackingObjectShift[i] = random8(1, 3); // присваивается случайный целочисленный радиус пузырям от 1 до 2
      }
      if (modes[currentMode].Scale != 1U)
        hue = modes[currentMode].Scale * 2.57;
    }
  }
  if (millis() - color_timer > 10000) {
    color_timer = millis();
    startnum++;
    if (startnum > 34) startnum = 1;
  }
  switch (startnum) {
    case 1:  currentPalette = CloudColors_p;         break;
    case 2:  currentPalette = LavaColors_p;          break;
    case 3:  currentPalette = PartyColors_p;         break;
    case 4:  currentPalette = RainbowColors_p;       break;
    case 5:  currentPalette = RainbowStripeColors_p; break;
    case 6:  currentPalette = ForestColors_p;        break;
    case 7:  currentPalette = AlcoholFireColors_p;   break;
    case 8:  currentPalette = HeatColors_p;          break;
    case 9:  currentPalette = WaterfallColors4in1_p; break;
    case 10: currentPalette = WoodFireColors_p;      break;
    case 11: currentPalette = NormalFire_p;          break;
    case 12: currentPalette = NormalFire2_p;         break;
    case 13: currentPalette = LithiumFireColors_p;   break;
    case 14: currentPalette = SodiumFireColors_p;    break;
    case 15: currentPalette = CopperFireColors_p;    break;
    case 16: currentPalette = RubidiumFireColors_p;  break;
    case 17: currentPalette = PotassiumFireColors_p; break;
    case 18: currentPalette = OceanColors_p;         break;
    case 19: currentPalette = Sunset_Real_gp;        break;
    case 20: currentPalette = dkbluered_gp;          break;
    case 21: currentPalette = Optimus_Prime_gp;      break;
    case 22: currentPalette = warmGrad_gp;           break;
    case 23: currentPalette = coldGrad_gp;           break;
    case 24: currentPalette = hotGrad_gp;            break;
    case 25: currentPalette = pinkGrad_gp;           break;
    case 26: currentPalette = comfy_gp;              break;
    case 27: currentPalette = cyperpunk_gp;          break;
    case 28: currentPalette = girl_gp;               break;
    case 29: currentPalette = xmas_gp;               break;
    case 30: currentPalette = acid_gp;               break;
    case 31: currentPalette = blueSmoke_gp;          break;
    case 32: currentPalette = gummy_gp;              break;
    case 33: currentPalette = aurora_gp;             break;
    case 34: currentPalette = redwhite_gp;           break;
    case 35: currentPalette = ib_jul01_gp;           break;
    case 36: currentPalette = rgi_15_gp;             break;
    case 37: currentPalette = retro2_16_gp;          break;
    case 38: currentPalette = Analogous_1_gp;        break;
    case 39: currentPalette = pinksplash_08_gp;      break;
    case 40: currentPalette = pinksplash_07_gp;      break;
    case 41: currentPalette = Coral_reef_gp;         break;
    case 42: currentPalette = ocean_breeze_gp;       break;
    case 43: currentPalette = landscape_64_gp;       break;
    case 44: currentPalette = landscape_33_gp;       break;
    case 45: currentPalette = rainbowsherbet_gp;     break;
    case 46: currentPalette = gr65_hult_gp;          break;
    case 47: currentPalette = GMT_drywet_gp;         break;
    case 48: currentPalette = emerald_dragon_gp;     break;
    case 49: currentPalette = Colorfull_gp;          break;
    case 50: currentPalette = Pink_Purple_gp;        break;
    case 51: currentPalette = autumn_19_gp;          break;
    case 52: currentPalette = daybreak_gp;           break;
    case 53: currentPalette = Blue_Cyan_Yellow_gp;   break;
    case 54: currentPalette = bhw1_28_gp;            break;
    case 55: currentPalette = rbw_gp;                break;
  }
  switch (lamp_type) {
    case 1: LiquidLampRoutine(currentPalette); break;
    case 2: LavaLampRoutine(); break;
  }
}

// ----------- Эффект "ДНК"
// База https://pastebin.com/jwvC1sNF адаптация и доработки kostyamat
// нормальные копирайты:
// https://pastebin.com/jwvC1sNF
//2 DNA spiral with subpixel
//16x16 rgb led matrix demo
//Yaroslaw Turbin 04.09.2020
//https://vk.com/ldirko
//https://www.reddit.com/user/ldirko/
//https://www.reddit.com/r/FastLED/comments/gogs4n/i_made_7x11_matrix_for_my_ntp_clock_project_then/
//this is update for DNA procedure https://pastebin.com/Qa8A5NvW
//add subpixel render foк nice smooth look
void wu_pixel(uint32_t x, uint32_t y, CRGB * col) {      //awesome wu_pixel procedure by reddit u/sutaburosu
  // extract the fractional parts and derive their inverses
  uint8_t xx = x & 0xff, yy = y & 0xff, ix = 255 - xx, iy = 255 - yy;
  // calculate the intensities for each affected pixel
#define WU_WEIGHT(a,b) ((uint8_t) (((a)*(b)+(a)+(b))>>8))
  uint8_t wu[4] = {WU_WEIGHT(ix, iy), WU_WEIGHT(xx, iy), WU_WEIGHT(ix, yy), WU_WEIGHT(xx, yy)};
  // multiply the intensities by the colour, and saturating-add them to the pixels
  for (uint8_t i = 0; i < 4; i++) {
    uint16_t xy = XY((x >> 8) + (i & 1), (y >> 8) + ((i >> 1) & 1));
    if (xy < NUM_LEDS) {
      leds[xy].r = qadd8(leds[xy].r, col->r * wu[i] >> 8);
      leds[xy].g = qadd8(leds[xy].g, col->g * wu[i] >> 8);
      leds[xy].b = qadd8(leds[xy].b, col->b * wu[i] >> 8);
    }
  }
}

void DNARoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    step = map8(modes[currentMode].Speed, 10U, 60U);
    hue = modes[currentMode].Scale;
    if (pWIDTH > pHEIGHT) direct = 0; //если матрица широкая - врубаем эффект горизонтально
    if (pWIDTH < pHEIGHT) direct = 1; //если матрица высокая - врубаем эффект вертикально
    if (pWIDTH == pHEIGHT) direct = random8(2); //если матрица квадратная - на все воля Великого Рандома, эффект может запуститься и так, и так
    if (direct == 0) hue = 101U - hue;
    hue = 255U - map( 51U - hue, 1U, 50U, 0, 255U);
  }
  double freq = 3000;
  float mn = 255.0 / 13.8;
  fadeToBlackBy(leds, NUM_LEDS, step);
  uint16_t ms = millis();
  if (direct == 0)
    for (uint8_t i = 0; i < pWIDTH; i++) {
      uint32_t x = beatsin16(step, 0, (pHEIGHT - 1) * 256, 0, i * freq);
      uint32_t y = i * 256;
      uint32_t x1 = beatsin16(step, 0, (pHEIGHT - 1) * 256, 0, i * freq + 32768);
      CRGB col = CHSV(ms / 29 + i * 255 / (pWIDTH - 1), 255, qadd8(hue, beatsin8(step, 60, 255U, 0, i * mn)));
      CRGB col1 = CHSV(ms / 29 + i * 255 / (pWIDTH - 1) + 128, 255, qadd8(hue, beatsin8(step, 60, 255U, 0, i * mn + 128)));
      wu_pixel (y , x, &col);
      wu_pixel (y , x1, &col1);
    }
  else
    for (uint8_t i = 0; i < pHEIGHT; i++) {
      uint32_t x = beatsin16(step, 0, (pWIDTH - 1) * 256, 0, i * freq);
      uint32_t y = i * 256;
      uint32_t x1 = beatsin16(step, 0, (pWIDTH - 1) * 256, 0, i * freq + 32768);
      CRGB col = CHSV(ms / 29 + i * 255 / (pHEIGHT - 1), 255, qadd8(hue, beatsin8(step, 60, 255U, 0, i * mn)));
      CRGB col1 = CHSV(ms / 29 + i * 255 / (pHEIGHT - 1) + 128, 255, qadd8(hue, beatsin8(step, 60, 255U, 0, i * mn + 128)));
      wu_pixel (x , y, &col);
      wu_pixel (x1 , y, &col1);
    }
  blurScreen(16);
}

// -------------- эффект пульс ------------
// Stefan Petrick's PULSE Effect mod by PalPalych for GyverLamp
uint8_t pulse_type = 0;
void draw_Circle(int x0, int y0, int radius, const CRGB &color) {
  int a = radius, b = 0;
  int radiusError = 1 - a;
  if (radius == 0) {
    drawPixelXY(x0, y0, color);
    return;
  }
  while (a >= b)  {
    drawPixelXY(a + x0, b + y0, color);
    drawPixelXY(b + x0, a + y0, color);
    drawPixelXY(-a + x0, b + y0, color);
    drawPixelXY(-b + x0, a + y0, color);
    drawPixelXY(-a + x0, -b + y0, color);
    drawPixelXY(-b + x0, -a + y0, color);
    drawPixelXY(a + x0, -b + y0, color);
    drawPixelXY(b + x0, -a + y0, color);
    b++;
    if (radiusError < 0)
      radiusError += 2 * b + 1;
    else
    {
      a--;
      radiusError += 2 * (b - a + 1);
    }
  }
}

void pulseRoutine() {
  if (loadingFlag) {
    loadingFlag = false;
    pulse_type = getEffectScaleParamValue2(MC_PULSE);
    if (pulse_type == 0 || pulse_type == 9) startnum = random8(1, 9); //Если Случайный выбор или Авто, задать произвольный вариант (в Авто от него начинается отсчет)
    else if (pulse_type > 0 || pulse_type < 9) startnum = pulse_type;  //Если что-то из вариантов 1-8, берем только это значение
    FastLED.clear();  // очистить
  }
  //ВНЕЗАПНО обнаружил, что там 8 вариантов вызова эффекта. Добавил все. Вывод: читайте код внимательно.
  if (pulse_type == 9) {  //автоперебор вариантов, если выбран вариант Авто, дается 10 сек на эффект
    if (millis() - color_timer > 10000) {
      color_timer = millis();
      startnum++;
      if (startnum > 8) startnum = 1;
    }
  }
  switch (startnum) {
    case 1:  pulse_routine(1U); break;
    case 2:  pulse_routine(2U); break;
    case 3:  pulse_routine(3U); break;
    case 4:  pulse_routine(4U); break;
    case 5:  pulse_routine(5U); break;
    case 6:  pulse_routine(6U); break;
    case 7:  pulse_routine(7U); break;
    case 8:  pulse_routine(8U); break;
  }
}
void pulse_routine(uint8_t PMode) {
  CRGB _pulse_color;
  dimAll(248U);
  uint8_t _sat;
  if (step <= pcnt) {
    for (uint8_t i = 0; i < step; i++ ) {
      uint8_t _dark = qmul8( 2U, cos8 (128U / (step + 1U) * (i + 1U))) ;
      switch (PMode) {
        case 1U:                    // 1 - случайные диски
          deltaHue = hue;
          _pulse_color = CHSV(deltaHue, 255U, _dark);
          break;
        case 2U:                    // 2...17 - перелив цвета дисков
          deltaHue2 = modes[currentMode].Scale;
          _pulse_color = CHSV(hue2, 255U, _dark);
          break;
        case 3U:                    // 18...33 - выбор цвета дисков
          deltaHue = modes[currentMode].Scale * 2.55;
          _pulse_color = CHSV(deltaHue, 255U, _dark);
          break;
        case 4U:                    // 34...50 - дискоцветы
          deltaHue += modes[currentMode].Scale;
          _pulse_color = CHSV(deltaHue, 255U, _dark);
          break;
        case 5U:                    // 51...67 - пузыри цветы
          _sat =  qsub8( 255U, cos8 (128U / (step + 1U) * (i + 1U))) ;
          deltaHue += modes[currentMode].Scale;
          _pulse_color = CHSV(deltaHue, _sat, _dark);
          break;
        case 6U:                    // 68...83 - выбор цвета пузырей
          _sat =  qsub8( 255U, cos8 (128U / (step + 1U) * (i + 1U))) ;
          deltaHue = modes[currentMode].Scale * 2.55;
          _pulse_color = CHSV(deltaHue, _sat, _dark);
          break;
        case 7U:                    // 84...99 - перелив цвета пузырей
          _sat =  qsub8( 255U, cos8 (128U / (step + 1U) * (i + 1U))) ;
          deltaHue2 = modes[currentMode].Scale;
          _pulse_color = CHSV(hue2, _sat, _dark);
          break;
        case 8U:                    // 100 - случайные пузыри
          _sat =  qsub8( 255U, cos8 (128U / (step + 1U) * (i + 1U))) ;
          deltaHue2 = modes[currentMode].Scale;
          _pulse_color = CHSV(hue2, _sat, _dark);
          break;
      }
      draw_Circle(emitterX, emitterY, i, _pulse_color  );
    }
  } else {
    emitterX = random8(pWIDTH - 5U) + 3U;
    emitterY = random8(pHEIGHT - 5U) + 3U;
    hue2 += deltaHue2;
    hue = random8(0U, 255U);
    pcnt = random8(pWIDTH >> 2U, (pWIDTH >> 1U) + 1U);
    step = 0;
  }
  step++;
}

// =====================================
//             Мечта Дизайнера
//                WebTools
//             © SlingMaster
// =====================================
/* --------------------------------- */
int getRandomPos(uint8_t STEP) {
  uint8_t val = floor(random(0, (STEP * 16 - pWIDTH - 1)) / STEP) * STEP;
  return -val;
}

/* --------------------------------- */
int getHue(uint8_t x, uint8_t y) {
  return ( x * 32 +  y * 24U );
}
/* --------------------------------- */
uint8_t getSaturationStep() {
  return (modes[currentMode].Speed > 170U) ? ((pHEIGHT > 24) ? 12 : 24) : 0;
}
/* --------------------------------- */
uint8_t getBrightnessStep() {
  return (modes[currentMode].Speed < 85U) ? ((pHEIGHT > 24) ? 16 : 24) : 0;
}
/* --------------------------------- */
void drawPalette(int posX, int posY, uint8_t STEP) {
  int PX, PY;
  const uint8_t SZ = STEP - 1;
  const uint8_t maxY = floor(pHEIGHT / SZ);
  uint8_t sat = getSaturationStep();
  uint8_t br  = getBrightnessStep();
  FastLED.clear();
  for (uint8_t y = 0; y < maxY; y++) {
    for (uint8_t x = 0; x < 16; x++) {
      PY = y * STEP;
      PX = posX + x * STEP;
      if ((PX >= - STEP ) && (PY >= - STEP) && (PX < pWIDTH) && (PY < pHEIGHT)) {
        // LOG.printf_P(PSTR("y: %03d | br • %03d | sat • %03d\n"), y, (240U - br * y), sat);
        drawRecCHSV(PX, PY, PX + SZ, PY + SZ, CHSV( getHue(x, y), (255U - sat * y), (240U - br * y)));
      }
    }
  }
}

void drawRecCHSV(uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY, CHSV color) {
  for (uint8_t y = startY; y < endY; y++) {
    for (uint8_t x = startX; x < endX; x++) {
      drawPixelXY(x, y, color);
    }
  }
}

/* --------------------------------- */
void selectColor(uint8_t sc) {
  uint8_t offset = (pWIDTH >= 16) ? pWIDTH * 0.25 : 0;
  hue = getHue(random(offset, pWIDTH - offset), random(pHEIGHT));
  uint8_t sat = getSaturationStep();
  uint8_t br  = getBrightnessStep();

  for (uint8_t y = 0; y < pHEIGHT; y++) {
    for (uint8_t x = offset; x < (pWIDTH - offset); x++) {
      CHSV curColor = CHSV(hue, (255U - sat * y), (240U - br * y));
      if (curColor == getPixColorXY(x, y)) {
        /* show srlect color */
        drawRecCHSV(x, y, x + sc, y + sc, CHSV( hue, 64U, 255U));
        FastLED.show();
        delay(400);
        drawRecCHSV(x, y, x + sc, y + sc, CHSV( hue, 255U, 255U));
        y = pHEIGHT;
        x = pWIDTH;
      }
    }
  }
}

// альтернативный градиент для ламп собраных из лент с вертикальной компоновкой
// gradientHorizontal | gradientVertical менее производительный но работает на всех видах ламп
//--------------------------------------
void gradientHorizontal(uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY, uint8_t start_color, uint8_t end_color, uint8_t start_br, uint8_t end_br, uint8_t saturate) {
  float step_color = 0;
  float step_br = 0;
  if (startX == endX) {
    endX++;
  }
  if (startY == endY) {
    endY++;
  }
  step_color = (end_color - start_color) / abs(startX - endX);
  if (start_color >  end_color) {
    step_color -= 1.2;
  } else {
    step_color += 1.2;
  }

  step_br = (end_br - start_br) / abs(startX - endX);
  if (start_br >  end_color) {
    step_br -= 1.2;
  } else {
    step_br += 1.2;
  }

  // LOG.printf_P(PSTR( "\n step_color: %f | step_br: %f \n\n\r"), step_color, step_br);
  for (uint8_t x = startX; x < endX; x++) {
    for (uint8_t y = startY; y < endY; y++) {
      CHSV thisColor = CHSV((uint8_t) validMinMax((start_color + (x - startX) * step_color), 1, 254), saturate,
                            (uint8_t) validMinMax((start_br + (x - startX) * step_br), 0, 255) );
      drawPixelXY(x, y, thisColor);
    }
  }
}

/* ------запускать отсюда-------- */
void WebTools() {
  const uint8_t FPS_D = 24U;
  static uint8_t STEP = 3U;
  static int posX = -STEP;
  static int posY = 0;
  static int nextX = -STEP * 2;
  static bool stop_moving = true;
  uint8_t speed = constrain (modes[currentMode].Speed, 65, 255);
  if (loadingFlag) {
    loadingFlag = false;
    FPSdelay = 1U;
    step = 0;
    STEP = 2U + floor(modes[currentMode].Scale / 35);
    posX = 0;
    posY = 0;
    drawPalette(posX, posY, STEP);
  }
  /* auto scenario */
  //switch (step) {
  if (step == 0) {    /* restart ----------- */
    nextX = 0;
    FPSdelay = FPS_D;
  }
  else if (step == speed / 16 + 1) { /* start move -------- 16*/
    nextX = getRandomPos(STEP);
    FPSdelay = FPS_D;
  }
  else if (step == speed / 10 + 1) { /* find --------------100 */
    nextX = getRandomPos(STEP);
    FPSdelay = FPS_D;
  }
  else if (step == speed / 7 + 1) { /* find 2 ----------- 150*/
    nextX = getRandomPos(STEP);
    FPSdelay = FPS_D;
  }
  else if (step == speed / 6 + 1) { /* find 3 -----------200 */
    nextX = - STEP * random(4, 8);
    // nextX = getRandomPos(STEP);
    FPSdelay = FPS_D;
  }
  else if (step == speed / 5 + 1) { /* select color ------220 */
    FPSdelay = 200U;
    selectColor(STEP - 1);
  }
  else if (step == speed / 4 + 1) { /* show color -------- 222*/
    FPSdelay = FPS_D;
    nextX = pWIDTH;
  }
  else if (step == speed / 4 + 3) {
    step = 252;
  }

  //}
  if (posX < nextX) posX++;
  if (posX > nextX) posX--;

  if (stop_moving)   {
    FPSdelay = 80U;
    step++;
  } else {
    drawPalette(posX, posY, STEP);
    if ((nextX == pWIDTH) || (nextX == 0)) {
      /* show select color bar gradient */
      // LOG.printf_P(PSTR("step: %03d | Next x: %03d • %03d | fps %03d\n"), step, nextX, posX, FPSdelay);
      if (posX > 1) {
        gradientHorizontal(0, 0, (posX - 1), pHEIGHT, hue, hue, 255U, 96U, 255U);
      }
      if (posX > 3) DrawLine(posX - 3, CENTER_Y_MINOR, posX - 3, CENTER_Y_MAJOR, CHSV( hue, 192U, 255U));
    }
  }
  stop_moving = (posX == nextX);
}

// =====================================
//                Contacts
//             © Yaroslaw Turbin
//        Adaptation © SlingMaster
// =====================================

static const uint8_t exp_gamma[256] = {
  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,
  1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
  1,   2,   2,   2,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,
  4,   4,   4,   4,   4,   5,   5,   5,   5,   5,   6,   6,   6,   7,   7,
  7,   7,   8,   8,   8,   9,   9,   9,   10,  10,  10,  11,  11,  12,  12,
  12,  13,  13,  14,  14,  14,  15,  15,  16,  16,  17,  17,  18,  18,  19,
  19,  20,  20,  21,  21,  22,  23,  23,  24,  24,  25,  26,  26,  27,  28,
  28,  29,  30,  30,  31,  32,  32,  33,  34,  35,  35,  36,  37,  38,  39,
  39,  40,  41,  42,  43,  44,  44,  45,  46,  47,  48,  49,  50,  51,  52,
  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,  64,  65,  66,  67,
  68,  70,  71,  72,  73,  74,  75,  77,  78,  79,  80,  82,  83,  84,  85,
  87,  89,  91,  92,  93,  95,  96,  98,  99,  100, 101, 102, 105, 106, 108,
  109, 111, 112, 114, 115, 117, 118, 120, 121, 123, 125, 126, 128, 130, 131,
  133, 135, 136, 138, 140, 142, 143, 145, 147, 149, 151, 152, 154, 156, 158,
  160, 162, 164, 165, 167, 169, 171, 173, 175, 177, 179, 181, 183, 185, 187,
  190, 192, 194, 196, 198, 200, 202, 204, 207, 209, 211, 213, 216, 218, 220,
  222, 225, 227, 229, 232, 234, 236, 239, 241, 244, 246, 249, 251, 253, 254,
  255
};

void Contacts() {
  if (loadingFlag) {
    loadingFlag = false;
    FPSdelay = 80U;
    FastLED.clear();
  }
  int a = millis() / floor((255 - modes[currentMode].Speed) / 10);
  hue = floor(modes[currentMode].Scale / 17);
  for (int x = 0; x < pWIDTH; x++) {
    for (int y = 0; y < pHEIGHT; y++) {
      int index = XY(x, y);
      uint8_t color1 = exp_gamma[sin8((x - 8) * cos8((y + 20) * 4) / 4)];
      uint8_t color2 = exp_gamma[(sin8(x * 16 + a / 3) + cos8(y * 8 + a / 2)) / 2];
      uint8_t color3 = exp_gamma[sin8(cos8(x * 8 + a / 3) + sin8(y * 8 + a / 4) + a)];
      if (hue == 0) {
        leds[index].b = color3 / 4;
        leds[index].g = color2;
        leds[index].r = 0;
      } else if (hue == 1) {
        leds[index].b = color1;
        leds[index].g = 0;
        leds[index].r = color3 / 4;
      } else if (hue == 2) {
        leds[index].b = 0;
        leds[index].g = color1 / 4;
        leds[index].r = color3;
      } else if (hue == 3) {
        leds[index].b = color1;
        leds[index].g = color2;
        leds[index].r = color3;
      } else if (hue == 4) {
        leds[index].b = color3;
        leds[index].g = color1;
        leds[index].r = color2;
      } else if (hue == 5) {
        leds[index].b = color2;
        leds[index].g = color3;
        leds[index].r = color1;
      }
    }
  }
}

// *********************  ЗВЕЗДОЧКИ ******************
#define STARS_FADE_STEP 5     // шаг уменьшения яркости
uint8_t drawRays = 0;
void starsRoutine() {
  if (loadingFlag) {
    // modeCode = MC_STARS;
    loadingFlag = false;
    loopCounter = 0;
    hue = 46;
    //   0               1         2        3        4
    // ">Случайный выбор,Без лучей,Лучи '+',Лучи 'X',Лучи '+' и 'X'"
    drawRays = getEffectScaleParamValue2(thisMode);
    if (drawRays == 0) drawRays = random8(1, 4);
    FastLED.clear();  // очистить
  }
  delay(5);
  fader(STARS_FADE_STEP);
  uint8_t spd = getEffectSpeedValue(thisMode);
  if (spd > 0 && loopCounter++ < map8(spd, 0, 30)) return;
  loopCounter = 0;
  uint8_t effectBrightness = getBrightnessCalculated(globalBrightness, getEffectContrastValue(thisMode));
  uint8_t fadeBrightness =  effectBrightness / 4 * 3;
  uint8_t the_color = getEffectScaleParamValue(thisMode);
  uint8_t color = the_color;
  int8_t  delta = random8(0, 12) - 6;
  if (the_color < 2) {
    color = random8(0, 255);
  } else if (the_color > 252) {
    color = hue += (spd == 0 ? 1 : 2);
  } else {
    color += delta;
  }
  uint8_t cnt = 0;
  while (cnt < 25) {
    cnt++;
    uint8_t x = random8(1, pWIDTH - 1);
    uint8_t y = random8(1, pHEIGHT - 1);
    bool enable = drawRays == 1 ||
                  getPixColorXY(x,   y  ) == 0 &&
                  getPixColorXY(x + 1, y  ) == 0 &&
                  getPixColorXY(x - 1, y  ) == 0 &&
                  getPixColorXY(x,   y + 1) == 0 &&
                  getPixColorXY(x,   y - 1) == 0 &&
                  getPixColorXY(x + 1, y + 1) == 0 &&
                  getPixColorXY(x + 1, y - 1) == 0 &&
                  getPixColorXY(x - 1, y + 1) == 0 &&
                  getPixColorXY(x - 1, y - 1) == 0;
    if (enable) {
      uint8_t sat = (random8(0, 100) % 10 == 0) ? 32 : 255;   // Одна из 10 звезд - белая
      CHSV star_color = CHSV(color, sat, effectBrightness);
      // Центр
      idx = getPixelNumber(x, y);
      if (idx >= 0) leds[idx] = star_color;
      if (drawRays > 1) {
        // Стороны лучей
        star_color = CHSV(color, sat, fadeBrightness);
        bool useXRay = random8(0, 50) % 2 == 0;
        if (drawRays == 3 || drawRays == 4 && useXRay) {
          // Тип - X
          idx = getPixelNumber(x + 1, y + 1);
          if (idx >= 0) leds[idx] = star_color;
          idx = getPixelNumber(x - 1, y + 1);
          if (idx >= 0) leds[idx] = star_color;
          idx = getPixelNumber(x + 1, y - 1);
          if (idx >= 0) leds[idx] = star_color;
          idx = getPixelNumber(x - 1, y - 1);
          if (idx >= 0) leds[idx] = star_color;
        } else if (drawRays == 2 || drawRays == 4 && !useXRay) {
          // Тип - крест +
          idx = getPixelNumber(x + 1, y);
          if (idx >= 0) leds[idx] = star_color;
          idx = getPixelNumber(x - 1, y);
          if (idx >= 0) leds[idx] = star_color;
          idx = getPixelNumber(x, y + 1);
          if (idx >= 0) leds[idx] = star_color;
          idx = getPixelNumber(x, y - 1);
          if (idx >= 0) leds[idx] = star_color;
        }
      }
      break;
    }
  }
}

// *********************  ЗВЕЗДОЧКИ-2 (Штора) ******************

#define STARS2_FADE_STEP 10   // шаг уменьшения яркости
#define BACK_BRIGHTNESS 20
#define STAR_BRIGHTNESS 36
int8_t  *starState;    // 0 - яркость не меняется 1 - яркость увеличивается -1 - яркость уменьшается
uint8_t *starBright;   // Текущая яркость звезды
uint8_t  numStarsWidth;
uint8_t  numStarsHeight;
uint16_t numStars;
void stars2Routine() {
  drawRays = getEffectScaleParamValue2(thisMode);
  uint8_t contrast = getEffectContrastValue(thisMode);
  uint8_t delta2 = 0;
  uint8_t delta = 255 - globalBrightness;
  if (drawRays < 2) {
    if (delta > 200) delta2 = delta / 3; else if (delta > 150) delta2 = delta / 4; else if (delta > 100) delta2 = delta / 5; else if (delta > 50 ) delta2 = delta / 6; else
      delta2 = delta / 7;
  } else {
    if (delta > 200) delta2 = delta / 4; else if (delta > 150) delta2 = delta / 5; else if (delta > 100) delta2 = delta / 6; else if (delta > 50 ) delta2 = delta / 7; else
      delta2 = delta / 8;
  }
  uint8_t backBrightness = BACK_BRIGHTNESS + delta2;
  uint8_t starBrightness = constrain(STAR_BRIGHTNESS + delta2, STAR_BRIGHTNESS, 255);
  uint8_t maxEffectBrightness = constrain(contrast, 2 * starBrightness, 255);
  uint8_t maxFadeBrightness = maxEffectBrightness / 4 * 3;
  if (loadingFlag) {
    // modeCode = MC_STARS2;
    loadingFlag = false;
    numStarsWidth = pWIDTH / 4;
    numStarsHeight = pHEIGHT / 4;
    numStars = numStarsWidth * numStarsHeight;
    hue = 0;
    if (starState  == NULL) {
      starState  = new int8_t  [numStars];
    }
    if (starBright == NULL) {
      starBright = new uint8_t [numStars];
    }
    // Заполнить массив начальной яркости звезд
    FOR_i(0, numStars) {
      starState[i] = 0;
      starBright[i] = starBrightness;
    }
    FastLED.clear();  // очистить
  }
  // На каждый 5 (или менее в зависимости от размера матрицы) шаг "зажигаем" следующую звезду
  loopCounter++;
  if (loopCounter == 5 - numStars / 256) {
    loopCounter = 0;
    idx = random16(0, numStars);
    if (starState[idx] == 0) {
      starState[idx] = 1;
      // Некоторые звезды зажигаются не плавно, а вспышкой и плавно угасает
      if (random8(0, 100) % 4 == 0) {
        starBright[idx] = maxEffectBrightness - STARS2_FADE_STEP;
      }
    }
  }
  // В режимах без фона крайние положения двиджка "вариант" включают режим прокрутки по радуге
  uint8_t color = getEffectScaleParamValue(thisMode);
  if (drawRays >= 2 && (color < 2 || color > 253)) {
    color = hue;
    loopCounter2++;
    if (loopCounter2 == 10) {
      loopCounter2 = 0;
      hue += 1;
    }
  }
  // Заливка поля цветом фона для режима с фоном или черным для режима без фона
  CHSV back_color = CHSV(color, 255, drawRays < 2 ? backBrightness : 0) ;
  fillAll(back_color);
  FOR_x(0, numStarsWidth) {
    FOR_y(0, numStarsHeight) {
      // Корректировка яркости (угасание/зажигания) звезды
      idx = x + numStarsWidth * y;
      uint16_t br = starBright[idx];
      br += starState[idx] * STARS2_FADE_STEP;
      if (br >= maxEffectBrightness) {
        // При достижении максимальной яркости - переключить на "угасание"
        starState[idx] = -1;
        br = maxEffectBrightness;
      } else if (br <= starBrightness) {
        // При достижении минимальной яркости - переключить на "ожидание"
        starState[idx] = 0;
        br = starBrightness;
      }
      starBright[idx] = br;
      // Отрисовать звезду
      uint8_t xp = x * 4 + 1;
      uint8_t yp = y * 4 + x % 2 + 1;
      uint8_t effectBrightness = constrain(starBright[x + y * numStarsWidth], starBrightness, maxEffectBrightness);
      // Центр
      idx = getPixelNumber(xp, yp);
      CHSV star_color = CHSV(color, 255, effectBrightness);
      if (idx >= 0) leds[idx] = star_color;
      if (drawRays == 1 || drawRays == 3) {
        // Стороны лучей
        uint8_t fadeBrightness = effectBrightness / 4 * 3;
        if (fadeBrightness > maxFadeBrightness) fadeBrightness = maxFadeBrightness;
        if (fadeBrightness < backBrightness) fadeBrightness = backBrightness;
        star_color = CHSV(color, 255, fadeBrightness);
        idx = getPixelNumber(xp + 1, yp);
        if (idx >= 0) leds[idx] = star_color;
        idx = getPixelNumber(xp - 1, yp);
        if (idx >= 0) leds[idx] = star_color;
        idx = getPixelNumber(xp, yp + 1);
        if (idx >= 0) leds[idx] = star_color;
        idx = getPixelNumber(xp, yp - 1);
        if (idx >= 0) leds[idx] = star_color;
      }
    }
  }
}
void stars2RoutineRelease() {
  if (starState == NULL) {
    delete [] starState;
    starState = NULL;
  }
  if (starBright == NULL) {
    delete [] starBright;
    starBright = NULL;
  }
}

// ============= Hourglass ==============
//             © SlingMaster
//             EFF_HOURGLASS
//             Песочные часы
//---------------------------------------
void Hourglass() {
  const float SIZE = 0.4;
  const uint8_t h = floor(SIZE * pHEIGHT);
  uint8_t posX = 0;
  const uint8_t topPos  = pHEIGHT - h;
  const uint8_t route = pHEIGHT - h - 1;
  const uint8_t STEP = 18U;
  if (loadingFlag) {
    loadingFlag = false;
    pcnt = 0;
    deltaHue2 = 0;
    hue2 = 0;
    FastLED.clear();
    hue = modes[currentMode].Scale * 2.55;
    for (uint8_t x = 0U; x < ((pWIDTH / 2)); x++) {
      for (uint8_t y = 0U; y < h; y++) {
        drawPixelXY(round(pWIDTH / 2) - x, pHEIGHT - y - 1, CHSV(hue, 255, 255 - x * STEP));
        drawPixelXY(round(pWIDTH / 2) + x, pHEIGHT - y - 1, CHSV(hue, 255, 255 - x * STEP));
      }
    }
  }
  if (hue2 == 0) {
    posX = floor(pcnt / 2);
    uint8_t posY = pHEIGHT - h - pcnt;
    if ((posY < (pHEIGHT - h - 2)) && (posY > deltaHue2)) {
      drawPixelXY(round(pWIDTH / 2), posY, CHSV(hue, 255, 255));
      drawPixelXY(round(pWIDTH / 2), posY - 2, CHSV(hue, 255, 255));
      drawPixelXY(round(pWIDTH / 2), posY - 4, CHSV(hue, 255, 255));
      if (posY < (pHEIGHT - h - 3)) {
        drawPixelXY(round(pWIDTH / 2), posY + 1, CHSV(hue, 255, 0 ));
      }
    }
    // draw body hourglass
    if (pcnt % 2 == 0) {
      drawPixelXY(round(pWIDTH / 2) - posX, pHEIGHT - deltaHue2 - 1, CHSV(hue, 255, 0));
      drawPixelXY(round(pWIDTH / 2) - posX, deltaHue2, CHSV(hue, 255, 255 - posX * STEP));
    } else {
      drawPixelXY(round(pWIDTH / 2) + posX, pHEIGHT - deltaHue2 - 1, CHSV(hue, 255, 0));
      drawPixelXY(round(pWIDTH / 2) + posX, deltaHue2, CHSV(hue, 255, 255 - posX * STEP));
    }
    if (pcnt > pWIDTH - 1) {
      deltaHue2++;
      pcnt = 0;
      if (modes[currentMode].Scale > 95) {
        hue += 4U;
      }
    }
    pcnt++;
    if (deltaHue2 > h) {
      deltaHue2 = 0;
      hue2 = 1;
    }
  }
  // имитация переворота песочных часов
  if (hue2 > 0) {
    for (uint8_t x = 0U; x < pWIDTH; x++) {
      for (uint8_t y = pHEIGHT; y > 0U; y--) {
        drawPixelXY(x, y, getPixColorXY(x, y - 1U));
        drawPixelXY(x, y - 1, 0x000000);
      }
    }
    hue2++;
    hue++;
    if (hue2 > route) {
      hue2 = 0;
    }
  }
}

// ============== ByEffect ==============
//             © SlingMaster
//             EFF_BY_EFFECT
//            Побочный Эффект
// --------------------------------------
void ByEffect() {
  uint8_t saturation;
  uint8_t delta;
  if (loadingFlag) {
    loadingFlag = false;
    deltaValue = 0;
    step = deltaValue;
    FastLED.clear();
  }
  hue = floor(step / 32) * 32U;
  dimAll(180);
  // ------
  saturation = 255U;
  delta = 0;
  for (uint8_t x = 0U; x < pWIDTH + 1 ; x++) {
    if (x % 8 == 0) {
      gradientVertical( x - deltaValue, floor(pHEIGHT * 0.75), x + 1U - deltaValue, pHEIGHT,  hue, hue + 2, 250U, 0U, 255U);
      if (modes[currentMode].Scale > 50) {
        delta = random8(200U);
      }
      drawPixelXY(x - 2 - deltaValue, floor(pHEIGHT * 0.7), CHSV(step, saturation - delta, 128 + random8(128)));
      drawPixelXY(x + 2 - deltaValue, floor(pHEIGHT * 0.7), CHSV(step, saturation, 128 + random8(128)));
      drawPixelXY(x - deltaValue, floor(pHEIGHT * 0.6), CHSV(hue, 255U, 190 + random8(65)));
      if (modes[currentMode].Scale > 50) {
        delta = random8(200U);
      }
      drawPixelXY(x - 1 - deltaValue, CENTER_Y_MINOR, CHSV(step, saturation, 128 + random8(128)));
      drawPixelXY(x + 1 - deltaValue, CENTER_Y_MINOR, CHSV(step, saturation - delta, 128 + random8(128)));
      drawPixelXY(x - deltaValue, floor(pHEIGHT * 0.4), CHSV(hue, 255U, 200U));
      if (modes[currentMode].Scale > 50) {
        delta = random8(200U);
      }
      drawPixelXY(x - 2 - deltaValue, floor(pHEIGHT * 0.3), CHSV(step, saturation - delta, 96 + random8(128)));
      drawPixelXY(x + 2 - deltaValue, floor(pHEIGHT * 0.3), CHSV(step, saturation, 96 + random8(128)));
      gradientVertical( x - deltaValue, 0U, x + 1U - deltaValue, floor(pHEIGHT * 0.25),  hue + 2, hue, 0U, 250U, 255U);
      if (modes[currentMode].Scale > 50) {
        drawPixelXY(x + 3 - deltaValue, pHEIGHT - 3U, CHSV(step, 255U, 255U));
        drawPixelXY(x - 3 - deltaValue, CENTER_Y_MINOR, CHSV(step, 255U, 255U));
        drawPixelXY(x + 3 - deltaValue, 2U, CHSV(step, 255U, 255U));
      }
    }
  }
  // ------
  deltaValue++;
  if (deltaValue >= 8) {
    deltaValue = 0;
  }
  step++;
}

//Здесь находится код вроде как работающих, но не задействованных в прошивке по тем или иным причинам эффектов
//волшебный фонарь
/*void MagicLantern() { //непонятный эффект, но что-то рисует

  static uint8_t saturation;
  static uint8_t brightness;
  static uint8_t low_br;
  uint8_t delta;
  const uint8_t PADDING = pHEIGHT * 0.25;
  const uint8_t WARM_LIGHT = 55U;
  const uint8_t STEP = 4U;
  if (loadingFlag) {
    loadingFlag = false;
    deltaValue = 0;
    step = deltaValue;
    if (modes[currentMode].Speed > 52) {
      // brightness = 50 + modes[currentMode].Speed;
      brightness = map(modes[currentMode].Speed, 1, 255, 50U, 250U);
      low_br = 50U;
    } else {
      brightness = 0U;
      low_br = 0U;
    }
    saturation = (modes[currentMode].Scale > 50U) ? 64U : 0U;
    if (abs (70 - modes[currentMode].Scale) <= 5) saturation = 170U;
    FastLED.clear();

  }
  dimAll(170);
  hue = (modes[currentMode].Scale > 95) ? floor(step / 32) * 32U : modes[currentMode].Scale * 2.55;

  // ------
  for (uint8_t x = 0U; x < pWIDTH + 1 ; x++) {

    // light ---
    if (low_br > 0) {
      gradientVertical( x - deltaValue, CENTER_Y_MAJOR, x + 1U - deltaValue, pHEIGHT - PADDING - 1,  WARM_LIGHT, WARM_LIGHT, brightness, low_br, saturation);
      gradientVertical( pWIDTH - x + deltaValue, CENTER_Y_MAJOR, pWIDTH - x + 1U + deltaValue, pHEIGHT - PADDING - 1,  WARM_LIGHT, WARM_LIGHT, brightness, low_br, saturation);
      gradientVertical( x - deltaValue, PADDING + 1, x + 1U - deltaValue, CENTER_Y_MAJOR, WARM_LIGHT, WARM_LIGHT, low_br + 10, brightness, saturation);
      gradientVertical( pWIDTH - x + deltaValue, PADDING + 1, pWIDTH - x + 1U + deltaValue, CENTER_Y_MAJOR, WARM_LIGHT, WARM_LIGHT, low_br + 10, brightness, saturation);
    } else {
      if (x % (STEP + 1) == 0) {
        leds[XY(random8(pWIDTH), random8(PADDING + 2, pHEIGHT - PADDING - 2))] = CHSV(step - 32U, random8(128U, 255U), 255U);
      }
      if ((modes[currentMode].Speed < 25) & (low_br == 0)) {
        deltaValue = 0;
        if (x % 2 != 0) {
          gradientVertical( x - deltaValue, pHEIGHT - PADDING, x + 1U - deltaValue, pHEIGHT,  hue, hue + 2, 64U, 20U, 255U);
          gradientVertical( (pWIDTH - x + deltaValue), 0U,  (pWIDTH - x + 1U + deltaValue), PADDING,  hue, hue, 42U, 64U, 255U);
        }
        //        deltaValue = 0;
      }
    }
    if (x % STEP == 0) {
      // body --
      gradientVertical( x - deltaValue, pHEIGHT - PADDING, x + 1U - deltaValue, pHEIGHT,  hue, hue + 2, 255U, 20U, 255U);
      gradientVertical( (pWIDTH - x + deltaValue), 0U,  (pWIDTH - x + 1U + deltaValue), PADDING,  hue, hue, 42U, 255U, 255U);
    }
  }
  // ------

  deltaValue++;
  if (deltaValue >= STEP) {
    deltaValue = 0;
  }

  step++;
  }*/

// ======== Digital Тurbulence =========
//             © SlingMaster
//        Цифровая турбулентность типа работает
// =====================================
/*uint8_t SpeedFactor(uint8_t spd) {
  uint8_t result = spd * NUM_LEDS / 1024.0;
  return result;
  }

  CRGB makeDarker( const CRGB& color, fract8 howMuchDarker)
  {
  CRGB newcolor = color;
  //newcolor.nscale8( 255 - howMuchDarker);
  newcolor.fadeToBlackBy(howMuchDarker);//эквивалент
  return newcolor;
  }

  void drawRandomCol(uint8_t x, uint8_t y, uint8_t offset, uint32_t count) {
  const byte STEP = 32;
  const byte D = pHEIGHT / 8;
  uint8_t color = floor(y / D) * STEP + offset;

  if (count == 0U) {
    drawPixelXY(x, y, CHSV(color, 255, random8(8U) == 0U ? (step % 2U ? 0 : 255) : 0));
  } else {
    drawPixelXY(x, y, CHSV(color, 255, (bitRead(count, y ) == 1U) ? (step % 5U ? 0 : 255) : 0));
  }
  }

  //---------------------------------------
  void Turbulence() {
  const byte STEP_COLOR = 255 / pHEIGHT;
  const byte STEP_OBJ = 8;
  const byte DEPTH = 2;
  static uint32_t count; // 16777216; = 65536
  uint32_t curColor;
  if (loadingFlag) {
    loadingFlag = false;
    step = 0U;
    deltaValue = 0;
    hue = 0;
    if (modes[currentMode].Speed < 20U) {
      FPSdelay = SpeedFactor(30);
    }
    FastLED.clear();
  }
  deltaValue++;
  for (uint8_t y = pHEIGHT; y > 0; y--) {
    drawRandomCol(0, y - 1, hue, count);
    drawRandomCol(pWIDTH - 1, y - 1, hue + 128U, count);
    for (uint8_t x = CENTER_X_MAJOR - 1; x > 0; x--) {
      if (x > CENTER_X_MAJOR) {
        if (random8(2) == 0U) {
          CRGB newColor = getPixColorXY(x, y - 1 );
        }
      }
      curColor = getPixColorXY(x - 1, y - 1);
      if (x < CENTER_X_MAJOR - DEPTH / 2) {
        drawPixelXY(x, y - 1, curColor);
      } else {
        if (curColor != 0U) drawPixelXY(x, y - 1, curColor);
      }
    }

    // right -----
    for (uint8_t x = CENTER_X_MAJOR + 1; x < pWIDTH; x++) {
      if (x < CENTER_X_MAJOR + DEPTH ) {
        if (random8(2) == 0U)  {
          CRGB newColor = getPixColorXY(x, y - 1 );
        }
      }

      curColor = getPixColorXY(x, y - 1);
      if (x > CENTER_X_MAJOR + DEPTH / 2 ) {
        drawPixelXY(x - 1, y - 1, curColor);
      } else {
        if (curColor != 0U) drawPixelXY(x - 1, y - 1, curColor);
      }
    }

    for (uint8_t x = CENTER_X_MAJOR - DEPTH; x < CENTER_X_MAJOR + DEPTH; x++) {
      drawPixelXY(x, y,  makeDarker(getPixColorXY(x, y - 1 ), 128 / y));
      if (y == 1) {
        drawPixelXY(x, 0, CRGB::Black);
      }
    }
  }

  if (modes[currentMode].Scale > 50) {
    count++;
    if (count % 256 == 0U) hue += 16U;
  } else {
    count = 0;
  }
  step++;
  }*/


/*
  // =============== Bamboo ===============
  //             © SlingMaster
  //                 Бамбук типа работает можно в принципе в продакшн
  // --------------------------------------
  uint8_t nextColor(uint8_t posY, uint8_t base, uint8_t next ) {
  const byte posLine = (pHEIGHT > 16) ? 4 : 3;
  if ((posY + 1 == posLine) | (posY == posLine)) {
    return next;
  } else {
    return base;
  }
  }

  // --------------------------------------
  void Bamboo() {
  const uint8_t gamma[7] = {0, 32, 144, 160, 196, 208, 230};
  static float index;
  const byte DELTA = 4U;
  const uint8_t VG_STEP = 64U;
  const uint8_t V_STEP = 32U;
  const byte posLine = (pHEIGHT > 16) ? 4 : 3;
  const uint8_t SX = 5;
  const uint8_t SY = 10;
  static float deltaX = 0;
  static bool direct = false;
  uint8_t posY;
  static uint8_t colLine;
  const float STP = 0.2;
  if (loadingFlag) {
    loadingFlag = false;
    index = STP;
    uint8_t idx = map(modes[currentMode].Scale, 5, 95, 0U, 6U);;
    colLine = gamma[idx];
    step = 0U;
  }

  // *** ---
  for (int y = 0; y < pHEIGHT + SY; y++) {
    if (modes[currentMode].Scale < 50U) {
      if (step % 128 == 0U) {
        deltaX += STP * ((direct) ? -1 : 1);
     if ((deltaX > 1) | (deltaX < -1)) direct = !direct;
      }
    } else {
      deltaX = 0;
    }
    posY = y;
    for (int x = 0; x < pWIDTH + SX; x++) {
      if (y == posLine) {
        drawPixelXYF(x , y - 1, CHSV(colLine, 255U, 128U));
        drawPixelXYF(x, y, CHSV(colLine, 255U, 96U));
        if (pHEIGHT > 16) {
          drawPixelXYF(x, y - 2, CHSV(colLine, 10U, 64U));
        }
      }
      if ((x % SX == 0U) & (y % SY == 0U)) {
        for (int i = 1; i < (SY - 3); i++) {
          if (i < 3) {
            posY = y - i + 1 - DELTA + index;
            drawPixelXYF(x - 3 + deltaX, posY, CHSV(nextColor(posY, 96, colLine), 255U, 255 - V_STEP * i));
            posY = y - i + index;
            drawPixelXYF(x + deltaX, posY, CHSV(nextColor(posY, 96, colLine), 255U, 255 - VG_STEP * i));
          }
          posY = y - i - DELTA + index;
          drawPixelXYF(x - 4 + deltaX, posY , CHSV(nextColor(posY, 96, colLine), 180U, 255 - V_STEP * i));
          posY = y - i + 1 + index;
          drawPixelXYF(x - 1 + deltaX, posY , CHSV(nextColor(posY, ((i == 1) ? 96 : 80), colLine), 255U, 255 - V_STEP * i));
        }
      }
    }
    step++;
  }
  if (index >= SY)  {
    index = 0;
  }
  fadeToBlackBy(leds, NUM_LEDS, 60);
  index += STP;
  }*/


/*
  // ============ Plasma Waves ============
  //              © Stepko
  //        Adaptation © alvikskor
  //             Плазменные Волны  можно объединить с осциллятором как второй вариант
  // --------------------------------------

  /* используется та же самая палитра из эффекта Контакт
  static const uint8_t exp_gamma[256] PROGMEM = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,
    1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
    1,   2,   2,   2,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,
    4,   4,   4,   4,   4,   5,   5,   5,   5,   5,   6,   6,   6,   7,   7,
    7,   7,   8,   8,   8,   9,   9,   9,   10,  10,  10,  11,  11,  12,  12,
    12,  13,  13,  14,  14,  14,  15,  15,  16,  16,  17,  17,  18,  18,  19,
    19,  20,  20,  21,  21,  22,  23,  23,  24,  24,  25,  26,  26,  27,  28,
    28,  29,  30,  30,  31,  32,  32,  33,  34,  35,  35,  36,  37,  38,  39,
    39,  40,  41,  42,  43,  44,  44,  45,  46,  47,  48,  49,  50,  51,  52,
    53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,  64,  65,  66,  67,
    68,  70,  71,  72,  73,  74,  75,  77,  78,  79,  80,  82,  83,  84,  85,
    87,  89,  91,  92,  93,  95,  96,  98,  99,  100, 101, 102, 105, 106, 108,
    109, 111, 112, 114, 115, 117, 118, 120, 121, 123, 125, 126, 128, 130, 131,
    133, 135, 136, 138, 140, 142, 143, 145, 147, 149, 151, 152, 154, 156, 158,
    160, 162, 164, 165, 167, 169, 171, 173, 175, 177, 179, 181, 183, 185, 187,
    190, 192, 194, 196, 198, 200, 202, 204, 207, 209, 211, 213, 216, 218, 220,
    222, 225, 227, 229, 232, 234, 236, 239, 241, 244, 246, 249, 251, 253, 254,
    255
  };*/

/*
  void Plasma_Waves() {
  static int64_t frameCount = 0;
  if (loadingFlag) {

    loadingFlag = false;
    hue = modes[currentMode].Scale / 10;
  }
  //EVERY_N_MILLIS(modes[currentMode].Speed) {//(1000 / 60) {
  //  frameCount++;
  //}
  FPSdelay = 1;//64 - modes[currentMode].Speed / 4;

  frameCount++;
  uint8_t t1 = cos8((42 * frameCount) / (132 - modes[currentMode].Speed / 2));
  uint8_t t2 = cos8((35 * frameCount) / (132 - modes[currentMode].Speed / 2));
  uint8_t t3 = cos8((38 * frameCount) / (132 - modes[currentMode].Speed / 2));

  for (uint16_t y = 0; y < pHEIGHT; y++) {
    for (uint16_t x = 0; x < pWIDTH; x++) {
      // Calculate 3 seperate plasma waves, one for each color channel
      uint8_t r = cos8((x << 3) + (t1 >> 1) + cos8(t2 + (y << 3) + modes[currentMode].Scale));
      uint8_t g = cos8((y << 3) + t1 + cos8((t3 >> 2) + (x << 3)) +modes[currentMode].Scale);
      uint8_t b = cos8((y << 3) + t2 + cos8(t1 + x + (g >> 2) + modes[currentMode].Scale));

      // uncomment the following to enable gamma correction
      // r = pgm_read_byte_near(exp_gamma + r);
      switch (hue) {
          case 0:
              r = pgm_read_byte(&exp_gamma[r]);
              g = pgm_read_byte(&exp_gamma[g]);
              b = pgm_read_byte(&exp_gamma[b]);
              break;
          case 1:
              r = pgm_read_byte(&exp_gamma[r]);
              b = pgm_read_byte(&exp_gamma[g]);
              g = pgm_read_byte(&exp_gamma[b]);
              break;
          case 2:
              g = pgm_read_byte(&exp_gamma[r]);
              r = pgm_read_byte(&exp_gamma[g]);
              b = pgm_read_byte(&exp_gamma[b]);
              break;
          case 3:
              r = pgm_read_byte(&exp_gamma[r])/2;
              g = pgm_read_byte(&exp_gamma[g]);
              b = pgm_read_byte(&exp_gamma[b]);
              break;
          case 4:
              r = pgm_read_byte(&exp_gamma[r]);
              g = pgm_read_byte(&exp_gamma[g])/2;
              b = pgm_read_byte(&exp_gamma[b]);
              break;
          case 5:
              r = pgm_read_byte(&exp_gamma[r]);
              g = pgm_read_byte(&exp_gamma[g]);
              b = pgm_read_byte(&exp_gamma[b])/2;
              break;
          case 6:
              r = pgm_read_byte(&exp_gamma[r])*3;
              g = pgm_read_byte(&exp_gamma[g]);
              b = pgm_read_byte(&exp_gamma[b]);
              break;
          case 7:
              r = pgm_read_byte(&exp_gamma[r]);
              g = pgm_read_byte(&exp_gamma[g])*3;
              b = pgm_read_byte(&exp_gamma[b]);
              break;
          case 8:
              r = pgm_read_byte(&exp_gamma[r]);
              g = pgm_read_byte(&exp_gamma[g]);
              b = pgm_read_byte(&exp_gamma[b])*3;
              break;

      }
      // g = pgm_read_byte_near(exp_gamma + g);
      // b = pgm_read_byte_near(exp_gamma + b);

      leds[XY(x, y)] = CRGB(r, g, b);
    }
    //hue++;
  }
  // blurScreen(beatsin8(3, 64, 80));
  }*/

// ------------------------------ РЕЖИМ / ЭФФЕКТ ЧАСЫ ----------------------
// (c) SottNick типа работают. можно что-то с ними сделать
/*
  #define CLOCK_SAVE_MODE     // удалите или закомментируйте эту строчку, чтобы цифры всегда оставались на одном месте, не двигались по вертикали (не хорошо для светодиодов. выгорают зря)
  #if (pHEIGHT > 12) || (pHEIGHT < 11)
  #define CLOCK_BLINKING      // удалите или закомментируйте эту строчку, чтобы точки не мигали
  #endif
  //uint8_t hue, hue2; // храним тут часы и минуты
  //uint8_t deltaHue, deltaHue2; // храним здесь задержки мигания точек
  //uint8_t deltaValue; // счётчик цикла / яркости точек на часах
  //uint8_t poleX, poleY; // храним здесь сдвиг циферблата по горизонтали и вертикали (переменные объявлены в эффекте Кубик Рубика)
  static const uint8_t clockFont3x5[10][3] PROGMEM = { // цифры зеркально и на левом боку (так проще рисовать в циклах и экономнее для памяти)
  { B11111,
    B10001,
    B11111
  },
  { B01001,
    B11111,
    B00001
  },
  { B10011,
    B10101,
    B01001
  },
  { B10001,
    B10101,
    B01010
  },
  { B11100,
    B00100,
    B11111
  },
  { B11101,
    B10101,
    B10010
  },
  { B01111,
    B10101,
    B10111
  },
  { B10011,
    B10100,
    B11000
  },
  { B11111,
    B10101,
    B11111
  },
  { B11101,
    B10101,
    B11110
  }
  };
  void drawDig3x5(uint8_t x, uint8_t y, uint8_t num, CRGB color) { // uint8_t hue, uint8_t sat, uint8_t bri = 255U
  for (uint8_t i = 0U; i < 3U; i++)
  {
    uint8_t m = pgm_read_byte(&clockFont3x5[num][i]);
    for (uint8_t j = 0U; j < 5U; j++)
      if ((m >> j) & 0x01)
        drawPixelXY((x + i) % pWIDTH, (y + j) % pHEIGHT, color);
  }
  }

  #if pHEIGHT > 10 // часы в столбик будут только если высота 11 пикселей и больше
  void clockRoutine() {
  if (loadingFlag)
  {
    loadingFlag = false;
    poleX = (modes[currentMode].Speed - 1U) % pWIDTH; //смещение цифр по горизонтали
  #ifdef CLOCK_BLINKING
  #if pHEIGHT > 13
    poleY = (modes[currentMode].Speed - 1U) / pWIDTH % (pHEIGHT - 13U);  //смещение цифр по вертикали (для режима CLOCK_SAVE_MODE будет меняться само)
  #else
    poleY = 0U;
  #endif
  #else
  #if pHEIGHT > 12
    poleY = (modes[currentMode].Speed - 1U) / pWIDTH % (pHEIGHT - 12U);  //смещение цифр по вертикали (для режима CLOCK_SAVE_MODE будет меняться само)
  #else // и для 12 и для 11 смещаться некуда. всё впритык
    poleY = 0U;
  #endif
  #endif
    hue2 = 255U; // количество минут в данный момент (первоначально запредельое значение)
    deltaHue2 = 0; // яркость точки в данный момент
    deltaValue = modes[currentMode].Scale * 2.55; // выбранный оттенок цифр
  }

  time_t currentLocalTime = now();

  if (minute(currentLocalTime) != hue2)
  {
  #ifdef CLOCK_SAVE_MODE
  #ifdef CLOCK_BLINKING
  #if pHEIGHT > 13
    poleY = (poleY + 1U) % (pHEIGHT - 13U);
  #endif
  #else
  #if pHEIGHT > 12
    poleY = (poleY + 1U) % (pHEIGHT - 12U);
  #endif
  #endif
  #endif
    step = 1U; // = CLOCK_REFRESH_DELAY; раньше делал постепенное затухание. получалось хуже
    hue = hour(currentLocalTime);
    hue2 = minute(currentLocalTime);
  }
  if (step > 0) // тут меняются цифры на часах
  {
    step--;
    //uint8_t bri = (CLOCK_REFRESH_DELAY - step) * 255.0 / CLOCK_REFRESH_DELAY;
    uint8_t sat = (modes[currentMode].Scale == 100) ? 0U : 255U;

    FastLED.clear();
    // рисуем цифры
  #ifdef CLOCK_BLINKING
    drawDig3x5(   poleX,               (poleY + 8U), hue  / 10U % 10U, CHSV(deltaValue, sat, 255U));
    drawDig3x5(  (poleX + 4U) % pWIDTH, (poleY + 8U), hue        % 10U, CHSV(deltaValue, sat, 255U));
  #else
  #if pHEIGHT > 11
    drawDig3x5( poleX,               (poleY + 7U), hue  / 10U % 10U, CHSV(deltaValue, sat, 255U));
    drawDig3x5((poleX + 4U) % pWIDTH, (poleY + 7U), hue        % 10U, CHSV(deltaValue, sat, 255U));
  #else // если матрица всего 11 пикселей в высоту, можно сэкономить 1 и впихнуть часы в неё. но если меньше, нужно брать код эффекта с высотой цифр 4 пикселя, а не 5
    drawDig3x5( poleX,               (poleY + 6U), hue  / 10U % 10U, CHSV(deltaValue, sat, 255U));
    drawDig3x5((poleX + 4U) % pWIDTH, (poleY + 6U), hue        % 10U, CHSV(deltaValue, sat, 255U));
  #endif
  #endif
    drawDig3x5(     poleX, poleY,                      hue2 / 10U % 10U, CHSV(deltaValue, sat, 255U));
    drawDig3x5(    (poleX + 4U) % pWIDTH, poleY,        hue2       % 10U, CHSV(deltaValue, sat, 255U));
  }

  #ifdef CLOCK_BLINKING
  // тут мигают точки
  //  if (deltaHue != 0U)
  //    deltaHue--;
  //  else
  //  {
  //    deltaHue = 4U; // множитель задержки 50 мс * 4+1U = 250 мс
  if (deltaHue2 & 0x01)
    deltaHue2 = deltaHue2 - ((deltaHue2 >  15U) ? 16U : 15U);//- ((deltaHue2 >  63U) ? 64U : 63U);
  else
    deltaHue2 = deltaHue2 + ((deltaHue2 < 240U) ? 16U : 15U);//+ ((deltaHue2 < 192U) ? 64U : 63U);

  drawPixelXY((poleX + 2U) % pWIDTH, poleY + 6U, CHSV(deltaValue, (modes[currentMode].Scale == 100) ? 0U : 255U, deltaHue2)); // цвет белый для .Scale=100
  drawPixelXY((poleX + 4U) % pWIDTH, poleY + 6U, CHSV(deltaValue, (modes[currentMode].Scale == 100) ? 0U : 255U, deltaHue2)); // цвет белый для .Scale=100
  //  }
  #endif //#ifdef CLOCK_BLINKING
  }
  #else // для матриц и гирлянд от 6 до 10 пикселей в высоту #if pHEIGHT > 10
  void clockRoutine() { // чтобы цифры были не в столбик, а в строчку
  if (loadingFlag)
  {
    loadingFlag = false;
    poleX = (modes[currentMode].Speed - 1U) % pWIDTH; //смещение цифр по горизонтали
    poleY = (modes[currentMode].Speed - 1U) / pWIDTH % (pHEIGHT - 5U);  //смещение цифр по вертикали (для режима CLOCK_SAVE_MODE будет меняться само)
    hue2 = 255U; // количество минут в данный момент (первоначально запредельое значение)
    deltaHue2 = 0; // яркость точки в данный момент
    deltaValue = modes[currentMode].Scale * 2.55; // выбранный оттенок цифр
  }
  // time_t currentLocalTime = getCurrentLocalTime();
  time_t currentLocalTime = now();
  if (minute(currentLocalTime) != hue2)
  {
  #ifdef CLOCK_SAVE_MODE
    poleY = (poleY + 1U) % (pHEIGHT - 5U);
  #endif
    step = 1U; // = CLOCK_REFRESH_DELAY; раньше делал постепенное затухание. получалось хуже
    hue = hour(currentLocalTime);
    hue2 = minute(currentLocalTime);
  }
  if (step > 0) // тут меняются цифры на часах
  {
    step--;
    //uint8_t bri = (CLOCK_REFRESH_DELAY - step) * 255.0 / CLOCK_REFRESH_DELAY;
    uint8_t sat = (modes[currentMode].Scale == 100) ? 0U : 255U;

    FastLED.clear();
    // рисуем цифры
    drawDig3x5( poleX               , poleY, hue  / 10U % 10U, CHSV(deltaValue, sat, 255U));
    drawDig3x5((poleX +  4U) % pWIDTH, poleY, hue        % 10U, CHSV(deltaValue, sat, 255U));
    drawDig3x5((poleX +  9U) % pWIDTH, poleY, hue2 / 10U % 10U, CHSV(deltaValue, sat, 255U));
    drawDig3x5((poleX + 13U) % pWIDTH, poleY, hue2       % 10U, CHSV(deltaValue, sat, 255U));
  }

  #ifdef CLOCK_BLINKING
  // тут мигают точки
  if (deltaHue2 & 0x01)
    deltaHue2 = deltaHue2 - ((deltaHue2 >  15U) ? 16U : 15U);//- ((deltaHue2 >  63U) ? 64U : 63U);
  else
    deltaHue2 = deltaHue2 + ((deltaHue2 < 240U) ? 16U : 15U);//+ ((deltaHue2 < 192U) ? 64U : 63U);

  drawPixelXY((poleX + 8U) % pWIDTH, poleY + 1U, CHSV(deltaValue, (modes[currentMode].Scale == 100) ? 0U : 255U, deltaHue2)); // цвет белый для .Scale=100
  drawPixelXY((poleX + 8U) % pWIDTH, poleY + 3U, CHSV(deltaValue, (modes[currentMode].Scale == 100) ? 0U : 255U, deltaHue2)); // цвет белый для .Scale=100
  //  }
  #endif //#ifdef CLOCK_BLINKING
  }
  #endif //#if pHEIGHT > 10
*/
