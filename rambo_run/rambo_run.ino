/*
  Rambo Run for Arduino Uno R3
  Display: Keyestudio 1.3" 128x64 OLED (SH1106) using the U8glib library

  Wiring:
    OLED (SPI)  GND -> GND    VCC -> 5V
                CLK -> D13    MOSI -> D11
                RES -> D8     DC -> D9      CS -> D10
    HC-SR04     VCC -> 5V     GND -> GND
                Trig -> D6    Echo -> D7
    Button      one leg -> D2, diagonal leg -> GND

  Library (Library Manager): U8glib by oliver

  How to play:
    Title screen: press the button to start.
    Cover the ultrasonic sensor with your hand to make Rambo jump.
    After you crash: press the button to go back to the title screen,
    then press it again to start a new run.
    The high score is saved, so it survives power off.
*/

#include "U8glib.h"
#include <EEPROM.h>

// ---------- Pins ----------
#define OLED_RES   8
#define OLED_DC    9
#define OLED_CS    10
// CLK = D13 and MOSI = D11 are used by U8glib directly.
const uint8_t TRIG_PIN   = 6;
const uint8_t ECHO_PIN   = 7;
const uint8_t BUTTON_PIN = 2;

// ---------- Display ----------
// 0 = software SPI, exactly as in the Keyestudio sample (always works, slower)
// 1 = hardware SPI (faster, smoother game). If it does not compile or the
//     screen stays blank, set this back to 0.
#define USE_HW_SPI 0

#if USE_HW_SPI
U8GLIB_SH1106_128X64 u8g(OLED_CS, OLED_DC);
#else
U8GLIB_SH1106_128X64 u8g(13, 11, OLED_CS, OLED_DC);  // SCK, MOSI, CS, A0(DC)
#endif

const int SCREEN_W = 128;
const int SCREEN_H = 64;

// ---------- Tuning ----------
// Set to 1 to print sensor distance (cm) to the Serial Monitor at 115200 baud.
// -1 means no echo was received. Useful for finding a good COVER_DISTANCE_CM.
// If jumps fail when your hand is very close (under about 2 cm), check here.
#define DEBUG_DISTANCE 0

const int COVER_DISTANCE_CM = 10;            // closer than this counts as "covered"
const unsigned long ECHO_TIMEOUT_US = 6000;  // about 100 cm max range
const unsigned long FRAME_MS = 33;           // target frame time
const float MAX_DT = 2.0;                    // cap on catch-up when frames are slow
const unsigned long DEBOUNCE_MS = 40;
const unsigned long DEAD_LOCKOUT_MS = 800;   // ignore button briefly after crashing

const int GROUND_Y = 58;
const int RAMBO_X = 10;
const int RAMBO_W = 16;
const int RAMBO_H = 16;
const float RAMBO_GROUND_Y = GROUND_Y - RAMBO_H;
const float JUMP_VELOCITY = -5.5;
const float GRAVITY = 0.55;

const float START_SPEED = 3.0;
const float MAX_SPEED = 6.0;
const float SPEED_PER_POINT = 0.008;
const float SCORE_PER_PIXEL = 0.1;

const int MIN_GAP_BASE = 50;
const int GAP_PER_SPEED = 14;
const int GAP_RANDOM = 70;

// Cactus size (used for both drawing and hitboxes)
const int CACTUS_W = 7;        // width of one cactus
const int CACTUS_SPACING = 8;  // distance between cacti in a group of two

// High score storage
const int HI_SCORE_ADDR = 0;   // EEPROM address (2 bytes)

// ---------- Rambo sprites (16x16, 1 bit per pixel) ----------
const uint8_t RAMBO_RUN1[] PROGMEM = {
  0x01, 0x80, 0x01, 0x80, 0x0F, 0xF0, 0x1F, 0xF8,
  0x7A, 0x5E, 0x9A, 0x59, 0x98, 0x19, 0x9A, 0x59,
  0x79, 0x9E, 0x0F, 0xF0, 0x00, 0x00, 0x07, 0xE0,
  0x1F, 0xF8, 0x07, 0xE0, 0x06, 0x60, 0x06, 0x00
};

const uint8_t RAMBO_RUN2[] PROGMEM = {
  0x01, 0x80, 0x01, 0x80, 0x0F, 0xF0, 0x1F, 0xF8,
  0x7A, 0x5E, 0x9A, 0x59, 0x98, 0x19, 0x9A, 0x59,
  0x79, 0x9E, 0x0F, 0xF0, 0x00, 0x00, 0x07, 0xE0,
  0x1F, 0xF8, 0x07, 0xE0, 0x06, 0x60, 0x00, 0x60
};

const uint8_t RAMBO_AIR[] PROGMEM = {
  0x01, 0x80, 0x01, 0x80, 0x0F, 0xF0, 0x1F, 0xF8,
  0x7A, 0x5E, 0x9A, 0x59, 0x98, 0x19, 0x9A, 0x59,
  0x79, 0x9E, 0x0F, 0xF0, 0x00, 0x00, 0x07, 0xE0,
  0x1F, 0xF8, 0x07, 0xE0, 0x06, 0x60, 0x06, 0x60
};

const uint8_t RAMBO_DEAD[] PROGMEM = {
  0x01, 0x80, 0x01, 0x80, 0x0F, 0xF0, 0x1F, 0xF8,
  0x7D, 0xBE, 0x9A, 0x59, 0x9D, 0xB9, 0x98, 0x19,
  0x7B, 0xDE, 0x0F, 0xF0, 0x00, 0x00, 0x07, 0xE0,
  0x1F, 0xF8, 0x07, 0xE0, 0x06, 0x60, 0x06, 0x60
};

// ---------- Game state ----------
enum GameState { STATE_TITLE, STATE_PLAYING, STATE_DEAD };
GameState state = STATE_TITLE;

struct Obstacle {
  float x;
  uint8_t h;
  uint8_t n;
  bool active;
};

const uint8_t MAX_OBSTACLES = 3;
Obstacle obstacles[MAX_OBSTACLES];

float ramboY = RAMBO_GROUND_Y;
float ramboVy = 0;
float speedPx = START_SPEED;
float scoreF = 0;
float groundScroll = 0;
float spawnCountdown = 40;
float runPhase = 0;
unsigned int hiScore = 0;
bool blinkOn = true;

unsigned long lastFrame = 0;
unsigned long deathTime = 0;

// ---------- Button (interrupt based, so short presses are never missed) ----------
volatile bool buttonFlag = false;
volatile unsigned long lastButtonIrq = 0;

void onButtonPress() {
  unsigned long now = millis();
  if (now - lastButtonIrq > DEBOUNCE_MS) {
    buttonFlag = true;
    lastButtonIrq = now;
  }
}

bool buttonJustPressed() {
  if (!buttonFlag) return false;
  noInterrupts();
  buttonFlag = false;
  interrupts();
  return true;
}

// ---------- Helpers ----------
int obstacleWidth(uint8_t n) {
  return CACTUS_W + CACTUS_SPACING * (n - 1);
}

unsigned int currentScore() {
  return (unsigned int)scoreF;
}

// Returns true when something is very close to the sensor.
bool sensorCovered() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  int cm = (duration == 0) ? -1 : (int)(duration / 58);

#if DEBUG_DISTANCE
  Serial.println(cm);
#endif

  return (cm >= 0 && cm <= COVER_DISTANCE_CM);
}

// ---------- Drawing (U8glib) ----------
// U8glib uses unsigned 8 bit coordinates, so everything is clipped here
// before it is handed to the library.
void box(int x, int y, int w, int h) {
  if (x < 0) { w += x; x = 0; }
  if (x + w > SCREEN_W) w = SCREEN_W - x;
  if (y < 0) { h += y; y = 0; }
  if (y + h > SCREEN_H) h = SCREEN_H - y;
  if (w <= 0 || h <= 0) return;
  u8g.drawBox(x, y, w, h);
}

void useFont(const u8g_fntpgm_uint8_t *font) {
  u8g.setFont(font);
  u8g.setFontPosTop();  // text y coordinate = top of the text
}

void drawCentered(int y, const char* s) {
  int w = u8g.getStrWidth(s);
  u8g.drawStr((SCREEN_W - w) / 2, y, s);
}

void drawRambo(int x, int y, bool airborne, bool dead) {
  const uint8_t *sprite;
  if (dead) sprite = RAMBO_DEAD;
  else if (airborne) sprite = RAMBO_AIR;
  else if (((int)runPhase / 3) % 2) sprite = RAMBO_RUN1;
  else sprite = RAMBO_RUN2;

  u8g.setColorIndex(1);
  u8g.drawBitmapP(x, y, 2, RAMBO_H, sprite);  // 2 bytes per row = 16 pixels wide
}

void drawCactus(int x, int h, int n) {
  u8g.setColorIndex(1);
  int top = GROUND_Y - h;
  for (int i = 0; i < n; i++) {
    int ux = x + i * CACTUS_SPACING;
    box(ux + 2, top, 3, h);            // trunk
    int r = top + h / 2;
    box(ux, r, 3, 2);                  // left arm
    box(ux, r - 3, 2, 5);
    if (h >= 14) {
      int r2 = top + h / 2 - 2;
      box(ux + 4, r2, 3, 2);           // right arm
      box(ux + 5, r2 - 3, 2, 5);
    }
  }
}

void drawGround() {
  u8g.setColorIndex(1);
  box(0, GROUND_Y, SCREEN_W, 1);
  int off = (int)groundScroll % 32;
  for (int i = 0; i < SCREEN_W + 32; i += 32) {
    int x1 = i - off;
    box(x1, 61, 4, 1);
    box(x1 + 16, 63, 2, 1);
  }
}

void drawHud() {
  char buf[12];
  u8g.setColorIndex(1);
  useFont(u8g_font_5x7);

  snprintf(buf, sizeof(buf), "%05u", currentScore());
  u8g.drawStr(SCREEN_W - u8g.getStrWidth(buf), 0, buf);

  if (hiScore > 0) {
    snprintf(buf, sizeof(buf), "HI %05u", hiScore);
    u8g.drawStr(0, 0, buf);
  }
}

void drawScene(bool dead) {
  drawGround();
  for (uint8_t i = 0; i < MAX_OBSTACLES; i++) {
    if (obstacles[i].active) {
      drawCactus((int)obstacles[i].x, obstacles[i].h, obstacles[i].n);
    }
  }
  drawRambo(RAMBO_X, (int)ramboY, ramboY < RAMBO_GROUND_Y, dead);
  drawHud();
}

void drawTitle() {
  u8g.setColorIndex(1);
  useFont(u8g_font_unifont);
  drawCentered(2, "RAMBO RUN");

  useFont(u8g_font_5x7);
  if (blinkOn) drawCentered(24, "Press button to start");
  drawCentered(34, "Cover sensor to jump");

  drawGround();
  drawRambo(RAMBO_X, (int)RAMBO_GROUND_Y, false, false);

  if (hiScore > 0) {
    char buf[12];
    useFont(u8g_font_5x7);
    snprintf(buf, sizeof(buf), "HI %05u", hiScore);
    u8g.drawStr(SCREEN_W - u8g.getStrWidth(buf) - 2, 46, buf);
  }
}

void drawGameOver() {
  drawScene(true);

  u8g.setColorIndex(0);
  u8g.drawBox(14, 8, 100, 32);
  u8g.setColorIndex(1);
  u8g.drawFrame(14, 8, 100, 32);

  char buf[16];
  useFont(u8g_font_5x7);
  drawCentered(12, "GAME OVER");
  snprintf(buf, sizeof(buf), "Score: %05u", currentScore());
  drawCentered(22, buf);
  drawCentered(31, "Press button");
}

// U8glib draws in pages: the draw code runs once per page, so it must not
// change any game state. Everything it needs is prepared before firstPage().
void renderFrame() {
  blinkOn = ((millis() / 500) % 2) == 0;
  u8g.firstPage();
  do {
    u8g.setColorIndex(1);
    if (state == STATE_TITLE) drawTitle();
    else if (state == STATE_PLAYING) drawScene(false);
    else drawGameOver();
  } while (u8g.nextPage());
}

// ---------- Game logic ----------
void resetGame() {
  ramboY = RAMBO_GROUND_Y;
  ramboVy = 0;
  speedPx = START_SPEED;
  scoreF = 0;
  groundScroll = 0;
  spawnCountdown = 40;
  runPhase = 0;
  for (uint8_t i = 0; i < MAX_OBSTACLES; i++) {
    obstacles[i].active = false;
  }
}

void startGame() {
  randomSeed(micros());
  resetGame();
  lastFrame = millis();
  state = STATE_PLAYING;
}

void spawnObstacle() {
  for (uint8_t i = 0; i < MAX_OBSTACLES; i++) {
    if (!obstacles[i].active) {
      uint8_t type = random(3);
      obstacles[i].active = true;
      obstacles[i].x = SCREEN_W;
      obstacles[i].h = (type == 1) ? 16 : 10;
      obstacles[i].n = (type == 2) ? 2 : 1;

      float gap = MIN_GAP_BASE + speedPx * GAP_PER_SPEED + random(0, GAP_RANDOM);
      spawnCountdown = obstacleWidth(obstacles[i].n) + gap;
      return;
    }
  }
  spawnCountdown = 10;  // no free slot, try again soon
}

bool hitsObstacle() {
  // Hitbox covers Rambo's head, body and feet (not the horns or antenna)
  int rx = RAMBO_X + 3;
  int rw = RAMBO_W - 6;
  int ry = (int)ramboY + 2;
  int rh = RAMBO_H - 2;

  for (uint8_t i = 0; i < MAX_OBSTACLES; i++) {
    if (!obstacles[i].active) continue;
    int ox = (int)obstacles[i].x + 1;
    int ow = obstacleWidth(obstacles[i].n) - 2;
    int oy = GROUND_Y - obstacles[i].h;
    int oh = obstacles[i].h;

    if (rx < ox + ow && rx + rw > ox && ry < oy + oh && ry + rh > oy) {
      return true;
    }
  }
  return false;
}

// dt is the elapsed time in "frames" (1.0 = one 33 ms frame), so the game
// runs at the same speed even if the display makes frames take longer.
void updateGame(float dt) {
  // Jump input: covering the sensor while standing on the ground
  bool covered = sensorCovered();
  bool onGround = (ramboY >= RAMBO_GROUND_Y);
  if (covered && onGround) {
    ramboVy = JUMP_VELOCITY;
  }

  // Rambo physics in fixed steps of at most 1.0, so jump height stays the
  // same no matter how long a frame took
  float t = dt;
  while (t > 0) {
    float h = (t > 1.0) ? 1.0 : t;
    ramboVy += GRAVITY * h;
    ramboY += ramboVy * h;
    if (ramboY >= RAMBO_GROUND_Y) {
      ramboY = RAMBO_GROUND_Y;
      ramboVy = 0;
    }
    t -= h;
  }

  // Speed, score, scrolling
  speedPx = START_SPEED + scoreF * SPEED_PER_POINT;
  if (speedPx > MAX_SPEED) speedPx = MAX_SPEED;
  float step = speedPx * dt;
  scoreF += step * SCORE_PER_PIXEL;
  groundScroll += step;
  if (groundScroll >= 32.0) groundScroll -= 32.0;
  runPhase += dt;
  if (runPhase > 600.0) runPhase -= 600.0;

  // Obstacles
  for (uint8_t i = 0; i < MAX_OBSTACLES; i++) {
    if (!obstacles[i].active) continue;
    obstacles[i].x -= step;
    if (obstacles[i].x + obstacleWidth(obstacles[i].n) < 0) {
      obstacles[i].active = false;
    }
  }
  spawnCountdown -= step;
  if (spawnCountdown <= 0) spawnObstacle();

  // Collision
  if (hitsObstacle()) {
    if (currentScore() > hiScore) {
      hiScore = currentScore();
      EEPROM.put(HI_SCORE_ADDR, hiScore);  // only written on a new high score
    }
    deathTime = millis();
    state = STATE_DEAD;
  }
}

// ---------- Arduino entry points ----------
void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(TRIG_PIN, LOW);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onButtonPress, FALLING);

#if DEBUG_DISTANCE
  Serial.begin(115200);
#endif

  // Load the saved high score (a blank EEPROM reads as 0xFFFF)
  EEPROM.get(HI_SCORE_ADDR, hiScore);
  if (hiScore == 0xFFFF) hiScore = 0;

  // Reset pulse for the OLED (RES pin)
  pinMode(OLED_RES, OUTPUT);
  digitalWrite(OLED_RES, LOW);
  delay(10);
  digitalWrite(OLED_RES, HIGH);
  delay(10);

  u8g.setRot180();       // flip the screen 180 degrees
  u8g.setColorIndex(1);  // white pixels

  randomSeed(analogRead(A0));
  resetGame();
  state = STATE_TITLE;
  lastFrame = millis();
}

void loop() {
  unsigned long now = millis();

  if (buttonJustPressed()) {
    if (state == STATE_TITLE) {
      startGame();
    } else if (state == STATE_DEAD && (now - deathTime) > DEAD_LOCKOUT_MS) {
      resetGame();
      state = STATE_TITLE;
    }
  }

  if (now - lastFrame >= FRAME_MS) {
    float dt = (float)(now - lastFrame) / (float)FRAME_MS;
    if (dt > MAX_DT) dt = MAX_DT;
    lastFrame = now;

    if (state == STATE_PLAYING) {
      updateGame(dt);
      renderFrame();  // draws the game over screen if we just crashed
    } else if (state == STATE_TITLE) {
      renderFrame();
    }
    // STATE_DEAD: the game over screen is static, nothing to redraw
  }
}