/*
 * Sharp Memory Display demo: a looping slide show that proves the display works.
 * For the Adafruit SHARP Memory Display breakout, 1.3" 144x168 monochrome (portrait).
 *
 * Scenes (about 35 s per loop, then it repeats):
 * 1. Splash screen with a compass icon
 * 2. Shapes: lines, rectangles, circles, triangles, text sizes
 * 3. "Image": a halftone (dithered) shaded sphere and gradient
 * 4. Compass: a needle sweeping around a dial with a live heading readout
 * 5. Radar: node dots (T114, LR30A, LR30B) with a sweeping scan line
 * 6. Pointer: the big arrow + distance + bearing screen the watch will show
 * 7. Bitmap: a 32x32 icon stored in flash
 *
 * Wiring (see the table in the instructions): only 5 wires are needed.
 * display Vin -> 3V3 display GND -> GND
 * display CLK -> SHARP_SCK display DI -> SHARP_MOSI display CS -> SHARP_SS
 * The other four pins on the breakout can stay unconnected.
 *
 * Libraries: "Adafruit SHARP Memory Display", "Adafruit GFX Library", "Adafruit BusIO".
 * The display keeps its picture without power for the pixels, but it needs a signal
 * flip (VCOM) at least about once a second. The library flips it on every refresh(),
 * so every wait below keeps calling refresh().
 */

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>

#ifndef BLACK
#define BLACK 0
#endif
#ifndef WHITE
#define WHITE 1
#endif

// ---- pins: any three free GPIOs work (the library bit-bangs the SPI) ----
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #define SHARP_SCK 4
  #define SHARP_MOSI 5
  #define SHARP_SS 6
#else // classic ESP32 dev board
  #define SHARP_SCK 18
  #define SHARP_MOSI 23
  #define SHARP_SS 5
#endif

#define ROTATION 0 // 0 = portrait 144 x 168; 1 or 3 = landscape 168 x 144

Adafruit_SharpMem display(SHARP_SCK, SHARP_MOSI, SHARP_SS, 144, 168);

// 32x32 location-pin icon, 1 bit per pixel (1 = black)
const uint8_t PIN_ICON[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xE0, 0x00,
  0x00, 0x3F, 0xFC, 0x00, 0x00, 0x7F, 0xFE, 0x00,
  0x00, 0xFF, 0xFF, 0x00, 0x01, 0xFF, 0xFF, 0x80,
  0x03, 0xFF, 0xFF, 0xC0, 0x03, 0xFF, 0xFF, 0xC0,
  0x07, 0xFC, 0x3F, 0xE0, 0x07, 0xF8, 0x1F, 0xE0,
  0x07, 0xF0, 0x0F, 0xE0, 0x07, 0xF0, 0x0F, 0xE0,
  0x0F, 0xE0, 0x07, 0xF0, 0x07, 0xF0, 0x0F, 0xE0,
  0x07, 0xF0, 0x0F, 0xE0, 0x07, 0xF8, 0x1F, 0xE0,
  0x07, 0xFC, 0x3F, 0xE0, 0x03, 0xFF, 0xFF, 0xC0,
  0x03, 0xFF, 0xFF, 0xC0, 0x01, 0xFF, 0xFF, 0x80,
  0x00, 0xFF, 0xFF, 0x00, 0x00, 0x7F, 0xFE, 0x00,
  0x00, 0x3F, 0xFC, 0x00, 0x00, 0x0F, 0xF0, 0x00,
  0x00, 0x0F, 0xF0, 0x00, 0x00, 0x07, 0xE0, 0x00,
  0x00, 0x03, 0xC0, 0x00, 0x00, 0x03, 0xC0, 0x00,
  0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static int W, H;

// ---------- helpers ----------

// Keep the picture on screen for `ms`, refreshing about twice a second (VCOM)
static void hold(uint32_t ms) {
  uint32_t end = millis() + ms;
  do {
    display.refresh();
    delay(250);
  } while ((int32_t)(end - millis()) > 0);
}

static void centered(const char *s, int y, int size) {
  int16_t x1, y1;
  uint16_t w, h;
  display.setTextSize(size);
  display.getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((W - (int)w) / 2 - x1, y);
  display.print(s);
}

static void polar(int cx, int cy, float r, float deg, int &x, int &y) { // 0 deg = up, clockwise
  float a = deg * DEG_TO_RAD;
  x = cx + (int)lroundf(r * sinf(a));
  y = cy - (int)lroundf(r * cosf(a));
}

// Arrow centred on (cx,cy), pointing `deg` clockwise from up
static void arrow(int cx, int cy, int len, int halfWidth, float deg) {
  int tx, ty, bx, by, lx, ly, rx, ry;
  polar(cx, cy, len, deg, tx, ty); // tip
  polar(cx, cy, len * 0.55f, deg + 180.0f, bx, by); // tail
  polar(cx, cy, halfWidth, deg - 90.0f, lx, ly); // shoulders
  polar(cx, cy, halfWidth, deg + 90.0f, rx, ry);
  display.fillTriangle(tx, ty, lx, ly, rx, ry, BLACK);
  display.fillTriangle(bx, by, lx, ly, rx, ry, BLACK);
  display.fillCircle(cx, cy, halfWidth / 3 + 1, WHITE);
}

// Classic arrow: triangular head plus a thick shaft, pointing `deg` clockwise from up
static void bigArrow(int cx, int cy, int len, int headHalf, int shaftHalf, float deg) {
  int tx, ty, nx, ny, hlx, hly, hrx, hry, tailx, taily, slx, sly, srx, sry, elx, ely, erx, ery;
  polar(cx, cy, len, deg, tx, ty); // tip
  polar(cx, cy, len * 0.15f, deg, nx, ny); // where the head meets the shaft
  polar(nx, ny, headHalf, deg - 90.0f, hlx, hly); // head corners
  polar(nx, ny, headHalf, deg + 90.0f, hrx, hry);
  polar(cx, cy, len * 0.7f, deg + 180.0f, tailx, taily); // tail end
  polar(nx, ny, shaftHalf, deg - 90.0f, slx, sly); // shaft corners at the head
  polar(nx, ny, shaftHalf, deg + 90.0f, srx, sry);
  polar(tailx, taily, shaftHalf, deg - 90.0f, elx, ely); // shaft corners at the tail
  polar(tailx, taily, shaftHalf, deg + 90.0f, erx, ery);
  display.fillTriangle(tx, ty, hlx, hly, hrx, hry, BLACK);
  display.fillTriangle(slx, sly, srx, sry, elx, ely, BLACK);
  display.fillTriangle(srx, sry, erx, ery, elx, ely, BLACK);
}

static void compassIcon(int cx, int cy, int r) {
  display.drawCircle(cx, cy, r, BLACK);
  display.drawCircle(cx, cy, r - 3, BLACK);
  for (int d = 0; d < 360; d += 45) {
    int x1, y1, x2, y2;
    polar(cx, cy, r - 3, d, x1, y1);
    polar(cx, cy, r - (d % 90 == 0 ? 10 : 6), d, x2, y2);
    display.drawLine(x1, y1, x2, y2, BLACK);
  }
  arrow(cx, cy, r - 8, 5, 35.0f);
}

// ---------- scenes ----------

static void sceneSplash() {
  Serial.println("SCENE 1 splash");
  display.clearDisplay();
  display.drawRect(0, 0, W, H, BLACK);
  display.drawRect(3, 3, W - 6, H - 6, BLACK);
  compassIcon(W / 2, 62, 34);
  display.setTextColor(BLACK);
  centered("LODESTONE", 112, 2);
  centered("Team 16", 138, 1);
  centered("SDP27", 150, 1);
  hold(2500);
}

static void sceneShapes() {
  Serial.println("SCENE 2 shapes");
  display.clearDisplay();
  display.setTextColor(BLACK);
  centered("Shapes", 2, 2);
  // star of lines
  int cx = 36, cy = 52;
  for (int d = 0; d < 360; d += 20) {
    int x, y;
    polar(cx, cy, 26, d, x, y);
    display.drawLine(cx, cy, x, y, BLACK);
  }
  // nested squares
  for (int i = 0; i < 5; i++) display.drawRect(80 + i * 4, 28 + i * 4, 48 - i * 8, 48 - i * 8, BLACK);
  // circles
  display.drawCircle(36, 108, 24, BLACK);
  display.fillCircle(36, 108, 14, BLACK);
  display.fillCircle(36, 108, 6, WHITE);
  // triangles and rounded rect
  display.fillTriangle(80, 130, 104, 86, 128, 130, BLACK);
  display.drawTriangle(86, 126, 104, 94, 122, 126, WHITE);
  display.drawRoundRect(70, 136, 66, 24, 6, BLACK);
  display.setTextSize(1);
  display.setCursor(78, 144);
  display.print("text 1x");
  display.setCursor(6, 140);
  display.setTextSize(2);
  display.print("2x");
  hold(3000);
}

// 4x4 Bayer matrix thresholds (0..15)
static const uint8_t BAYER4[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};

static void sceneImage() {
  Serial.println("SCENE 3 dithered image");
  display.clearDisplay();
  // every pixel gets a brightness 0..1, then a Bayer threshold turns it into black/white
  const float cx = W / 2.0f, cy = 100.0f, R = 50.0f;
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      float v;
      float dx = (x - cx) / R, dy = (y - cy) / R;
      float d2 = dx * dx + dy * dy;
      if (y < 30) {
        v = x / (float)(W - 1); // horizontal gradient strip
      } else if (d2 <= 1.0f) {
        float nz = sqrtf(1.0f - d2); // sphere lit from the upper left
        v = 0.12f + 0.88f * fmaxf(0.0f, (-dx * 0.5f - dy * 0.5f + nz * 0.7f) / 1.1f);
      } else {
        v = 0.92f; // light background
      }
      if (v * 16.0f < BAYER4[y & 3][x & 3] + 0.5f) display.drawPixel(x, y, BLACK);
    }
  }
  display.fillRect(0, 30, W, 12, WHITE);
  display.setTextColor(BLACK);
  centered("Dithered image", 32, 1);
  hold(3500);
}

static void sceneCompass() {
  Serial.println("SCENE 4 compass");
  const int cx = W / 2, cy = 100, R = 52;
  for (int heading = 0; heading <= 360; heading += 6) {
    display.clearDisplay();
    display.setTextColor(BLACK);
    char buf[16];
    snprintf(buf, sizeof(buf), "HDG %03d", heading % 360);
    centered(buf, 4, 2);
    display.drawCircle(cx, cy, R, BLACK);
    // the dial rotates opposite to the heading so N stays where the real north is
    for (int d = 0; d < 360; d += 15) {
      int x1, y1, x2, y2;
      float a = d - heading;
      polar(cx, cy, R, a, x1, y1);
      polar(cx, cy, R - (d % 90 == 0 ? 10 : (d % 45 == 0 ? 7 : 4)), a, x2, y2);
      display.drawLine(x1, y1, x2, y2, BLACK);
    }
    const char *names[4] = {"N", "E", "S", "W"};
    for (int i = 0; i < 4; i++) {
      int x, y;
      polar(cx, cy, R - 20, i * 90 - heading, x, y);
      display.setTextSize(1);
      display.setCursor(x - 3, y - 4);
      display.print(names[i]);
    }
    // fixed lubber line at the top: the direction the watch points
    display.fillTriangle(cx, cy - R - 1, cx - 6, cy - R - 12, cx + 6, cy - R - 12, BLACK);
    display.refresh();
    delay(40);
  }
  hold(1000);
}

struct Blip { const char *name; float brg; float frac; };

static void sceneRadar() {
  Serial.println("SCENE 5 radar");
  const int cx = W / 2, cy = 96, R = 58;
  const Blip blips[3] = {{"T114", 20.0f, 0.45f}, {"LR30A", 110.0f, 0.75f}, {"LR30B", 235.0f, 0.60f}};
  for (int sweep = 0; sweep < 720; sweep += 12) {
    float sw = (float)(sweep % 360);
    display.clearDisplay();
    display.setTextColor(BLACK);
    centered("NODES", 4, 2);
    display.drawCircle(cx, cy, R, BLACK);
    display.drawCircle(cx, cy, R * 2 / 3, BLACK);
    display.drawCircle(cx, cy, R / 3, BLACK);
    display.drawLine(cx - R, cy, cx + R, cy, BLACK);
    display.drawLine(cx, cy - R, cx, cy + R, BLACK);
    int sx, sy;
    polar(cx, cy, R, sw, sx, sy);
    display.drawLine(cx, cy, sx, sy, BLACK);
    display.fillCircle(cx, cy, 3, BLACK); // you
    for (const Blip &b : blips) {
      int x, y;
      polar(cx, cy, R * b.frac, b.brg, x, y);
      float diff = fabsf(fmodf(sw - b.brg + 540.0f, 360.0f) - 180.0f); // how far the scan line is
      bool lit = diff < 40.0f;
      display.fillCircle(x, y, lit ? 5 : 3, BLACK);
      if (lit) display.drawCircle(x, y, 8, BLACK);
      display.setTextSize(1);
      int lw = 6 * (int)strlen(b.name);
      int lx = (x + 7 + lw > W) ? x - 7 - lw : x + 7; // flip to the left near the edge
      display.setCursor(lx, constrain(y - 3, 0, H - 8));
      display.print(b.name);
    }
    display.setCursor(2, H - 10);
    display.print("3 nodes");
    display.refresh();
    delay(40);
  }
  hold(500);
}

static void scenePointer() {
  Serial.println("SCENE 6 pointer");
  const float target = 132.0f;
  const int cx = W / 2, cy = 68, R = 44;
  for (float a = 0; a <= target; a += 6.0f) {
    display.clearDisplay();
    display.setTextColor(BLACK);
    centered("T114", 4, 2);
    display.drawCircle(cx, cy, R, BLACK);
    bigArrow(cx, cy, 36, 13, 4, a);
    display.refresh();
    delay(30);
  }
  centered("85 m", 118, 3);
  centered("brg 132", 146, 2);
  hold(3500);
}

static void sceneBitmap() {
  Serial.println("SCENE 7 bitmap");
  display.clearDisplay();
  display.setTextColor(BLACK);
  centered("Bitmap", 4, 2);
  display.drawBitmap((W - 32) / 2, 40, PIN_ICON, 32, 32, BLACK);
  display.drawBitmap(W / 2 - 52, 90, PIN_ICON, 32, 32, BLACK);
  display.drawBitmap(W / 2 + 20, 90, PIN_ICON, 32, 32, BLACK);
  centered("32x32 icon in flash", 134, 1);
  centered("drawBitmap()", 148, 1);
  hold(3000);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Sharp memory display demo");
  if (!display.begin()) {
    Serial.println("display.begin() failed - check wiring and the three pin numbers");
    while (true) delay(1000);
  }
  display.setRotation(ROTATION);
  W = display.width();
  H = display.height();
  Serial.printf("display %d x %d\n", W, H);
  display.clearDisplay();
  display.refresh();
}

void loop() {
  sceneSplash();
  sceneShapes();
  sceneImage();
  sceneCompass();
  sceneRadar();
  scenePointer();
  sceneBitmap();
}
