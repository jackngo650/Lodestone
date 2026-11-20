/*
 * Heltec WiFi LoRa 32 V4 - LoRa receiver with OLED display + acknowledgements.
 *
 * Receives telemetry from the nRF54L15:
 *   "TEL <seq> <yaw> <pitch> <roll> <acc> <lat> <lon> <alt> <sats> <fix>"
 *      yaw/pitch/roll : tenths of a degree
 *      acc            : IMU calibration 0-3 (-1 = no IMU)
 *      lat/lon        : millionths of a degree
 *      alt            : metres
 *      sats, fix      : satellites used, fix quality (0 none, -1 no GNSS data)
 * and replies "OK <seq> <rssi> <snr>" so the nRF can count and log round trips.
 * Older "IMU ..." and "hello N" messages are still accepted and acknowledged.
 *
 * Settings match the nRF54L15 lora_tx app:
 *   915 MHz, 125 kHz BW, SF9, CR 4/5, sync word 0x12 (private), preamble 8
 *
 * Two screen pages - short press PRG to switch:
 *
 *   IMU page                    GPS page
 *   RX 42 ACK 42 c3             RX 42 ACK 42 c3
 *   Y 123.4      (large)        Lat  40.712345
 *   P  -1.2  R   3.4            Lon -74.001234
 *   RSSI -22 SNR 10.0           Alt 10m Sat 8 Fix 1
 *                               RSSI -22 SNR 10.0
 *   Lost 0 CRC 0 1s             Lost 0 CRC 0 1s
 *
 * "Lost" counts gaps in the sequence numbers, handy for range tests.
 * Hold PRG for 1 second to reset the counters.
 *
 * Works without USB (on battery). Serial output is still printed when
 * plugged into a computer.
 *
 * Attach the Heltec's antenna before powering it.
 */

#include <Arduino.h>
#include <Wire.h>
#include <RadioLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO08x.h>

// SX1262 pins on the Heltec V4
#define LORA_NSS 8
#define LORA_SCK 9
#define LORA_MOSI 10
#define LORA_MISO 11
#define LORA_RST 12
#define LORA_BUSY 13
#define LORA_DIO1 14

// Board power / front-end (GC1109) control
#define VEXT_CTRL 36 // powers OLED + RF front end (LOW = on on most boards)
#define FEM_PWR 7 // front-end LDO enable
#define FEM_CSD 2 // front-end chip enable
#define FEM_TX_EN 46 // front-end TX enable: HIGH in TX, LOW in RX (RadioLib drives it)

// OLED (SSD1306 128x64, I2C)
#define OLED_SDA 17
#define OLED_SCL 18
#define OLED_RST 21
#define OLED_ADDR 0x3C

// PRG button (resets counters)
#define PRG_BTN 0

// Reply settings
#define ACK_DELAY_MS 100 // give the nRF time to switch to receive
#define ACK_TX_POWER 0 // dBm from the SX1262 (the board's PA adds gain on top)

// Show "NO SIGNAL" after this long without a packet
#define SIGNAL_TIMEOUT_MS 6000

// ---------- NEW: point-to-nRF settings ----------
// Where THIS Heltec is sitting (decimal degrees x 1,000,000) and which compass
// direction the top edge of the OLED faces (0 = north, 90 = east, ...).
// The Heltec has no GPS or compass, so you type these in and re-flash.
#define HOME_LAT_E6 42393518L // 42.393518 N
#define HOME_LON_E6 -72528339L // 72.528339 W
#define HOME_HEADING_DEG 0.0f // direction the screen's top edge points
#define PAGE_COUNT 5 // 0 = DK IMU, 1 = GPS, 2 = point to nRF, 3 = other nodes, 4 = compass
#define MAX_NODES 6
#define NODE_STALE_MS 30000 // show :lost" when a node is quiet for too long

// ---------- local IMU (a BNO085 wired to this board) ----------
// It uses its own I2C bus because the OLED already uses GPIO17/18.
// Free header pins on the V4: GPIO4, 6, 33 (47 and 48 also work). Change these two if yours are used.
#define IMU_SDA 33
#define IMU_SCL 47
#define HEADING_SIGN -1.0f // flip to 1.0f if the arrow turns the wrong way
#define HEADING_OFFSET_DEG 0.0f // adjust so the top edge of the screen reads 0 facing north
#define DECLINATION_DEG -14.0f // east positive, Amherst/Lexington MA is about 14 W
#define SHOW_TRUE_NORTH 0 // 1 = compass page shows true north, 0 = magnetic (use 0 while tuning)
#define IMU_STALE_MS 1000 // no new report for this long = IMU lost
#define IMU_SAVE_AFTER_MS 10000 // save the calibration once cal has been 3 this long
#define START_PAGE 4 // page shown at boot

// Only print to Serial when a computer is connected (so battery use never blocks)
#define LOG(...) do { if (Serial) Serial.printf(__VA_ARGS__); } while (0)

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RST);

volatile bool packetReady = false;

void IRAM_ATTR onPacket() {
  packetReady = true;
}

// ---------- stats ----------
bool haveDisplay = false;
String lastMsg = "";
bool haveImu = false; // last packet carried IMU data
int imuYaw10 = 0;
int imuPitch10 = 0;
int imuRoll10 = 0;
int imuAcc = -1;
bool haveGnss = false; // last packet carried GNSS data
long gpsLatE6 = 0;
long gpsLonE6 = 0;
long gpsAltM = 0;
int gpsSats = 0;
int gpsFix = -1;
int page = START_PAGE; // 0 = DK IMU, 1 = GPS, 2 = point to nRF, 3 = nodes, 4 = compass
float lastRssi = 0;
float lastSnr = 0;
uint32_t rxCount = 0;
uint32_t crcErrors = 0;
uint32_t missed = 0;
long lastSeq = -1;
uint32_t lastRxMs = 0;
uint32_t acksSent = 0;
uint32_t ackFails = 0;

struct Node {
  bool used;
  char name[8];
  long latE6, lonE6;
  int fix;
  long seq;
  float rssi, snr;
  uint32_t lastMs;
};
Node nodes[MAX_NODES];
uint32_t beaconCount = 0;

void nodeUpdate(const char *name, long lat, long lon, int fix, long seq, float rssi, float snr) {
  int slot = -1;
  for (int i = 0; i < MAX_NODES; i++)
    if (nodes[i].used && strncmp(nodes[i].name, name, sizeof(nodes[i].name) - 1) == 0) { slot = i; break; }
  if (slot < 0) for (int i = 0; i < MAX_NODES; i++) if (!nodes[i].used) { slot = i; break; }
  if (slot < 0) { slot = 0; for (int i = 1; i < MAX_NODES; i++) if (nodes[i].lastMs < nodes[slot].lastMs) slot = i; }
  Node &n = nodes[slot];
  n.used = true;
  strncpy(n.name, name, sizeof(n.name) - 1);
  n.name[sizeof(n.name) - 1] = '\0';
  n.latE6 = lat; n.lonE6 = lon; n.fix = fix; n.seq = seq;
  n.rssi = rssi; n.snr = snr; n.lastMs = millis();
}

void resetStats() {
  lastMsg = "";
  haveImu = false;
  haveGnss = false;
  rxCount = 0;
  crcErrors = 0;
  missed = 0;
  lastSeq = -1;
  acksSent = 0;
  ackFails = 0;
  memset(nodes, 0, sizeof(nodes));
  beaconCount = 0;
}

// ---------- NEW: bearing / distance to the nRF ----------
// Great-circle initial bearing (0-360, clockwise from north) and haversine
// distance from the Heltec's fixed position to the nRF's last GNSS fix.
void pointToNrf(float &bearingDeg, float &distM) {
  const double D2R = 0.017453292519943295;
  double lat1 = (HOME_LAT_E6 / 1e6) * D2R;
  double lon1 = (HOME_LON_E6 / 1e6) * D2R;
  double lat2 = (gpsLatE6 / 1e6) * D2R;
  double lon2 = (gpsLonE6 / 1e6) * D2R;
  double dLat = lat2 - lat1;
  double dLon = lon2 - lon1;

  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  double b = atan2(y, x) / D2R;
  bearingDeg = (float)fmod(b + 360.0, 360.0);

  double h = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  distM = (float)(2.0 * 6371000.0 * asin(sqrt(h)));
}

// Same maths as pointToNrf(), for any position (used for the other nodes)
void bearingFromHome(long latE6, long lonE6, float &bearingDeg, float &distM) {
  const double D2R = 0.017453292519943295;
  double lat1 = (HOME_LAT_E6 / 1e6) * D2R;
  double lon1 = (HOME_LON_E6 / 1e6) * D2R;
  double lat2 = (latE6 / 1e6) * D2R;
  double lon2 = (lonE6 / 1e6) * D2R;
  double dLat = lat2 - lat1;
  double dLon = lon2 - lon1;

  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  bearingDeg = (float)fmod(atan2(y, x) / D2R + 360.0, 360.0);

  double h = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  distM = (float)(2.0 * 6371000.0 * asin(sqrt(h)));
}

// Arrow centred at (cx,cy); angleDeg is clockwise from the top of the screen.

// Arrow centred at (cx,cy); angleDeg is clockwise from the top of the screen.
void drawArrow(int cx, int cy, int r, float angleDeg) {
  const float D2R = 0.017453293f;
  float a = angleDeg * D2R;
  int tx = cx + (int)lroundf(r * sinf(a));
  int ty = cy - (int)lroundf(r * cosf(a));
  int bx = cx - (int)lroundf(r * sinf(a));
  int by = cy + (int)lroundf(r * cosf(a));
  int lx = tx + (int)lroundf(7 * sinf(a + 2.62f)); // arrow head, +/-150 deg
  int ly = ty - (int)lroundf(7 * cosf(a + 2.62f));
  int rx = tx + (int)lroundf(7 * sinf(a - 2.62f));
  int ry = ty - (int)lroundf(7 * cosf(a - 2.62f));
  display.drawCircle(cx, cy, r + 3, SSD1306_WHITE);
  display.drawLine(bx, by, tx, ty, SSD1306_WHITE);
  display.drawLine(tx, ty, lx, ly, SSD1306_WHITE);
  display.drawLine(tx, ty, rx, ry, SSD1306_WHITE);
}

// ---------- display helpers ----------
bool oledPresent() {
  Wire.beginTransmission(OLED_ADDR);
  return Wire.endTransmission() == 0;
}

void resetOled() {
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  delay(20);
}

// Bring up the OLED. Tries Vext LOW first (the usual Heltec setting),
// then HIGH in case this board revision uses the opposite polarity.
bool startDisplay() {
  Wire.begin(OLED_SDA, OLED_SCL);

  pinMode(VEXT_CTRL, OUTPUT);
  digitalWrite(VEXT_CTRL, LOW);
  delay(100);
  resetOled();
  if (!oledPresent()) {
    digitalWrite(VEXT_CTRL, HIGH);
    delay(100);
    resetOled();
    if (!oledPresent()) {
      digitalWrite(VEXT_CTRL, LOW); // back to the setting the radio is known to work with
      return false;
    }
    LOG("OLED found with Vext HIGH\n");
  }

  // reset already done, Wire already started
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, false, false)) {
    return false;
  }
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);
  display.cp437(true); // so write(247) draws the degree sign
  return true;
}

void showStatus(const char *line1, const char *line2) {
  if (!haveDisplay) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(line1);
  display.setCursor(0, 16);
  display.print(line2);
  display.display();
}

// Page 3: list of other nodes. Draws the whole screen (including its bottom line).
void drawNodesPage() {
  int shown = 0, active = 0;
  for (int i = 0; i < MAX_NODES; i++) if (nodes[i].used) active++;

  for (int i = 0; i < MAX_NODES && shown < 4; i++) {
    if (!nodes[i].used) continue;
    const Node &n = nodes[i];
    int y = 12 + shown * 10;
    uint32_t age = (millis() - n.lastMs) / 1000;
    display.setCursor(0, y);
    if (n.fix > 0) {
      float brg, dist;
      bearingFromHome(n.latE6, n.lonE6, brg, dist);
      char d[8];
      if (dist < 1000.0f) snprintf(d, sizeof(d), "%dm", (int)lroundf(dist));
      else snprintf(d, sizeof(d), "%.1fkm", dist / 1000.0f);
      if (millis() - n.lastMs > NODE_STALE_MS) display.printf("%-5.5s%5s %3d lost", n.name, d, (int)lroundf(brg) % 360);
      else display.printf("%-5.5s%5s %3d %3lus", n.name, d, (int)lroundf(brg) % 360, (unsigned long)age);
    } else {
      display.printf("%-5.5s no fix %3lus", n.name, (unsigned long)age);
    }
    shown++;
  }
  if (shown == 0) {
    display.setCursor(0, 22);
    display.print("No nodes heard yet");
  }

  display.setCursor(0, 54);
  if (active > shown) display.printf("Beacons %lu (+%d more)", (unsigned long)beaconCount, active - shown);
  else display.printf("Beacons heard: %lu", (unsigned long)beaconCount);
  display.display();
}


// ---------- local IMU: heading from the BNO085 on this board ----------
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t locValue;
bool locHave = false;
float locYaw = 0;
int locCal = 0;
uint32_t locLastMs = 0;
uint32_t locReports = 0;
uint32_t locCal3Since = 0;
bool locSaved = false;

float wrap360(float a) {
  a = fmodf(a, 360.0f);
  return a < 0 ? a + 360.0f : a;
}

void enableLocalReports() {
  bno08x.enableReport(SH2_ROTATION_VECTOR, 20000);
  sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
}

bool startLocalImu() {
  Wire1.begin(IMU_SDA, IMU_SCL);
  Wire1.setClock(400000);
  const uint8_t addrs[] = {0x4A, 0x4B};
  for (uint8_t a : addrs) {
    if (bno08x.begin_I2C(a, &Wire1)) {
      LOG("Local IMU found at 0x%02X\n", a);
      enableLocalReports();
      return true;
    }
  }
  return false;
}

void pollLocalImu() {
  if (!locHave) return;
  if (bno08x.wasReset()) enableLocalReports();
  while (bno08x.getSensorEvent(&locValue)) {
    if (locValue.sensorId != SH2_ROTATION_VECTOR) continue;
    float w = locValue.un.rotationVector.real;
    float x = locValue.un.rotationVector.i;
    float y = locValue.un.rotationVector.j;
    float z = locValue.un.rotationVector.k;
    locYaw = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * 57.2957795f;
    locCal = locValue.status & 0x03;
    locLastMs = millis();
    locReports++;
  }

  // Save the calibration to the sensor once it has held at 3 for a while
  if (locCal >= 3) {
    if (locCal3Since == 0) locCal3Since = millis();
    if (!locSaved && millis() - locCal3Since > IMU_SAVE_AFTER_MS) {
      sh2_saveDcdNow();
      locSaved = true;
      LOG("Local IMU calibration saved\n");
    }
  } else {
    locCal3Since = 0;
  }
}

bool localFresh() {
  return locHave && locReports > 0 && millis() - locLastMs < IMU_STALE_MS;
}

float localHeadingMag() {
  return wrap360(HEADING_SIGN * locYaw + HEADING_OFFSET_DEG);
}

float localHeadingTrue() {
  return wrap360(localHeadingMag() + DECLINATION_DEG);
}

const char *cardinal(float deg) {
  static const char *const names[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  return names[((int)((deg + 22.5f) / 45.0f)) % 8];
}

// Page 4: compass. The arrow points to north and turns as the board turns.
// Draws the whole screen.
void drawCompassPage() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.printf("%s c%d", SHOW_TRUE_NORTH ? "TRUE" : "MAG", locCal);

  if (!localFresh()) {
    display.setTextSize(2);
    display.setCursor(0, 22);
    display.print(locHave ? "IMU lost" : "No IMU");
    display.display();
    return;
  }

  float heading = SHOW_TRUE_NORTH ? localHeadingTrue() : localHeadingMag();
  int deg = (int)lroundf(heading) % 360;

  display.setTextSize(3);
  display.setCursor(0, 14);
  display.printf("%3d", deg);
  display.setTextSize(2);
  display.write(247);
  display.setCursor(0, 42);
  display.print(cardinal(heading));

  drawArrow(98, 34, 19, wrap360(-heading));
  display.drawFastVLine(98, 6, 4, SSD1306_WHITE); // fixed mark: the way the board faces
  display.display();
}

void drawScreen() {
  if (!haveDisplay) return;

  uint32_t ageMs = millis() - lastRxMs;
  bool noSignal = (rxCount == 0) || (ageMs > SIGNAL_TIMEOUT_MS);

  display.clearDisplay();
  display.setTextSize(1);

  // Top line: settings + packet count
  display.setCursor(0, 0);
  display.printf("RX %lu ACK %lu", (unsigned long)rxCount, (unsigned long)acksSent);
  if (haveImu) {
    display.setCursor(104, 0);
    if (imuAcc >= 0) display.printf("c%d", imuAcc);
    else display.print("noI");
  }

  if (rxCount == 0 && page != 3 && page != 4) {
    display.setTextSize(2);
    display.setCursor(0, 22);
    display.print("Waiting..");
    display.setTextSize(1);
    display.setCursor(0, 54);
    display.printf("CRC err %lu", (unsigned long)crcErrors);
    display.display();
    return;
  }

  if (page == 1 && haveGnss) {
    // GPS page
    if (gpsFix > 0) {
      display.setTextSize(1);
      display.setCursor(0, 12);
      display.printf("Lat %s%ld.%06ld", gpsLatE6 < 0 ? "-" : " ",
                     labs(gpsLatE6) / 1000000L, labs(gpsLatE6) % 1000000L);
      display.setCursor(0, 22);
      display.printf("Lon %s%ld.%06ld", gpsLonE6 < 0 ? "-" : " ",
                     labs(gpsLonE6) / 1000000L, labs(gpsLonE6) % 1000000L);
      display.setCursor(0, 32);
      display.printf("Alt %ldm Sat %d Fix %d", gpsAltM, gpsSats, gpsFix);
    } else {
      display.setTextSize(2);
      display.setCursor(0, 12);
      display.print(gpsFix < 0 ? "No GNSS" : "No fix");
      display.setTextSize(1);
      display.setCursor(0, 32);
      if (gpsFix == 0) display.printf("Sats %d, searching", gpsSats);
      else display.print("Check GNSS wiring");
    }
    display.setCursor(0, 42);
    display.printf("RSSI %d SNR %.1f", (int)lroundf(lastRssi), lastSnr);
  } else if (page == 2) {
    // NEW: point-to-nRF page
    if (haveGnss && gpsFix > 0) {
      float brg, dist;
      pointToNrf(brg, dist);
      float facing = localFresh() ? localHeadingTrue() : HOME_HEADING_DEG;
      float rel = fmodf(brg - facing + 360.0f, 360.0f);

      display.setTextSize(1);
      display.setCursor(0, 12);
      display.print("nRF is");
      display.setTextSize(2);
      display.setCursor(0, 22);
      if (dist < 1000.0f) display.printf("%dm", (int)lroundf(dist));
      else display.printf("%.1fkm", dist / 1000.0f);
      display.setTextSize(1);
      display.setCursor(0, 42);
      display.printf("Brg %3d", (int)lroundf(brg) % 360);
      drawArrow(100, 33, 15, rel);
    } else {
      display.setTextSize(2);
      display.setCursor(0, 12);
      display.print("No nRF");
      display.setCursor(0, 30);
      display.print(gpsFix < 0 ? "GNSS" : "fix");
    }
  } else if (page == 4) {
    drawCompassPage();
    return;
  } else if (page == 3) {
      drawNodesPage();
      return;
  } else if (haveImu && imuAcc >= 0) {
    // IMU page: yaw large, pitch/roll below
    display.setTextSize(2);
    display.setCursor(0, 12);
    display.printf("Y %5.1f", imuYaw10 / 10.0f);

    display.setTextSize(1);
    display.setCursor(0, 32);
    display.printf("P %6.1f R %6.1f", imuPitch10 / 10.0f, imuRoll10 / 10.0f);
    display.setCursor(0, 42);
    display.printf("RSSI %d SNR %.1f", (int)lroundf(lastRssi), lastSnr);
  } else {
    // Plain message (or IMU not found on the nRF side), large
    display.setTextSize(2);
    display.setCursor(0, 12);
    display.print(haveImu ? "No IMU" : lastMsg.substring(0, 10));

    display.setTextSize(1);
    display.setCursor(0, 32);
    display.printf("RSSI %6.1f dBm", lastRssi);
    display.setCursor(0, 42);
    display.printf("SNR  %6.1f dB", lastSnr);
  }

  // Bottom line: lost packets, CRC errors, seconds since last packet
  display.setCursor(0, 54);
  if (noSignal) {
    display.printf("NO SIGNAL %lus", (unsigned long)(ageMs / 1000));
  } else {
    display.printf("Lost %lu CRC %lu %lus",
                   (unsigned long)missed, (unsigned long)crcErrors,
                   (unsigned long)(ageMs / 1000));
  }

  display.display();
}

// ---------- setup ----------
void setup() {
  Serial.begin(115200);
  delay(500);
  LOG("Heltec V4 LoRa receiver starting\n");

  pinMode(PRG_BTN, INPUT_PULLUP);

  // Display first (also powers Vext), so errors can be shown on screen
  haveDisplay = startDisplay();
  locHave = startLocalImu();
  LOG(locHave ? "Local IMU ready\n" : "Local IMU not found - check SDA/SCL wiring and power\n");
  LOG(haveDisplay ? "OLED ready\n" : "OLED not found - continuing without it\n");
  showStatus("Starting radio...", "");

  // Power up the RF front end
  pinMode(FEM_PWR, OUTPUT); digitalWrite(FEM_PWR, HIGH);
  pinMode(FEM_CSD, OUTPUT); digitalWrite(FEM_CSD, HIGH);
  delay(50);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  // freq, bw, sf, cr, sync word, tx power, preamble, TCXO voltage
  int state = radio.begin(915.0, 125.0, 9, 5, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, ACK_TX_POWER, 8, 1.8);
  if (state != RADIOLIB_ERR_NONE) {
    char buf[24];
    snprintf(buf, sizeof(buf), "code %d", state);
    showStatus("radio.begin FAILED", buf);
    while (true) {
      LOG("radio.begin failed, code %d\n", state);
      delay(2000);
    }
  }

  radio.setDio2AsRfSwitch(true);
  radio.setRfSwitchPins(RADIOLIB_NC, FEM_TX_EN); // front end into TX only while transmitting
  radio.setCRC(true);
  radio.setRxBoostedGainMode(true); // ~3 dB better sensitivity for range tests
  radio.setPacketReceivedAction(onPacket);

  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    char buf[24];
    snprintf(buf, sizeof(buf), "code %d", state);
    showStatus("startReceive FAILED", buf);
    while (true) {
      LOG("startReceive failed, code %d\n", state);
      delay(2000);
    }
  }

  LOG("Listening on 915 MHz, SF9, 125 kHz...\n");
  drawScreen();
}

// ---------- loop ----------
void loop() {
  static uint32_t lastDraw = 0;
  static uint32_t lastBeat = 0;
  static bool lastBtn = HIGH;

  // PRG button: short press = switch page, hold 1 s = reset counters
  static uint32_t pressStart = 0;
  static bool longDone = false;
  bool btn = digitalRead(PRG_BTN);
  if (lastBtn == HIGH && btn == LOW) { // pressed
    pressStart = millis();
    longDone = false;
    delay(30); // debounce
  } else if (btn == LOW && !longDone && millis() - pressStart > 1000) {
    longDone = true; // held 1 s
    resetStats();
    LOG("Counters reset\n");
    drawScreen();
  } else if (lastBtn == LOW && btn == HIGH) { // released
    if (!longDone) {
      page = (page + 1) % PAGE_COUNT;
      drawScreen();
    }
    delay(30);
  }
  lastBtn = btn;

  pollLocalImu();

  // Refresh the screen twice a second so the "seconds since" counter ticks
  if (millis() - lastDraw > (page == 4 ? 100 : 500)) {
    lastDraw = millis();
    drawScreen();
  }

  // Serial heartbeat
  if (millis() - lastBeat > 5000) {
    lastBeat = millis();
    LOG("...still listening\n");
  }

  static uint32_t lastHdg = 0;
  if (localFresh() && millis() - lastHdg > 1000) {
    lastHdg = millis();
    LOG("Local IMU: mag %.1f true %.1f cal %d\n", localHeadingMag(), localHeadingTrue(), locCal);
  }

  if (!packetReady) return;
  packetReady = false;

  String msg;
  int state = radio.readData(msg);

  // ---- Beacon from a dummy node ----
  if (state == RADIOLIB_ERR_NONE && msg.startsWith("BCN ")) {
    char nm[12]; long bseq, blat, blon; int bfix;
    float brssi = radio.getRSSI(), bsnr = radio.getSNR();
    if (sscanf(msg.c_str(), "BCN %11s %ld %ld %ld %d", nm, &bseq, &blat, &blon, &bfix) == 5) {
      nodeUpdate(nm, blat, blon, bfix, bseq, brssi, bsnr);
      beaconCount++;
    }
    drawScreen();
    radio.startReceive();
    return; // never ack beacons, never count them as nRF packets
  }

  if (state == RADIOLIB_ERR_NONE) {
    lastMsg = msg;
    lastRssi = radio.getRSSI();
    lastSnr = radio.getSNR();
    lastRxMs = millis();
    rxCount++;

    // Work out the sequence number from either message type
    long seq = -1;
    long s, lat, lon, alt;
    int y, p, r, acc, sats, fix;
    if (sscanf(msg.c_str(), "TEL %ld %d %d %d %d %ld %ld %ld %d %d",
               &s, &y, &p, &r, &acc, &lat, &lon, &alt, &sats, &fix) == 10) {
      seq = s;
      haveImu = true;
      imuYaw10 = y;
      imuPitch10 = p;
      imuRoll10 = r;
      imuAcc = acc;
      haveGnss = true;
      gpsLatE6 = lat;
      gpsLonE6 = lon;
      gpsAltM = alt;
      gpsSats = sats;
      gpsFix = fix;
      LOG("TEL #%ld  yaw %.1f pitch %.1f roll %.1f cal %d | ",
          seq, y / 10.0f, p / 10.0f, r / 10.0f, acc);
      if (fix > 0) {
        LOG("lat %.6f lon %.6f alt %ldm sats %d fix %d",
            lat / 1e6, lon / 1e6, alt, sats, fix);
      } else {
        LOG("%s (sats %d)", fix < 0 ? "no GNSS data" : "no fix", sats);
      }
      LOG(" | RSSI %.1f dBm SNR %.2f dB\n", lastRssi, lastSnr);
    } else if (sscanf(msg.c_str(), "IMU %ld %d %d %d %d", &s, &y, &p, &r, &acc) == 5) {
      seq = s;
      haveImu = true;
      haveGnss = false;
      imuYaw10 = y;
      imuPitch10 = p;
      imuRoll10 = r;
      imuAcc = acc;
      if (acc >= 0) {
        LOG("IMU #%ld  yaw %.1f  pitch %.1f  roll %.1f  cal %d   RSSI %.1f dBm  SNR %.2f dB\n",
            seq, y / 10.0f, p / 10.0f, r / 10.0f, acc, lastRssi, lastSnr);
      } else {
        LOG("IMU #%ld  (no IMU on transmitter)   RSSI %.1f dBm  SNR %.2f dB\n",
            seq, lastRssi, lastSnr);
      }
    } else if (msg.startsWith("hello ")) {
      haveImu = false;
      haveGnss = false;
      seq = msg.substring(6).toInt();
      LOG("Got: \"%s\"  RSSI %.1f dBm  SNR %.2f dB\n", msg.c_str(), lastRssi, lastSnr);
    }

    if (seq >= 0) {
      // Count gaps in the numbering
      if (lastSeq >= 0 && seq > lastSeq + 1) {
        missed += seq - lastSeq - 1;
      }
      lastSeq = seq; // also resyncs if the transmitter restarted

      // Reply "OK N <rssi> <snr>"
      char reply[40];
      snprintf(reply, sizeof(reply), "OK %ld %ld %ld",
               seq, lroundf(lastRssi), lroundf(lastSnr));
      delay(ACK_DELAY_MS);
      int t = radio.transmit(reply);
      packetReady = false; // ignore the TX-done interrupt from our own reply
      if (t == RADIOLIB_ERR_NONE) {
        acksSent++;
        LOG("  replied: %s\n", reply);
      } else {
        ackFails++;
        LOG("  reply FAILED, code %d\n", t);
      }
    } else {
      LOG("Got: \"%s\"  RSSI %.1f dBm  SNR %.2f dB  (unknown format, no reply)\n",
          msg.c_str(), lastRssi, lastSnr);
    }
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    crcErrors++;
    LOG("Packet received with CRC error\n");
  } else {
    LOG("readData failed, code %d\n", state);
  }

  drawScreen();
  radio.startReceive();
}

// /*
//  * Heltec WiFi LoRa 32 V4 - LoRa receiver with OLED display + acknowledgements.
//  *
//  * Receives telemetry from the nRF54L15:
//  *   "TEL <seq> <yaw> <pitch> <roll> <acc> <lat> <lon> <alt> <sats> <fix>"
//  *      yaw/pitch/roll : tenths of a degree
//  *      acc            : IMU calibration 0-3 (-1 = no IMU)
//  *      lat/lon        : millionths of a degree
//  *      alt            : metres
//  *      sats, fix      : satellites used, fix quality (0 none, -1 no GNSS data)
//  * and replies "OK <seq> <rssi> <snr>" so the nRF can count and log round trips.
//  * Older "IMU ..." and "hello N" messages are still accepted and acknowledged.
//  *
//  * Settings match the nRF54L15 lora_tx app:
//  *   915 MHz, 125 kHz BW, SF9, CR 4/5, sync word 0x12 (private), preamble 8
//  *
//  * Two screen pages - short press PRG to switch:
//  *
//  *   IMU page                    GPS page
//  *   RX 42 ACK 42 c3             RX 42 ACK 42 c3
//  *   Y 123.4      (large)        Lat  40.712345
//  *   P  -1.2  R   3.4            Lon -74.001234
//  *   RSSI -22 SNR 10.0           Alt 10m Sat 8 Fix 1
//  *                               RSSI -22 SNR 10.0
//  *   Lost 0 CRC 0 1s             Lost 0 CRC 0 1s
//  *
//  * "Lost" counts gaps in the sequence numbers, handy for range tests.
//  * Hold PRG for 1 second to reset the counters.
//  *
//  * Works without USB (on battery). Serial output is still printed when
//  * plugged into a computer.
//  *
//  * Attach the Heltec's antenna before powering it.
//  */

// #include <Arduino.h>
// #include <Wire.h>
// #include <RadioLib.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>

// // SX1262 pins on the Heltec V4
// #define LORA_NSS   8
// #define LORA_SCK   9
// #define LORA_MOSI  10
// #define LORA_MISO  11
// #define LORA_RST   12
// #define LORA_BUSY  13
// #define LORA_DIO1  14

// // Board power / front-end (GC1109) control
// #define VEXT_CTRL  36  // powers OLED + RF front end (LOW = on on most boards)
// #define FEM_PWR    7   // front-end LDO enable
// #define FEM_CSD    2   // front-end chip enable
// #define FEM_TX_EN  46  // front-end TX enable: HIGH in TX, LOW in RX (RadioLib drives it)

// // OLED (SSD1306 128x64, I2C)
// #define OLED_SDA   17
// #define OLED_SCL   18
// #define OLED_RST   21
// #define OLED_ADDR  0x3C

// // PRG button (resets counters)
// #define PRG_BTN    0

// // Reply settings
// #define ACK_DELAY_MS  100  // give the nRF time to switch to receive
// #define ACK_TX_POWER  0 // dBm from the SX1262 (the board's PA adds gain on top)

// // Show "NO SIGNAL" after this long without a packet
// #define SIGNAL_TIMEOUT_MS 6000

// // ---------- NEW: point-to-nRF settings ----------
// // Where THIS Heltec is sitting (decimal degrees x 1,000,000) and which compass
// // direction the top edge of the OLED faces (0 = north, 90 = east, ...).
// // The Heltec has no GPS or compass, so you type these in and re-flash.
// #define HOME_LAT_E6 42393518L // 42.393518 N
// #define HOME_LON_E6 -72528339L // 72.528339 W
// #define HOME_HEADING_DEG  0.0f // direction the screen's top edge points
// #define PAGE_COUNT 4  // 0 = IMU, 1 = GPS, 2 = point to nRF, 3 = other nodes
// #define MAX_NODES 6
// #define NODE_STALE_MS 30000 // show :lost" when a node is quiet for too long

// // Only print to Serial when a computer is connected (so battery use never blocks)
// #define LOG(...) do { if (Serial) Serial.printf(__VA_ARGS__); } while (0)

// SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
// Adafruit_SSD1306 display(128, 64, &Wire, OLED_RST);

// volatile bool packetReady = false;

// void IRAM_ATTR onPacket() {
//   packetReady = true;
// }

// // ---------- stats ----------
// bool     haveDisplay = false;
// String   lastMsg     = "";
// bool     haveImu     = false; // last packet carried IMU data
// int      imuYaw10    = 0;
// int      imuPitch10  = 0;
// int      imuRoll10   = 0;
// int      imuAcc      = -1;
// bool     haveGnss    = false; // last packet carried GNSS data
// long     gpsLatE6    = 0;
// long     gpsLonE6    = 0;
// long     gpsAltM     = 0;
// int      gpsSats     = 0;
// int      gpsFix      = -1;
// int      page        = 0; // 0 = IMU, 1 = GPS, 2 = point to nRF
// float    lastRssi    = 0;
// float    lastSnr     = 0;
// uint32_t rxCount     = 0;
// uint32_t crcErrors   = 0;
// uint32_t missed      = 0;
// long     lastSeq     = -1;
// uint32_t lastRxMs    = 0;
// uint32_t acksSent    = 0;
// uint32_t ackFails    = 0;

// struct Node {
//   bool     used;
//   char     name[8];
//   long     latE6, lonE6;
//   int      fix;
//   long     seq;
//   float    rssi, snr;
//   uint32_t lastMs;
// };
// Node     nodes[MAX_NODES];
// uint32_t beaconCount = 0;

// void nodeUpdate(const char *name, long lat, long lon, int fix, long seq, float rssi, float snr) {
//   int slot = -1;
//   for (int i = 0; i < MAX_NODES; i++)
//     if (nodes[i].used && strncmp(nodes[i].name, name, sizeof(nodes[i].name) - 1) == 0) { slot = i; break; }
//   if (slot < 0) for (int i = 0; i < MAX_NODES; i++) if (!nodes[i].used) { slot = i; break; }
//   if (slot < 0) { slot = 0; for (int i = 1; i < MAX_NODES; i++) if (nodes[i].lastMs < nodes[slot].lastMs) slot = i; }
//   Node &n = nodes[slot];
//   n.used = true;
//   strncpy(n.name, name, sizeof(n.name) - 1);
//   n.name[sizeof(n.name) - 1] = '\0';
//   n.latE6 = lat;  n.lonE6 = lon;  n.fix = fix;  n.seq = seq;
//   n.rssi = rssi;  n.snr = snr;    n.lastMs = millis();
// }

// void resetStats() {
//   lastMsg   = "";
//   haveImu   = false;
//   haveGnss  = false;
//   rxCount   = 0;
//   crcErrors = 0;
//   missed    = 0;
//   lastSeq   = -1;
//   acksSent  = 0;
//   ackFails  = 0;
//   memset(nodes, 0, sizeof(nodes));
//   beaconCount = 0;
// }

// // ---------- NEW: bearing / distance to the nRF ----------
// // Great-circle initial bearing (0-360, clockwise from north) and haversine
// // distance from the Heltec's fixed position to the nRF's last GNSS fix.
// void pointToNrf(float &bearingDeg, float &distM) {
//   const double D2R = 0.017453292519943295;
//   double lat1 = (HOME_LAT_E6 / 1e6) * D2R;
//   double lon1 = (HOME_LON_E6 / 1e6) * D2R;
//   double lat2 = (gpsLatE6 / 1e6) * D2R;
//   double lon2 = (gpsLonE6 / 1e6) * D2R;
//   double dLat = lat2 - lat1;
//   double dLon = lon2 - lon1;

//   double y = sin(dLon) * cos(lat2);
//   double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
//   double b = atan2(y, x) / D2R;
//   bearingDeg = (float)fmod(b + 360.0, 360.0);

//   double h = sin(dLat / 2) * sin(dLat / 2) +
//              cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
//   distM = (float)(2.0 * 6371000.0 * asin(sqrt(h)));
// }

// // Same maths as pointToNrf(), for any position (used for the other nodes)
// void bearingFromHome(long latE6, long lonE6, float &bearingDeg, float &distM) {
//   const double D2R = 0.017453292519943295;
//   double lat1 = (HOME_LAT_E6 / 1e6) * D2R;
//   double lon1 = (HOME_LON_E6 / 1e6) * D2R;
//   double lat2 = (latE6 / 1e6) * D2R;
//   double lon2 = (lonE6 / 1e6) * D2R;
//   double dLat = lat2 - lat1;
//   double dLon = lon2 - lon1;

//   double y = sin(dLon) * cos(lat2);
//   double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
//   bearingDeg = (float)fmod(atan2(y, x) / D2R + 360.0, 360.0);

//   double h = sin(dLat / 2) * sin(dLat / 2) +
//              cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
//   distM = (float)(2.0 * 6371000.0 * asin(sqrt(h)));
// }

// // Arrow centred at (cx,cy); angleDeg is clockwise from the top of the screen.

// // Arrow centred at (cx,cy); angleDeg is clockwise from the top of the screen.
// void drawArrow(int cx, int cy, int r, float angleDeg) {
//   const float D2R = 0.017453293f;
//   float a = angleDeg * D2R;
//   int tx = cx + (int)lroundf(r * sinf(a));
//   int ty = cy - (int)lroundf(r * cosf(a));
//   int bx = cx - (int)lroundf(r * sinf(a));
//   int by = cy + (int)lroundf(r * cosf(a));
//   int lx = tx + (int)lroundf(7 * sinf(a + 2.62f)); // arrow head, +/-150 deg
//   int ly = ty - (int)lroundf(7 * cosf(a + 2.62f));
//   int rx = tx + (int)lroundf(7 * sinf(a - 2.62f));
//   int ry = ty - (int)lroundf(7 * cosf(a - 2.62f));
//   display.drawCircle(cx, cy, r + 3, SSD1306_WHITE);
//   display.drawLine(bx, by, tx, ty, SSD1306_WHITE);
//   display.drawLine(tx, ty, lx, ly, SSD1306_WHITE);
//   display.drawLine(tx, ty, rx, ry, SSD1306_WHITE);
// }

// // ---------- display helpers ----------
// bool oledPresent() {
//   Wire.beginTransmission(OLED_ADDR);
//   return Wire.endTransmission() == 0;
// }

// void resetOled() {
//   pinMode(OLED_RST, OUTPUT);
//   digitalWrite(OLED_RST, LOW);
//   delay(20);
//   digitalWrite(OLED_RST, HIGH);
//   delay(20);
// }

// // Bring up the OLED. Tries Vext LOW first (the usual Heltec setting),
// // then HIGH in case this board revision uses the opposite polarity.
// bool startDisplay() {
//   Wire.begin(OLED_SDA, OLED_SCL);

//   pinMode(VEXT_CTRL, OUTPUT);
//   digitalWrite(VEXT_CTRL, LOW);
//   delay(100);
//   resetOled();
//   if (!oledPresent()) {
//     digitalWrite(VEXT_CTRL, HIGH);
//     delay(100);
//     resetOled();
//     if (!oledPresent()) {
//       digitalWrite(VEXT_CTRL, LOW); // back to the setting the radio is known to work with
//       return false;
//     }
//     LOG("OLED found with Vext HIGH\n");
//   }

//   // reset already done, Wire already started
//   if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, false, false)) {
//     return false;
//   }
//   display.setTextColor(SSD1306_WHITE);
//   display.setTextWrap(false);
//   return true;
// }

// void showStatus(const char *line1, const char *line2) {
//   if (!haveDisplay) return;
//   display.clearDisplay();
//   display.setTextSize(1);
//   display.setCursor(0, 0);
//   display.print(line1);
//   display.setCursor(0, 16);
//   display.print(line2);
//   display.display();
// }

// // Page 3: list of other nodes. Draws the whole screen (including its bottom line).
// void drawNodesPage() {
//   int shown = 0, active = 0;
//   for (int i = 0; i < MAX_NODES; i++) if (nodes[i].used) active++;

//   for (int i = 0; i < MAX_NODES && shown < 4; i++) {
//     if (!nodes[i].used) continue;
//     const Node &n = nodes[i];
//     int y = 12 + shown * 10;
//     uint32_t age = (millis() - n.lastMs) / 1000;
//     display.setCursor(0, y);
//     if (n.fix > 0) {
//       float brg, dist;
//       bearingFromHome(n.latE6, n.lonE6, brg, dist);
//       char d[8];
//       if (dist < 1000.0f) snprintf(d, sizeof(d), "%dm", (int)lroundf(dist));
//       else                snprintf(d, sizeof(d), "%.1fkm", dist / 1000.0f);
//       if (millis() - n.lastMs > NODE_STALE_MS) display.printf("%-5.5s%5s %3d lost", n.name, d, (int)lroundf(brg) % 360);
//       else                                     display.printf("%-5.5s%5s %3d %3lus", n.name, d, (int)lroundf(brg) % 360, (unsigned long)age);
//     } else {
//       display.printf("%-5.5s no fix %3lus", n.name, (unsigned long)age);
//     }
//     shown++;
//   }
//   if (shown == 0) {
//     display.setCursor(0, 22);
//     display.print("No nodes heard yet");
//   }

//   display.setCursor(0, 54);
//   if (active > shown) display.printf("Beacons %lu (+%d more)", (unsigned long)beaconCount, active - shown);
//   else                display.printf("Beacons heard: %lu", (unsigned long)beaconCount);
//   display.display();
// }


// void drawScreen() {
//   if (!haveDisplay) return;

//   uint32_t ageMs = millis() - lastRxMs;
//   bool noSignal = (rxCount == 0) || (ageMs > SIGNAL_TIMEOUT_MS);

//   display.clearDisplay();
//   display.setTextSize(1);

//   // Top line: settings + packet count
//   display.setCursor(0, 0);
//   display.printf("RX %lu ACK %lu", (unsigned long)rxCount, (unsigned long)acksSent);
//   if (haveImu) {
//     display.setCursor(104, 0);
//     if (imuAcc >= 0) display.printf("c%d", imuAcc);
//     else             display.print("noI");
//   }

//   if (rxCount == 0 && page != 3) {
//     display.setTextSize(2);
//     display.setCursor(0, 22);
//     display.print("Waiting..");
//     display.setTextSize(1);
//     display.setCursor(0, 54);
//     display.printf("CRC err %lu", (unsigned long)crcErrors);
//     display.display();
//     return;
//   }

//   if (page == 1 && haveGnss) {
//     // GPS page
//     if (gpsFix > 0) {
//       display.setTextSize(1);
//       display.setCursor(0, 12);
//       display.printf("Lat %s%ld.%06ld", gpsLatE6 < 0 ? "-" : " ",
//                      labs(gpsLatE6) / 1000000L, labs(gpsLatE6) % 1000000L);
//       display.setCursor(0, 22);
//       display.printf("Lon %s%ld.%06ld", gpsLonE6 < 0 ? "-" : " ",
//                      labs(gpsLonE6) / 1000000L, labs(gpsLonE6) % 1000000L);
//       display.setCursor(0, 32);
//       display.printf("Alt %ldm Sat %d Fix %d", gpsAltM, gpsSats, gpsFix);
//     } else {
//       display.setTextSize(2);
//       display.setCursor(0, 12);
//       display.print(gpsFix < 0 ? "No GNSS" : "No fix");
//       display.setTextSize(1);
//       display.setCursor(0, 32);
//       if (gpsFix == 0) display.printf("Sats %d, searching", gpsSats);
//       else             display.print("Check GNSS wiring");
//     }
//     display.setCursor(0, 42);
//     display.printf("RSSI %d SNR %.1f", (int)lroundf(lastRssi), lastSnr);
//   } else if (page == 2) {
//     // NEW: point-to-nRF page
//     if (haveGnss && gpsFix > 0) {
//       float brg, dist;
//       pointToNrf(brg, dist);
//       float rel = fmodf(brg - HOME_HEADING_DEG + 360.0f, 360.0f);

//       display.setTextSize(1);
//       display.setCursor(0, 12);
//       display.print("nRF is");
//       display.setTextSize(2);
//       display.setCursor(0, 22);
//       if (dist < 1000.0f) display.printf("%dm", (int)lroundf(dist));
//       else                display.printf("%.1fkm", dist / 1000.0f);
//       display.setTextSize(1);
//       display.setCursor(0, 42);
//       display.printf("Brg %3d", (int)lroundf(brg) % 360);
//       drawArrow(100, 33, 15, rel);
//     } else {
//       display.setTextSize(2);
//       display.setCursor(0, 12);
//       display.print("No nRF");
//       display.setCursor(0, 30);
//       display.print(gpsFix < 0 ? "GNSS" : "fix");
//     }
//   } else if (page == 3) {            
//       drawNodesPage();                 
//       return;
//   } else if (haveImu && imuAcc >= 0) {
//     // IMU page: yaw large, pitch/roll below
//     display.setTextSize(2);
//     display.setCursor(0, 12);
//     display.printf("Y %5.1f", imuYaw10 / 10.0f);

//     display.setTextSize(1);
//     display.setCursor(0, 32);
//     display.printf("P %6.1f R %6.1f", imuPitch10 / 10.0f, imuRoll10 / 10.0f);
//     display.setCursor(0, 42);
//     display.printf("RSSI %d SNR %.1f", (int)lroundf(lastRssi), lastSnr);
//   } else {
//     // Plain message (or IMU not found on the nRF side), large
//     display.setTextSize(2);
//     display.setCursor(0, 12);
//     display.print(haveImu ? "No IMU" : lastMsg.substring(0, 10));

//     display.setTextSize(1);
//     display.setCursor(0, 32);
//     display.printf("RSSI %6.1f dBm", lastRssi);
//     display.setCursor(0, 42);
//     display.printf("SNR  %6.1f dB", lastSnr);
//   }

//   // Bottom line: lost packets, CRC errors, seconds since last packet
//   display.setCursor(0, 54);
//   if (noSignal) {
//     display.printf("NO SIGNAL %lus", (unsigned long)(ageMs / 1000));
//   } else {
//     display.printf("Lost %lu CRC %lu %lus",
//                    (unsigned long)missed, (unsigned long)crcErrors,
//                    (unsigned long)(ageMs / 1000));
//   }

//   display.display();
// }

// // ---------- setup ----------
// void setup() {
//   Serial.begin(115200);
//   delay(500);
//   LOG("Heltec V4 LoRa receiver starting\n");

//   pinMode(PRG_BTN, INPUT_PULLUP);

//   // Display first (also powers Vext), so errors can be shown on screen
//   haveDisplay = startDisplay();
//   LOG(haveDisplay ? "OLED ready\n" : "OLED not found - continuing without it\n");
//   showStatus("Starting radio...", "");

//   // Power up the RF front end
//   pinMode(FEM_PWR, OUTPUT); digitalWrite(FEM_PWR, HIGH);
//   pinMode(FEM_CSD, OUTPUT); digitalWrite(FEM_CSD, HIGH);
//   delay(50);

//   SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

//   // freq, bw, sf, cr, sync word, tx power, preamble, TCXO voltage
//   int state = radio.begin(915.0, 125.0, 9, 5, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, ACK_TX_POWER, 8, 1.8);
//   if (state != RADIOLIB_ERR_NONE) {
//     char buf[24];
//     snprintf(buf, sizeof(buf), "code %d", state);
//     showStatus("radio.begin FAILED", buf);
//     while (true) {
//       LOG("radio.begin failed, code %d\n", state);
//       delay(2000);
//     }
//   }

//   radio.setDio2AsRfSwitch(true);
//   radio.setRfSwitchPins(RADIOLIB_NC, FEM_TX_EN); // front end into TX only while transmitting
//   radio.setCRC(true);
//   radio.setRxBoostedGainMode(true); // ~3 dB better sensitivity for range tests
//   radio.setPacketReceivedAction(onPacket);

//   state = radio.startReceive();
//   if (state != RADIOLIB_ERR_NONE) {
//     char buf[24];
//     snprintf(buf, sizeof(buf), "code %d", state);
//     showStatus("startReceive FAILED", buf);
//     while (true) {
//       LOG("startReceive failed, code %d\n", state);
//       delay(2000);
//     }
//   }

//   LOG("Listening on 915 MHz, SF9, 125 kHz...\n");
//   drawScreen();
// }

// // ---------- loop ----------
// void loop() {
//   static uint32_t lastDraw = 0;
//   static uint32_t lastBeat = 0;
//   static bool lastBtn = HIGH;

//   // PRG button: short press = switch page, hold 1 s = reset counters
//   static uint32_t pressStart = 0;
//   static bool longDone = false;
//   bool btn = digitalRead(PRG_BTN);
//   if (lastBtn == HIGH && btn == LOW) {  // pressed
//     pressStart = millis();
//     longDone = false;
//     delay(30); // debounce
//   } else if (btn == LOW && !longDone && millis() - pressStart > 1000) {
//     longDone = true; // held 1 s
//     resetStats();
//     LOG("Counters reset\n");
//     drawScreen();
//   } else if (lastBtn == LOW && btn == HIGH) { // released
//     if (!longDone) {
//       page = (page + 1) % PAGE_COUNT;
//       drawScreen();
//     }
//     delay(30);
//   }
//   lastBtn = btn;

//   // Refresh the screen twice a second so the "seconds since" counter ticks
//   if (millis() - lastDraw > 500) {
//     lastDraw = millis();
//     drawScreen();
//   }

//   // Serial heartbeat
//   if (millis() - lastBeat > 5000) {
//     lastBeat = millis();
//     LOG("...still listening\n");
//   }

//   if (!packetReady) return;
//   packetReady = false;

//   String msg;
//   int state = radio.readData(msg);

//   // ---- Beacon from a dummy node ----
//   if (state == RADIOLIB_ERR_NONE && msg.startsWith("BCN ")) {
//     char nm[12]; long bseq, blat, blon; int bfix;
//     float brssi = radio.getRSSI(), bsnr = radio.getSNR();
//     if (sscanf(msg.c_str(), "BCN %11s %ld %ld %ld %d", nm, &bseq, &blat, &blon, &bfix) == 5) {
//       nodeUpdate(nm, blat, blon, bfix, bseq, brssi, bsnr);
//       beaconCount++;
//     }
//     drawScreen();
//     radio.startReceive();
//     return;   // never ack beacons, never count them as nRF packets
//   }

//   if (state == RADIOLIB_ERR_NONE) {
//     lastMsg  = msg;
//     lastRssi = radio.getRSSI();
//     lastSnr  = radio.getSNR();
//     lastRxMs = millis();
//     rxCount++;

//     // Work out the sequence number from either message type
//     long seq = -1;
//     long s, lat, lon, alt;
//     int y, p, r, acc, sats, fix;
//     if (sscanf(msg.c_str(), "TEL %ld %d %d %d %d %ld %ld %ld %d %d",
//                &s, &y, &p, &r, &acc, &lat, &lon, &alt, &sats, &fix) == 10) {
//       seq        = s;
//       haveImu    = true;
//       imuYaw10   = y;
//       imuPitch10 = p;
//       imuRoll10  = r;
//       imuAcc     = acc;
//       haveGnss   = true;
//       gpsLatE6   = lat;
//       gpsLonE6   = lon;
//       gpsAltM    = alt;
//       gpsSats    = sats;
//       gpsFix     = fix;
//       LOG("TEL #%ld  yaw %.1f pitch %.1f roll %.1f cal %d | ",
//           seq, y / 10.0f, p / 10.0f, r / 10.0f, acc);
//       if (fix > 0) {
//         LOG("lat %.6f lon %.6f alt %ldm sats %d fix %d",
//             lat / 1e6, lon / 1e6, alt, sats, fix);
//       } else {
//         LOG("%s (sats %d)", fix < 0 ? "no GNSS data" : "no fix", sats);
//       }
//       LOG(" | RSSI %.1f dBm SNR %.2f dB\n", lastRssi, lastSnr);
//     } else if (sscanf(msg.c_str(), "IMU %ld %d %d %d %d", &s, &y, &p, &r, &acc) == 5) {
//       seq        = s;
//       haveImu    = true;
//       haveGnss   = false;
//       imuYaw10   = y;
//       imuPitch10 = p;
//       imuRoll10  = r;
//       imuAcc     = acc;
//       if (acc >= 0) {
//         LOG("IMU #%ld  yaw %.1f  pitch %.1f  roll %.1f  cal %d   RSSI %.1f dBm  SNR %.2f dB\n",
//             seq, y / 10.0f, p / 10.0f, r / 10.0f, acc, lastRssi, lastSnr);
//       } else {
//         LOG("IMU #%ld  (no IMU on transmitter)   RSSI %.1f dBm  SNR %.2f dB\n",
//             seq, lastRssi, lastSnr);
//       }
//     } else if (msg.startsWith("hello ")) {
//       haveImu  = false;
//       haveGnss = false;
//       seq = msg.substring(6).toInt();
//       LOG("Got: \"%s\"  RSSI %.1f dBm  SNR %.2f dB\n", msg.c_str(), lastRssi, lastSnr);
//     }

//     if (seq >= 0) {
//       // Count gaps in the numbering
//       if (lastSeq >= 0 && seq > lastSeq + 1) {
//         missed += seq - lastSeq - 1;
//       }
//       lastSeq = seq; // also resyncs if the transmitter restarted

//       // Reply "OK N <rssi> <snr>"
//       char reply[40];
//       snprintf(reply, sizeof(reply), "OK %ld %ld %ld",
//                seq, lroundf(lastRssi), lroundf(lastSnr));
//       delay(ACK_DELAY_MS);
//       int t = radio.transmit(reply);
//       packetReady = false; // ignore the TX-done interrupt from our own reply
//       if (t == RADIOLIB_ERR_NONE) {
//         acksSent++;
//         LOG("  replied: %s\n", reply);
//       } else {
//         ackFails++;
//         LOG("  reply FAILED, code %d\n", t);
//       }
//     } else {
//       LOG("Got: \"%s\"  RSSI %.1f dBm  SNR %.2f dB  (unknown format, no reply)\n",
//           msg.c_str(), lastRssi, lastSnr);
//     }
//   } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
//     crcErrors++;
//     LOG("Packet received with CRC error\n");
//   } else {
//     LOG("readData failed, code %d\n", state);
//   }

//   drawScreen();
//   radio.startReceive();
// }