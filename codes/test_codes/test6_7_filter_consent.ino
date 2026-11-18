/*
  TEST 6 + TEST 7 (v2) - Watch-side request filtering and consent gate, ON THE WATCH SCREEN
  Project I (247-519-VA) | Tests: REQSU-01 (filtering), REQSU-06 (consent), R3
  Hardware: Waveshare ESP32-S3-Touch-AMOLED-2.06. NO radio and NO second device needed.
  You do NOT need the Serial Monitor: instructions, buttons and results are all on the watch.

  SELF-CONTAINED MODEL (nothing external):
    8 abstract skills A..H, one bit each in a uint8_t (A=bit0 ... H=bit7).
    Profile = { skills mask, available flag }, Request = { required skills mask }.
    Rule    : RELEVANT  <=>  profile.available AND (profile.skills & required) != 0
    Two criteria (skill overlap, availability) are tested alone, then combined (R3 mitigation).

  TEST 6 (24 cases): hand-written expected answers vs isRelevant().  PASS: 24 of 24.

  TEST 7 (12 runs): consent gate. YOU are the participant. Before each run the screen tells you
    what to do; 3 s later the request alert appears with two big buttons:
        left half = REJECT, right half = ACCEPT (touch the lower half of the screen).
      ACCEPT run   -> tap ACCEPT           REJECT run   -> tap REJECT
      TIMEOUT run  -> do nothing (8 s)     FILTERED run -> do nothing (no alert should appear)
    The profile is "sent" only by a TX line (shown on screen, stand-in for the radio).
    PASS (log before running): all 12 runs show the expected outcome, the profile is sent exactly
    once in each ACCEPT run and never otherwise, and there are zero violations.

  BUTTON FALLBACK: if the touch chip is not found at start-up, the screen says so and the BOOT
    button is used instead: short press = ACCEPT, hold about 1 s = REJECT. Write down which mode ran.

  LIMITS (state in the logbook): the screen tap stands in for the final user interface, and the TX
  line stands in for the radio send. This supports REQSU-06 on the watch side only.

  PINS (Waveshare docs): QSPI display CS12 CLK11 D0-D3 = 4,5,6,7 RST8; touch FT3168 on I2C
  SDA15 SCL14 RST9 (INT38 not used); BOOT button GPIO0.

  LIBRARIES: "GFX Library for Arduino" (Arduino_GFX) v1.6.0 from the Library Manager. Nothing else.
  ARDUINO IDE SETTINGS (Waveshare): ESP32S3 Dev Module, USB CDC On Boot Enabled, Flash 32MB,
    PSRAM OPI, a large partition scheme, esp32 core 3.3.x.  Serial Monitor (optional): 115200.

  UNVERIFIED (could not be compiled against the library here): the Arduino_CO5300 constructor
  arguments (22 = column offset from the Waveshare docs) and the touch I2C address 0x38. If compile
  or start-up fails, send me the first error line or the Serial lines starting with "I2C found".
*/

#include <Arduino_GFX_Library.h>
#include <Wire.h>

#define LCD_CS 12
#define LCD_SCLK 11
#define LCD_SDIO0 4
#define LCD_SDIO1 5
#define LCD_SDIO2 6
#define LCD_SDIO3 7
#define LCD_RESET 8
#define LCD_W 410
#define LCD_H 502
#define TOUCH_SDA 15
#define TOUCH_SCL 14
#define TOUCH_RST 9
#define TOUCH_ADDR 0x38
#define BOOT_BTN 0

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_CO5300 *gfx = new Arduino_CO5300(bus, LCD_RESET, 0, LCD_W, LCD_H, 22, 0, 0, 0);

#define N_SKILLS 8
static const char SKILL_NAME[N_SKILLS] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'};
#define CONSENT_MS 8000UL
#define GAP_MS     3000UL
#define BTN_ZONE_Y 280      // touches above this line are ignored

const uint16_t BLK = 0x0000, WHT = 0xFFFF;   // black and white only: legible in a photo or print

// ======================= model (no hardware) =======================
struct Profile { const char *id; uint8_t skills; bool available; };
struct Case    { uint8_t id; const char *criterion; uint8_t skills; bool avail; uint8_t req; bool expect; };

bool isRelevant(uint8_t skills, bool available, uint8_t req) {
  return available && ((skills & req) != 0);
}

void maskStr(uint8_t m, char *out) {   // out >= 20 chars
  int n = 0;
  out[n++] = '{';
  bool first = true;
  for (int i = 0; i < N_SKILLS; i++) {
    if (m & (1 << i)) {
      if (!first) out[n++] = ',';
      out[n++] = SKILL_NAME[i];
      first = false;
    }
  }
  out[n++] = '}';
  out[n] = 0;
}

// Expected answers are written by hand from the rule above, NOT computed by isRelevant().
// masks: A=0x01 B=0x02 C=0x04 D=0x08 E=0x10 F=0x20 G=0x40 H=0x80
const Case CASES[] = {
  // --- skill criterion alone (participant available) ---
  { 1, "skill", 0x01, true,  0x01, true  },
  { 2, "skill", 0x01, true,  0x02, false },
  { 3, "skill", 0x03, true,  0x02, true  },
  { 4, "skill", 0x03, true,  0x0C, false },
  { 5, "skill", 0x05, true,  0x0C, true  },
  { 6, "skill", 0xFF, true,  0x80, true  },
  { 7, "skill", 0x80, true,  0x7F, false },
  { 8, "skill", 0x00, true,  0x01, false },
  { 9, "skill", 0x01, true,  0x00, false },
  {10, "skill", 0x18, true,  0x18, true  },
  {11, "skill", 0x07, true,  0x38, false },
  {12, "skill", 0x20, true,  0x38, true  },
  // --- availability criterion alone (skills would match) ---
  {13, "avail", 0x01, false, 0x01, false },
  {14, "avail", 0x02, true,  0x02, true  },
  {15, "avail", 0xFF, false, 0xFF, false },
  {16, "avail", 0x00, true,  0x00, false },
  // --- both criteria combined ---
  {17, "both",  0x05, true,  0x04, true  },
  {18, "both",  0x05, true,  0x0A, false },
  {19, "both",  0x05, false, 0x04, false },
  {20, "both",  0x05, false, 0x02, false },
  {21, "both",  0x40, true,  0xC0, true  },
  {22, "both",  0x40, true,  0x80, false },
  {23, "both",  0xAA, true,  0x55, false },
  {24, "both",  0xAA, true,  0x95, true  },
};
const int N_CASES = sizeof(CASES) / sizeof(CASES[0]);

// plain ints, not enums: the Arduino IDE auto-generates prototypes before type definitions
#define O_FILTERED 0
#define O_ACCEPTED 1
#define O_REJECTED 2
#define O_TIMEOUT  3
const char *OUT_NAME[] = {"FILTERED", "ACCEPT", "REJECT", "TIMEOUT"};
const char *OUT_SHORT[] = {"FILTER", "ACCEPT", "REJECT", "TIMEOUT"};

struct Run { uint8_t req; int expect; };
const Run RUNS[] = {
  {0x04, O_ACCEPTED}, {0x01, O_ACCEPTED}, {0x09, O_ACCEPTED}, {0x04, O_ACCEPTED},
  {0x04, O_REJECTED}, {0x05, O_REJECTED}, {0x01, O_REJECTED},
  {0x04, O_TIMEOUT},  {0x01, O_TIMEOUT},  {0x0D, O_TIMEOUT},
  {0x02, O_FILTERED}, {0x18, O_FILTERED},
};
const int N_RUNS = sizeof(RUNS) / sizeof(RUNS[0]);

Profile me = {"P01", 0x05, true};   // skills {A,C}, available
bool consented = false;
uint32_t txProfile = 0, blocked = 0, violations = 0;
uint32_t decideAt = 0, txAt = 0;
int observedRun[N_RUNS];
bool okRun[N_RUNS];

// The only path by which profile data leaves the watch; called after EVERY matching request.
bool shareProfileGate() {
  if (!consented) {
    blocked++;
    Serial.println("  gate: BLOCKED (no consent)");
    return false;
  }
  txAt = millis();
  if (txAt < decideAt) {   // sent before the decision: impossible if the logic is right
    violations++;
    Serial.println("  !! VIOLATION: sent before decision");
  }
  txProfile++;
  char m[20];
  maskStr(me.skills, m);
  Serial.printf("  TX PROFILE id=%s skills=%s\n", me.id, m);
  return true;
}

// ======================= watch input =======================
bool touchOk = false;
uint32_t btnDownAt = 0;
#define IN_NONE 0
#define IN_ACCEPT 1
#define IN_REJECT 2

bool readTouch(int &x, int &y) {
  Wire.beginTransmission(TOUCH_ADDR);
  Wire.write(0x02);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(TOUCH_ADDR, 5) != 5) return false;
  uint8_t n = Wire.read() & 0x0F;
  uint8_t xh = Wire.read(), xl = Wire.read(), yh = Wire.read(), yl = Wire.read();
  if (n == 0) return false;
  x = ((xh & 0x0F) << 8) | xl;
  y = ((yh & 0x0F) << 8) | yl;
  return true;
}

int pollInput() {
  if (touchOk) {
    int x, y;
    if (readTouch(x, y) && y >= BTN_ZONE_Y) return (x >= LCD_W / 2) ? IN_ACCEPT : IN_REJECT;
    return IN_NONE;
  }
  bool down = (digitalRead(BOOT_BTN) == LOW);
  if (down && btnDownAt == 0) btnDownAt = millis();
  if (!down && btnDownAt != 0) {
    uint32_t d = millis() - btnDownAt;
    btnDownAt = 0;
    if (d >= 40) return (d >= 1000) ? IN_REJECT : IN_ACCEPT;
  }
  return IN_NONE;
}

void waitRelease() {   // a finger or press from the previous screen must not count
  uint32_t quiet = millis(), giveUp = millis() + 2000;
  while (millis() < giveUp) {
    int x, y;
    bool t = touchOk ? readTouch(x, y) : (digitalRead(BOOT_BTN) == LOW);
    if (t) quiet = millis();
    if (millis() - quiet >= 200) break;
    delay(10);
  }
  btnDownAt = 0;
}

// ======================= screens =======================
void txtc(int x, int y, uint8_t size, const char *s, bool inverse) {   // no default args (Arduino prototypes)
  gfx->setTextSize(size);
  gfx->setTextColor(inverse ? BLK : WHT, inverse ? WHT : BLK);
  gfx->setCursor(x, y);
  gfx->print(s);
}

void txt(int x, int y, uint8_t size, const char *s) { txtc(x, y, size, s, false); }

void screenPlan(int k, int e) {
  gfx->fillScreen(BLK);
  char b[24];
  snprintf(b, sizeof b, "RUN %d / %d", k, N_RUNS);
  txt(20, 30, 4, b);
  txt(20, 110, 2, "Next, you are the participant:");
  switch (e) {
    case O_ACCEPTED:
      if (touchOk) { txt(20, 170, 4, "TAP"); txt(20, 215, 4, "ACCEPT"); }
      else         { txt(20, 170, 3, "SHORT-PRESS"); txt(20, 215, 3, "BOOT BUTTON"); }
      txt(20, 300, 2, "when the alert shows");
      break;
    case O_REJECTED:
      if (touchOk) { txt(20, 170, 4, "TAP"); txt(20, 215, 4, "REJECT"); }
      else         { txt(20, 170, 3, "HOLD BOOT"); txt(20, 215, 3, "ABOUT 1 S"); }
      txt(20, 300, 2, "when the alert shows");
      break;
    case O_TIMEOUT:
      txt(20, 170, 4, "DO NOTHING");
      txt(20, 300, 2, "let the alert time out (8 s)");
      break;
    default:
      txt(20, 170, 4, "DO NOTHING");
      txt(20, 300, 2, "no alert should appear");
      break;
  }
  txt(20, 440, 2, "request arrives in 3 s");
}

void screenRequest(uint8_t req) {
  gfx->fillScreen(BLK);
  char m[20];
  maskStr(req, m);
  txt(20, 25, 3, "TASK REQUEST");
  txt(20, 80, 2, "task needs:");
  txt(20, 110, 3, m);
  txt(20, 165, 2, "Share your profile?");
  if (touchOk) {
    gfx->drawRect(10, 300, 190, 170, WHT);
    gfx->drawRect(11, 301, 188, 168, WHT);
    txt(35, 370, 3, "REJECT");
    gfx->fillRect(210, 300, 190, 170, WHT);
    txtc(235, 370, 3, "ACCEPT", true);
  } else {
    txt(20, 330, 2, "BOOT button:");
    txt(20, 360, 2, "short press = ACCEPT");
    txt(20, 390, 2, "hold about 1 s = REJECT");
  }
}

void drawCountdown(int secs) {
  gfx->fillRect(20, 215, 370, 50, BLK);
  char b[24];
  snprintf(b, sizeof b, "Time left: %d s", secs);
  txt(20, 225, 3, b);
}

void screenResult(int o, bool sent) {
  gfx->fillScreen(BLK);
  const char *t = (o == O_ACCEPTED) ? "ACCEPTED" : (o == O_REJECTED) ? "REJECTED" : (o == O_TIMEOUT) ? "TIMED OUT" : "FILTERED";
  txt(20, 120, 4, t);
  txt(20, 220, 3, sent ? "profile SENT" : "profile NOT sent");
}

// ======================= test 7 core =======================
int handleRequest(uint8_t req, bool &sent) {
  consented = false;
  sent = false;
  if (!isRelevant(me.skills, me.available, req)) {
    Serial.println("  filtered: no notification, nothing sent");
    return O_FILTERED;
  }
  char m[20];
  maskStr(req, m);
  Serial.printf("  NOTIFY: task needs %s (accept/reject on the watch)\n", m);
  waitRelease();
  screenRequest(req);
  uint32_t t0 = millis();
  int lastSec = -1;
  int o = O_TIMEOUT;
  while (millis() - t0 < CONSENT_MS) {
    int sec = (int)((CONSENT_MS - (millis() - t0) + 999) / 1000);
    if (sec != lastSec) { lastSec = sec; drawCountdown(sec); }
    int in = pollInput();
    if (in == IN_ACCEPT) { consented = true; o = O_ACCEPTED; break; }
    if (in == IN_REJECT) { o = O_REJECTED; break; }
    delay(10);
  }
  decideAt = millis();
  sent = shareProfileGate();
  return o;
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);   // never wait for a Serial Monitor
  pinMode(BOOT_BTN, INPUT_PULLUP);
  pinMode(TOUCH_RST, OUTPUT);
  digitalWrite(TOUCH_RST, LOW);
  delay(20);
  digitalWrite(TOUCH_RST, HIGH);
  delay(100);
  Wire.begin(TOUCH_SDA, TOUCH_SCL);
  Wire.setClock(400000);

  bool gfxOk = gfx->begin();
  gfx->setBrightness(255);
  Serial.printf("\n===== TEST 6 + 7 (on-watch): display %s =====\n", gfxOk ? "OK" : "FAILED");

  for (uint8_t a = 0x08; a < 0x78; a++) {   // diagnostic only
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Serial.printf("I2C found 0x%02X\n", a);
  }
  Wire.beginTransmission(TOUCH_ADDR);
  touchOk = (Wire.endTransmission() == 0);
  Serial.printf("touch %s at 0x%02X\n", touchOk ? "FOUND" : "NOT FOUND (BOOT-button mode)", TOUCH_ADDR);

  gfx->fillScreen(BLK);
  txt(20, 40, 4, "TEST 6 + 7");
  txt(20, 120, 2, "Filtering and consent gate");
  txt(20, 160, 2, touchOk ? "Input: touch screen" : "Input: BOOT button");
  txt(20, 200, 2, "Model: 8 skills A..H");
  delay(3000);

  // ---- Test 6 ----
  Serial.println("\n--- TEST 6: filtering table ---");
  int pass6 = 0;
  char failIds[64] = "";
  for (int i = 0; i < N_CASES; i++) {
    const Case &c = CASES[i];
    bool got = isRelevant(c.skills, c.avail, c.req);
    bool ok = (got == c.expect);
    char sm[20], rm[20];
    maskStr(c.skills, sm);
    maskStr(c.req, rm);
    Serial.printf("[%s] case %02d %-5s skills=%-18s avail=%d req=%-18s expect=%d got=%d\n",
                  ok ? "PASS" : "FAIL", c.id, c.criterion, sm, c.avail, rm, c.expect, got);
    if (ok) pass6++;
    else {
      char t[8];
      snprintf(t, sizeof t, "%d ", c.id);
      strncat(failIds, t, sizeof(failIds) - strlen(failIds) - 1);
    }
  }
  bool t6 = (pass6 == N_CASES);
  Serial.printf("TEST 6 RESULT: %d/%d correct -> %s\n", pass6, N_CASES, t6 ? "PASS" : "FAIL");
  gfx->fillScreen(BLK);
  char b[64];
  txt(20, 40, 4, "TEST 6");
  snprintf(b, sizeof b, "%d / %d correct", pass6, N_CASES);
  txt(20, 130, 3, b);
  txt(20, 200, 4, t6 ? "PASS" : "FAIL");
  if (!t6) { txt(20, 290, 2, "failed cases:"); txt(20, 320, 2, failIds); }
  delay(4000);

  // ---- Test 7 ----
  Serial.println("\n--- TEST 7: consent gate (you are the participant) ---");
  gfx->fillScreen(BLK);
  txt(20, 40, 4, "TEST 7");
  txt(20, 120, 2, "You are participant P01");
  txt(20, 150, 2, "skills {A,C}, available");
  txt(20, 200, 2, "Follow the instruction");
  txt(20, 230, 2, "shown before each run.");
  txt(20, 300, 2, "starts in 5 s");
  delay(5000);

  int pass7 = 0;
  for (int k = 0; k < N_RUNS; k++) {
    Serial.printf("[PLAN] run=%d expect=%s\n", k + 1, OUT_NAME[RUNS[k].expect]);
    screenPlan(k + 1, RUNS[k].expect);
    delay(GAP_MS);
    char rm[20];
    maskStr(RUNS[k].req, rm);
    Serial.printf("RUN %d/%d request needs %s\n", k + 1, N_RUNS, rm);
    uint32_t txBefore = txProfile;
    bool sent = false;
    int o = handleRequest(RUNS[k].req, sent);
    uint32_t txDelta = txProfile - txBefore;
    uint32_t wantTx = (RUNS[k].expect == O_ACCEPTED) ? 1 : 0;
    bool ok = (o == RUNS[k].expect) && (txDelta == wantTx);
    observedRun[k] = o;
    okRun[k] = ok;
    Serial.printf("[%s] run %d expected=%s observed=%s tx=%lu (want %lu)\n",
                  ok ? "PASS" : "FAIL", k + 1, OUT_NAME[RUNS[k].expect], OUT_NAME[o],
                  (unsigned long)txDelta, (unsigned long)wantTx);
    if (ok) pass7++;
    screenResult(o, sent);
    delay(1500);
  }

  bool all = t6 && (pass7 == N_RUNS) && violations == 0;
  Serial.println("\n===== SUMMARY (paste into logbook) =====");
  Serial.printf("Input mode: %s\n", touchOk ? "touch screen" : "BOOT button");
  Serial.printf("Test 6: %d/%d cases correct\n", pass6, N_CASES);
  Serial.printf("Test 7: %d/%d runs as expected | profile TX total=%lu | blocked by gate=%lu | violations=%lu\n",
                pass7, N_RUNS, (unsigned long)txProfile, (unsigned long)blocked, (unsigned long)violations);
  Serial.printf("OVERALL: %s\n", all ? "PASS" : "FAIL");

  gfx->fillScreen(BLK);
  txt(20, 10, 3, "RESULT");
  snprintf(b, sizeof b, "Test 6: %d/%d", pass6, N_CASES);
  txt(20, 50, 2, b);
  snprintf(b, sizeof b, "Test 7: %d/%d", pass7, N_RUNS);
  txt(20, 75, 2, b);
  snprintf(b, sizeof b, "sent %lu  blocked %lu  viol %lu", (unsigned long)txProfile, (unsigned long)blocked, (unsigned long)violations);
  txt(20, 100, 2, b);
  for (int k = 0; k < N_RUNS; k++) {
    snprintf(b, sizeof b, "%02d %-7s %s", k + 1, OUT_SHORT[observedRun[k]], okRun[k] ? "ok" : "FAIL");
    txt(k < 6 ? 15 : 215, 140 + (k % 6) * 28, 2, b);
  }
  txt(20, 340, 4, all ? "OVERALL PASS" : "OVERALL FAIL");
  txt(20, 400, 2, touchOk ? "mode: touch" : "mode: BOOT button");
  txt(20, 430, 2, "photograph or copy this screen");
}

void loop() { delay(1000); }
