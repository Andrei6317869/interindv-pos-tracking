/*
  WATCH FIRMWARE for Tests 4, 5 and 11: request delivery, two or three participants at once, group size and leader
  Project I (247-519-VA) | Waveshare ESP32-S3-Touch-AMOLED-2.06 | works with test4_5_11_board.ino on B0, R1, R2
  Requirements: REQSU-01 (request reaches the watch), REQSU-03 (more than one participant at once),
                REQSU-04 (request/response), REQSU-06 (consent over the radio), REQSU-07 (group size and leader).
  You do NOT need the Serial Monitor: instructions, buttons and results are all on the watch.

  SETUP: power B0, R1 and R2 first (USB, within about 1 m of the watch, BLE is not used). Then start the watch.

  WHAT HAPPENS IN EACH OF THE 10 RUNS
    1. The watch shows what YOU must do (tap ACCEPT, tap REJECT, or nothing), then after 3 s it sends GO to the beacon.
    2. The beacon B0 broadcasts a request (a set of skills). R1 and R2 answer by themselves, as scripted.
    3. If the request is relevant to the watch (profile {A,C}), the alert appears: you do what the screen said earlier.
       The profile is sent to the beacon only after ACCEPT.
    4. After 9.5 s the beacon reports the group: size, leader and members. The watch compares it with the answer
       written in advance in the RUNS table below, and shows ok or FAIL.

  WHAT IS TESTED
    Test 4  : the request reaches the watch in every run, the profile and the report come back over the radio.
    Test 5  : runs 1, 3 and 8 have two or three participants answering in the same run (watch + R1, + R2).
    Test 11 : the beacon's report has the right size, leader and members in every run.

  PASS (written before running): in all 10 runs the request reached the watch, the report arrived, and its size,
  leader and members equal the expected ones; the watch sent its profile exactly in the ACCEPT runs and never
  otherwise (0 violations).

  LIMITS (state in the logbook): R1 and R2 are scripted boards, not watches; skills are abstract; one request at
  a time; ESP-NOW only; 1 m indoors. REQSU-03 is supported only in the sense of one watch plus scripted stand-ins.

  IDE settings as in Tests 6/7. Library: GFX Library for Arduino (Arduino_GFX) v1.6.0.
*/

#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

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
#define BTN_ZONE_Y 280

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_CO5300 *gfx = new Arduino_CO5300(bus, LCD_RESET, 0, LCD_W, LCD_H, 22, 0, 0, 0);

#define MAGIC 0xC7
#define T_GO   1
#define T_REQ  2
#define T_PROF 3
#define T_REP  4
#define ID_W 9
#define NONE 255
#define CONSENT_MS 8000UL
#define GAP_MS 3000UL
#define RUN_WAIT_MS 13000UL

const uint16_t BLK = 0x0000, WHT = 0xFFFF;

struct __attribute__((packed)) Msg {
  uint8_t magic;
  uint8_t type;
  uint8_t run;
  uint8_t id;
  uint8_t arg;       // GO/REQ: request mask. PROF: skills mask
  uint8_t size;      // REP
  uint8_t leader;    // REP: 9 = watch, 1 = R1, 2 = R2, 255 = none
  uint8_t members;   // REP: bit0 = R1, bit1 = R2, bit2 = watch
};
struct RxItem { Msg m; uint8_t src[6]; };
#define QN 8
RxItem rxq[QN];
volatile uint8_t qHead = 0, qTail = 0;
RxItem cur;
static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
volatile uint32_t sendFail = 0, rxDrop = 0;

// ======================= model (same as Tests 6/7) =======================
#define MY_SKILLS 0x05      // watch profile {A,C}, available
bool isRelevant(uint8_t skills, bool available, uint8_t req) { return available && ((skills & req) != 0); }

void maskStr(uint8_t m, char *out) {   // out >= 20 chars
  int n = 0;
  out[n++] = '{';
  bool first = true;
  for (int i = 0; i < 8; i++) {
    if (m & (1 << i)) {
      if (!first) out[n++] = ',';
      out[n++] = 'A' + i;
      first = false;
    }
  }
  out[n++] = '}';
  out[n] = 0;
}

// Plain ints, not enums (Arduino auto-prototypes)
#define O_FILTERED 0
#define O_ACCEPTED 1
#define O_REJECTED 2
#define O_TIMEOUT  3
#define O_NOREQ    4
const char *OUT_NAME[] = {"FILTERED", "ACCEPT", "REJECT", "TIMEOUT", "NO REQUEST"};
const char *OUT_SHORT[] = {"FILT", "ACC", "REJ", "TMO", "NOREQ"};

// RUNS: written by hand BEFORE running, from the rule and the profiles below. NOT computed by the code.
//   W (watch, id 9) {A,C} = 0x05   R1 (id 1) {B,C} = 0x06   R2 (id 2) {D,E} = 0x18   all available
//   skills: A=0x01 B=0x02 C=0x04 D=0x08 E=0x10 F=0x20
//   leader = member covering the most requested skills, tie -> lowest id.   members: bit0 R1, bit1 R2, bit2 W
//   run req   watch   R1     R2     -> expected group                        size leader members
//    1  {C}   ACCEPT  accept filter   W and R1 (each covers 1, tie)           2    R1     0b101
//    2  {A}   ACCEPT  filter filter   W                                       1    W      0b100
//    3  {B,C,D} ACCEPT accept accept  W, R1, R2 (cover 1, 2, 1)               3    R1     0b111
//    4  {D,E} filter  filter accept   R2                                      1    R2     0b010
//    5  {C}   REJECT  accept filter   R1 only (watch said no)                 1    R1     0b001
//    6  {C}   ACCEPT  reject filter   W only                                  1    W      0b100
//    7  {B,C,D} timeout accept reject R1 only                                 1    R1     0b001
//    8  {A,C} ACCEPT  accept filter   W and R1 (cover 2 and 1)                2    W      0b101
//    9  {F}   filter  filter filter   nobody relevant                         0    none   0
//   10  {C}   REJECT  reject filter   nobody accepts                          0    none   0
struct Run { uint8_t req; int act; uint8_t size; uint8_t leader; uint8_t members; };
const Run RUNS[] = {
  {0x04, O_ACCEPTED, 2, 1,    5},
  {0x01, O_ACCEPTED, 1, ID_W, 4},
  {0x0E, O_ACCEPTED, 3, 1,    7},
  {0x18, O_FILTERED, 1, 2,    2},
  {0x04, O_REJECTED, 1, 1,    1},
  {0x04, O_ACCEPTED, 1, ID_W, 4},
  {0x0E, O_TIMEOUT,  1, 1,    1},
  {0x05, O_ACCEPTED, 2, ID_W, 5},
  {0x20, O_FILTERED, 0, NONE, 0},
  {0x04, O_REJECTED, 0, NONE, 0},
};
const int N_RUNS = sizeof(RUNS) / sizeof(RUNS[0]);

// ======================= radio =======================
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 0)
void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
#else
void onSent(const uint8_t *mac, esp_now_send_status_t status) {
#endif
  if (status != ESP_NOW_SEND_SUCCESS) sendFail++;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  const uint8_t *src = info->src_addr;
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  const uint8_t *src = mac;
#endif
  if (len != (int)sizeof(Msg)) return;
  uint8_t next = (uint8_t)((qHead + 1) % QN);
  if (next == qTail) { rxDrop++; return; }
  memcpy(&rxq[qHead].m, data, sizeof(Msg));
  memcpy(rxq[qHead].src, src, 6);
  qHead = next;
}

bool popRx() {
  if (qTail == qHead) return false;
  cur = rxq[qTail];
  qTail = (uint8_t)((qTail + 1) % QN);
  return cur.m.magic == MAGIC;
}

void ensurePeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;
  p.encrypt = false;
  p.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&p);
}

void sendMsg(const uint8_t *to, uint8_t type, uint8_t run, uint8_t id, uint8_t arg) {
  Msg m;
  m.magic = MAGIC; m.type = type; m.run = run; m.id = id; m.arg = arg;
  m.size = 0; m.leader = 0; m.members = 0;
  if (esp_now_send(to, (uint8_t *)&m, sizeof(m)) != ESP_OK) sendFail++;
}

// state of the run in progress (filled by pumpRx)
uint8_t curRun = 0;
bool reqSeen = false, repGot = false;
uint8_t reqMask = 0, repSize = 0, repLeader = NONE, repMembers = 0;
uint8_t reqSrc[6];
uint32_t reqAt = 0, repAt = 0;

void pumpRx() {
  while (popRx()) {
    if (cur.m.run != curRun) continue;
    if (cur.m.type == T_REQ && !reqSeen) {
      reqSeen = true;
      reqMask = cur.m.arg;
      memcpy(reqSrc, cur.src, 6);
      reqAt = millis();
    } else if (cur.m.type == T_REP && !repGot) {
      repGot = true;
      repSize = cur.m.size;
      repLeader = cur.m.leader;
      repMembers = cur.m.members;
      repAt = millis();
    }
  }
}

// ======================= consent gate (as in Test 7, now with a real send) =======================
bool consented = false;
uint32_t decideAt = 0, txProfile = 0, blocked = 0, violations = 0;

bool shareProfileGate(uint8_t run) {
  if (!consented) {
    blocked++;
    Serial.println("  gate: BLOCKED (no consent)");
    return false;
  }
  if (millis() < decideAt) {
    violations++;
    Serial.println("  !! VIOLATION: sent before decision");
  }
  ensurePeer(reqSrc);
  sendMsg(reqSrc, T_PROF, run, ID_W, MY_SKILLS);
  txProfile++;
  char m[20];
  maskStr(MY_SKILLS, m);
  Serial.printf("  TX PROFILE (unicast to the beacon) id=%d skills=%s\n", ID_W, m);
  return true;
}

// ======================= watch input =======================
bool touchOk = false;
bool espOk = false;
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

void waitRelease() {
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

void waitForTap() {   // any touch or BOOT press
  waitRelease();
  while (true) {
    int x, y;
    bool t = touchOk ? readTouch(x, y) : (digitalRead(BOOT_BTN) == LOW);
    if (t) break;
    delay(20);
  }
  waitRelease();
}

// ======================= screens (text kept clear of the rounded corners) =======================
void txtc(int x, int y, uint8_t size, const char *s, bool inverse) {
  gfx->setTextSize(size);
  gfx->setTextColor(inverse ? BLK : WHT, inverse ? WHT : BLK);
  gfx->setCursor(x, y);
  gfx->print(s);
}
void txt(int x, int y, uint8_t size, const char *s) { txtc(x, y, size, s, false); }

void screenPlan(int k, int act) {
  gfx->fillScreen(BLK);
  char b[24];
  snprintf(b, sizeof b, "RUN %d / %d", k, N_RUNS);
  txt(40, 45, 4, b);
  txt(30, 110, 2, "You are the watch W:");
  switch (act) {
    case O_ACCEPTED:
      if (touchOk) { txt(30, 150, 4, "TAP"); txt(30, 195, 4, "ACCEPT"); }
      else         { txt(30, 150, 3, "SHORT-PRESS"); txt(30, 195, 3, "BOOT BUTTON"); }
      txt(30, 270, 2, "when the alert shows");
      break;
    case O_REJECTED:
      if (touchOk) { txt(30, 150, 4, "TAP"); txt(30, 195, 4, "REJECT"); }
      else         { txt(30, 150, 3, "HOLD BOOT"); txt(30, 195, 3, "ABOUT 1 S"); }
      txt(30, 270, 2, "when the alert shows");
      break;
    case O_TIMEOUT:
      txt(30, 150, 4, "DO NOTHING");
      txt(30, 270, 2, "let the alert time out");
      break;
    default:
      txt(30, 150, 4, "DO NOTHING");
      txt(30, 270, 2, "no alert should appear");
      break;
  }
  txt(30, 400, 2, "request goes out in 3 s");
}

void screenRequest(uint8_t req) {
  gfx->fillScreen(BLK);
  char m[20];
  maskStr(req, m);
  txt(30, 45, 3, "TASK REQUEST");
  txt(30, 100, 2, "task needs:");
  txt(30, 130, 3, m);
  txt(30, 185, 2, "Share your profile?");
  if (touchOk) {
    gfx->drawRect(30, 300, 170, 140, WHT);
    gfx->drawRect(31, 301, 168, 138, WHT);
    txt(45, 355, 3, "REJECT");
    gfx->fillRect(210, 300, 170, 140, WHT);
    txtc(220, 355, 3, "ACCEPT", true);
  } else {
    txt(30, 330, 2, "BOOT button:");
    txt(30, 360, 2, "short press = ACCEPT");
    txt(30, 390, 2, "hold about 1 s = REJECT");
  }
}

void drawCountdown(int secs) {
  gfx->fillRect(30, 225, 340, 50, BLK);
  char b[24];
  snprintf(b, sizeof b, "Time left: %d s", secs);
  txt(30, 235, 3, b);
}

void screenWaiting(int k) {
  gfx->fillScreen(BLK);
  char b[24];
  snprintf(b, sizeof b, "RUN %d / %d", k, N_RUNS);
  txt(40, 45, 4, b);
  txt(30, 150, 3, "waiting for the");
  txt(30, 190, 3, "beacon report...");
}

const char *leaderName(uint8_t id) {
  if (id == ID_W) return "W";
  if (id == 1) return "R1";
  if (id == 2) return "R2";
  return "-";
}

void membersStr(uint8_t mm, char *out) {   // out >= 12
  out[0] = 0;
  if (mm & 4) strcat(out, "W ");
  if (mm & 1) strcat(out, "R1 ");
  if (mm & 2) strcat(out, "R2 ");
  if (!out[0]) strcpy(out, "none");
}

void screenRunResult(int k, int o, bool sent, bool ok) {
  gfx->fillScreen(BLK);
  char b[40], mm[12];
  snprintf(b, sizeof b, "RUN %d: %s", k + 1, ok ? "ok" : "FAIL");
  txt(40, 45, 4, b);
  txt(30, 110, 2, OUT_NAME[o]);
  txt(30, 140, 2, sent ? "profile SENT" : "profile NOT sent");
  if (repGot) {
    membersStr(repMembers, mm);
    snprintf(b, sizeof b, "report: size %u", (unsigned)repSize);
    txt(30, 190, 2, b);
    snprintf(b, sizeof b, "leader %s", leaderName(repLeader));
    txt(30, 220, 2, b);
    snprintf(b, sizeof b, "members %s", mm);
    txt(30, 250, 2, b);
  } else {
    txt(30, 190, 2, "no report received");
  }
  if (!ok) {
    const Run &r = RUNS[k];
    membersStr(r.members, mm);
    snprintf(b, sizeof b, "expected %u, %s, %s", (unsigned)r.size, leaderName(r.leader), mm);
    txt(30, 320, 2, b);
  }
}

// ======================= one run =======================
int observed[N_RUNS];
bool sentRun[N_RUNS], okRun[N_RUNS], gotReq[N_RUNS], gotRep[N_RUNS];
uint8_t outSize[N_RUNS], outLeader[N_RUNS], outMembers[N_RUNS];

int handleRequest(uint8_t run, bool &sent) {
  consented = false;
  sent = false;
  char m[20];
  maskStr(reqMask, m);
  if (!isRelevant(MY_SKILLS, true, reqMask)) {
    Serial.printf("  request needs %s: not relevant, no alert, nothing sent\n", m);
    return O_FILTERED;
  }
  Serial.printf("  NOTIFY: request needs %s (accept/reject on the watch)\n", m);
  waitRelease();
  screenRequest(reqMask);
  uint32_t t0 = millis();
  int lastSec = -1;
  int o = O_TIMEOUT;
  while (millis() - t0 < CONSENT_MS) {
    int sec = (int)((CONSENT_MS - (millis() - t0) + 999) / 1000);
    if (sec != lastSec) { lastSec = sec; drawCountdown(sec); }
    pumpRx();                      // keep receiving (the report copies arrive later, but never lose a frame)
    int in = pollInput();
    if (in == IN_ACCEPT) { consented = true; o = O_ACCEPTED; break; }
    if (in == IN_REJECT) { o = O_REJECTED; break; }
    delay(10);
  }
  decideAt = millis();
  sent = shareProfileGate(run);
  return o;
}

void runOne(int k) {
  const Run &r = RUNS[k];
  curRun = (uint8_t)(k + 1);
  reqSeen = false;
  repGot = false;
  repSize = 0; repLeader = NONE; repMembers = 0;
  while (popRx()) {}               // clear anything left from the previous run
  screenPlan(k + 1, r.act);
  delay(GAP_MS);
  waitRelease();

  char rm[20];
  maskStr(r.req, rm);
  Serial.printf("RUN %d/%d request needs %s, expect watch=%s, group size %u leader %s\n", k + 1, N_RUNS, rm,
                OUT_NAME[r.act], (unsigned)r.size, leaderName(r.leader));
  uint32_t goAt = millis();
  for (int c = 0; c < 3; c++) {    // GO is broadcast without acknowledgement, so it is sent 3 times
    sendMsg(BCAST, T_GO, curRun, ID_W, r.req);
    uint32_t t = millis();
    while (millis() - t < 100) { pumpRx(); delay(5); }
  }
  while (!reqSeen && millis() - goAt < 3000) { pumpRx(); delay(5); }

  bool sent = false;
  int o = O_NOREQ;
  if (reqSeen) {
    Serial.printf("  request reached the watch %lu ms after GO\n", (unsigned long)(reqAt - goAt));
    o = handleRequest(curRun, sent);
  } else {
    Serial.println("  !! request did NOT reach the watch");
  }
  screenWaiting(k + 1);
  while (!repGot && millis() - goAt < RUN_WAIT_MS) { pumpRx(); delay(5); }
  if (repGot) Serial.printf("  report arrived %lu ms after GO\n", (unsigned long)(repAt - goAt));
  else Serial.println("  !! no report arrived");

  bool profileOk = (sent == (r.act == O_ACCEPTED));
  bool repOk = repGot && repSize == r.size && repLeader == r.leader && repMembers == r.members;
  bool ok = reqSeen && (o == r.act) && profileOk && repOk;
  observed[k] = o; sentRun[k] = sent; okRun[k] = ok; gotReq[k] = reqSeen; gotRep[k] = repGot;
  outSize[k] = repSize; outLeader[k] = repLeader; outMembers[k] = repMembers;
  Serial.printf("[%s] run %d: watch %s (want %s), profile sent=%d (want %d), report size=%u leader=%s members=%u (want %u, %s, %u)\n",
                ok ? "PASS" : "FAIL", k + 1, OUT_NAME[o], OUT_NAME[r.act], sent, r.act == O_ACCEPTED,
                (unsigned)repSize, leaderName(repLeader), (unsigned)repMembers,
                (unsigned)r.size, leaderName(r.leader), (unsigned)r.members);
  screenRunResult(k, o, sent, ok);
  delay(2000);
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
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
  Wire.beginTransmission(TOUCH_ADDR);
  touchOk = (Wire.endTransmission() == 0);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  espOk = (esp_now_init() == ESP_OK) &&
          (esp_now_register_send_cb(onSent) == ESP_OK) &&
          (esp_now_register_recv_cb(onRecv) == ESP_OK);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = 0;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  espOk = espOk && (esp_now_add_peer(&peer) == ESP_OK);
  Serial.printf("\n===== TESTS 4/5/11 (watch): display %s, touch %s, ESP-NOW %s =====\n",
                gfxOk ? "OK" : "FAILED", touchOk ? "OK" : "none (BOOT button)", espOk ? "OK" : "FAILED");

  gfx->fillScreen(BLK);
  if (!espOk) {
    txt(40, 120, 3, "ESP-NOW FAILED");
    txt(40, 180, 2, "restart the watch");
    return;
  }
  txt(40, 45, 3, "TESTS 4 / 5 / 11");
  txt(30, 100, 2, "Beacon B0 + R1 + R2 must");
  txt(30, 125, 2, "be powered and within 1 m.");
  txt(30, 170, 2, "10 runs. Before each one the");
  txt(30, 195, 2, "screen tells you what to do.");
  txt(30, 240, 2, touchOk ? "Input: touch screen" : "Input: BOOT button");
  gfx->fillRect(30, 310, 350, 70, WHT);
  txtc(100, 330, 3, "TAP TO START", true);
  waitForTap();

  for (int k = 0; k < N_RUNS; k++) runOne(k);

  int pass = 0, nReq = 0, nRep = 0;
  for (int k = 0; k < N_RUNS; k++) {
    if (okRun[k]) pass++;
    if (gotReq[k]) nReq++;
    if (gotRep[k]) nRep++;
  }
  bool all = (pass == N_RUNS) && nReq == N_RUNS && nRep == N_RUNS && violations == 0;
  Serial.println("\n===== SUMMARY (paste into logbook) =====");
  Serial.printf("Input mode: %s\n", touchOk ? "touch screen" : "BOOT button");
  Serial.printf("Runs as expected: %d/%d | request reached watch: %d/%d | report received: %d/%d\n", pass, N_RUNS, nReq, N_RUNS, nRep, N_RUNS);
  Serial.printf("Watch profile TX total=%lu | blocked by gate=%lu | violations=%lu | send fails=%lu | rx dropped=%lu\n",
                (unsigned long)txProfile, (unsigned long)blocked, (unsigned long)violations, (unsigned long)sendFail, (unsigned long)rxDrop);
  for (int k = 0; k < N_RUNS; k++) {
    char mm[12];
    membersStr(outMembers[k], mm);
    Serial.printf("run %02d %s: watch %s, size %u, leader %s, members %s\n", k + 1, okRun[k] ? "ok  " : "FAIL",
                  OUT_NAME[observed[k]], (unsigned)outSize[k], leaderName(outLeader[k]), mm);
  }
  Serial.printf("OVERALL: %s\n", all ? "PASS" : "FAIL");

  gfx->fillScreen(BLK);
  char b[48], mm[12];
  txt(40, 45, 3, "RESULT 4/5/11");
  snprintf(b, sizeof b, "ok %d/%d  req %d  rep %d", pass, N_RUNS, nReq, nRep);
  txt(30, 85, 2, b);
  snprintf(b, sizeof b, "sent %lu  blocked %lu  viol %lu", (unsigned long)txProfile, (unsigned long)blocked, (unsigned long)violations);
  txt(30, 110, 2, b);
  for (int k = 0; k < N_RUNS; k++) {
    membersStr(outMembers[k], mm);
    snprintf(b, sizeof b, "%02d n%u %-2s %s", k + 1, (unsigned)outSize[k], leaderName(outLeader[k]), okRun[k] ? "ok" : "FAIL");
    txt(k < 5 ? 30 : 215, 150 + (k % 5) * 30, 2, b);
  }
  txt(30, 320, 3, all ? "OVERALL PASS" : "OVERALL FAIL");
  txt(30, 370, 2, touchOk ? "mode: touch" : "mode: BOOT button");
  txt(30, 400, 2, "photograph this screen");
}

void loop() { delay(1000); }
