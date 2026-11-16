/*
  WATCH FIRMWARE for Test 1b, the through-walls survey and the trilateration data
  Project I (247-519-VA) | Waveshare ESP32-S3-Touch-AMOLED-2.06 | works with node_link_board.ino on B0, R1, R2

  HOW IT WORKS (all on the watch, no Serial Monitor needed)
    Tap RUN: the watch sends 30 numbered ESP-NOW pings, 100 ms apart (about 4 s, keep the watch still).
    Each board that hears a ping answers with a pong that carries how many pings it has heard and the RSSI
    it measured. The screen then shows, for B0, B1 and B2:
        RT     pongs the watch received, of 30 pings sent (round trip = both directions worked)
        heard  pings the board says it received (a lower bound: it is read from the last pong that arrived)
        dBm W  average RSSI at the WATCH of the pongs   (this is the RSSI used for position)
        dBm B  average RSSI at the BOARD of the pings
    Every round is kept; tap HISTORY to see the last 14. Each round number = one place you stood: write
    where you were in your notebook. Serial (115200) repeats each round as a log line, optional.

  USE 1  TEST 1b (desk, about 1 m): PASS (write BEFORE running) = RT >= 27 of 30 for each of B0, R1, R2
         and all board start-up steps PASS. Supports C2 and REQSU-04 (watch <-> beacon exchange).
  USE 2  SURVEY: boards stay put in different rooms; carry the watch between places; one round each.
  USE 3  POSITION (C6): boards at three known points; one round at each known watch spot.

  LIMITS: indoor, walls, one watch; this is not the 30 m test (C3).
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

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_CO5300 *gfx = new Arduino_CO5300(bus, LCD_RESET, 0, LCD_W, LCD_H, 22, 0, 0, 0);

#define MAGIC 0xA7
#define T_PING 1
#define T_PONG 2
#define WATCH_ID 9
#define NB 3
#define NPING 30
#define PING_GAP_MS 100
#define MAXH 14
const uint16_t BLK = 0x0000, WHT = 0xFFFF;

struct __attribute__((packed)) Msg {
  uint8_t magic;
  uint8_t type;
  uint8_t id;
  uint8_t round;
  uint8_t seq;
  uint8_t heard;
  int8_t  rssi;
};

static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// per-round counters (written by the receive callback)
volatile uint8_t curRound = 0;
volatile uint32_t gotMask[NB];
volatile uint8_t gotCnt[NB];
volatile int32_t sumW[NB];       // RSSI at watch
volatile int32_t sumB[NB];       // RSSI at board
volatile uint8_t heardMax[NB];
volatile uint32_t txFail = 0, strayRx = 0;

// history (ring of MAXH rounds)
uint16_t nRounds = 0;
uint8_t hRT[MAXH][NB], hH[MAXH][NB];
int8_t hW[MAXH][NB], hB[MAXH][NB];   // 0 = no data

bool touchOk = false;
bool espOk = false;
uint32_t btnDownAt = 0;

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 0)
void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
#else
void onSent(const uint8_t *mac, esp_now_send_status_t status) {
#endif
  if (status != ESP_NOW_SEND_SUCCESS) txFail++;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  int rssi = info->rx_ctrl->rssi;
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  int rssi = 0;
#endif
  if (len != (int)sizeof(Msg)) { strayRx++; return; }
  Msg m;
  memcpy(&m, data, sizeof(m));
  if (m.magic != MAGIC || m.type != T_PONG || m.id >= NB || m.seq >= 32 || m.round != curRound) { strayRx++; return; }
  uint32_t bit = 1UL << m.seq;
  if (gotMask[m.id] & bit) return;       // duplicate
  gotMask[m.id] |= bit;
  gotCnt[m.id]++;
  sumW[m.id] += rssi;
  sumB[m.id] += m.rssi;
  if (m.heard > heardMax[m.id]) heardMax[m.id] = m.heard;
}

// ======================= touch / button (same as Tests 6 and 7) =======================
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

#define Z_NONE 0
#define Z_RUN 1
#define Z_HIST 2
int pollZone() {
  if (touchOk) {
    int x, y;
    if (readTouch(x, y)) {
      if (y >= 300 && y < 385) return Z_RUN;
      if (y >= 385 && y < 440) return Z_HIST;
    }
    return Z_NONE;
  }
  bool down = (digitalRead(BOOT_BTN) == LOW);
  if (down && btnDownAt == 0) btnDownAt = millis();
  if (!down && btnDownAt != 0) {
    uint32_t d = millis() - btnDownAt;
    btnDownAt = 0;
    if (d >= 40) return (d >= 1000) ? Z_HIST : Z_RUN;
  }
  return Z_NONE;
}

// ======================= screens (margins keep text clear of the rounded corners) =======================
void txt(int x, int y, uint8_t size, const char *s) {
  gfx->setTextSize(size);
  gfx->setTextColor(WHT, BLK);
  gfx->setCursor(x, y);
  gfx->print(s);
}

void txtInv(int x, int y, uint8_t size, const char *s) {
  gfx->setTextSize(size);
  gfx->setTextColor(BLK, WHT);
  gfx->setCursor(x, y);
  gfx->print(s);
}

// one result row of the last stored round (idx = ring index)
void cell(int x, int y, int rt, int rssi, int mode) {
  char b[12];
  if (mode == 0) snprintf(b, sizeof b, "%2d", rt);
  else if (rssi == 0) snprintf(b, sizeof b, "--");
  else snprintf(b, sizeof b, "%d", rssi);
  txt(x, y, 2, b);
}

void screenMain() {
  gfx->fillScreen(BLK);
  txt(40, 45, 3, "NODE LINK TEST");
  char b[40];
  if (nRounds == 0) {
    txt(30, 90, 2, "No round yet. Tap RUN.");
  } else {
    snprintf(b, sizeof b, "Round %u done (%d pings)", (unsigned)nRounds, NPING);
    txt(30, 90, 2, b);
    int i = (nRounds - 1) % MAXH;
    txt(150, 125, 2, "B0");
    txt(230, 125, 2, "B1");
    txt(310, 125, 2, "B2");
    txt(30, 155, 2, "RT/30");
    txt(30, 185, 2, "heard");
    txt(30, 215, 2, "dBm W");
    txt(30, 245, 2, "dBm B");
    for (int k = 0; k < NB; k++) {
      int x = 150 + 80 * k;
      cell(x, 155, hRT[i][k], 0, 0);
      cell(x, 185, hH[i][k], 0, 0);
      cell(x, 215, 0, hW[i][k], 1);
      cell(x, 245, 0, hB[i][k], 1);
    }
    txt(30, 280, 1, "RT: pongs received of 30 pings sent");
    txt(30, 292, 1, "heard: pings the board received (at least)");
  }
  gfx->fillRect(30, 310, 350, 70, WHT);
  snprintf(b, sizeof b, "RUN ROUND %u", (unsigned)(nRounds + 1));
  txtInv(60, 330, 3, b);
  gfx->drawRect(30, 392, 350, 42, WHT);
  txt(120, 403, 2, "HISTORY");
}

void screenRunning(int s) {
  gfx->fillScreen(BLK);
  char b[40];
  snprintf(b, sizeof b, "ROUND %u", (unsigned)(nRounds + 1));
  txt(60, 60, 4, b);
  txt(40, 150, 2, "Keep the watch still");
  snprintf(b, sizeof b, "ping %d / %d", s, NPING);
  txt(40, 200, 3, b);
}

void screenHistory() {
  gfx->fillScreen(BLK);
  txt(40, 45, 3, "HISTORY");
  txt(30, 85, 2, "rd B0     B1     B2");
  int cnt = nRounds < MAXH ? nRounds : MAXH;
  for (int k = 0; k < cnt; k++) {
    int r = nRounds - cnt + 1 + k;
    int i = (r - 1) % MAXH;
    char b[48];
    int n = snprintf(b, sizeof b, "%02d", r);
    for (int j = 0; j < NB; j++) {
      if (hW[i][j] == 0) n += snprintf(b + n, sizeof b - n, " %2d/--", hRT[i][j]);
      else n += snprintf(b + n, sizeof b - n, " %2d/%d", hRT[i][j], hW[i][j]);
    }
    txt(30, 112 + k * 22, 2, b);
  }
  txt(30, 425, 1, "RT/dBm = pongs of 30 / avg signal at watch");
  txt(30, 440, 1, "tap anywhere to go back");
}

// ======================= one round =======================
void runRound() {
  curRound++;
  for (int b = 0; b < NB; b++) { gotMask[b] = 0; gotCnt[b] = 0; sumW[b] = 0; sumB[b] = 0; heardMax[b] = 0; }
  strayRx = 0;
  uint32_t fail0 = txFail;
  screenRunning(0);
  for (int s = 0; s < NPING; s++) {
    Msg m;
    m.magic = MAGIC;
    m.type = T_PING;
    m.id = WATCH_ID;
    m.round = curRound;
    m.seq = (uint8_t)s;
    m.heard = 0;
    m.rssi = 0;
    if (esp_now_send(BCAST, (uint8_t *)&m, sizeof(m)) != ESP_OK) txFail++;
    if (s % 5 == 4) screenRunning(s + 1);
    delay(PING_GAP_MS);
  }
  delay(500);   // let the last pongs arrive

  int i = nRounds % MAXH;
  nRounds++;
  Serial.printf("ROUND %u (%d pings, tx fail %lu, stray rx %lu)\n", (unsigned)nRounds, NPING,
                (unsigned long)(txFail - fail0), (unsigned long)strayRx);
  for (int k = 0; k < NB; k++) {
    hRT[i][k] = gotCnt[k];
    hH[i][k] = heardMax[k];
    hW[i][k] = gotCnt[k] ? (int8_t)((sumW[k] >= 0 ? sumW[k] + gotCnt[k] / 2 : sumW[k] - gotCnt[k] / 2) / gotCnt[k]) : 0;
    hB[i][k] = gotCnt[k] ? (int8_t)((sumB[k] >= 0 ? sumB[k] + gotCnt[k] / 2 : sumB[k] - gotCnt[k] / 2) / gotCnt[k]) : 0;
    Serial.printf("  B%d RT=%2d/%d heard>=%2d dBmW=%d dBmB=%d\n", k, hRT[i][k], NPING, hH[i][k], hW[i][k], hB[i][k]);
  }
  Serial.printf("CSV,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", (unsigned)nRounds,
                hRT[i][0], hW[i][0], hB[i][0], hRT[i][1], hW[i][1], hB[i][1], hRT[i][2], hW[i][2], hB[i][2],
                hH[i][0], hH[i][1], hH[i][2]);
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

  Serial.printf("\n===== NODE LINK TEST (watch): display %s, touch %s, ESP-NOW %s =====\n",
                gfxOk ? "OK" : "FAILED", touchOk ? "OK" : "none (BOOT button)", espOk ? "OK" : "FAILED");
  if (!espOk) {
    gfx->fillScreen(BLK);
    txt(40, 120, 3, "ESP-NOW FAILED");
    txt(40, 180, 2, "restart the watch");
    return;
  }
  screenMain();
}

void loop() {
  if (!espOk) { delay(1000); return; }
  int z = pollZone();
  if (z == Z_RUN) {
    waitRelease();
    runRound();
    screenMain();
    waitRelease();
  } else if (z == Z_HIST) {
    waitRelease();
    screenHistory();
    uint32_t t0 = millis();
    while (millis() - t0 < 120000) {          // any tap goes back (2 min safety)
      int x, y;
      bool t = touchOk ? readTouch(x, y) : (digitalRead(BOOT_BTN) == LOW);
      if (t) break;
      delay(20);
    }
    waitRelease();
    screenMain();
  }
  delay(10);
}
