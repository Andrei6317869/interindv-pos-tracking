/*
  BOARD FIRMWARE for Test 1b, the through-walls survey and the trilateration data (B0, R1, R2)
  Project I (247-519-VA)  |  Boards: ELEGOO ESP32 (classic ESP-WROOM-32). Flash this to all three.

  BEFORE UPLOADING EACH BOARD: set NODE_ID below.   0 = beacon B0,  1 = reference point R1,  2 = R2.

  WHAT IT DOES
    - Starts ESP-NOW and BLE advertising ("N0", "N1", "N2") and prints PASS/FAIL per step (Serial, 115200).
    - Listens for PING broadcasts from the watch. For every new ping it answers with a PONG that says:
        which ping (seq), which round, how many pings of this round it has heard so far, and the RSSI at
        which it heard this ping. Replies are staggered by node id so the three boards do not collide.
    - Blue LED blinks on every ping heard.
    - Serial is a log only. You type nothing. The watch screen shows the results.

  SETTINGS: Board "ESP32 Dev Module", Serial Monitor 115200, esp32 core 3.3.x (same as the watch).
  If the upload stalls on "Connecting...", hold the BOOT button on the board until it starts.
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <BLEDevice.h>
#include <BLEServer.h>

#define NODE_ID 0          // <<< CHANGE PER BOARD: 0, 1 or 2

#ifndef LED_PIN
#define LED_PIN 2
#endif

#define MAGIC 0xA7
#define T_PING 1
#define T_PONG 2

struct __attribute__((packed)) Msg {
  uint8_t magic;
  uint8_t type;     // T_PING (watch -> boards) or T_PONG (board -> watch)
  uint8_t id;       // sender: 9 = watch, 0..2 = board
  uint8_t round;
  uint8_t seq;      // 0..29
  uint8_t heard;    // PONG: pings of this round this board has heard so far
  int8_t  rssi;     // PONG: RSSI (dBm) at which this board heard that ping
};

static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

volatile bool pending = false;
volatile uint8_t pRound = 0, pSeq = 0;
volatile int8_t pRssi = 0;
volatile uint32_t replyAt = 0;
volatile uint8_t curRound = 0, heardCnt = 0;
volatile uint32_t seenMask = 0;
volatile uint32_t pingsTotal = 0;
volatile bool haveRound = false;
volatile uint32_t sendFail = 0;
int failures = 0;
uint32_t lastLog = 0;
uint8_t myMac[6];

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 0)
void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
#else
void onSent(const uint8_t *mac, esp_now_send_status_t status) {
#endif
  if (status != ESP_NOW_SEND_SUCCESS) sendFail++;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  int rssi = info->rx_ctrl->rssi;
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  int rssi = 0;
#endif
  if (len != (int)sizeof(Msg)) return;
  Msg m;
  memcpy(&m, data, sizeof(m));
  if (m.magic != MAGIC || m.type != T_PING || m.seq >= 32) return;
  if (!haveRound || m.round != curRound) {   // new round: start counting again
    haveRound = true;
    curRound = m.round;
    heardCnt = 0;
    seenMask = 0;
  }
  uint32_t bit = 1UL << m.seq;
  if (!(seenMask & bit)) { seenMask |= bit; heardCnt++; }
  pingsTotal++;
  pRound = m.round;
  pSeq = m.seq;
  pRssi = (int8_t)rssi;
  replyAt = millis() + 5 + 12 * NODE_ID;     // stagger: B0 +5 ms, R1 +17 ms, R2 +29 ms
  pending = true;
}

void step(const char *name, bool ok) {
  Serial.printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok) failures++;
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  pinMode(LED_PIN, OUTPUT);
  delay(300);

  Serial.printf("\n===== BOARD NODE %d (%s) =====\n", NODE_ID,
                NODE_ID == 0 ? "beacon B0" : (NODE_ID == 1 ? "reference R1" : "reference R2"));
  Serial.printf("Chip: %s rev %d | cores %d | %u MHz | core %d.%d.%d | IDF %s\n",
                ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(), getCpuFrequencyMhz(),
                ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH,
                ESP.getSdkVersion());
  Serial.printf("Heap free at boot: %u B\n", ESP.getFreeHeap());

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_err_t er = esp_wifi_get_mac(WIFI_IF_STA, myMac);
  Serial.printf("MAC=%02X:%02X:%02X:%02X:%02X:%02X   <- write this on the board's tape\n",
                myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
  step("1 Wi-Fi STA mode + MAC read", er == ESP_OK);
  step("2 esp_now_init", esp_now_init() == ESP_OK);
  step("3 register send/recv callbacks",
       esp_now_register_send_cb(onSent) == ESP_OK && esp_now_register_recv_cb(onRecv) == ESP_OK);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = 0;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  step("4 add broadcast peer", esp_now_add_peer(&peer) == ESP_OK);

  char nm[8];
  snprintf(nm, sizeof nm, "N%d", NODE_ID);
  BLEDevice::init(nm);
  BLEServer *server = BLEDevice::createServer();
  BLEService *svc = server->createService("6e5a1a00-0000-4a1a-9a1a-00000000001b");
  svc->start();
  step("5 BLE init + service", server != nullptr && svc != nullptr);
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID("6e5a1a00-0000-4a1a-9a1a-00000000001b");
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  step("6 BLE advertising started", true);

  Serial.printf("Heap free after start: %u B\n", ESP.getFreeHeap());
  Serial.printf("Step failures: %d  -> %s\n", failures, failures == 0 ? "BOARD OK" : "BOARD PROBLEM");
  Serial.println("Waiting for pings from the watch...");
}

void loop() {
  if (pending && (int32_t)(millis() - replyAt) >= 0) {
    Msg r;
    r.magic = MAGIC;
    r.type = T_PONG;
    r.id = NODE_ID;
    r.round = pRound;
    r.seq = pSeq;
    r.heard = heardCnt;
    r.rssi = pRssi;
    pending = false;
    esp_err_t e = esp_now_send(BCAST, (uint8_t *)&r, sizeof(r));
    if (e != ESP_OK) sendFail++;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
  if (millis() - lastLog >= 5000) {
    lastLog = millis();
    Serial.printf("node %d | round %u: heard %u pings | total pings %lu | send fails %lu\n",
                  NODE_ID, (unsigned)curRound, (unsigned)heardCnt, (unsigned long)pingsTotal,
                  (unsigned long)sendFail);
  }
  delay(1);
}
