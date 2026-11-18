/*
  TEST 1a - Can the watch run the candidate links? (ESP-NOW and BLE)
  Project I (247-519-VA)  |  Tests: REQSU-04 (interface), supports R1
  Hardware: Waveshare ESP32-S3-Touch-AMOLED-2.06 (watch only, no second device needed)

  WHAT THIS TESTS
    Steps 1-7 : each stack starts without error (PASS/FAIL printed per step)
    Step 8    : ESP-NOW broadcast + BLE advertising run together for 30 s
  WHAT THIS DOES NOT TEST
    Whether any other device RECEIVES anything. A broadcast "send success"
    only means the radio transmitted. Range and delivery are Test 2.

  PASS CRITERION (write this in your logbook BEFORE running):
    Steps 1-7 all PASS and no send failures in the 30 s soak.

  ARDUINO IDE SETTINGS (check against the Waveshare wiki for this board):
    Board: ESP32S3 Dev Module
    USB CDC On Boot: Enabled      <- needed or Serial prints nothing
    Flash Size: 16MB, PSRAM: OPI PSRAM (if the board has it)
    Partition Scheme: a large-app one (e.g. Huge APP) if "sketch too big"
    Serial Monitor: 115200 baud
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <BLEDevice.h>
#include <BLEServer.h>

#define BLE_NAME      "T1a-watch"
#define BLE_SVC_UUID  "6e5a1a00-0000-4a1a-9a1a-000000000001"
#define SOAK_MS       30000UL
#define SEND_EVERY_MS 1000UL

static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct __attribute__((packed)) Msg {
  uint32_t seq;
  uint32_t t_ms;
};

volatile uint32_t txOk = 0, txFail = 0, rxCount = 0, lastRxSeq = 0;
volatile int lastRxRssi = 0;
volatile bool sendCbSeen = false;

int failures = 0;
uint32_t seqNo = 0, soakStart = 0, lastSend = 0, lastLive = 0;
bool soakDone = false;

// ---- ESP-NOW callbacks. Signatures changed between Arduino-ESP32 versions. ----
// If one of these gives a compile error, tell me your core version
// (Tools > Board > Boards Manager > "esp32") and I will adjust it.
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 0)
void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
#else
void onSent(const uint8_t *mac, esp_now_send_status_t status) {
#endif
  if (status == ESP_NOW_SEND_SUCCESS) txOk++; else txFail++;
  sendCbSeen = true;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  lastRxRssi = info->rx_ctrl->rssi;
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  if (len == (int)sizeof(Msg)) {
    Msg m;
    memcpy(&m, data, sizeof(m));
    lastRxSeq = m.seq;
  }
  rxCount++;
}

// ---- helpers ----
void step(const char *name, bool ok, const char *note = "") {
  Serial.printf("[%s] %s %s\n", ok ? "PASS" : "FAIL", name, note);
  if (!ok) failures++;
}

void heap(const char *label) {
  Serial.printf("  heap @ %-14s free=%u of %u B | PSRAM free=%u of %u B\n",
                label, ESP.getFreeHeap(), ESP.getHeapSize(),
                ESP.getFreePsram(), ESP.getPsramSize());
}

void sendOne() {
  Msg m = {++seqNo, (uint32_t)millis()};
  esp_err_t e = esp_now_send(BCAST, (uint8_t *)&m, sizeof(m));
  if (e != ESP_OK) txFail++;  // call itself failed
}

// ---- setup: steps 0-7 ----
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 5000) delay(10);
  delay(500);

  Serial.println("\n===== TEST 1a: watch link check =====");
  Serial.printf("Chip: %s rev %d | cores: %d | CPU: %u MHz\n",
                ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(), getCpuFrequencyMhz());
  Serial.printf("Arduino-ESP32 core: %d.%d.%d | IDF: %s\n",
                ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR,
                ESP_ARDUINO_VERSION_PATCH, ESP.getSdkVersion());
  Serial.printf("Flash: %u B\n", ESP.getFlashChipSize());
  heap("boot");

  // Step 1: Wi-Fi radio in station mode (needed for ESP-NOW), read MAC
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  uint8_t mac[6] = {0};
  esp_err_t er = esp_wifi_get_mac(WIFI_IF_STA, mac);
  char macStr[24];
  snprintf(macStr, sizeof(macStr), "MAC=%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  step("1 Wi-Fi STA mode + MAC read", er == ESP_OK, macStr);
  heap("after WiFi");

  // Step 2: ESP-NOW init
  step("2 esp_now_init", esp_now_init() == ESP_OK);

  // Step 3: callbacks
  bool cb = (esp_now_register_send_cb(onSent) == ESP_OK) &&
            (esp_now_register_recv_cb(onRecv) == ESP_OK);
  step("3 register send/recv callbacks", cb);

  // Step 4: broadcast peer
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = 0;      // current channel
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  step("4 add broadcast peer", esp_now_add_peer(&peer) == ESP_OK);
  heap("after ESP-NOW");

  // Step 5: one broadcast, wait up to 1 s for the send callback
  sendCbSeen = false;
  Msg packet = {++seqNo, (uint32_t)millis()};
  esp_err_t se = esp_now_send(BCAST, (uint8_t *)&packet, sizeof(Msg));
  uint32_t w = millis();
  while (!sendCbSeen && millis() - w < 1000) delay(5);
  step("5 ESP-NOW broadcast sent + callback fired",
       se == ESP_OK && sendCbSeen && txFail == 0,
       "(send OK only means transmitted, not received)");

  // Step 6: BLE init
  BLEDevice::init(BLE_NAME);
  BLEServer *server = BLEDevice::createServer();
  BLEService *svc = server->createService(BLE_SVC_UUID);
  svc->start();
  Serial.printf("  BLE address: %s\n", BLEDevice::getAddress().toString().c_str());
  step("6 BLE init + service created", server != nullptr && svc != nullptr);
  heap("after BLE");

  // Step 7: BLE advertising
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SVC_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  step("7 BLE advertising started", true,
       "(check with a phone scanner app: look for 'T1a-watch')");

  Serial.printf("\nSteps 1-7 failures so far: %d\n", failures);
  Serial.printf("Starting 30 s soak: ESP-NOW broadcast every %lu ms + BLE advertising...\n",
                SEND_EVERY_MS);
  soakStart = millis();
  lastSend = millis();
}

// ---- loop: step 8 (soak) then idle monitor ----
void loop() {
  uint32_t now = millis();

  if (!soakDone) {
    if (now - lastSend >= SEND_EVERY_MS) {
      lastSend = now;
      sendOne();
    }
    if (now - lastLive >= 5000) {
      lastLive = now;
      Serial.printf("  soak t=%2lus  sent=%u txOk=%u txFail=%u rx=%u\n",
                    (now - soakStart) / 1000, seqNo, txOk, txFail, rxCount);
    }
    if (now - soakStart >= SOAK_MS) {
      soakDone = true;
      delay(200);  // let last callback land
      bool soakOk = (txFail == 0) && (txOk >= seqNo - 1);
      step("8 30 s ESP-NOW + BLE coexistence", soakOk);
      heap("end of soak");
      Serial.println("\n===== SUMMARY (paste into logbook) =====");
      Serial.printf("Messages sent: %u | tx OK: %u | tx fail: %u | received from others: %u\n",
                    seqNo, txOk, txFail, rxCount);
      Serial.printf("Step failures: %d\n", failures);
      Serial.printf("OVERALL: %s\n", (failures == 0) ? "PASS" : "FAIL");
      Serial.println("Note: PASS = stacks run on this watch. It does NOT show another device hears it.");
      Serial.println("========================================");
      Serial.println("Idle monitor: prints received-packet count every 10 s.");
      lastLive = millis();
    }
  } else {
    if (now - lastLive >= 10000) {
      lastLive = now;
      Serial.printf("  idle: rx=%u lastSeq=%u lastRSSI=%d dBm\n", rxCount, lastRxSeq, lastRxRssi);
    }
  }
  delay(2);
}
