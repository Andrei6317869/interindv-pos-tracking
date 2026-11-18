/*
  BOARD FIRMWARE for Tests 4, 5 and 11 (beacon B0 + stand-in participants R1, R2)
  Project I (247-519-VA)  |  Boards: ELEGOO ESP32 (classic ESP-WROOM-32). Flash this to all three.
  Works with test4_5_11_watch.ino on the watch. BLE is NOT used (L-10, L-11: BLE on the boards costs pings).

  BEFORE UPLOADING EACH BOARD: set NODE_ID below.
      0 = beacon B0 (coordinator)       1 = R1 (stands in for a second watch)       2 = R2 (stands in for a third)

  SELF-CONTAINED MODEL (the same abstract skills as Tests 6, 7 and 10)
    8 skills A..H = bits 0..7. A request needs a set of skills (mask). A participant is RELEVANT when it is
    available AND shares at least one skill with the request.   Profiles:
        watch W  id 9  skills {A,C} = 0x05      R1  id 1  skills {B,C} = 0x06      R2  id 2  skills {D,E} = 0x18

  WHAT EACH BOARD DOES (nothing to type; the watch screen shows the result)
    B0 (beacon):  the watch sends GO(run, request mask). B0 broadcasts the request 3 times (copies 150 ms apart),
                  collects the profiles that come back for 9.5 s, then computes the group and broadcasts the
                  report (size, leader, members) 3 times.
                  GROUP  = the participants whose profile arrived (they accepted).
                  LEADER = the member covering the most requested skills; a tie goes to the lowest id.
    R1, R2:       on a request, if relevant, they answer after a short "thinking" delay according to a scripted
                  policy for that run (A = accept and send profile, R = reject, send nothing). If not relevant
                  they stay silent. The profile goes only to the beacon that asked (unicast), only on accept.
  Serial (115200) is a log only.

  LIMITS (state in the logbook): R1 and R2 are scripted stand-ins, not watches; the watch is the only real
  participant with a human consent step. One request at a time. ESP-NOW only.

  SETTINGS: Board "ESP32 Dev Module", Partition Scheme "Huge APP" (as before), core 3.3.x.
  If the upload stalls: lower Upload Speed to 115200, or hold BOOT on the board while it says "Connecting...".
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define NODE_ID 2          // <<< CHANGE PER BOARD: 0, 1 or 2

#ifndef LED_PIN
#define LED_PIN 2
#endif

#define MAGIC 0xC7
#define T_GO   1           // watch  -> beacon (broadcast): start run, with the request mask
#define T_REQ  2           // beacon -> all    (broadcast): the request
#define T_PROF 3           // participant -> beacon (unicast): profile, sent only after ACCEPT
#define T_REP  4           // beacon -> all    (broadcast): group report for the run
#define ID_W 9
#define NONE 255
#define COLLECT_MS 9500UL
#define COPY_GAP_MS 150UL
#define RUN_SPAN_MS 30000UL

struct __attribute__((packed)) Msg {
  uint8_t magic;
  uint8_t type;
  uint8_t run;       // 1..10
  uint8_t id;        // sender id (PROF), 0 otherwise
  uint8_t arg;       // GO/REQ: request mask. PROF: skills mask
  uint8_t size;      // REP: group size
  uint8_t leader;    // REP: leader id (9, 1, 2) or 255 = none
  uint8_t members;   // REP: bit0 = R1, bit1 = R2, bit2 = watch
};

struct RxItem { Msg m; uint8_t src[6]; };
#define QN 8
RxItem rxq[QN];
volatile uint8_t qHead = 0, qTail = 0;
RxItem cur;                 // item being processed (global: Arduino prototypes cannot use custom types)

static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
volatile uint32_t sendFail = 0, rxDrop = 0;
int failures = 0;
uint32_t lastLog = 0;
uint8_t myMac[6];

// scripted policy of the stand-in participants, one letter per run 1..10 (A = accept, R = reject)
// R1: rejects in runs 6 and 10.  R2: rejects in run 7.  (Where a participant is not relevant it stays silent anyway.)
static const char POLICY_R1[] = "AAAAARAAAR";
static const char POLICY_R2[] = "AAAAAARAAA";

// ---- beacon state ----
bool active = false;
uint8_t sessRun = 0, sessReq = 0;
uint32_t sessStart = 0;
int reqLeft = 0;
uint32_t reqNextAt = 0;
uint8_t gotMask = 0;
uint8_t gotSkills[3];
bool reported = false;
int repLeft = 0;
uint32_t repNextAt = 0;
uint8_t repSize = 0, repLeader = NONE, repMembers = 0;

// ---- participant state ----
uint8_t lastRun = 0;
uint32_t lastReqAt = 0;
bool replyPending = false;
uint32_t replyAt = 0;
uint8_t replyRun = 0;
uint8_t replyTo[6];

uint8_t mySkills() { return NODE_ID == 1 ? 0x06 : 0x18; }

int slotOf(int id) { return id == 1 ? 0 : (id == 2 ? 1 : (id == ID_W ? 2 : -1)); }
int idOfSlot(int s) { return s == 0 ? 1 : (s == 1 ? 2 : ID_W); }
int popc8(uint8_t v) { int n = 0; while (v) { n += v & 1; v >>= 1; } return n; }

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

void sendMsg(const uint8_t *to, uint8_t type, uint8_t run, uint8_t id, uint8_t arg, uint8_t size, uint8_t leader, uint8_t members) {
  Msg m;
  m.magic = MAGIC; m.type = type; m.run = run; m.id = id; m.arg = arg;
  m.size = size; m.leader = leader; m.members = members;
  if (esp_now_send(to, (uint8_t *)&m, sizeof(m)) != ESP_OK) sendFail++;
}

void step(const char *name, bool ok) {
  Serial.printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok) failures++;
}

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

// ======================= beacon =======================
void startSession(uint8_t run, uint8_t req) {
  active = true;
  reported = false;
  sessRun = run;
  sessReq = req;
  sessStart = millis();
  gotMask = 0;
  gotSkills[0] = gotSkills[1] = gotSkills[2] = 0;
  reqLeft = 3;
  reqNextAt = millis();
  repLeft = 0;
  char b[20];
  maskStr(req, b);
  Serial.printf("GO run %u  request needs %s\n", (unsigned)run, b);
}

void finishSession() {
  // group = those whose profile arrived; leader = most requested skills covered, tie -> lowest id
  repSize = 0;
  repMembers = 0;
  repLeader = NONE;
  int best = 0;
  for (int s = 0; s < 3; s++) {                    // slots are in ascending id order (1, 2, 9)
    if (!(gotMask & (1 << s))) continue;
    repSize++;
    repMembers |= (uint8_t)(1 << s);
    int cover = popc8(gotSkills[s] & sessReq);
    if (cover > best) { best = cover; repLeader = (uint8_t)idOfSlot(s); }
  }
  reported = true;
  repLeft = 3;
  repNextAt = millis();
  Serial.printf("REPORT run %u: size %u, leader %d, members mask %u (bit0 R1, bit1 R2, bit2 watch)\n",
                (unsigned)sessRun, (unsigned)repSize, (int)repLeader, (unsigned)repMembers);
}

void beaconRx() {
  if (cur.m.type == T_GO) {
    bool same = active && cur.m.run == sessRun && (millis() - sessStart) < RUN_SPAN_MS;
    if (!same) startSession(cur.m.run, cur.m.arg);   // copies of the same GO are ignored
  } else if (cur.m.type == T_PROF) {
    if (active && !reported && cur.m.run == sessRun) {
      int s = slotOf(cur.m.id);
      if (s >= 0 && !(gotMask & (1 << s))) {
        gotMask |= (uint8_t)(1 << s);
        gotSkills[s] = cur.m.arg;
        char b[20];
        maskStr(cur.m.arg, b);
        Serial.printf("  profile from id %u, skills %s (%lu ms after GO)\n", (unsigned)cur.m.id, b,
                      (unsigned long)(millis() - sessStart));
      }
    }
  }
}

void beaconTick() {
  if (!active) return;
  if (reqLeft > 0 && (int32_t)(millis() - reqNextAt) >= 0) {
    sendMsg(BCAST, T_REQ, sessRun, 0, sessReq, 0, 0, 0);
    reqLeft--;
    reqNextAt = millis() + COPY_GAP_MS;
  }
  if (!reported && (millis() - sessStart) >= COLLECT_MS) finishSession();
  if (reported && repLeft > 0 && (int32_t)(millis() - repNextAt) >= 0) {
    sendMsg(BCAST, T_REP, sessRun, 0, 0, repSize, repLeader, repMembers);
    repLeft--;
    repNextAt = millis() + COPY_GAP_MS;
    if (repLeft == 0) active = false;
  }
}

// ======================= participant =======================
void participantRx() {
  if (cur.m.type != T_REQ) return;
  uint8_t run = cur.m.run;
  if (run == lastRun && (millis() - lastReqAt) < RUN_SPAN_MS) return;   // copy of a request already handled
  lastRun = run;
  lastReqAt = millis();
  uint8_t req = cur.m.arg;
  char b[20];
  maskStr(req, b);
  bool relevant = (mySkills() & req) != 0;                                // available = true for both stand-ins
  if (!relevant) {
    Serial.printf("REQ run %u needs %s: not relevant, silent\n", (unsigned)run, b);
    return;
  }
  char pol = 'A';
  if (run >= 1 && run <= 10) pol = (NODE_ID == 1) ? POLICY_R1[run - 1] : POLICY_R2[run - 1];
  if (pol == 'A') {
    Serial.printf("REQ run %u needs %s: relevant, ACCEPT (profile sent in about %d ms)\n", (unsigned)run, b, 1000 + 700 * NODE_ID);
    replyPending = true;
    replyRun = run;
    replyAt = millis() + 1000 + 700 * NODE_ID;
    memcpy(replyTo, cur.src, 6);
  } else {
    Serial.printf("REQ run %u needs %s: relevant, REJECT (nothing sent)\n", (unsigned)run, b);
  }
}

void participantTick() {
  if (replyPending && (int32_t)(millis() - replyAt) >= 0) {
    replyPending = false;
    ensurePeer(replyTo);
    sendMsg(replyTo, T_PROF, replyRun, (uint8_t)NODE_ID, mySkills(), 0, 0, 0);
    Serial.printf("  profile sent for run %u\n", (unsigned)replyRun);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  pinMode(LED_PIN, OUTPUT);
  delay(300);

  Serial.printf("\n===== TESTS 4/5/11 BOARD NODE %d (%s) =====\n", NODE_ID,
                NODE_ID == 0 ? "beacon B0, coordinator" : (NODE_ID == 1 ? "R1, stand-in participant" : "R2, stand-in participant"));
  Serial.printf("Heap free at boot: %u B\n", ESP.getFreeHeap());

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_err_t er = esp_wifi_get_mac(WIFI_IF_STA, myMac);
  Serial.printf("MAC=%02X:%02X:%02X:%02X:%02X:%02X\n", myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
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
  Serial.println("BLE: not used in this sketch");
  Serial.printf("Heap free after start: %u B\n", ESP.getFreeHeap());
  Serial.printf("Step failures: %d  -> %s\n", failures, failures == 0 ? "BOARD OK" : "BOARD PROBLEM");
  Serial.println(NODE_ID == 0 ? "Waiting for GO from the watch..." : "Waiting for requests from the beacon...");
}

void loop() {
  while (popRx()) {
    if (NODE_ID == 0) beaconRx();
    else participantRx();
  }
  if (NODE_ID == 0) beaconTick();
  else participantTick();
  if (millis() - lastLog >= 10000) {
    lastLog = millis();
    Serial.printf("node %d alive | send fails %lu | rx dropped %lu\n", NODE_ID, (unsigned long)sendFail, (unsigned long)rxDrop);
  }
  delay(1);
}
