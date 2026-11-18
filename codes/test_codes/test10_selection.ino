/*
  TEST 10 - Subgroup selection covers the task skills (REQSU-08, claim C5)
  Project I (247-519-VA)
  Hardware: Waveshare ESP32-S3-Touch-AMOLED-2.06. NO radio, NO second device, NO input from you.
  Just flash it, wait about 10 s, and photograph the result screen.

  QUESTION
    Given the participants who said "yes" (a roster), can a simple selection rule pick a subgroup
    whose skills together cover everything the task needs, and does it say so honestly when no such
    subgroup exists?

  SELF-CONTAINED MODEL (same as Tests 6 and 7, nothing external)
    8 abstract skills A..H, one bit each in a uint8_t (A=0x01 ... H=0x80).
    Participant = { skills mask, available flag }.  Task = { required skills mask }.
    A participant may be picked only if available AND shares at least one required skill.
    A "valid subgroup exists" if the available participants' skills together contain every
    required skill.

  SELECTION RULE UNDER TEST (a candidate rule, not fixed by the proposal): greedy cover.
    Repeatedly pick the available participant who adds the most still-missing required skills
    (lowest index wins ties) until nothing is missing. If a required skill cannot be added, report
    "cannot cover" and pick nobody.

  PART A - 8 hand-written cases. The expected picks were worked out by hand BEFORE writing the code
    and are compared with the code's picks. PASS: 8 of 8.

  PART B - 3 batches x 1000 random trials (fixed seeds 1, 2, 3, so the run is repeatable).
    Roster of 2..10 participants (10 participants plus the beacon = the 11 nodes of the connectivity
    model), each skill held with probability 1/4, 80 % available, task needs 2..4 skills.
    Two independent checks per trial, neither using the selection code:
      - FEASIBLE?  union of available skills contains the task skills.
      - OPTIMAL SIZE: brute-force over all subsets of the roster (at most 1024).
    PASS (write this in your logbook BEFORE running), in every batch:
      * covered >= 90 % of the trials where a valid subgroup exists  (claim C5, REQSU-08)
      * 0 invalid picks (unavailable participant or one sharing no required skill)
      * 0 false "covered" claims in trials where no valid subgroup exists
      * at least 100 feasible and at least 1 infeasible trial (so the checks are really exercised)
    The subgroup SIZE compared with the brute-force minimum is REPORTED, not a pass/fail item.

  LIMITS (state in the logbook): abstract skills, simulated rosters, no radio. This tests the
  selection logic only. Whether the chosen participants can actually be reached is R1 and R2,
  tested with the boards.

  LIBRARY: "GFX Library for Arduino" (Arduino_GFX) v1.6.0. Settings as in Tests 6/7.
  Note: the first line of the Serial output tells you whether the display started; Serial is a log
  only, nothing is typed into it.
*/

#include <Arduino_GFX_Library.h>

#define LCD_CS 12
#define LCD_SCLK 11
#define LCD_SDIO0 4
#define LCD_SDIO1 5
#define LCD_SDIO2 6
#define LCD_SDIO3 7
#define LCD_RESET 8
#define LCD_W 410
#define LCD_H 502

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_CO5300 *gfx = new Arduino_CO5300(bus, LCD_RESET, 0, LCD_W, LCD_H, 22, 0, 0, 0);

#define MAXN 10
#define TRIALS 1000
#define N_BATCH 3
const uint16_t BLK = 0x0000, WHT = 0xFFFF;

// ======================= model (no hardware) =======================
// Plain arrays and ints in every function signature (the Arduino IDE auto-prototypes break on custom types).
// sk[i] = skills of participant i, availMask bit i = participant i available, n = roster size.

int popc(unsigned v) { return __builtin_popcount(v); }

// Independent of the selection code: does a valid subgroup exist at all?
int feasible(const uint8_t *sk, unsigned availMask, int n, uint8_t req) {
  uint8_t u = 0;
  for (int i = 0; i < n; i++) if (availMask & (1u << i)) u |= sk[i];
  return (u & req) == req;
}

// Independent of the selection code: smallest valid subgroup by brute force (0 if none).
int optimalSize(const uint8_t *sk, unsigned availMask, int n, uint8_t req) {
  int best = 0;
  for (unsigned s = 1; s < (1u << n); s++) {
    if ((s & ~availMask) != 0) continue;           // only available participants
    uint8_t u = 0;
    for (int i = 0; i < n; i++) if (s & (1u << i)) u |= sk[i];
    if ((u & req) == req) {
      int c = popc(s);
      if (best == 0 || c < best) best = c;
    }
  }
  return best;
}

// THE RULE UNDER TEST. Returns 1 and sets *chosen (bit i = participant i picked) if the task is covered,
// else returns 0 and *chosen = 0.
int greedySelect(const uint8_t *sk, unsigned availMask, int n, uint8_t req, unsigned *chosen) {
  uint8_t missing = req;
  unsigned pick = 0;
  while (missing) {
    int best = -1, bestGain = 0;
    for (int i = 0; i < n; i++) {
      if (!(availMask & (1u << i)) || (pick & (1u << i))) continue;
      int gain = popc(sk[i] & missing);
      if (gain > bestGain) { bestGain = gain; best = i; }
    }
    if (best < 0) break;                            // nobody adds anything: cannot cover
    pick |= (1u << best);
    missing &= (uint8_t)~sk[best];
  }
  if (missing) { *chosen = 0; return 0; }
  *chosen = pick;
  return 1;
}

// Independent rule check of a result: every picked participant must be available and share a required skill.
int picksValid(const uint8_t *sk, unsigned availMask, int n, uint8_t req, unsigned chosen) {
  for (int i = 0; i < n; i++) {
    if (!(chosen & (1u << i))) continue;
    if (!(availMask & (1u << i))) return 0;
    if ((sk[i] & req) == 0) return 0;
  }
  return 1;
}

// Small repeatable random numbers (xorshift32), so the same seed gives the same trials everywhere.
uint32_t rngState = 1;
uint32_t rnd() {
  rngState ^= rngState << 13;
  rngState ^= rngState >> 17;
  rngState ^= rngState << 5;
  return rngState;
}

// ======================= Part A: hand-written cases =======================
// masks: A=0x01 B=0x02 C=0x04 D=0x08 E=0x10 F=0x20 G=0x40 H=0x80. Participants are P0, P1, ...
// 'avail' bit i = Pi available. 'expect' = picks worked out by hand (bit i = Pi picked), 0 = cannot cover.
struct HCase {
  uint8_t n;
  uint8_t sk[MAXN];
  uint16_t avail;
  uint8_t req;
  uint16_t expect;
  uint8_t optimal;      // hand-derived smallest valid subgroup size, 0 if none
  const char *what;
};

const HCase HAND[] = {
  // 1 two helpers needed, irrelevant one ignored: A from P0, C from P1; P2 has only D
  {3, {0x03, 0x04, 0x08}, 0x07, 0x05, 0x03, 2, "two needed, one ignored"},
  // 2 one participant covers everything: pick only P0
  {3, {0x07, 0x01, 0x02}, 0x07, 0x07, 0x01, 1, "one covers all"},
  // 3 greedy is not minimal: P0{B,C}, P1{A,B}, P2{C,D}; needs A-D. greedy picks P0,P1,P2; best is P1+P2
  {3, {0x06, 0x03, 0x0C}, 0x07, 0x0F, 0x07, 2, "greedy not minimal"},
  // 4 unavailable participant excluded: only P0 has A but is unavailable -> cannot cover
  {2, {0x01, 0x02}, 0x02, 0x03, 0x00, 0, "needed skill unavailable"},
  // 5 nobody has skill C -> cannot cover
  {2, {0x01, 0x02}, 0x03, 0x04, 0x00, 0, "skill nobody has"},
  // 6 duplicate skills: P0 and P1 both A; pick P0 only (lowest index), then P2 for B
  {3, {0x01, 0x01, 0x02}, 0x07, 0x03, 0x05, 2, "duplicate skills"},
  // 7 only participant has a skill the task does not need -> cannot cover
  {1, {0x10}, 0x01, 0x01, 0x00, 0, "no relevant skill"},
  // 8 four helpers cover all eight skills, the empty-skill participant is never picked
  {5, {0x03, 0x0C, 0x30, 0xC0, 0x00}, 0x1F, 0xFF, 0x0F, 4, "all eight skills"},
};
const int N_HAND = sizeof(HAND) / sizeof(HAND[0]);

// ======================= Part B: random batches =======================
struct Batch {
  uint32_t feas, covered, infeas, falseClaims, invalid, sizeEq, sizeSumG, sizeSumO;
  int maxExcess;
};
Batch B[N_BATCH];

void runBatch(int b, uint32_t seed) {
  rngState = seed;
  Batch &x = B[b];
  memset(&x, 0, sizeof(x));
  for (int t = 0; t < TRIALS; t++) {
    int n = 2 + (int)(rnd() % 9);                    // 2..10
    uint8_t sk[MAXN];
    unsigned availMask = 0;
    for (int i = 0; i < n; i++) {
      sk[i] = (uint8_t)(rnd() & rnd());              // each skill held with probability 1/4
      if ((rnd() % 5) != 0) availMask |= (1u << i);  // 80 % available
    }
    int k = 2 + (int)(rnd() % 3);                    // task needs 2..4 distinct skills
    uint8_t req = 0;
    while (popc(req) < k) req |= (uint8_t)(1u << (rnd() % 8));

    int feas = feasible(sk, availMask, n, req);
    unsigned chosen = 0;
    int cov = greedySelect(sk, availMask, n, req, &chosen);

    if (!picksValid(sk, availMask, n, req, chosen)) x.invalid++;
    if (feas) {
      x.feas++;
      if (cov) {
        // covered must really cover (independent re-check)
        uint8_t u = 0;
        for (int i = 0; i < n; i++) if (chosen & (1u << i)) u |= sk[i];
        if ((u & req) == req) {
          x.covered++;
          int g = popc(chosen), o = optimalSize(sk, availMask, n, req);
          x.sizeSumG += g;
          x.sizeSumO += o;
          if (g == o) x.sizeEq++;
          if (g - o > x.maxExcess) x.maxExcess = g - o;
        }
      }
    } else {
      x.infeas++;
      if (cov) x.falseClaims++;
    }
  }
}

// ======================= display =======================
void txtc(int x, int y, uint8_t size, const char *s) {
  gfx->setTextSize(size);
  gfx->setTextColor(WHT, BLK);
  gfx->setCursor(x, y);
  gfx->print(s);
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  bool gfxOk = gfx->begin();
  gfx->setBrightness(255);
  Serial.printf("\n===== TEST 10: selection covers task skills (display %s) =====\n", gfxOk ? "OK" : "FAILED");
  gfx->fillScreen(BLK);
  txtc(20, 40, 4, "TEST 10");
  txtc(20, 120, 2, "Subgroup selection");
  txtc(20, 150, 2, "No input needed.");
  txtc(20, 200, 2, "Running ...");

  // ---- Part A ----
  Serial.println("\n--- PART A: hand-written cases ---");
  int passA = 0;
  for (int i = 0; i < N_HAND; i++) {
    const HCase &h = HAND[i];
    unsigned chosen = 0;
    int cov = greedySelect(h.sk, h.avail, h.n, h.req, &chosen);
    int opt = optimalSize(h.sk, h.avail, h.n, h.req);
    bool ok = (chosen == h.expect) && (cov == (h.expect != 0)) && (opt == h.optimal) &&
              (feasible(h.sk, h.avail, h.n, h.req) == (h.optimal != 0)) &&
              picksValid(h.sk, h.avail, h.n, h.req, chosen);
    Serial.printf("[%s] case %d %-26s picks=0x%02X (hand 0x%02X) covered=%d optimal=%d (hand %d)\n",
                  ok ? "PASS" : "FAIL", i + 1, h.what, chosen, h.expect, cov, opt, h.optimal);
    if (ok) passA++;
  }
  Serial.printf("PART A RESULT: %d/%d\n", passA, N_HAND);

  // ---- Part B ----
  Serial.println("\n--- PART B: random trials vs brute-force checks ---");
  uint32_t t0 = millis();
  bool batchOk[N_BATCH];
  bool allB = true;
  for (int b = 0; b < N_BATCH; b++) {
    runBatch(b, (uint32_t)(b + 1));
    const Batch &x = B[b];
    float covPct = x.feas ? 100.0f * x.covered / x.feas : 0.0f;
    batchOk[b] = (x.feas >= 100) && (x.infeas >= 1) && (covPct >= 90.0f) &&
                 (x.invalid == 0) && (x.falseClaims == 0);
    if (!batchOk[b]) allB = false;
    float mg = x.covered ? (float)x.sizeSumG / x.covered : 0.0f;
    float mo = x.covered ? (float)x.sizeSumO / x.covered : 0.0f;
    float eqPct = x.covered ? 100.0f * x.sizeEq / x.covered : 0.0f;
    Serial.printf("[%s] seed %d: trials=%d feasible=%lu infeasible=%lu covered=%lu (%.1f %%) invalid=%lu falseClaims=%lu\n",
                  batchOk[b] ? "PASS" : "FAIL", b + 1, TRIALS, (unsigned long)x.feas, (unsigned long)x.infeas,
                  (unsigned long)x.covered, covPct, (unsigned long)x.invalid, (unsigned long)x.falseClaims);
    Serial.printf("        size: greedy mean %.2f, minimum mean %.2f, equal in %.1f %% of covered trials, worst excess %d\n",
                  mg, mo, eqPct, x.maxExcess);
  }
  uint32_t ms = millis() - t0;

  bool all = (passA == N_HAND) && allB;
  Serial.println("\n===== SUMMARY (paste into logbook) =====");
  Serial.printf("Part A: %d/%d hand cases\n", passA, N_HAND);
  Serial.printf("Part B: %d x %d random trials, all batches pass: %s | run time %lu ms\n", N_BATCH, TRIALS,
                allB ? "yes" : "NO", (unsigned long)ms);
  Serial.printf("OVERALL: %s\n", all ? "PASS" : "FAIL");

  // ---- result screen ----
  gfx->fillScreen(BLK);
  char b[64];
  txtc(15, 10, 3, "TEST 10");
  snprintf(b, sizeof b, "Part A hand: %d/%d", passA, N_HAND);
  txtc(15, 50, 2, b);
  txtc(15, 85, 2, "seed feas  cov%  bad");
  uint32_t eqC = 0, covC = 0, sg = 0, so = 0;
  int worst = 0;
  for (int i = 0; i < N_BATCH; i++) {
    const Batch &x = B[i];
    float covPct = x.feas ? 100.0f * x.covered / x.feas : 0.0f;
    snprintf(b, sizeof b, " %d   %4lu %5.1f  %lu", i + 1, (unsigned long)x.feas, covPct,
             (unsigned long)(x.invalid + x.falseClaims));
    txtc(15, 112 + i * 28, 2, b);
    eqC += x.sizeEq; covC += x.covered; sg += x.sizeSumG; so += x.sizeSumO;
    if (x.maxExcess > worst) worst = x.maxExcess;
  }
  txtc(15, 205, 2, "bad = invalid + false claims");
  snprintf(b, sizeof b, "infeasible: %lu / %lu / %lu", (unsigned long)B[0].infeas, (unsigned long)B[1].infeas, (unsigned long)B[2].infeas);
  txtc(15, 235, 2, b);
  snprintf(b, sizeof b, "size greedy %.2f min %.2f", covC ? (float)sg / covC : 0.0f, covC ? (float)so / covC : 0.0f);
  txtc(15, 270, 2, b);
  snprintf(b, sizeof b, "equal to min: %.1f %%  worst +%d", covC ? 100.0f * eqC / covC : 0.0f, worst);
  txtc(15, 300, 2, b);
  txtc(15, 350, 4, all ? "OVERALL PASS" : "OVERALL FAIL");
  snprintf(b, sizeof b, "run time %lu ms", (unsigned long)ms);
  txtc(15, 420, 2, b);
  txtc(15, 450, 2, "photograph or copy this screen");
}

void loop() { delay(1000); }
