// ============================================================================
//  UPF_MCU_Benchmark.ino   --   ONE sketch for BOTH boards:  Teensy 4.1  and  ESP32-S3
// ----------------------------------------------------------------------------
//  Runs the SAME computation as your 8 Python codes (4 codes x FIM off / FIM on) on synthetic
//  data generated on the board, and measures how long every filter step takes, so you can see
//  which update rates (10 / 20 / 50 / 100 / 200 Hz) the MCU can sustain.
//
//   C1 = static A, B moving, UWB 0.01 m, 32 heading particles
//   C2 = static A, B moving, UWB 0.25 m, 128 heading particles
//   C3 = both bots moving,   UWB 0.01 m, 16x16 = 256 particles
//   C4 = both bots moving,   UWB 0.25 m, 16x16 = 256 particles
//   each with and without the FIM monitor (FIM runs every step, window 500, like the Python code)
//
//  What is timed: only the estimator (filter predict+update, and the FIM push+report).
//  What is NOT timed: the on-board simulator (random motion, noise) -- on a real robot this is
//  replaced by your sensor reading / radio code, which you must budget on top.
//
//  HOW TO USE
//   1. Put upf_core.h next to this .ino (same folder, folder name = UPF_MCU_Benchmark).
//   2. Teensy 4.1: Tools > Optimize > "Faster" (or Fastest), CPU 600 MHz (default).
//      ESP32-S3:   Tools > "USB CDC On Boot" = Enabled if you use the native USB port,
//                  CPU frequency 240 MHz, PSRAM setting does not matter (data lives in internal RAM).
//   3. Open the Serial Monitor at 115200 baud, wait ~1-4 minutes. A summary table + CSV lines
//      are printed at the end. Send any character to run it again.
//   4. Copy the whole serial output back to me / into your report.
// ============================================================================
#include <Arduino.h>
#include <stdlib.h>
#ifndef USE_FAST_TRIG
#define USE_FAST_TRIG 0        // 0 = literal port of the Python maths (use this for the main benchmark)
#endif                         // 1 = same maths with fewer sin/cos calls (run it a 2nd time to see the gain)
#if USE_FAST_TRIG
#define UPF_FAST_TRIG
#endif
#include "upf_core.h"

// ------------------------------ user settings ------------------------------
#ifndef COMPUTE_STEPS
#define COMPUTE_STEPS   3000   // steps timed in phase A (3000 = 60 s of simulation at 50 Hz, like your dashboards)
#endif
#ifndef RUN_PHASE_B
#define RUN_PHASE_B     1      // 1 = also run real-time paced checks at every rate (slower, most honest)
#endif
#ifndef RT_SECONDS
#define RT_SECONDS      4      // seconds per rate in phase B
#endif
static const int RATES[5] = {10, 20, 50, 100, 200};
// ---------------------------------------------------------------------------

#if defined(__IMXRT1062__)
  #define BOARD_NAME "Teensy 4.1"
  #define CPU_MHZ ((int)(F_CPU_ACTUAL / 1000000))
#elif defined(ARDUINO_ARCH_ESP32)
  #define BOARD_NAME "ESP32-S3"
  #define CPU_MHZ ((int)getCpuFrequencyMhz())
#else
  #define BOARD_NAME "HOST-MOCK"
  #define CPU_MHZ 0
#endif

// ------------------------------ configurations ------------------------------
struct Config { const char* name; uint8_t mode; uint16_t n1, n2; float uwb; bool fim; };
static const Config CFGS[8] = {
  {"C1  static A  N=32      UWB 0.01   FIM off", 0,  32,  0, 0.01f, false},
  {"C1  static A  N=32      UWB 0.01   FIM ON ", 0,  32,  0, 0.01f, true },
  {"C2  static A  N=128     UWB 0.25   FIM off", 0, 128,  0, 0.25f, false},
  {"C2  static A  N=128     UWB 0.25   FIM ON ", 0, 128,  0, 0.25f, true },
  {"C3  both move 16x16=256 UWB 0.01   FIM off", 1,  16, 16, 0.01f, false},
  {"C3  both move 16x16=256 UWB 0.01   FIM ON ", 1,  16, 16, 0.01f, true },
  {"C4  both move 16x16=256 UWB 0.25   FIM off", 1,  16, 16, 0.25f, false},
  {"C4  both move 16x16=256 UWB 0.25   FIM ON ", 1,  16, 16, 0.25f, true },
};
static const int NCFG = 8;

// ------------------------------ random numbers ------------------------------
struct Rng {
  uint32_t s = 2463534242u; bool have = false; ureal cache = 0;
  uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
  ureal uni() { return ((ureal)(next() >> 8) + (ureal)0.5) / (ureal)16777216.0; }   // (0,1)
  ureal range(ureal a, ureal b) { return a + (b - a) * uni(); }
  ureal gauss(ureal sd) {
    if (have) { have = false; return cache * sd; }
    ureal r = RSQRT((ureal)-2 * RLOG(uni())), t = 2 * UPF_PI * uni();
    cache = r * RSIN(t); have = true; return r * RCOS(t) * sd;
  }
};

// ------------------------------ on-board simulator (same as the dashboards) ------------------------------
struct Sim {
  Rng rng; int mode = 0; uint32_t k = 0;
  ureal ax, ay, ayaw, bx, by, byaw, va, wa, vb, wb, phi;
  ureal ks = 1, sks = 1;                 // dynamics scale for rates other than 50 Hz (ks = 50/rate)
  ureal oa[3], ob[3], uwb;               // outputs of the last step()
  void init(int m, uint32_t seed, ureal rate_hz) {
    mode = m; rng.s = seed * 2654435761u + 12345u; rng.have = false; k = 0;
    ks = (ureal)50 / rate_hz; sks = RSQRT(ks);
    phi = rng.range(-UPF_PI, UPF_PI);
    ax = ay = 0; ayaw = (m == 1) ? rng.range(-UPF_PI, UPF_PI) : 0;
    bx = RCOS(phi); by = RSIN(phi); byaw = rng.range(-UPF_PI, UPF_PI);
    va = (ureal)0.05; wa = (ureal)0.01; vb = (ureal)0.05; wb = (ureal)-0.01;
  }
  static void move(ureal& x, ureal& y, ureal& yaw, ureal v, ureal w) {
    x += RCOS(yaw) * v; y += RSIN(yaw) * v; yaw = wrap_a(yaw + w);
  }
  void step(ureal uwb_sigma, int change_every) {
    k++;
    if (change_every > 0 && (k % (uint32_t)change_every) == 0) {
      va = rng.range((ureal)0.01, (ureal)0.04); wa = rng.range((ureal)-0.02, (ureal)0.02);
      vb = rng.range((ureal)0.01, (ureal)0.04); wb = rng.range((ureal)-0.02, (ureal)0.02);
    }
    if (mode == 1) move(ax, ay, ayaw, va * ks, wa * ks);
    move(bx, by, byaw, vb * ks, wb * ks);
    if (mode == 1) { oa[0] = va * ks + rng.gauss((ureal)0.003 * sks); oa[1] = rng.gauss((ureal)0.002 * sks); oa[2] = wa * ks + rng.gauss((ureal)0.002 * sks); }
    else { oa[0] = oa[1] = oa[2] = 0; }
    ob[0] = vb * ks + rng.gauss((ureal)0.003 * sks); ob[1] = rng.gauss((ureal)0.002 * sks); ob[2] = wb * ks + rng.gauss((ureal)0.002 * sks);
    ureal dx = bx - ax, dy = by - ay;
    uwb = RSQRT(dx * dx + dy * dy) + rng.gauss(uwb_sigma);
  }
  // truth in the frame the estimator reports (static: A frame after rotating the gauge estimate; both: B in A's body frame)
  void error(const ureal est[3], ureal& pos_err, ureal& yaw_err) const {
    if (mode == 0) {
      ureal cp = RCOS(phi), sp = RSIN(phi);
      ureal ex = cp * est[0] - sp * est[1], ey = sp * est[0] + cp * est[1];
      ureal eyaw = wrap_a(est[2] + phi);
      pos_err = RSQRT((bx - ex) * (bx - ex) + (by - ey) * (by - ey)); yaw_err = RFABS(wrap_a(byaw - eyaw));
    } else {
      ureal c = RCOS(ayaw), s = RSIN(ayaw), dx = bx - ax, dy = by - ay;
      ureal tx = c * dx + s * dy, ty = -s * dx + c * dy, tyaw = wrap_a(byaw - ayaw);
      pos_err = RSQRT((tx - est[0]) * (tx - est[0]) + (ty - est[1]) * (ty - est[1])); yaw_err = RFABS(wrap_a(tyaw - est[2]));
    }
  }
};

// ------------------------------ globals (static RAM: no heap, no fragmentation) ------------------------------
static ParticleUKF upf;
static FIMMonitor fimmon;
static Sim sim;
static uint32_t stepUs[COMPUTE_STEPS];

struct Result {
  uint32_t avg_total, p50, p95, p99, mx, mn; uint32_t avg_filter, avg_fim;
  float err_mean, err_last10, head_deg; uint32_t chol_fixes, nan_events;
  uint32_t miss[5]; uint32_t rt_overrun[5]; uint32_t rt_maxlate[5]; uint32_t rt_steps[5];
};
static Result res[8];

static int cmpu(const void* a, const void* b) { uint32_t x = *(const uint32_t*)a, y = *(const uint32_t*)b; return (x > y) - (x < y); }

static void setup_case(const Config& c, ureal rate_hz, uint32_t seed) {
  if (c.mode == 0) upf.configure_static(c.n1, (ureal)c.uwb); else upf.configure_rel(c.n1, c.n2, (ureal)c.uwb);
  upf.chol_fixes = 0; upf.nan_events = 0;
  fimmon.begin((ureal)c.uwb);
  sim.init(c.mode, seed, rate_hz);
}

// one estimator step: returns microseconds spent in filter and in fim
static inline void estimator_step(const Config& c, ureal ksig, uint32_t& tf, uint32_t& tm, ureal est[3]) {
  const ureal sx = (ureal)0.003 * ksig, sy = (ureal)0.002 * ksig, sth = (ureal)0.002 * ksig;
  uint32_t t0 = micros();
  if (c.mode == 0) {
    upf.step_static(sim.ob[0], sim.ob[1], sim.ob[2], sx, sy, sth, sim.uwb);
  } else {
    const ureal s[3] = {sx, sy, sth};
    upf.step_rel(sim.oa, sim.ob, s, s, sim.uwb);
  }
  upf.best_pose(est);
  uint32_t t1 = micros();
  tf = t1 - t0; tm = 0;
  if (c.fim) {
    fimmon.push(sim.oa, sim.ob);
    FIMReport r = fimmon.report(est);
    volatile ureal sink = r.cond + r.sigma_t; (void)sink;      // keep the result "used"
    tm = micros() - t1;
  }
}

// ------------------------------ phase A: pure compute timing (50 Hz dynamics) ------------------------------
static void phaseA(int ci) {
  const Config& c = CFGS[ci]; Result& R = res[ci];
  setup_case(c, 50, 7);
  uint64_t sumTot = 0, sumF = 0, sumM = 0; double errSum = 0, errLast = 0, headSum = 0; int nLast = 0;
  for (int k = 0; k < COMPUTE_STEPS; k++) {
    sim.step((ureal)c.uwb, 100);
    uint32_t tf, tm; ureal est[3];
    estimator_step(c, 1, tf, tm, est);
    stepUs[k] = tf + tm; sumTot += tf + tm; sumF += tf; sumM += tm;
    ureal pe, ye; sim.error(est, pe, ye);
    errSum += pe; headSum += ye;
    if (k >= COMPUTE_STEPS * 9 / 10) { errLast += pe; nLast++; }
    if ((k & 63) == 0) yield();
  }
  for (int r = 0; r < 5; r++) {
    uint32_t period = 1000000UL / RATES[r], m = 0;
    for (int k = 0; k < COMPUTE_STEPS; k++) if (stepUs[k] > period) m++;
    R.miss[r] = m;
  }
  qsort(stepUs, COMPUTE_STEPS, sizeof(uint32_t), cmpu);
  R.avg_total = (uint32_t)(sumTot / COMPUTE_STEPS); R.avg_filter = (uint32_t)(sumF / COMPUTE_STEPS); R.avg_fim = (uint32_t)(sumM / COMPUTE_STEPS);
  R.mn = stepUs[0]; R.p50 = stepUs[COMPUTE_STEPS / 2]; R.p95 = stepUs[COMPUTE_STEPS * 95 / 100]; R.p99 = stepUs[COMPUTE_STEPS * 99 / 100]; R.mx = stepUs[COMPUTE_STEPS - 1];
  R.err_mean = (float)(errSum / COMPUTE_STEPS); R.err_last10 = (float)(errLast / nLast); R.head_deg = (float)(headSum / COMPUTE_STEPS * 57.29578);
  R.chol_fixes = upf.chol_fixes; R.nan_events = upf.nan_events;
}

// ------------------------------ phase B: real-time paced run at each rate ------------------------------
static void phaseB(int ci, int ri) {
  const Config& c = CFGS[ci]; Result& R = res[ci];
  const int rate = RATES[ri]; const uint32_t period = 1000000UL / rate;
  const int steps = rate * RT_SECONDS;
  setup_case(c, (ureal)rate, 11);
  const ureal ks = (ureal)50 / rate, ksig = RSQRT(ks);
  uint32_t overrun = 0, maxlate = 0;
  uint32_t t_next = micros() + 1000;
  for (int k = 0; k < steps; k++) {
    sim.step((ureal)c.uwb, rate * 2);                  // new random velocities every 2 s of simulated time
    while ((int32_t)(micros() - t_next) < 0) { }        // wait for the slot
    uint32_t start = micros();
    int32_t late = (int32_t)(start - t_next); if (late > 0 && (uint32_t)late > maxlate) maxlate = (uint32_t)late;
    uint32_t tf, tm; ureal est[3];
    estimator_step(c, ksig, tf, tm, est);
    if (tf + tm > period) overrun++;
    t_next += period;
  }
  R.rt_overrun[ri] = overrun; R.rt_maxlate[ri] = maxlate; R.rt_steps[ri] = steps;
}

// ------------------------------ reporting ------------------------------
static const char* verdict(const Result& R, int ri) {
  uint32_t period = 1000000UL / RATES[ri];
  if (R.avg_total > period) return "FAIL (avg > period)";
  if (R.miss[ri] > 0) return "MISSES";
  if (R.avg_total * 2 > period) return "OK (tight)";
  return "OK";
}

static void report() {
  Serial.println();
  Serial.println(F("=================== UPF MCU BENCHMARK RESULTS ==================="));
  Serial.printf("board=%s  cpu=%d MHz  real=%s (%d bytes)  fast_trig=%d  steps/config=%d\n", BOARD_NAME, CPU_MHZ,
                sizeof(ureal) == 4 ? "float" : "double", (int)sizeof(ureal), USE_FAST_TRIG, COMPUTE_STEPS);
  Serial.printf("RAM used by estimator objects: ParticleUKF=%u B, FIMMonitor=%u B, timing buffer=%u B\n",
                (unsigned)sizeof(ParticleUKF), (unsigned)sizeof(FIMMonitor), (unsigned)sizeof(stepUs));
  for (int ci = 0; ci < NCFG; ci++) {
    const Result& R = res[ci];
    Serial.printf("\n--- %s ---\n", CFGS[ci].name);
    Serial.printf(" compute per step [us]: avg %lu (filter %lu + fim %lu) | min %lu | p50 %lu | p95 %lu | p99 %lu | max %lu\n",
                  (unsigned long)R.avg_total, (unsigned long)R.avg_filter, (unsigned long)R.avg_fim, (unsigned long)R.mn,
                  (unsigned long)R.p50, (unsigned long)R.p95, (unsigned long)R.p99, (unsigned long)R.mx);
    Serial.printf(" max sustainable rate: %.0f Hz (from avg) | %.0f Hz (from p99) | %.0f Hz (from worst step)\n",
                  1e6 / (double)R.avg_total, 1e6 / (double)R.p99, 1e6 / (double)R.mx);
    Serial.printf(" accuracy of this run (50 Hz dynamics): mean pos err %.3f m | last 10%% %.3f m | heading err %.2f deg | chol_fixes %lu | NaN %lu\n",
                  R.err_mean, R.err_last10, R.head_deg, (unsigned long)R.chol_fixes, (unsigned long)R.nan_events);
    Serial.println(F(" rate[Hz] period[us] util[%]  steps over deadline   real-time check (overruns / max lateness us)   verdict"));
    for (int ri = 0; ri < 5; ri++) {
      uint32_t period = 1000000UL / RATES[ri];
      Serial.printf("  %4d    %7lu    %5.1f   %5lu / %d", RATES[ri], (unsigned long)period, 100.0 * R.avg_total / period, (unsigned long)R.miss[ri], COMPUTE_STEPS);
#if RUN_PHASE_B
      Serial.printf("          %5lu / %5lu  (late max %lu)", (unsigned long)R.rt_overrun[ri], (unsigned long)R.rt_steps[ri], (unsigned long)R.rt_maxlate[ri]);
#endif
      Serial.printf("      %s\n", verdict(R, ri));
    }
  }
  Serial.println(F("\n----- CSV (paste into a spreadsheet) -----"));
  Serial.println(F("CSV,board,cpu_mhz,ureal,fast_trig,config,avg_us,filter_us,fim_us,p50,p95,p99,max,maxrate_avg_hz,maxrate_p99_hz,err_mean_m,err_last10_m,head_deg,chol_fixes,nan,miss10,miss20,miss50,miss100,miss200,rt_over10,rt_over20,rt_over50,rt_over100,rt_over200"));
  for (int ci = 0; ci < NCFG; ci++) {
    const Result& R = res[ci];
    Serial.printf("CSV,%s,%d,%s,%d,\"%s\",%lu,%lu,%lu,%lu,%lu,%lu,%lu,%.0f,%.0f,%.3f,%.3f,%.2f,%lu,%lu",
                  BOARD_NAME, CPU_MHZ, sizeof(ureal) == 4 ? "float" : "double", USE_FAST_TRIG, CFGS[ci].name,
                  (unsigned long)R.avg_total, (unsigned long)R.avg_filter, (unsigned long)R.avg_fim, (unsigned long)R.p50, (unsigned long)R.p95,
                  (unsigned long)R.p99, (unsigned long)R.mx, 1e6 / (double)R.avg_total, 1e6 / (double)R.p99,
                  R.err_mean, R.err_last10, R.head_deg, (unsigned long)R.chol_fixes, (unsigned long)R.nan_events);
    for (int ri = 0; ri < 5; ri++) Serial.printf(",%lu", (unsigned long)R.miss[ri]);
    for (int ri = 0; ri < 5; ri++) Serial.printf(",%lu", (unsigned long)R.rt_overrun[ri]);
    Serial.println();
  }
  Serial.println(F("\nDone. Send any character to run again."));
}

static void run_all() {
  for (int ci = 0; ci < NCFG; ci++) {
    Serial.printf("[phase A] %s ...\n", CFGS[ci].name);
    phaseA(ci);
#if RUN_PHASE_B
    for (int ri = 0; ri < 5; ri++) {
      Serial.printf("[phase B] %s @ %d Hz ...\n", CFGS[ci].name, RATES[ri]);
      phaseB(ci, ri);
    }
#endif
  }
  report();
}

void setup() {
  Serial.begin(115200);
  uint32_t t = millis();
  while (!Serial && (millis() - t) < 4000) { }
  delay(500);
  Serial.println(F("UPF MCU benchmark starting in 2 s ..."));
  delay(2000);
  run_all();
}

void loop() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    run_all();
  }
  delay(50);
}
