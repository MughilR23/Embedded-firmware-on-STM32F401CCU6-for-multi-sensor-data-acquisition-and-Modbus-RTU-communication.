// ============================================================================
// water_quality_monitor.ino  —  Industrial Edition v2.0
// Carbelim Sensor Gateway — STM32F103 (Blue Pill)
//
// Water + Air Quality Monitor
// Full fault handling · IWatchdog · Modbus RTU · Non-blocking I/O
//
// Target:    STM32F103C8T6 via STM32duino / Arduino IDE
// Slave ID:  1   Baud: 9600   Parity: None   Stop: 1
//
// MODBUS REGISTER MAP:
//   REG  0  TURB       Turbidity    ×10  (NTU)        0xFFFF=fault  0xFFFE=stale
//   REG  1  PH         pH           ×100              0xFFFF=fault  0xFFFE=stale
//   REG  2  TDS        TDS          ×10  (ppm)        0xFFFF=fault  0xFFFE=stale
//   REG  3  PM1        PM1.0        (µg/m³)           0xFFFF=fault
//   REG  4  PM25       PM2.5        (µg/m³)           0xFFFF=fault
//   REG  5  PM10       PM10         (µg/m³)           0xFFFF=fault
//   REG  6  CO2        CO2          (ppm)             0xFFFF=fault
//   REG  7  VOC        VOC index                      0xFFFF=fault
//   REG  8  TEMP       Temperature  ×10  (°C)         0xFFFF=fault
//   REG  9  HUM        Humidity     (%)               0xFFFF=fault
//   REG 10  CH2O       Formaldehyde ×1000 (mg/m³)     0xFFFF=fault
//   REG 11  CO         CO           ×10  (ppm)        0xFFFF=fault
//   REG 12  O3         Ozone        ×100 (ppm)        0xFFFF=fault
//   REG 13  NO2        NO2          ×100 (ppm)        0xFFFF=fault
//   REG 14  STATUS     Bitmask — see STATUS_* constants
//   REG 15  UPTIME_LO  Uptime seconds — low  16 bits
//   REG 16  UPTIME_HI  Uptime seconds — high 16 bits
//   REG 17  DATA_AGE   Consecutive faulted cycles (alarm threshold: 30)
//   REG 18  FAULT_CODE Bitmask — see FAULT_* constants
//
// STATUS register bits (REG 14):
//   bit 0 = Air sensor OK       bit 1 = pH OK
//   bit 2 = TDS OK              bit 3 = Turbidity OK
//   bit 4 = ADC rail OK         bit 7 = Device running
//   Master MUST read STATUS before trusting any data register.
//
// LED BLINK CODES (PC13, active-low):
//   Startup  — 2 slow (300 ms) : self-test passed, all sensors healthy
//   Startup  — 3 fast (100 ms) : self-test failed, one or more faults
//   Running  — 1 Hz blink      : normal operation
//   Degraded — 4 Hz blink      : active fault, check REG_FAULT_CODE
//
// ============================================================================

#include <ModbusRTU.h>
#include <IWatchdog.h>        // STM32 Independent Watchdog

// ============================================================================
// SECTION 1 — CONFIGURATION
// All tuneable parameters centralised here. No magic numbers elsewhere.
// ============================================================================
namespace Cfg {

  // ── Serial ────────────────────────────────────────────────────────────────
  constexpr uint32_t ZPHS_BAUD       = 9600;
  constexpr uint32_t RS485_BAUD      = 9600;
  constexpr uint8_t  MODBUS_SLAVE_ID = 1;

  // ── Timing (ms) ───────────────────────────────────────────────────────────
  constexpr uint32_t READ_INTERVAL_MS  = 2000UL;   // sensor poll period
  constexpr uint32_t ZPHS_TIMEOUT_MS   = 1000UL;   // per-request timeout
  constexpr uint32_t ZPHS_BACKOFF_MS   = 5000UL;   // wait after 3 timeouts
  constexpr uint32_t STARTUP_SETTLE_MS = 300UL;    // analog rail stabilise
  constexpr uint32_t SELFTEST_ZPHS_MS  = 2000UL;   // ZPHS startup probe

  // ── Blink periods (ms) ────────────────────────────────────────────────────
  constexpr uint32_t BLINK_NORMAL_MS  = 500UL;     // 1 Hz  — healthy
  constexpr uint32_t BLINK_FAULT_MS   = 125UL;     // 4 Hz  — degraded

  // ── Watchdog ──────────────────────────────────────────────────────────────
  constexpr uint32_t WDT_TIMEOUT_US   = 4000000UL; // 4 s (2× read interval)

  // ── ADC ───────────────────────────────────────────────────────────────────
  constexpr uint8_t  ADC_SAMPLES      = 9;          // must be odd (median filter)
  constexpr uint16_t ADC_RAIL_MIN     = 10;         // ≤ this counts as GND short
  constexpr uint16_t ADC_RAIL_MAX     = 4085;       // ≥ this counts as open circuit
  constexpr float    ADC_VREF         = 3.3f;
  constexpr float    ADC_COUNTS_MAX   = 4095.0f;  // renamed: STM32 core defines ADC_RESOLUTION

  // ── IIR filter alphas (0 < α < 1; smaller α = more smoothing) ────────────
  constexpr float    IIR_TDS_ALPHA    = 0.30f;
  constexpr float    IIR_TURB_ALPHA   = 0.30f;

  // ── Stuck-value detection ─────────────────────────────────────────────────
  constexpr float    STUCK_DELTA      = 0.02f;      // minimum meaningful change
  constexpr uint8_t  STUCK_LIMIT      = 10;         // consecutive counts → fault

  // ── ZPHS retry ────────────────────────────────────────────────────────────
  constexpr uint8_t  ZPHS_MAX_RETRIES = 3;

  // ── Recovery ──────────────────────────────────────────────────────────────
  constexpr uint8_t  RECOVERY_CYCLES  = 3;          // clean cycles before RUNNING

  // ── Sensor valid ranges ───────────────────────────────────────────────────
  constexpr float    PH_MIN           =  0.0f;
  constexpr float    PH_MAX           = 14.0f;
  constexpr float    TDS_MAX_PPM      = 2000.0f;
  constexpr float    TURB_MAX_NTU     = 3000.0f;

  // ── Calibration — pH: pH = SLOPE × V + INTERCEPT ─────────────────────────
  constexpr float    PH_SLOPE         =  15.0f;
  constexpr float    PH_INTERCEPT     = -11.6f;     // net sign: pH = 15V − 11.6

  // ── Calibration — Turbidity ───────────────────────────────────────────────
  constexpr float    TURB_VCLEAN      = 1.87f;      // voltage at 0 NTU (clean water)
  constexpr float    TURB_SLOPE       = 1796.0f;    // NTU/V

  // ── Calibration — TDS ─────────────────────────────────────────────────────
  constexpr float    TDS_OFFSET_V     = 0.65f;
  constexpr float    TDS_SCALE        = 700.0f;     // ppm/V after offset

  // ── Modbus sentinels ──────────────────────────────────────────────────────
  constexpr uint16_t MODBUS_FAULT_VAL = 0xFFFFu;   // sensor fault
  constexpr uint16_t MODBUS_STALE_VAL = 0xFFFEu;   // data not yet available

} // namespace Cfg

// ============================================================================
// SECTION 2 — PIN DEFINITIONS
// ============================================================================
namespace Pin {
  constexpr uint8_t LED        = PC13;
  constexpr uint8_t RS485_CTRL = PA8;    // DE/RE direction control
  constexpr uint8_t PH         = PA0;
  constexpr uint8_t TDS        = PA1;
  constexpr uint8_t TURB       = PA4;
} // namespace Pin

// ============================================================================
// SECTION 3 — MODBUS REGISTER MAP
// REG_COUNT is automatically correct — always add new registers before it.
// ============================================================================
enum ModbusReg : uint16_t {
  REG_TURB      =  0,
  REG_PH        =  1,
  REG_TDS       =  2,
  REG_PM1       =  3,
  REG_PM25      =  4,
  REG_PM10      =  5,
  REG_CO2       =  6,
  REG_VOC       =  7,
  REG_TEMP      =  8,
  REG_HUM       =  9,
  REG_CH2O      = 10,
  REG_CO        = 11,
  REG_O3        = 12,
  REG_NO2       = 13,
  REG_STATUS    = 14,
  REG_UPTIME_LO = 15,
  REG_UPTIME_HI = 16,
  REG_DATA_AGE  = 17,
  REG_FAULT_CODE= 18,
  REG_COUNT           // sentinel — keep last
};

// STATUS register bitmasks
constexpr uint16_t STATUS_AIR_OK  = (1u << 0);
constexpr uint16_t STATUS_PH_OK   = (1u << 1);
constexpr uint16_t STATUS_TDS_OK  = (1u << 2);
constexpr uint16_t STATUS_TURB_OK = (1u << 3);
constexpr uint16_t STATUS_ADC_OK  = (1u << 4);
constexpr uint16_t STATUS_RUNNING = (1u << 7);

// FAULT_CODE bitmasks (REG_FAULT_CODE)
constexpr uint16_t FAULT_PH       = (1u << 0);
constexpr uint16_t FAULT_TDS      = (1u << 1);
constexpr uint16_t FAULT_TURB     = (1u << 2);
constexpr uint16_t FAULT_AIR      = (1u << 3);
constexpr uint16_t FAULT_ADC      = (1u << 4);

// ============================================================================
// SECTION 4 — DATA STRUCTURES
// ============================================================================

// ── Fault flags (1 byte bitfield) ────────────────────────────────────────────
struct FaultFlags {
  uint8_t ph    : 1;
  uint8_t tds   : 1;
  uint8_t turb  : 1;
  uint8_t air   : 1;
  uint8_t adc   : 1;
  uint8_t _rsvd : 3;   // reserved for future fault sources

  inline bool    any()  const { return (ph | tds | turb | air | adc) != 0; }
  inline void    clear()      { *reinterpret_cast<uint8_t*>(this) = 0; }

  inline uint16_t toFaultCode() const {
    uint16_t fc = 0;
    if (ph)   fc |= FAULT_PH;
    if (tds)  fc |= FAULT_TDS;
    if (turb) fc |= FAULT_TURB;
    if (air)  fc |= FAULT_AIR;
    if (adc)  fc |= FAULT_ADC;
    return fc;
  }
};

// ── Device state machine ─────────────────────────────────────────────────────
enum class DeviceState : uint8_t {
  INIT     = 0,   // hardware initialising (pre-setup)
  WARMUP   = 1,   // self-test in progress, no valid data yet
  RUNNING  = 2,   // all sensors healthy
  DEGRADED = 3,   // one or more sensors faulted, partial data valid
  RECOVERY = 4,   // fault cleared, verifying stability
};

// ── ZPHS protocol state machine ───────────────────────────────────────────────
enum class ZphsState : uint8_t {
  IDLE    = 0,   // idle, waiting for next poll trigger
  WAITING = 1,   // command sent, awaiting response bytes
  FAULT   = 2,   // repeated failures; honouring backoff timer
};

// ── ZPHS context ─────────────────────────────────────────────────────────────
struct ZphsCtx {
  ZphsState     state       = ZphsState::IDLE;
  uint8_t       buf[26]     = {};
  uint8_t       idx         = 0;
  uint8_t       failCount   = 0;
  uint8_t       cksumFails  = 0;     // tracked separately for diagnostics
  unsigned long sentAt      = 0;
  unsigned long backoffAt   = 0;

  inline bool isTimedOut()        const { return (millis() - sentAt)    >= Cfg::ZPHS_TIMEOUT_MS;  }
  inline bool isBackoffExpired()  const { return (millis() - backoffAt) >= Cfg::ZPHS_BACKOFF_MS;  }
};

// ── Sensor data ───────────────────────────────────────────────────────────────
struct SensorData {
  // Water quality (analog sensors)
  float    ph;
  float    tds;
  float    turb;
  // Air quality (ZPHS01B digital sensor)
  float    temp;
  float    hum;
  uint16_t pm1;
  uint16_t pm25;
  uint16_t pm10;
  uint16_t co2;
  uint8_t  voc;
  float    ch2o;
  float    co;
  float    o3;
  float    no2;
  bool     air_valid;   // true only after first successful ZPHS frame
};

// ── IIR filter state ─────────────────────────────────────────────────────────
struct FilterState {
  float tds  = 0.0f;
  float turb = 0.0f;
};

// ── Stuck-value tracker (per sensor) ─────────────────────────────────────────
struct StuckTracker {
  float   lastVal = -1.0f;
  uint8_t count   = 0;

  /**
   * Call once per reading. Returns true if stuck for STUCK_LIMIT cycles.
   * Auto-resets when the value moves again — self-recovering.
   */
  bool update(float newVal) {
    if (fabsf(newVal - lastVal) < Cfg::STUCK_DELTA) {
      if (++count >= Cfg::STUCK_LIMIT) return true;
    } else {
      count   = 0;
      lastVal = newVal;
    }
    return false;
  }
};

// ============================================================================
// SECTION 5 — GLOBALS
// ============================================================================
HardwareSerial ZphsSerial (PB7, PB6);   // ZPHS01B sensor UART (RX, TX)
HardwareSerial rs485Serial(PA3, PA2);   // RS485 Modbus UART  (RX, TX)

ModbusRTU    mb;
FaultFlags   fault    = {};
DeviceState  devState = DeviceState::INIT;
SensorData   sensor   = {};
ZphsCtx      zphs     = {};
FilterState  filt     = {};

StuckTracker phStuck, tdsStuck, turbStuck;

unsigned long lastReadMs  = 0;
unsigned long lastBlinkMs = 0;
uint16_t      staleCycles = 0;

static uint8_t recoveryGoodCount = 0;

// ZPHS query command (constant, stored once)
static const uint8_t ZPHS_CMD[9] = { 0xFF, 0x01, 0x86, 0, 0, 0, 0, 0, 0x79 };

// ============================================================================
// SECTION 6 — UTILITY
// ============================================================================

/**
 * @brief Blocking blink sequence — for startup signalling only.
 * @param n        Number of blinks
 * @param periodMs On/off period in ms
 */
static void blinkCode(uint8_t n, uint16_t periodMs) {
  for (uint8_t i = 0; i < n; i++) {
    digitalWrite(Pin::LED, LOW);  delay(periodMs);
    digitalWrite(Pin::LED, HIGH); delay(periodMs);
  }
  delay(600);
}

/**
 * @brief In-place insertion sort for uint16_t arrays.
 *        Optimal for N ≤ 9 (avoids heap overhead of qsort).
 */
static void insertionSort(uint16_t* arr, uint8_t n) {
  for (uint8_t i = 1; i < n; i++) {
    const uint16_t key = arr[i];
    int8_t j = static_cast<int8_t>(i) - 1;
    while (j >= 0 && arr[j] > key) { arr[j + 1] = arr[j]; j--; }
    arr[j + 1] = key;
  }
}

/**
 * @brief Parse a big-endian uint16_t from two bytes.
 */
static inline uint16_t be16(const uint8_t* p) {
  return (static_cast<uint16_t>(p[0]) << 8) | p[1];
}

// ============================================================================
// SECTION 7 — ADC: MEDIAN FILTER + RAIL SANITY CHECK
// ============================================================================

/**
 * @brief Sample analog pin, apply median filter, check for rail faults.
 *
 * Takes ADC_SAMPLES readings (odd), sorts, returns median as voltage.
 * Sets fault.adc and returns -1.0f if signal is railed (open / short).
 *
 * Using median (vs trimmed mean) gives:
 *  • True outlier rejection (single-spike immunity)
 *  • Guaranteed worst-case of (N/2) outliers handled
 *  • Constant O(N log N) time — no branching on data content
 *
 * @param  pin  Analog pin identifier
 * @return Measured voltage in [0, 3.3] V, or -1.0f on ADC fault.
 */
static float readVoltage(uint8_t pin) {
  uint16_t s[Cfg::ADC_SAMPLES];
  for (uint8_t i = 0; i < Cfg::ADC_SAMPLES; i++) {
    s[i] = static_cast<uint16_t>(analogRead(pin));
  }
  insertionSort(s, Cfg::ADC_SAMPLES);
  const uint16_t median = s[Cfg::ADC_SAMPLES / 2];

  if (median <= Cfg::ADC_RAIL_MIN || median >= Cfg::ADC_RAIL_MAX) {
    fault.adc = 1;
    return -1.0f;
  }
  fault.adc = 0;
  return static_cast<float>(median) * (Cfg::ADC_VREF / Cfg::ADC_COUNTS_MAX);
}

// ============================================================================
// SECTION 8 — SENSOR CALCULATIONS
// Each function returns -1.0f and sets the relevant fault bit on any error.
// On success, the fault bit is explicitly cleared to enable auto-recovery.
// ============================================================================

/**
 * @brief Convert voltage to pH. Linear model: pH = SLOPE × V + INTERCEPT.
 */
static float calcPH(float v) {
  if (v < 0.0f) { fault.ph = 1; return -1.0f; }
  const float ph = Cfg::PH_SLOPE * v + Cfg::PH_INTERCEPT;
  if (ph < Cfg::PH_MIN || ph > Cfg::PH_MAX) { fault.ph = 1; return -1.0f; }
  fault.ph = 0;
  return ph;
}

/**
 * @brief Convert voltage to TDS (ppm). Applies zero-offset, range-clamp, scale.
 *        Returns 0.0f (valid) for clean water (voltage below offset threshold).
 */
static float calcTDS(float v) {
  if (v < 0.0f) { fault.tds = 1; return -1.0f; }
  const float adj = v - Cfg::TDS_OFFSET_V;
  if (adj <= 0.0f)  { fault.tds = 0; return 0.0f; }         // clean water, valid zero
  const float clamped = (adj > 1.8f) ? 1.8f : adj;          // clamp sensor non-linearity
  const float tds = clamped * Cfg::TDS_SCALE;
  if (tds > Cfg::TDS_MAX_PPM) { fault.tds = 1; return -1.0f; }
  fault.tds = 0;
  return tds;
}

/**
 * @brief Convert voltage to turbidity (NTU). Voltage ≥ TURB_VCLEAN → 0 NTU.
 */
static float calcTurbidity(float v) {
  if (v < 0.0f)               { fault.turb = 1; return -1.0f; }
  if (v >= Cfg::TURB_VCLEAN)  { fault.turb = 0; return 0.0f; }  // clean water
  const float ntu = (Cfg::TURB_VCLEAN - v) * Cfg::TURB_SLOPE;
  if (ntu > Cfg::TURB_MAX_NTU){ fault.turb = 1; return -1.0f; }
  fault.turb = 0;
  return ntu;
}

// ============================================================================
// SECTION 9 — WATER SENSOR READ CYCLE
// Pipeline: readVoltage → calcXxx → stuckDetect → IIR filter → sensor struct
// On fault: fault flag set, sensor struct holds last known-good value.
// ============================================================================
static void readWaterSensors() {
  float raw;

  // ── pH (no IIR — fast response required for process control) ────────────
  raw = calcPH(readVoltage(Pin::PH));
  if (raw >= 0.0f) {
    if (phStuck.update(raw)) fault.ph = 1;
    else                      sensor.ph = raw;
  }

  // ── TDS ──────────────────────────────────────────────────────────────────
  raw = calcTDS(readVoltage(Pin::TDS));
  if (raw >= 0.0f) {
    if (tdsStuck.update(raw)) {
      fault.tds = 1;
    } else {
      filt.tds   = (1.0f - Cfg::IIR_TDS_ALPHA) * filt.tds + Cfg::IIR_TDS_ALPHA * raw;
      sensor.tds = filt.tds;
    }
  }

  // ── Turbidity ────────────────────────────────────────────────────────────
  raw = calcTurbidity(readVoltage(Pin::TURB));
  if (raw >= 0.0f) {
    if (turbStuck.update(raw)) {
      fault.turb = 1;
    } else {
      filt.turb   = (1.0f - Cfg::IIR_TURB_ALPHA) * filt.turb + Cfg::IIR_TURB_ALPHA * raw;
      sensor.turb = filt.turb;
    }
  }
}

// ============================================================================
// SECTION 10 — ZPHS01B AIR SENSOR (NON-BLOCKING)
// ============================================================================

/**
 * @brief Send measurement query to ZPHS01B.
 *
 * Respects fault/backoff state — will not hammer a dead sensor.
 * Flushes RX before sending to eliminate stale bytes.
 */
static void zphs_sendCmd() {
  if (zphs.state == ZphsState::FAULT) {
    if (!zphs.isBackoffExpired()) return;   // still in backoff
    zphs.state     = ZphsState::IDLE;
    zphs.failCount = 0;
  }

  while (ZphsSerial.available()) ZphsSerial.read();   // flush stale RX

  zphs.idx    = 0;
  zphs.sentAt = millis();
  zphs.state  = ZphsState::WAITING;
  ZphsSerial.write(ZPHS_CMD, sizeof(ZPHS_CMD));
}

/**
 * @brief Non-blocking ZPHS response processor. Must be called every loop().
 *
 * Handles all error paths internally:
 *  • Timeout       → increments failCount; enters FAULT+backoff at threshold
 *  • Bad start byte→ discards byte (re-sync on 0xFF)
 *  • Bad checksum  → discards frame, increments cksumFails
 *  • Valid frame   → parses payload, clears all fault state
 */
static void zphs_process() {
  if (zphs.state != ZphsState::WAITING) return;

  // ── Handle timeout ────────────────────────────────────────────────────────
  if (zphs.isTimedOut()) {
    zphs.idx   = 0;
    zphs.state = ZphsState::IDLE;
    if (++zphs.failCount >= Cfg::ZPHS_MAX_RETRIES) {
      fault.air        = 1;
      sensor.air_valid = false;
      zphs.state       = ZphsState::FAULT;
      zphs.backoffAt   = millis();
    }
    return;
  }

  // ── Receive bytes ─────────────────────────────────────────────────────────
  while (ZphsSerial.available()) {
    const uint8_t b = static_cast<uint8_t>(ZphsSerial.read());

    if (zphs.idx == 0 && b != 0xFF) continue;   // wait for frame-start byte
    zphs.buf[zphs.idx++] = b;
    if (zphs.idx < 26) continue;                // accumulate complete frame
    zphs.idx = 0;

    // Validate command echo (byte 1 must be 0x86)
    if (zphs.buf[1] != 0x86) { zphs.cksumFails++; return; }

    // Validate checksum: (~sum(bytes 1–24)) + 1 == byte 25
    uint8_t sum = 0;
    for (uint8_t j = 1; j <= 24; j++) sum += zphs.buf[j];
    if (static_cast<uint8_t>((~sum) + 1) != zphs.buf[25]) {
      zphs.cksumFails++;
      return;   // corrupt frame — discard
    }

    // ── Valid frame — parse big-endian payload ─────────────────────────────
    sensor.pm1   = be16(&zphs.buf[2]);
    sensor.pm25  = be16(&zphs.buf[4]);
    sensor.pm10  = be16(&zphs.buf[6]);
    sensor.co2   = be16(&zphs.buf[8]);
    sensor.voc   = zphs.buf[10];
    sensor.temp  = static_cast<float>(be16(&zphs.buf[11]) - 500u) * 0.1f;
    sensor.hum   = static_cast<float>(be16(&zphs.buf[13]));
    sensor.ch2o  = static_cast<float>(be16(&zphs.buf[15])) * 0.001f;
    sensor.co    = static_cast<float>(be16(&zphs.buf[17])) * 0.1f;
    sensor.o3    = static_cast<float>(be16(&zphs.buf[19])) * 0.01f;
    sensor.no2   = static_cast<float>(be16(&zphs.buf[21])) * 0.01f;

    // Clear all ZPHS fault state on successful frame
    sensor.air_valid = true;
    fault.air        = 0;
    zphs.failCount   = 0;
    zphs.cksumFails  = 0;
    zphs.state       = ZphsState::IDLE;
  }
}

// ============================================================================
// SECTION 11 — MODBUS REGISTER UPDATE
// ============================================================================

/**
 * @brief Push all sensor data (or sentinels) to Modbus holding registers.
 *
 * Write order: data registers → STATUS → diagnostics.
 * Master must read REG_STATUS before trusting any data register value.
 * 0xFFFF in a data register = that sensor is faulted.
 * 0xFFFE = data not yet available (startup state only).
 */
static void updateModbus() {

  // ── Water sensor registers ───────────────────────────────────────────────
  mb.Hreg(REG_TURB, fault.turb ? Cfg::MODBUS_FAULT_VAL
                                : static_cast<uint16_t>(sensor.turb * 10.0f));
  mb.Hreg(REG_PH,   fault.ph   ? Cfg::MODBUS_FAULT_VAL
                                : static_cast<uint16_t>(sensor.ph   * 100.0f));
  mb.Hreg(REG_TDS,  fault.tds  ? Cfg::MODBUS_FAULT_VAL
                                : static_cast<uint16_t>(sensor.tds  * 10.0f));

  // ── Air sensor registers ─────────────────────────────────────────────────
  if (fault.air || !sensor.air_valid) {
    for (uint8_t i = REG_PM1; i <= REG_NO2; i++) {
      mb.Hreg(i, Cfg::MODBUS_FAULT_VAL);
    }
  } else {
    mb.Hreg(REG_PM1,  sensor.pm1);
    mb.Hreg(REG_PM25, sensor.pm25);
    mb.Hreg(REG_PM10, sensor.pm10);
    mb.Hreg(REG_CO2,  sensor.co2);
    mb.Hreg(REG_VOC,  sensor.voc);
    mb.Hreg(REG_TEMP, static_cast<uint16_t>(sensor.temp * 10.0f));
    mb.Hreg(REG_HUM,  static_cast<uint16_t>(sensor.hum));
    mb.Hreg(REG_CH2O, static_cast<uint16_t>(sensor.ch2o * 1000.0f));
    mb.Hreg(REG_CO,   static_cast<uint16_t>(sensor.co   * 10.0f));
    mb.Hreg(REG_O3,   static_cast<uint16_t>(sensor.o3   * 100.0f));
    mb.Hreg(REG_NO2,  static_cast<uint16_t>(sensor.no2  * 100.0f));
  }

  // ── STATUS register ───────────────────────────────────────────────────────
  uint16_t status = 0;
  if (!fault.air  && sensor.air_valid) status |= STATUS_AIR_OK;
  if (!fault.ph)                       status |= STATUS_PH_OK;
  if (!fault.tds)                      status |= STATUS_TDS_OK;
  if (!fault.turb)                     status |= STATUS_TURB_OK;
  if (!fault.adc)                      status |= STATUS_ADC_OK;
  if (devState == DeviceState::RUNNING  ||
      devState == DeviceState::DEGRADED ||
      devState == DeviceState::RECOVERY)  status |= STATUS_RUNNING;
  mb.Hreg(REG_STATUS, status);

  // ── Diagnostic registers ──────────────────────────────────────────────────
  const uint32_t uptime_s = millis() / 1000UL;
  mb.Hreg(REG_UPTIME_LO, static_cast<uint16_t>(uptime_s & 0xFFFFu));
  mb.Hreg(REG_UPTIME_HI, static_cast<uint16_t>(uptime_s >> 16));
  mb.Hreg(REG_DATA_AGE,  staleCycles);
  mb.Hreg(REG_FAULT_CODE, fault.toFaultCode());
}

// ============================================================================
// SECTION 12 — STATE MACHINE
// ============================================================================

/**
 * @brief Advance device state based on current fault flags.
 *
 * Transitions:
 *   Any fault  → DEGRADED  (immediately, staleCycles increments)
 *   DEGRADED   → RECOVERY  (first clean cycle after fault clears)
 *   RECOVERY   → RUNNING   (after RECOVERY_CYCLES consecutive clean cycles)
 *   *          → RUNNING   (direct, if no fault and not in recovery)
 *
 * The RECOVERY gate prevents rapid DEGRADED↔RUNNING oscillation on intermittent
 * sensor faults (e.g. EMI-induced glitches on analog inputs).
 */
static void advanceStateMachine() {
  if (fault.any()) {
    staleCycles++;
    recoveryGoodCount = 0;
    devState = DeviceState::DEGRADED;
    return;
  }

  // No fault
  staleCycles = 0;

  switch (devState) {
    case DeviceState::DEGRADED:
      devState = DeviceState::RECOVERY;
      recoveryGoodCount = 1;
      break;

    case DeviceState::RECOVERY:
      if (++recoveryGoodCount >= Cfg::RECOVERY_CYCLES) {
        devState = DeviceState::RUNNING;
        recoveryGoodCount = 0;
      }
      break;

    default:
      devState = DeviceState::RUNNING;
      break;
  }
}

// ============================================================================
// SECTION 13 — STARTUP SELF-TEST
// ============================================================================

/**
 * @brief Blocking startup self-test. Runs once in setup() before main loop.
 *
 * Tests:
 *  1. ADC voltages in plausible range (0.1–3.2 V) — detects disconnected sensors
 *  2. ZPHS responds within SELFTEST_ZPHS_MS — detects dead sensor / UART fault
 *
 * Does NOT prevent the device from entering RUNNING state — the master receives
 * fault codes via Modbus and can decide the appropriate response.
 *
 * Result signalled via LED blink code (see header comment).
 */
static void runSelfTest() {
  float v;

  // Test 1 — ADC channels
  v = readVoltage(Pin::PH);
  if (v < 0.1f || v > 3.2f) fault.ph   = 1;

  v = readVoltage(Pin::TDS);
  if (v < 0.1f || v > 3.2f) fault.tds  = 1;

  v = readVoltage(Pin::TURB);
  if (v < 0.1f || v > 3.2f) fault.turb = 1;

  // Test 2 — ZPHS UART response
  zphs_sendCmd();
  const unsigned long t0 = millis();
  bool zphs_ok = false;
  while ((millis() - t0) < Cfg::SELFTEST_ZPHS_MS) {
    if (ZphsSerial.available()) { zphs_ok = true; break; }
  }
  if (!zphs_ok) fault.air = 1;

  // Reset ZPHS state — main loop takes over from here
  while (ZphsSerial.available()) ZphsSerial.read();
  zphs.state = ZphsState::IDLE;
  zphs.idx   = 0;

  // Signal result
  if (fault.any()) blinkCode(3, 100);   // 3 fast = fault(s) detected
  else             blinkCode(2, 300);   // 2 slow = all healthy
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // LED — off (active-low)
  pinMode(Pin::LED, OUTPUT);
  digitalWrite(Pin::LED, HIGH);

  // RS485 — default to receive mode
  pinMode(Pin::RS485_CTRL, OUTPUT);
  digitalWrite(Pin::RS485_CTRL, LOW);

  analogReadResolution(12);            // 12-bit, 0–4095
  delay(Cfg::STARTUP_SETTLE_MS);       // let analog supply rails stabilise

  ZphsSerial.begin(Cfg::ZPHS_BAUD);
  rs485Serial.begin(Cfg::RS485_BAUD);

  // Modbus — pre-fill all registers with STALE sentinel so master gets 0xFFFE
  // on any register it reads before the first sensor cycle completes.
  mb.begin(&rs485Serial, Pin::RS485_CTRL);
  mb.slave(Cfg::MODBUS_SLAVE_ID);
  for (uint8_t i = 0; i < REG_COUNT; i++) {
    mb.addHreg(i, Cfg::MODBUS_STALE_VAL);
  }

  devState = DeviceState::WARMUP;
  runSelfTest();
  devState  = DeviceState::RUNNING;
  lastReadMs = millis();

  // Start watchdog LAST — after the 2 s ZPHS self-test probe so it cannot fire.
  // From this point: IWatchdog.reload() MUST be called within WDT_TIMEOUT_US.
  IWatchdog.begin(Cfg::WDT_TIMEOUT_US);
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  const unsigned long now = millis();

  mb.task();        // service Modbus — must run every iteration
  zphs_process();   // non-blocking ZPHS response handler — must run every iteration

  // ── Sensor read cycle — every READ_INTERVAL_MS ───────────────────────────
  // Guard: do not start a new cycle while ZPHS response is still pending.
  if (zphs.state != ZphsState::WAITING && (now - lastReadMs >= Cfg::READ_INTERVAL_MS)) {
    lastReadMs = now;

    zphs_sendCmd();       // fire ZPHS query (response arrives asynchronously)
    readWaterSensors();   // sample + validate analog sensors (blocking, ~1 ms)

    advanceStateMachine();  // update devState & staleCycles from fault flags
    updateModbus();         // push all values (or sentinels) to holding registers

    // Pet the watchdog here — proves the read cycle completed without hanging.
    // A blocked readWaterSensors() or zphs_process() will miss this → WDT reset.
    IWatchdog.reload();
  }

  // ── LED heartbeat (non-blocking) ─────────────────────────────────────────
  // DEGRADED: 4 Hz fast blink.  All other states: 1 Hz normal blink.
  const uint32_t blinkPeriod = (devState == DeviceState::DEGRADED)
                               ? Cfg::BLINK_FAULT_MS
                               : Cfg::BLINK_NORMAL_MS;

  if ((now - lastBlinkMs) >= blinkPeriod) {
    lastBlinkMs = now;
    digitalWrite(Pin::LED, !digitalRead(Pin::LED));
  }
}
// ============================================================================
// END OF FILE
// ============================================================================
