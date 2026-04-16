// ═══════════════════════════════════════════════
//  CARBELIM — Black Pill STM32F401CC
//  No FTDI, No delays
//  ZphsSerial (PB6/PB7)  → ZPHS01B
//  rs485Serial(PA3/PA2)  → MAX485 Modbus RTU
// ═══════════════════════════════════════════════

#include <ModbusRTU.h>

// ── UART ─────────────────────────────────────────
HardwareSerial ZphsSerial(PB7, PB6);
HardwareSerial rs485Serial(PA3, PA2);

// ── RS485 ────────────────────────────────────────
#define RS485_CTRL  PA8
ModbusRTU mb;

// ── ANALOG PINS ──────────────────────────────────
#define PH_PIN    PA0
#define TDS_PIN   PA1
#define TURB_PIN  PA4

// ── pH CALIBRATION ───────────────────────────────
#define PH4_VOLTAGE  2.23f
#define PH7_VOLTAGE  1.65f
float pH_slope     = (7.0f - 4.0f) / (PH7_VOLTAGE - PH4_VOLTAGE);
float pH_intercept = 7.0f - pH_slope * PH7_VOLTAGE;

// ── ZPHS01B ──────────────────────────────────────
#define ZPHS_RESP_LEN 26
byte zphs_resp[26];
bool zphs_sent       = false;
unsigned long zphs_t = 0;
#define ZPHS_WAIT    200

// ── Timing ───────────────────────────────────────
unsigned long lastRead = 0;
#define READ_INTERVAL  2000

// ── Sensor values ────────────────────────────────
float sv_ph   = 7.0f;
float sv_tds  = 0.0f;
float sv_turb = 0.0f;
float sv_temp = 25.0f;
int   sv_pm1  = 0, sv_pm25 = 0, sv_pm10 = 0;
int   sv_co2  = 0, sv_voc  = 0;
float sv_hum  = 0, sv_ch2o = 0;
float sv_co   = 0, sv_o3   = 0, sv_no2  = 0;

// ═══════════════════════════════════════════════
//  VOLTAGE READ — NO DELAY
// ═══════════════════════════════════════════════

float readVoltage(int pin) {
  long sum = 0;
  for (int i = 0; i < 10; i++) sum += analogRead(pin);
  return (sum / 10.0f) * (3.3f / 4095.0f);
}

// ═══════════════════════════════════════════════
//  WATER SENSORS
// ═══════════════════════════════════════════════

float calcPH(float v) {
  return constrain(pH_slope * v + pH_intercept, 0.0f, 14.0f);
}

float calcTDS(float v) {
  float comp = 1.0f + 0.02f * (sv_temp - 25.0f);
  float cv   = v / comp;
  return (133.42f * cv * cv * cv
        - 255.86f * cv * cv
        + 857.39f * cv) * 0.5f;
}

float calcTurbidity(float v) {
  float realV = v * 1.5f;
  if (realV < 2.5f) return 3000.0f;
  float ntu = -1120.4f * realV * realV
              + 5742.3f * realV - 4352.9f;
  return (ntu < 0.0f) ? 0.0f : ntu;
}

// ═══════════════════════════════════════════════
//  ZPHS01B — NON BLOCKING
// ═══════════════════════════════════════════════

void zphs_sendCmd() {
  byte cmd[9] = {0xFF,0x01,0x86,0,0,0,0,0,0x79};
  while (ZphsSerial.available()) ZphsSerial.read();
  ZphsSerial.write(cmd, 9);
  zphs_sent = true;
  zphs_t    = millis();
}

bool zphs_readResp() {
  if (!zphs_sent) return false;
  if (millis() - zphs_t < ZPHS_WAIT) return false;
  zphs_sent = false;

  if (ZphsSerial.available() < ZPHS_RESP_LEN) return false;

  int i = 0;
  while (ZphsSerial.available() && i < ZPHS_RESP_LEN)
    zphs_resp[i++] = ZphsSerial.read();

  if (zphs_resp[0] != 0xFF || zphs_resp[1] != 0x86) return false;

  byte sum = 0;
  for (int j = 1; j <= 24; j++) sum += zphs_resp[j];
  if (((~sum) + 1) != zphs_resp[25]) return false;

  sv_pm1  = (zphs_resp[2]  << 8) | zphs_resp[3];
  sv_pm25 = (zphs_resp[4]  << 8) | zphs_resp[5];
  sv_pm10 = (zphs_resp[6]  << 8) | zphs_resp[7];
  sv_co2  = (zphs_resp[8]  << 8) | zphs_resp[9];
  sv_voc  =  zphs_resp[10];
  sv_temp = (((zphs_resp[11] << 8) | zphs_resp[12]) - 500) * 0.1f;
  sv_hum  =  (zphs_resp[13] << 8) | zphs_resp[14];
  sv_ch2o = ((zphs_resp[15] << 8) | zphs_resp[16]) * 0.001f;
  sv_co   = ((zphs_resp[17] << 8) | zphs_resp[18]) * 0.1f;
  sv_o3   = ((zphs_resp[19] << 8) | zphs_resp[20]) * 0.01f;
  sv_no2  = ((zphs_resp[21] << 8) | zphs_resp[22]) * 0.01f;
  return true;
}

// ═══════════════════════════════════════════════
//  UPDATE MODBUS REGISTERS
// ═══════════════════════════════════════════════

void updateModbus() {
  mb.Hreg(0,  (uint16_t)(sv_turb * 10));
  mb.Hreg(1,  (uint16_t)(sv_ph   * 100));
  mb.Hreg(2,  (uint16_t)(sv_tds  * 10));
  mb.Hreg(3,  sv_pm1);
  mb.Hreg(4,  sv_pm25);
  mb.Hreg(5,  sv_pm10);
  mb.Hreg(6,  sv_co2);
  mb.Hreg(7,  sv_voc);
  mb.Hreg(8,  (uint16_t)(sv_temp  * 10));
  mb.Hreg(9,  (uint16_t)(sv_hum));
  mb.Hreg(10, (uint16_t)(sv_ch2o  * 1000));
  mb.Hreg(11, (uint16_t)(sv_co    * 10));
  mb.Hreg(12, (uint16_t)(sv_o3    * 100));
  mb.Hreg(13, (uint16_t)(sv_no2   * 100));
  mb.Hreg(14, 0);
  mb.Hreg(15, 0);
  mb.Hreg(16, 0);
  mb.Hreg(17, 1);
}

// ═══════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════

void setup() {
  pinMode(PC13,      OUTPUT);
  pinMode(RS485_CTRL, OUTPUT);
  digitalWrite(RS485_CTRL, LOW);
  digitalWrite(PC13, HIGH);

  ZphsSerial.begin(9600);
  rs485Serial.begin(9600);
  analogReadResolution(12);

  mb.begin(&rs485Serial, RS485_CTRL);
  mb.slave(1);
  for (int i = 0; i < 18; i++) mb.addHreg(i);

  // Initial values in registers
  updateModbus();
}

// ═══════════════════════════════════════════════
//  LOOP — FULLY NON BLOCKING
// ═══════════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  // ── 1. Modbus always first ───────────────────
  mb.task();

  // ── 2. Check ZPHS response ───────────────────
  if (zphs_readResp()) {
    updateModbus();
  }

  // ── 3. Read sensors every 2s ─────────────────
  if (now - lastRead >= READ_INTERVAL) {
    lastRead = now;

    // Send ZPHS command
    zphs_sendCmd();

    // Read water sensors
    sv_ph   = calcPH(readVoltage(PH_PIN));
    sv_tds  = calcTDS(readVoltage(TDS_PIN));
    sv_turb = calcTurbidity(readVoltage(TURB_PIN));

    updateModbus();

    // Blink LED to show alive
    digitalWrite(PC13, !digitalRead(PC13));
  }
}
