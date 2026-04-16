// FINAL4
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
float pH_slope = 15.0f;
float pH_intercept = -11.6f;

// ── FILTER STORAGE ───────────────────────────────
float tds_filtered  = 0;
float turb_filtered = 0;

// ── ZPHS01B ──────────────────────────────────────
#define ZPHS_RESP_LEN  26
byte zphs_resp[26];
int  zphs_idx        = 0;

bool zphs_sent        = false;
unsigned long zphs_t  = 0;
#define ZPHS_TIMEOUT  1000

// ── Timing ───────────────────────────────────────
unsigned long lastRead  = 0;
unsigned long lastBlink = 0;
#define READ_INTERVAL   2000
#define BLINK_INTERVAL  500

// ── Sensor values ────────────────────────────────
float sv_ph   = 0;
float sv_tds  = 0;
float sv_turb = 0;
float sv_temp = 25.0f;
int   sv_pm1  = 0, sv_pm25 = 0, sv_pm10 = 0;
int   sv_co2  = 0, sv_voc  = 0;
float sv_hum  = 0, sv_ch2o = 0;
float sv_co   = 0, sv_o3   = 0, sv_no2  = 0;

bool air_valid = false;

// ═══════════════════════════════════════════════
// VOLTAGE READ
// ═══════════════════════════════════════════════
float readVoltage(int pin) {
  long sum = 0;
  int minVal = 4095, maxVal = 0;

  for (int i = 0; i < 20; i++) {
    int val = analogRead(pin);
    sum += val;
    if (val < minVal) minVal = val;
    if (val > maxVal) maxVal = val;
  }

  sum = sum - minVal - maxVal;
  float avg = sum / 18.0f;
  return avg * (3.3f / 4095.0f);
}

// ═══════════════════════════════════════════════
// CALCULATIONS
// ═══════════════════════════════════════════════
float calcPH(float v) {
  float ph = pH_slope * v + pH_intercept;
  if (ph < 0)  ph = 0;
  if (ph > 14) ph = 14;
  return ph;
}

float calcTDS(float v) {
  float comp = 1.0f + 0.02f * (sv_temp - 25.0f);
  float cv   = v / comp;
  float tds  = (133.42f * cv * cv * cv
              - 255.86f * cv * cv
              + 857.39f * cv) * 0.5f;
  return tds * 1.36f;
}

/*float calcTurbidity(float v) {
  float Vclean = 3.34f;
  float Vdirty = 0.05f;

  if (v >= 3.25f) return 0;

  float scale = 3000.0f / (Vclean - Vdirty);
  float ntu   = (Vclean - v) * scale;

  if (ntu < 0)    ntu = 0;
  if (ntu > 3000) ntu = 3000;

  return ntu;
}*/
float calcTurbidity(float v) {

  float Vclean = 1.87f;
  float slope  = 1796.0f;

  if (v >= Vclean) return 0;

  float ntu = (Vclean - v) * slope;

  if (ntu < 0) ntu = 0;
  if (ntu > 3000) ntu = 3000;

  return ntu;
}

// ═══════════════════════════════════════════════
// SEND COMMAND
// ═══════════════════════════════════════════════
void zphs_sendCmd() {
  byte cmd[9] = {0xFF,0x01,0x86,0,0,0,0,0,0x79};

  while (ZphsSerial.available()) ZphsSerial.read(); // flush

  zphs_idx  = 0;    // reset frame index
  zphs_sent = true;
  zphs_t    = millis();

  ZphsSerial.write(cmd, 9);
}

// ═══════════════════════════════════════════════
// CONTINUOUS UART READ
// ═══════════════════════════════════════════════
void zphs_readResp() {

  // Timeout — give up waiting
  if (zphs_sent && (millis() - zphs_t > ZPHS_TIMEOUT)) {
    zphs_sent = false;
    zphs_idx  = 0;
    air_valid = false;
  }

  while (ZphsSerial.available()) {

    byte b = ZphsSerial.read();

    // Wait for 0xFF start byte
    if (zphs_idx == 0 && b != 0xFF) continue;

    zphs_resp[zphs_idx++] = b;

    if (zphs_idx == ZPHS_RESP_LEN) {

      zphs_idx = 0;

      // Validate second header byte
      if (zphs_resp[1] != 0x86) return;

      // ✅ FIXED: explicit byte cast prevents int promotion bug
      byte sum = 0;
      for (int j = 1; j <= 24; j++) sum += zphs_resp[j];
      byte checksum = (byte)((~sum) + 1);
      if (checksum != zphs_resp[25]) return;

      // ── Parse all values ──────────────────────
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

      air_valid = true;
      zphs_sent = false;
    }
  }
}

// ═══════════════════════════════════════════════
// MODBUS UPDATE
// ═══════════════════════════════════════════════
void updateModbus() {
  if (sv_turb < 0)    sv_turb = 0;
  if (sv_turb > 3000) sv_turb = 3000;

  mb.Hreg(0, (uint16_t)(sv_turb * 10));
  mb.Hreg(1, (uint16_t)(sv_ph   * 100));
  mb.Hreg(2, (uint16_t)(sv_tds  * 10));

  if (air_valid) {
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
  } else {
    for (int i = 3; i <= 13; i++) mb.Hreg(i, 0xFFFF);
  }

  mb.Hreg(17, air_valid ? 1 : 0);
}

// ═══════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════
void setup() {
  pinMode(PC13, OUTPUT);
  pinMode(RS485_CTRL, OUTPUT);
  pinMode(TURB_PIN, INPUT_PULLDOWN);

  digitalWrite(RS485_CTRL, LOW);
  digitalWrite(PC13, HIGH);

  ZphsSerial.begin(9600);
  rs485Serial.begin(9600);

  analogReadResolution(12);

  mb.begin(&rs485Serial, RS485_CTRL);
  mb.slave(1);

  for (int i = 0; i < 18; i++) mb.addHreg(i);

  updateModbus();
}

// ═══════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  mb.task();

  zphs_readResp();  // must run every loop

  if (!zphs_sent && (now - lastRead >= READ_INTERVAL)) {
    lastRead = now;

    zphs_sendCmd();

    sv_ph = calcPH(readVoltage(PH_PIN));

    float tds_raw  = calcTDS(readVoltage(TDS_PIN));
    tds_filtered   = 0.7f * tds_filtered + 0.3f * tds_raw;
    sv_tds         = tds_filtered;

    float turb_raw = calcTurbidity(readVoltage(TURB_PIN));
    turb_filtered  = 0.7f * turb_filtered + 0.3f * turb_raw;
    sv_turb        = turb_filtered;

    updateModbus();
  }

  // LED blink on timer
  if (now - lastBlink >= BLINK_INTERVAL) {
    lastBlink = now;
    digitalWrite(PC13, !digitalRead(PC13));
  }
}