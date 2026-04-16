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

// ── CONSTANTS ────────────────────────────────────
#define READ_INTERVAL   2000
#define BLINK_INTERVAL  500
#define ZPHS_TIMEOUT    1000

#define PH_MIN   0.0f
#define PH_MAX   14.0f
#define TURB_MAX 3000.0f
#define TDS_OFFSET_V 0.65f

// ── SENSOR STRUCT ────────────────────────────────
typedef struct {
  float ph;
  float tds;
  float turb;
  float temp;
  float hum;

  uint16_t pm1, pm25, pm10;
  uint16_t co2, voc;

  float ch2o, co, o3, no2;

  bool air_valid;
} SensorData;

SensorData sensor;

// ── MODBUS REGISTER MAP ──────────────────────────
typedef enum {
  REG_TURB = 0,
  REG_PH,
  REG_TDS,
  REG_PM1,
  REG_PM25,
  REG_PM10,
  REG_CO2,
  REG_VOC,
  REG_TEMP,
  REG_HUM,
  REG_CH2O,
  REG_CO,
  REG_O3,
  REG_NO2,
  REG_RESERVED1,
  REG_RESERVED2,
  REG_RESERVED3,
  REG_STATUS
} ModbusRegs;

// ── FILTER STORAGE ───────────────────────────────
float tds_filtered  = 0;
float turb_filtered = 0;

// ── ZPHS SENSOR ──────────────────────────────────
#define ZPHS_RESP_LEN 26
byte zphs_resp[26];
int zphs_idx = 0;

bool zphs_sent = false;
unsigned long zphs_t = 0;

// ── TIMING ───────────────────────────────────────
unsigned long lastRead = 0;
unsigned long lastBlink = 0;

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
// SENSOR CALCULATIONS
// ═══════════════════════════════════════════════
float calcPH(float v) {
  float ph = 15.0f * v - 11.6f;
  if (ph < PH_MIN) ph = PH_MIN;
  if (ph > PH_MAX) ph = PH_MAX;
  return ph;
}

float calcTDS(float v) {

  v = v - TDS_OFFSET_V;   // ✅ shift baseline

  if (v <= 0) return 0;
  if (v > 1.8f) v = 1.8f;

  float tds = v * 700.0f;   // initial scaling

  return tds;
}

float calcTurbidity(float v) {
  float Vclean = 1.87f;
  float slope  = 1796.0f;

  if (v >= Vclean) return 0;

  float ntu = (Vclean - v) * slope;

  if (ntu < 0) ntu = 0;
  if (ntu > TURB_MAX) ntu = TURB_MAX;

  return ntu;
}

// ═══════════════════════════════════════════════
// AIR SENSOR
// ═══════════════════════════════════════════════
void zphs_sendCmd() {
  byte cmd[9] = {0xFF,0x01,0x86,0,0,0,0,0,0x79};

  while (ZphsSerial.available()) ZphsSerial.read();

  zphs_idx = 0;
  zphs_sent = true;
  zphs_t = millis();

  ZphsSerial.write(cmd, 9);
}

void zphs_readResp() {
  if (zphs_sent && (millis() - zphs_t > ZPHS_TIMEOUT)) {
    zphs_sent = false;
    zphs_idx = 0;
    sensor.air_valid = false;
  }

  while (ZphsSerial.available()) {
    byte b = ZphsSerial.read();

    if (zphs_idx == 0 && b != 0xFF) continue;

    zphs_resp[zphs_idx++] = b;

    if (zphs_idx == ZPHS_RESP_LEN) {
      zphs_idx = 0;

      if (zphs_resp[1] != 0x86) return;

      byte sum = 0;
      for (int j = 1; j <= 24; j++) sum += zphs_resp[j];
      byte checksum = (~sum) + 1;
      if (checksum != zphs_resp[25]) return;

      sensor.pm1  = (zphs_resp[2] << 8) | zphs_resp[3];
      sensor.pm25 = (zphs_resp[4] << 8) | zphs_resp[5];
      sensor.pm10 = (zphs_resp[6] << 8) | zphs_resp[7];
      sensor.co2  = (zphs_resp[8] << 8) | zphs_resp[9];
      sensor.voc  = zphs_resp[10];
      sensor.temp = (((zphs_resp[11] << 8) | zphs_resp[12]) - 500) * 0.1f;
      sensor.hum  = (zphs_resp[13] << 8) | zphs_resp[14];
      sensor.ch2o = ((zphs_resp[15] << 8) | zphs_resp[16]) * 0.001f;
      sensor.co   = ((zphs_resp[17] << 8) | zphs_resp[18]) * 0.1f;
      sensor.o3   = ((zphs_resp[19] << 8) | zphs_resp[20]) * 0.01f;
      sensor.no2  = ((zphs_resp[21] << 8) | zphs_resp[22]) * 0.01f;

      sensor.air_valid = true;
      zphs_sent = false;
    }
  }
}

// ═══════════════════════════════════════════════
// SENSOR READ
// ═══════════════════════════════════════════════
void readSensors() {
  sensor.ph = calcPH(readVoltage(PH_PIN));

  float tds_raw = calcTDS(readVoltage(TDS_PIN));
  tds_filtered = 0.7f * tds_filtered + 0.3f * tds_raw;
  sensor.tds = tds_filtered;

  float turb_raw = calcTurbidity(readVoltage(TURB_PIN));
  turb_filtered = 0.7f * turb_filtered + 0.3f * turb_raw;
  sensor.turb = turb_filtered;
}

// ═══════════════════════════════════════════════
// MODBUS UPDATE
// ═══════════════════════════════════════════════
void updateModbus() {
  mb.Hreg(REG_TURB, (uint16_t)(sensor.turb * 10));
  mb.Hreg(REG_PH,   (uint16_t)(sensor.ph * 100));
  mb.Hreg(REG_TDS,  (uint16_t)(sensor.tds * 10));

  if (sensor.air_valid) {
    mb.Hreg(REG_PM1, sensor.pm1);
    mb.Hreg(REG_PM25, sensor.pm25);
    mb.Hreg(REG_PM10, sensor.pm10);
    mb.Hreg(REG_CO2, sensor.co2);
    mb.Hreg(REG_VOC, sensor.voc);
    mb.Hreg(REG_TEMP, (uint16_t)(sensor.temp * 10));
    mb.Hreg(REG_HUM, (uint16_t)(sensor.hum));
    mb.Hreg(REG_CH2O, (uint16_t)(sensor.ch2o * 1000));
    mb.Hreg(REG_CO, (uint16_t)(sensor.co * 10));
    mb.Hreg(REG_O3, (uint16_t)(sensor.o3 * 100));
    mb.Hreg(REG_NO2, (uint16_t)(sensor.no2 * 100));
  } else {
    for (int i = REG_PM1; i <= REG_NO2; i++) {
      mb.Hreg(i, 0xFFFF);
    }
  }

  mb.Hreg(REG_STATUS, sensor.air_valid ? 1 : 0);
}

// ═══════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════
void setup() {
  pinMode(PC13, OUTPUT);
  pinMode(RS485_CTRL, OUTPUT);

  digitalWrite(RS485_CTRL, LOW);

  ZphsSerial.begin(9600);
  rs485Serial.begin(9600);

  analogReadResolution(12);

  mb.begin(&rs485Serial, RS485_CTRL);
  mb.slave(1);

  for (int i = 0; i <= REG_STATUS; i++) {
    mb.addHreg(i);
  }
}

// ═══════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  mb.task();
  zphs_readResp();

  if (!zphs_sent && (now - lastRead >= READ_INTERVAL)) {
    lastRead = now;

    zphs_sendCmd();
    readSensors();
    updateModbus();
  }

  if (now - lastBlink >= BLINK_INTERVAL) {
    lastBlink = now;
    digitalWrite(PC13, !digitalRead(PC13));
  }
}