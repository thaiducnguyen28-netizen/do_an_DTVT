// =====================================================================
//  GIAM SAT CHAT LUONG KHONG KHI - ESP32 + Blynk
//  DHT11 + MQ-2 + GP2Y1010 (bui) + LCD 20x4 I2C
// =====================================================================

// ---------- Blynk template ----------
#define BLYNK_TEMPLATE_ID   "TMPL6PUkET05B"
#define BLYNK_TEMPLATE_NAME "Chat Luong Khong Khi"
// !!! Token cu da bi lo -> vao Blynk Console tao lai Auth Token roi dan vao day.
#define BLYNK_AUTH_TOKEN    "DAN_TOKEN_MOI_VAO_DAY"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Preferences.h>
#include <Arduino.h>
// ---------- WiFi ----------
const char auth[] = BLYNK_AUTH_TOKEN;
const char ssid[] = "TEN_WIFI";
const char pass[] = "MAT_KHAU_WIFI";

// ---------- Chan ----------
#define DHTPIN       17
#define DHTTYPE      DHT11
#define MQ2_PIN      33
#define PM25_PIN     32
#define LED_CRT_PIN  27
#define NOT1_PIN     26
#define NOT2_PIN     25
#define SDA_PIN      21
#define SCL_PIN      22

// Dau ra canh bao: gia su ACTIVE-LOW (LOW = bat), trang thai nghi = HIGH
// (giong code goc). Neu mach cua ban active-high thi doi 2 dong duoi.
#define NOT_ON   LOW
#define NOT_OFF  HIGH

// ---------- Phan cung / dien ap (KIEM TRA LAI THEO MACH THUC TE) ----------
// MQ2_VCC        : dien ap cap cho module MQ-2 (3.3 hoac 5.0).
// *_ADC_DIVIDER  : he so cau phan ap truoc chan ADC = (R1+R2)/R2.
//                  Khong co cau phan ap -> 1.0. Cap 5V qua 10k/20k -> 1.5.
// ADC ESP32 CHI CHIU TOI DA 3.3V, neu cam bien ra 5V BAT BUOC phai co cau phan ap!
const float MQ2_VCC           = 3.3;
const float MQ2_ADC_DIVIDER   = 1.0;
const float DUST_ADC_DIVIDER  = 1.0;

// ---------- Nguong mac dinh (don vi: ppm, ug/m3, C, %) ----------
float threshold_mq2   = 300.0;   // ppm
float threshold_dust  = 150.0;   // ug/m3  (truoc day la 0.5 mg/m3 nhung so sanh voi ug/m3 -> sai)
float threshold_temp  = 40.0;    // C
float threshold_humid = 30.0;    // %  (canh bao khi DO AM THAP HON nguong)

// ---------- Canh bao ----------
const float    HYST_RATIO         = 0.10;       // tat canh bao khi xuong duoi nguong 10%
const uint32_t NOTIFY_REPEAT_MS   = 300000UL;   // nhac lai thong bao toi thieu 5 phut
const uint32_t BLINK_MS           = 250;        // chu ky nhap nhay khi co gas

// ---------- MQ-2 ----------
const uint8_t  MQ2_SAMPLES         = 20;
const float    MQ2_FILTER_ALPHA    = 0.2;
const uint32_t MQ2_WARMUP_MS       = 180000UL;  // 3 phut (lan dau nen de 24h cho on dinh)
const float    MQ2_CLEAN_AIR_RATIO = 9.83;      // Rs/R0 trong khong khi sach
const float    MQ2_CURVE_A         = 574.25;    // duong cong LPG
const float    MQ2_CURVE_B         = -2.222;

float mq2_filtered_mv   = 0.0;
float mq2_r0            = 0.0;     // 0 = chua hieu chuan
bool  mq2_ready         = false;
bool  calibrateRequested = false;
unsigned long mq2_start_ms = 0;

// ---------- Bui ----------
const uint8_t DUST_SAMPLES      = 10;
const float   DUST_FILTER_ALPHA = 0.3;
const int samplingTime = 280;
const int deltaTime    = 40;
const int sleepTime    = 9680;
bool dustInit = false;

// ---------- Gia tri do ----------
float temperature = 0, humidity = 0, mq2_ppm = 0, dustDensity = 0;
uint8_t dhtFailCount = 0;
const uint8_t DHT_FAIL_LIMIT = 5;
bool dhtOk() { return dhtFailCount < DHT_FAIL_LIMIT; }

// ---------- Trang thai canh bao ----------
struct AlarmState {
  bool     active     = false;
  bool     notified   = false;
  uint32_t lastNotify = 0;
};
AlarmState gasAlarm, dustAlarm, tempAlarm, humAlarm;

// ---------- Doi tuong ----------
LiquidCrystal_I2C lcd(0x27, 20, 4);
DHT dht(DHTPIN, DHTTYPE);
BlynkTimer timer;
Preferences prefs;

// =====================================================================
//  NGUONG: luu vao flash de offline van giu nguyen sau khi reset
// =====================================================================
void setThreshold(float &var, float v, const char *key) {
  if (fabsf(var - v) > 0.001f) {
    var = v;
    prefs.putFloat(key, v);
  }
}

BLYNK_WRITE(V4) { setThreshold(threshold_mq2,   param.asFloat(), "mq2");  }
BLYNK_WRITE(V5) { setThreshold(threshold_dust,  param.asFloat(), "dust"); }
BLYNK_WRITE(V6) { setThreshold(threshold_temp,  param.asFloat(), "temp"); }
BLYNK_WRITE(V7) { setThreshold(threshold_humid, param.asFloat(), "humi"); }
// V8: nut bam (Button) de hieu chuan lai MQ-2 - CHI bam khi chac chan khong khi sach
BLYNK_WRITE(V8) { if (param.asInt() == 1) calibrateRequested = true; }

BLYNK_CONNECTED() {
  Blynk.syncAll();
}

// =====================================================================
//  DOC CAM BIEN
// =====================================================================
float readAveragedMilliVolts(uint8_t pin, uint8_t samples) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogReadMilliVolts(pin);
    delay(2);
  }
  return (float)sum / samples;
}

void readDHT() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h) && !isnan(t)) {
    humidity = h;
    temperature = t;
    dhtFailCount = 0;
  } else if (dhtFailCount < 255) {
    dhtFailCount++;
  }
}

float mq2Rs() {
  float vout = (mq2_filtered_mv / 1000.0f) * MQ2_ADC_DIVIDER;   // ve dien ap that tren cam bien
  vout = constrain(vout, 0.01f, MQ2_VCC - 0.01f);
  return (MQ2_VCC - vout) / vout;                                // RL triet tieu khi chia cho R0
}

void updateMQ2() {
  float mv = readAveragedMilliVolts(MQ2_PIN, MQ2_SAMPLES);
  mq2_filtered_mv = (mq2_filtered_mv <= 0.0f)
                      ? mv
                      : MQ2_FILTER_ALPHA * mv + (1.0f - MQ2_FILTER_ALPHA) * mq2_filtered_mv;

  // Cho heater nong truoc khi dung gia tri
  if (millis() - mq2_start_ms < MQ2_WARMUP_MS) {
    mq2_ready = false;
    mq2_ppm = 0;
    return;
  }

  // Hieu chuan R0: lan dau (chua co trong flash) hoac khi bam nut V8
  if (calibrateRequested || mq2_r0 <= 0.0f) {
    calibrateRequested = false;
    mq2_r0 = max(mq2Rs() / MQ2_CLEAN_AIR_RATIO, 0.01f);
    prefs.putFloat("r0", mq2_r0);
    Serial.printf("MQ-2 R0 = %.4f (da luu)\n", mq2_r0);
  }

  mq2_ready = true;
  float ratio = mq2Rs() / mq2_r0;
  mq2_ppm = constrain(MQ2_CURVE_A * powf(ratio, MQ2_CURVE_B), 0.0f, 10000.0f);
}

void updateDust() {
  float mv = 0;
  for (uint8_t i = 0; i < DUST_SAMPLES; i++) {
    digitalWrite(LED_CRT_PIN, LOW);            // LED IR bat
    delayMicroseconds(samplingTime);
    mv += analogReadMilliVolts(PM25_PIN);
    delayMicroseconds(deltaTime);
    digitalWrite(LED_CRT_PIN, HIGH);           // LED IR tat
    delayMicroseconds(sleepTime);
  }
  mv /= DUST_SAMPLES;

  float v = (mv / 1000.0f) * DUST_ADC_DIVIDER;             // dien ap thuc tren cam bien
  float d = max(0.0f, (0.17f * v - 0.1f) * 1000.0f);       // mg/m3 -> ug/m3

  if (!dustInit) { dustDensity = d; dustInit = true; }
  else dustDensity = DUST_FILTER_ALPHA * d + (1.0f - DUST_FILTER_ALPHA) * dustDensity;
}

void readSensors() {
  readDHT();
  updateMQ2();
  updateDust();
}

// =====================================================================
//  CANH BAO (co hysteresis, khong chan, chay duoc khi offline)
// =====================================================================
// Tra ve true neu luc nay can gui thong bao len Blynk.
bool evalAlarm(AlarmState &a, float value, float thr, bool lowIsBad) {
  float hyst = fabsf(thr) * HYST_RATIO;
  bool trigger = lowIsBad ? (value < thr)         : (value > thr);
  bool clear   = lowIsBad ? (value > thr + hyst)  : (value < thr - hyst);

  if (!a.active && trigger) {
    a.active = true;
    a.notified = false;
  } else if (a.active && clear) {
    a.active = false;
    a.notified = false;
  }

  if (!a.active || !Blynk.connected()) return false;   // offline: van bao dong cuc bo, doi co mang moi gui

  if (!a.notified || (millis() - a.lastNotify >= NOTIFY_REPEAT_MS)) {
    a.notified = true;
    a.lastNotify = millis();
    return true;
  }
  return false;
}

void checkAlarms() {
  if (mq2_ready &&
      evalAlarm(gasAlarm, mq2_ppm, threshold_mq2, false)) {
    // Giu nguyen event code de khop template Blynk
    Blynk.logEvent("canhbaoCO",
                   "Cảnh báo! Khí gas = " + String((int)mq2_ppm) + " ppm vượt quá mức cho phép!");
  }

  if (evalAlarm(dustAlarm, dustDensity, threshold_dust, false)) {
    Blynk.logEvent("canhbaodust",
                   "Cảnh báo! Nồng độ bụi = " + String(dustDensity, 1) + " ug/m3 vượt quá mức cho phép!");
  }

  if (dhtOk()) {
    if (evalAlarm(tempAlarm, temperature, threshold_temp, false)) {
      Blynk.logEvent("canhbaotemp",
                     "Cảnh báo! Nhiệt độ = " + String(temperature, 1) + " C vượt quá mức cho phép!");
    }
    if (evalAlarm(humAlarm, humidity, threshold_humid, true)) {
      Blynk.logEvent("canhbaohum",
                     "Cảnh báo! Độ ẩm = " + String(humidity, 1) + " % thấp hơn mức cho phép!");
    }
  }
}

// Goi moi 250ms: dieu khien dau ra tu trang thai canh bao (khong con timer tat chong cheo)
//   Nhiet do : NOT1 sang lien tuc
//   Bui      : NOT2 sang lien tuc
//   Gas      : NOT1 + NOT2 nhap nhay (uu tien cao nhat)
// Do am thap chi gui thong bao, khong keu coi.
void updateOutputs() {
  bool blink = ((millis() / BLINK_MS) % 2) == 0;
  bool n1 = tempAlarm.active;
  bool n2 = dustAlarm.active;
  if (gasAlarm.active) { n1 = blink; n2 = blink; }

  digitalWrite(NOT1_PIN, n1 ? NOT_ON : NOT_OFF);
  digitalWrite(NOT2_PIN, n2 ? NOT_ON : NOT_OFF);
}

// =====================================================================
//  BLYNK + LCD
// =====================================================================
void sendToBlynk() {
  if (!Blynk.connected()) return;
  if (dhtOk()) {
    Blynk.virtualWrite(V0, temperature);
    Blynk.virtualWrite(V1, humidity);
  }
  if (mq2_ready) Blynk.virtualWrite(V2, mq2_ppm);
  Blynk.virtualWrite(V3, dustDensity);
}

// Ghi 1 dong 20 ky tu; cot 19 hien '!' neu dang canh bao
void lcdLine(uint8_t row, const char *text, bool alarm) {
  char buf[21];
  snprintf(buf, sizeof(buf), "%-20s", text);
  if (alarm) buf[19] = '!';
  if (row == 0) {                                   // trang thai mang: W = WiFi, B = Blynk
    buf[15] = (WiFi.status() == WL_CONNECTED) ? 'W' : '-';
    buf[16] = Blynk.connected() ? 'B' : '-';
  }
  lcd.setCursor(0, row);
  lcd.print(buf);
}

void updateLCD() {
  char line[24];

  if (dhtOk()) snprintf(line, sizeof(line), "Temp: %.1f C", temperature);
  else         snprintf(line, sizeof(line), "Temp: ERR");
  lcdLine(0, line, tempAlarm.active);

  if (dhtOk()) snprintf(line, sizeof(line), "Humi: %.1f %%", humidity);
  else         snprintf(line, sizeof(line), "Humi: ERR");
  lcdLine(1, line, humAlarm.active);

  if (mq2_ready) {
    snprintf(line, sizeof(line), "MQ2: %d ppm", (int)mq2_ppm);
  } else {
    long left = (long)(MQ2_WARMUP_MS - (millis() - mq2_start_ms)) / 1000;
    snprintf(line, sizeof(line), "MQ2: warmup %lds", left < 0 ? 0 : left);
  }
  lcdLine(2, line, gasAlarm.active);

  snprintf(line, sizeof(line), "Dust: %.1f ug/m3", dustDensity);
  lcdLine(3, line, dustAlarm.active);
}

// =====================================================================
//  KET NOI (khong chan)
// =====================================================================
void checkConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
  } else if (!Blynk.connected()) {
    Blynk.connect(3000);               // timeout ngan 3s
  }
}

// =====================================================================
//  VONG LAP CHINH CUA CAM BIEN (moi 2s)
// =====================================================================
void sensorCycle() {
  readSensors();
  checkAlarms();
  sendToBlynk();
  updateLCD();

  Serial.printf("T=%.1f H=%.1f MQ2=%.0fppm Dust=%.1f | gas:%d dust:%d temp:%d hum:%d\n",
                temperature, humidity, mq2_ppm, dustDensity,
                gasAlarm.active, dustAlarm.active, tempAlarm.active, humAlarm.active);
}

// =====================================================================
void setup() {
  Serial.begin(115200);

  pinMode(NOT1_PIN, OUTPUT);
  pinMode(NOT2_PIN, OUTPUT);
  pinMode(LED_CRT_PIN, OUTPUT);
  digitalWrite(LED_CRT_PIN, HIGH);     // LED cam bien bui tat
  digitalWrite(NOT1_PIN, NOT_OFF);
  digitalWrite(NOT2_PIN, NOT_OFF);

  analogReadResolution(12);
  analogSetPinAttenuation(MQ2_PIN, ADC_11db);
  analogSetPinAttenuation(PM25_PIN, ADC_11db);

  // Nap nguong + R0 da luu
  prefs.begin("air", false);
  threshold_mq2   = prefs.getFloat("mq2",  threshold_mq2);
  threshold_dust  = prefs.getFloat("dust", threshold_dust);
  threshold_temp  = prefs.getFloat("temp", threshold_temp);
  threshold_humid = prefs.getFloat("humi", threshold_humid);
  mq2_r0          = prefs.getFloat("r0",   0.0f);
  mq2_start_ms = millis();

  Wire.begin(SDA_PIN, SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Starting...");

  dht.begin();

  // KHONG cho WiFi: mat mang van do va bao dong cuc bo binh thuong
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, pass);
  Blynk.config(auth);

  timer.setInterval(2000L,  sensorCycle);
  timer.setInterval(250L,   updateOutputs);
  timer.setInterval(10000L, checkConnection);
}

void loop() {
  if (Blynk.connected()) Blynk.run();
  timer.run();
}
