/*
 * esp32_fan_node.ino — "เครื่องจักรจริง" (พัดลม + สาย tach) ใช้แทน simulator.js12
 * -----------------------------------------------------------
 *  - นับพัลส์ tach -> คำนวณ RPM ทุก WINDOW_MS
 *  - PUBLISH  <PREFIX>/status   (JSON array รูปแบบเดียวกับ simulator.js, retained)
 *  - SUBSCRIBE <PREFIX>/command (รับ status / reset จาก dashboard และ predictor.js)
 *
 * ไลบรารีที่ต้องติดตั้ง (Library Manager): PubSubClient, ArduinoJson (v7)
 *
 * การต่อสาย:
 *   ไดรเวอร์มอเตอร์: ENA -> GPIO25, IN1 -> GPIO26, IN2 -> GPIO27 (เหมือน motor_button_control.ino)
 *   สัญญาณรอบ (tach) -> TACH_PIN (GPIO33 ตาม #define ด้านล่าง) ใช้ INPUT_PULLUP, ต้อง common ground กับ ESP32
 *   ห้ามให้สัญญาณเกิน 3.3V เข้าขา ESP32 โดยตรง
 * -----------------------------------------------------------
 */
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>

// ---------------- ตั้งค่า ----------------
const char* WIFI_SSID  = "Ketie";
const char* WIFI_PASS  = "kt141048";
const char* MQTT_HOST  = "broker.hivemq.com";
const uint16_t MQTT_PORT = 1883;
const char* PREFIX     = "mydemo123";   // ต้องตรงกับ predictor.js และ dashboard.html

const char* MACHINE_ID   = "fan1";
const char* MACHINE_NAME = "Fan-01";
const char* MACHINE_LOC  = "โต๊ะทดลอง";

#define TACH_PIN        34     // ขาที่ต่อสัญญาณ tach/เซนเซอร์รอบ (ห้ามซ้ำ 25/26/27 ที่ใช้ขับมอเตอร์) ; แก้ให้ตรงกับที่ต่อจริง
#define PULSES_PER_REV  1      // 1 = ใช้จำนวนพัลส์ต่อนาทีเป็นค่า RPM ตรง ๆ ไม่หารเพิ่ม (ถ้าอยากได้รอบจริงของพัดลม PC ให้ใส่ 2 ตามดาตาชีต)
#define MAX_VALID_RPM   6000   // เกินนี้ถือว่าเป็นสัญญาณรบกวน ไม่ใช่ค่าจริง
#define MIN_PULSE_US    1500   // พัลส์ที่ถี่กว่านี้ (>666 Hz) ทิ้ง = ตัดสัญญาณรบกวน
#define WINDOW_MS       1000   // ช่วงเวลานับพัลส์ / ส่งข้อมูล (สั้นลง = ค่า RPM อัปเดตถี่ขึ้น เกือบเรียลไทม์)
#define RAW_RPM_MODE        1    // 1 = ส่งค่าที่นับได้ในแต่ละช่วงขึ้น dashboard ตรง ๆ ไม่คำนวณ/ไม่หาร ; 0 = คำนวณเป็น RPM จริง (ใช้ PULSES_PER_REV)
#define RANDOM_FAULT_ENABLED 1   // 0 = ยังไม่สุ่มซ่อมบำรุง/ขัดข้อง (เครื่องทำงานไปเรื่อย ๆ) ; 1 = เปิดการสุ่ม
// --- จำลองเครื่องเสียแบบสุ่ม (ซ่อมบำรุง / ขัดข้อง) --- ใช้เมื่อ RANDOM_FAULT_ENABLED = 1 ---
// predictor.js ต้องเก็บตัวอย่าง RPM ให้ครบ CALIBRATION_N (=10) ค่าก่อน ถึงจะเริ่มประเมินได้
// จึงห้ามสุ่มเสียก่อนที่จะส่งข้อมูลครบ ไม่งั้น dashboard จะค้างที่ "กำลังเรียนรู้ (x/10)"
#define ENABLE_RANDOM_FAULT 1    // 0 = ปิดสุ่มเสีย (เครื่องทำงานไปเรื่อย ๆ จนกว่าจะสั่งเองหรือ AI สั่ง) ; 1 = เปิดสุ่มซ่อมบำรุง/ขัดข้อง
#define CALIB_SAMPLES       10   // ต้องเท่ากับ CALIBRATION_N ใน predictor.js
#define CALIB_SPARE         5    // เผื่อไว้ (ตัวอย่างอุ่นเครื่อง WARMUP_N=3 ที่ predictor ทิ้ง + ข้อความ publish ตอนรับคำสั่ง)
#define FAULT_EXTRA_MIN_SEC 30   // หลังเรียนรู้ครบแล้ว ให้ทำงานต่อเพิ่มอย่างน้อยกี่วินาที ก่อนจะสุ่มเสียได้
#define FAULT_EXTRA_MAX_SEC 120  // ...และอย่างมากกี่วินาที (สุ่มใหม่ทุกครั้งที่เริ่มทำงาน) ; 0 = ปิดการสุ่มเสียทั้งหมด
#define ERROR_CHANCE_PCT    40   // โอกาส % ที่จะเสียแบบ "ขัดข้อง" (error) ; ที่เหลือคือ "ซ่อมบำรุง" (maintenance)
// --- ขับมอเตอร์ผ่านไดรเวอร์ (ต่อเหมือน motor_button_control.ino ที่ใช้งานได้) ---
#define MOTOR_ENA       25     // ENA  (คุมความเร็ว/เปิด-ปิด)
#define MOTOR_IN1       26     // IN1
#define MOTOR_IN2       27     // IN2
#define MOTOR_SPEED     180    // ความเร็วตอนเปิด (0-255)
// -----------------------------------------

volatile uint32_t pulseCount = 0;
volatile uint32_t edgeCount = 0;     // ขอบสัญญาณทั้งหมดที่ขา tach (ก่อนกรอง) ใช้ไล่ปัญหา: ถ้า edges=0 = ไม่มีสัญญาณเข้าขาเลย
volatile uint32_t lastPulseUs = 0;
uint8_t zeroWindows = 0;             // จำนวนช่วงติดกันที่นับพัลส์ได้ 0 ขณะ running

void IRAM_ATTR onTach() {
  // ผูกกับขอบขาลง (FALLING) โดยตรง จึงไม่ต้องเรียก digitalRead ใน ISR
  // (digitalRead ไม่ใช่ฟังก์ชัน IRAM -> เสี่ยงค้าง/รีบูตเมื่อ WiFi เขียน flash และอ่านระดับสัญญาณพลาดถ้าพัลส์สั้น)
  edgeCount++;
  uint32_t now = micros();
  if (now - lastPulseUs < MIN_PULSE_US) return;  // debounce / กรองสัญญาณรบกวน
  lastPulseUs = now;
  pulseCount++;
}

WiFiClient net;
PubSubClient mqtt(net);

char statusTopic[96], commandTopic[96];
String status = "stopped";   // เปิดเครื่องมาแล้วยังไม่หมุน รอสั่ง "ทำงานปกติ" จากหน้า Dashboard
String note = "";
float lastRpm = 0;
bool rpmInvalid = false;
uint32_t lastPulses = 0;
uint32_t lastEdges = 0;    // ขอบสัญญาณที่เห็นในช่วงล่าสุด (ก่อนกรอง) ส่งขึ้น dashboard ไว้ไล่ปัญหา
bool tachWarn = false;     // true = note ปัจจุบันเป็นคำเตือนเรื่อง tach ที่เราตั้งเอง (จะได้ล้างเฉพาะอันนี้)
uint32_t runStartMs = 0;   // เวลาที่เริ่ม running (0 = ยังไม่เริ่มนับ)
uint32_t faultAfterMs = 0; // สุ่มได้กี่ ms (นับจากเริ่ม running) ถึงจะเสีย
bool faultIsError = false; // true = จะเสียแบบ "ขัดข้อง", false = "ซ่อมบำรุง"
uint32_t lastWindowMs = 0;
uint32_t lastReconnectMs = 0;

void applyFan() {
  // running = มอเตอร์หมุน ; สถานะอื่น (stopped/maintenance/error) = หยุดมอเตอร์
  if (status == "running") {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_ENA, MOTOR_SPEED);
  } else {
    analogWrite(MOTOR_ENA, 0);
  }
}

void publishStatus() {
  if (!mqtt.connected()) return;

  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();
  JsonObject o = arr.add<JsonObject>();
  o["id"] = MACHINE_ID;
  o["name"] = MACHINE_NAME;
  o["loc"] = MACHINE_LOC;
  o["status"] = status;
  o["note"] = note;
  o["rpm"] = (int)lastRpm;
  o["pulses"] = lastPulses;
  o["edges"] = lastEdges;
  o["tachPin"] = TACH_PIN;
  o["tachLevel"] = digitalRead(TACH_PIN);
  // ไม่ใส่ temp / vibration เพราะ ESP32 ตัวนี้ไม่มีเซนเซอร์ — ห้ามส่งเลขปลอม
  o["updatedAt"] = (uint64_t)time(nullptr) * 1000ULL;

  static char buf[640];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(statusTopic, (const uint8_t*)buf, n, true);  // retained
}

void onCommand(char* topic, byte* payload, unsigned int len) {
  JsonDocument doc;
  if (deserializeJson(doc, payload, len)) return;
  const char* id = doc["id"] | "";
  if (strcmp(id, MACHINE_ID) != 0) return;

  if (strcmp(doc["action"] | "", "reset") == 0) {
    status = "running";
    tachWarn = false;
    note = "🔄 รีเซ็ตแล้ว";
    runStartMs = 0;   // เริ่มนับเวลาสุ่มเสียใหม่
    rpmInvalid = false;
    Serial.println("CMD reset");
  } else if (doc["status"].is<const char*>()) {
    status = doc["status"].as<String>();
    tachWarn = false;
    runStartMs = 0;
    bool autoCmd = doc["auto"] | false;
    if (status == "maintenance")
      note = autoCmd ? "🤖 AI สั่งหยุดเครื่องเพื่อซ่อมบำรุงอัตโนมัติ — รอทีมยืนยันซ่อมเสร็จ"
                     : "ซ่อมบำรุงตามคำสั่ง";
    else if (status == "running") note = "▶️ ทีมยืนยันซ่อมเสร็จแล้ว กลับมาทำงานปกติ";
    else note = "";
    Serial.printf("CMD status=%s\n", status.c_str());
  }
  applyFan();
  publishStatus();
}

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\nWiFi OK  IP=%s\n", WiFi.localIP().toString().c_str());
}

void ensureMqtt() {
  if (mqtt.connected()) return;
  if (millis() - lastReconnectMs < 3000) return;
  lastReconnectMs = millis();
  String cid = "esp32-fan-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  if (mqtt.connect(cid.c_str())) {
    mqtt.subscribe(commandTopic);
    Serial.println("MQTT OK");
    publishStatus();
  } else {
    Serial.printf("MQTT fail rc=%d\n", mqtt.state());
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  applyFan();   // เริ่มที่สถานะ stopped = มอเตอร์ยังไม่หมุน

  pinMode(TACH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), onTach, FALLING);  // นับเฉพาะขอบขาลง

  connectWifi();
  configTime(0, 0, "pool.ntp.org", "time.google.com");  // ให้ updatedAt เป็นเวลาจริง
  for (int i = 0; i < 20 && time(nullptr) < 1700000000; i++) delay(250);

  snprintf(statusTopic,  sizeof(statusTopic),  "%s/status",  PREFIX);
  snprintf(commandTopic, sizeof(commandTopic), "%s/command", PREFIX);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(768);
  mqtt.setCallback(onCommand);
  lastWindowMs = millis();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWifi();
  ensureMqtt();
  mqtt.loop();

#if RANDOM_FAULT_ENABLED
  // --- จำลองเครื่องเสียแบบสุ่ม: รอให้ predictor เรียนรู้ครบก่อน (CALIB_SAMPLES ค่า) + สุ่มเวลาเพิ่ม แล้วเสียแบบ "ซ่อมบำรุง" หรือ "ขัดข้อง" ---
  if (!ENABLE_RANDOM_FAULT) {
    runStartMs = 0;   // ปิดการสุ่มเสีย ข้ามส่วนนี้ทั้งหมด
  } else if (status != "running") {
    runStartMs = 0;
  } else if (runStartMs == 0) {
    runStartMs = millis();
    uint32_t calibMs = (uint32_t)(CALIB_SAMPLES + CALIB_SPARE) * WINDOW_MS;
    uint32_t extraMs = (FAULT_EXTRA_MAX_SEC > FAULT_EXTRA_MIN_SEC)
        ? (uint32_t)random((long)FAULT_EXTRA_MIN_SEC * 1000L, (long)FAULT_EXTRA_MAX_SEC * 1000L + 1)
        : (uint32_t)FAULT_EXTRA_MIN_SEC * 1000UL;
    faultAfterMs = calibMs + extraMs;
    faultIsError = (random(100) < ERROR_CHANCE_PCT);
    Serial.printf("เริ่มทำงาน — จะเสียแบบ%s ในอีก ~%.1f วินาที (เรียนรู้ ~%.0f วิ + สุ่ม ~%.0f วิ)\n",
                  faultIsError ? "ขัดข้อง" : "ซ่อมบำรุง", faultAfterMs / 1000.0f,
                  calibMs / 1000.0f, extraMs / 1000.0f);
  } else if (FAULT_EXTRA_MAX_SEC > 0 && millis() - runStartMs >= faultAfterMs) {
    if (faultIsError) {
      status = "error";
      note = "🛑 เครื่องขัดข้อง (สุ่ม) — หยุดเครื่องฉุกเฉิน กด \"รีเซ็ต\" เพื่อเริ่มใหม่";
      Serial.println("RANDOM FAULT -> error");
    } else {
      status = "maintenance";
      note = "🔧 ถึงรอบซ่อมบำรุง (สุ่ม) — หยุดเครื่อง รอทีมยืนยันซ่อมเสร็จ";
      Serial.println("RANDOM MAINTENANCE -> maintenance");
    }
    runStartMs = 0;
    applyFan();
    publishStatus();
  }
#endif

  uint32_t now = millis();
  uint32_t dt = now - lastWindowMs;
  if (dt < WINDOW_MS) return;
  lastWindowMs = now;

  noInterrupts();
  uint32_t cnt = pulseCount;
  uint32_t edges = edgeCount;
  pulseCount = 0;
  edgeCount = 0;
  interrupts();

  lastPulses = cnt;
  lastEdges = edges;
#if RAW_RPM_MODE
  float rpm = (float)cnt;   // ค่าดิบ: จำนวนพัลส์ที่นับได้ในช่วง WINDOW_MS ไม่คำนวณ
#else
  float rpm = cnt * 60000.0f / ((float)dt * PULSES_PER_REV);
#endif
  Serial.printf("pulses=%u  edges=%u  pin=%d  window=%ums  rpm=%.0f\n", cnt, edges, digitalRead(TACH_PIN), dt, rpm);

  // วินิจฉัย: มอเตอร์สั่งหมุนแล้วแต่ไม่มีพัลส์เข้ามาเลย
  if (status == "running" && cnt == 0) {
    if (zeroWindows < 255) zeroWindows++;
    if (zeroWindows == 3) {
      Serial.println(edges == 0
        ? "!! ไม่มีสัญญาณเข้าขา TACH_PIN เลย (edges=0) — ตรวจสายสัญญาณ/GND ร่วม/ว่ามีเซนเซอร์รอบจริงหรือไม่"
        : "!! มีสัญญาณแต่ถูกกรองทิ้งหมด (edges>0, pulses=0) — สัญญาณถี่เกิน MIN_PULSE_US หรือเป็นสัญญาณรบกวน");
    }
    if (zeroWindows >= 3) {   // แสดงสาเหตุบน dashboard ด้วย (เดิมพิมพ์แค่ Serial)
      String pin = String(TACH_PIN);
      if (edges == 0) note = "⚠️ ไม่มีสัญญาณเข้าขา GPIO" + pin + " (edges=0) — ตรวจสายสัญญาณ/GND ร่วม/เซนเซอร์รอบ";
      else            note = "⚠️ มีสัญญาณเข้าขา GPIO" + pin + " แต่ถูกกรองทิ้ง (edges>0, pulses=0) — ตรวจสัญญาณรบกวน";
      tachWarn = true;
    }
  } else {
    zeroWindows = 0;
    if (tachWarn) { note = ""; tachWarn = false; }   // กลับมามีพัลส์แล้ว (หรือไม่ได้ running) ล้างคำเตือน tach
  }

  if (rpm > MAX_VALID_RPM) {
    // ค่าเป็นไปไม่ได้สำหรับพัดลม = เกือบแน่ใจว่าเป็นสัญญาณรบกวน -> ไม่เอามาใช้
    rpmInvalid = true;
    if (status == "running") note = "⚠️ อ่านสัญญาณ tach ผิดปกติ (สงสัยสัญญาณรบกวน/สายหลวม) — ตรวจการต่อสาย";
  } else {
    if (rpmInvalid && status == "running") note = "";
    rpmInvalid = false;
    lastRpm = rpm;
  }
  publishStatus();
}
