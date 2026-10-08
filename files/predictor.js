/**
 * predictor.js — "AI วิเคราะห์ความผิดปกติ" (Predictive Maintenance Engine)
 * -----------------------------------------------------------
 *  - SUBSCRIBE ข้อมูลเซนเซอร์จาก   <PREFIX>/status
 *  - PUBLISH   ผลวิเคราะห์ไปที่    <PREFIX>/predict
 *  - PUBLISH   คำสั่งอัตโนมัติไปที่ <PREFIX>/command
 *              (เมื่อพบความเสี่ยงสูง จะสั่งเข้าโหมด "ซ่อมบำรุง" ให้เองทันที
 *               ไม่ต้องรอคนกด — แต่การกลับไป "ทำงานปกติ" ยังต้องให้คนยืนยันเอง)
 *
 * รัน:  node predictor.js <topic-prefix>
 * -----------------------------------------------------------
 */

const mqtt = require("mqtt");
const crypto = require("crypto");

const PREFIX = process.argv[2] || "mydemo123";
const BROKER_URL = "mqtt://broker.hivemq.com:1883";
const STATUS_TOPIC = `${PREFIX}/status`;
const PREDICT_TOPIC = `${PREFIX}/predict`;
const COMMAND_TOPIC = `${PREFIX}/command`;

console.log(`🧠 Predictive Maintenance Engine`);
console.log(`🔌 เชื่อมต่อ MQTT broker: ${BROKER_URL}`);
console.log(`📡 Topic prefix: "${PREFIX}"`);

const client = mqtt.connect(BROKER_URL, {
  clientId: "predictor-" + crypto.randomBytes(4).toString("hex"),
});

const HISTORY_LEN = 8;
const history = {};       // { machineId: [vibration, ...] }
const autoPending = new Set(); // กันไม่ให้สั่งซ่อมอัตโนมัติซ้ำระหว่างรอเครื่องเปลี่ยนสถานะ

client.on("connect", () => {
  console.log("✅ เชื่อมต่อ broker สำเร็จ กำลังวิเคราะห์ข้อมูลแบบเรียลไทม์...");
  client.subscribe(STATUS_TOPIC);
});

client.on("message", (topic, payload) => {
  if (topic !== STATUS_TOPIC) return;
  try {
    const machines = JSON.parse(payload.toString());
    const predictions = machines.map((m) => analyzeAndMaybeAct(m));
    client.publish(PREDICT_TOPIC, JSON.stringify(predictions), { retain: true, qos: 0 });
  } catch (e) {
    console.error("วิเคราะห์ข้อมูลผิดพลาด:", e.message);
  }
});

function analyzeAndMaybeAct(m) {
  const prediction = analyzeMachine(m);

  // เคลียร์สถานะ "รอดำเนินการ" เมื่อเครื่องไม่ได้ running แล้ว (แปลว่าคำสั่งก่อนหน้าถูกทำแล้ว)
  if (m.status !== "running") autoPending.delete(m.id);

  // ---- สั่งซ่อมบำรุงอัตโนมัติ เมื่อความเสี่ยงสูง และเครื่องยังทำงานอยู่ ----
  highCount[m.id] = prediction.risk === "high" ? (highCount[m.id] || 0) + 1 : 0;
  if (prediction.risk === "high" && highCount[m.id] >= HIGH_CONFIRM_N && m.status === "running" && !autoPending.has(m.id)) {
    autoPending.add(m.id);
    client.publish(
      COMMAND_TOPIC,
      JSON.stringify({ id: m.id, status: "maintenance", auto: true })
    );
    console.log(`🤖 AI สั่งหยุดเครื่อง "${m.name}" เพื่อซ่อมบำรุงอัตโนมัติ (${prediction.message})`);
  }

  return prediction;
}

// ---------- โหมด RPM (ใช้เมื่อเครื่องไม่มีเซนเซอร์สั่นสะเทือน เช่น ESP32 วัด tach อย่างเดียว) ----------
const CALIBRATION_N = 10;   // จำนวนตัวอย่างที่ใช้เรียนรู้ "รอบปกติ" (ต้องตรงกับ CALIB_SAMPLES ใน esp32_fan_node.ino)
const WARMUP_N = 3;         // ทิ้งตัวอย่างแรก ๆ หลังเริ่มทำงาน (มอเตอร์กำลังออกตัว RPM ยังไม่นิ่ง) ไม่เอามาคิด baseline
const HIGH_CONFIRM_N = 2;   // ต้องเจอความเสี่ยงสูงติดกันกี่ครั้ง ถึงจะสั่งหยุดอัตโนมัติ (กันค่ากระชากครั้งเดียว)
const rpmState = {};        // { machineId: { warm: 0, calib: [], baseline: null, hist: [] } }
const highCount = {};       // { machineId: จำนวนครั้งติดกันที่ความเสี่ยงสูง }

function scoreByVibration(m) {
  if (!history[m.id]) history[m.id] = [];
  const h = history[m.id];
  h.push(m.vibration);
  if (h.length > HISTORY_LEN) h.shift();

  let trend = 0;
  if (h.length >= 4) {
    const mid = Math.floor(h.length / 2);
    trend = avg(h.slice(mid)) - avg(h.slice(0, mid));
  }

  const VIBRATION_CRITICAL = 4.5;
  let health = 100 - (m.vibration / VIBRATION_CRITICAL) * 70 - Math.max(0, trend) * 60;
  return {
    health,
    trend,
    highMsg: `พบแนวโน้มการสั่นสะเทือนเพิ่มขึ้นต่อเนื่อง (${m.vibration.toFixed(2)} mm/s) คาดว่าจะเสียหายในไม่ช้า`,
    medMsg: `ค่าการสั่นสะเทือนเริ่มสูงกว่าปกติ (${m.vibration.toFixed(2)} mm/s)`,
    extra: { vibration: m.vibration },
  };
}

function scoreByRpm(m) {
  const st = rpmState[m.id] || (rpmState[m.id] = { warm: 0, calib: [], baseline: null, hist: [] });

  // ไม่ได้ running -> ลืมค่าเดิม แล้วเรียนรู้ใหม่ตอนกลับมาทำงาน (เช่น หลังซ่อมเสร็จ)
  if (m.status !== "running") {
    st.warm = 0; st.calib = []; st.baseline = null; st.hist = [];
    return { health: 100, trend: 0, extra: { rpm: m.rpm } };
  }

  // ช่วงอุ่นเครื่อง: ข้ามตัวอย่างแรก ๆ ที่ RPM ยังไม่นิ่ง ไม่งั้น baseline จะเพี้ยนแล้ว AI สั่งหยุดตอนเรียนรู้ครบ
  if (st.warm < WARMUP_N) {
    st.warm++;
    return {
      health: 100, trend: 0, calibrating: true,
      calibMsg: `กำลังอุ่นเครื่อง รอรอบนิ่ง (${st.warm}/${WARMUP_N})`,
      extra: { rpm: m.rpm },
    };
  }

  st.hist.push(m.rpm);
  if (st.hist.length > HISTORY_LEN) st.hist.shift();

  if (st.baseline === null) {
    st.calib.push(m.rpm);
    if (st.calib.length >= CALIBRATION_N) {
      const b = avg(st.calib);
      if (b < 1) {
        // ไม่มีสัญญาณรอบเลย (ยังไม่ต่อ tach / มอเตอร์ไม่หมุน) -> ประเมินไม่ได้ ห้ามหารด้วย 0
        st.calib = [];
        return {
          health: 100, trend: 0, calibrating: true,
          calibMsg: "ไม่พบสัญญาณรอบ (RPM = 0) — ตรวจการต่อ tach หรือสั่งให้มอเตอร์ทำงาน",
          extra: { rpm: m.rpm },
        };
      }
      st.baseline = b;
      console.log(`📏 ${m.name}: เรียนรู้รอบปกติได้ ${st.baseline.toFixed(0)} RPM`);
    } else {
      return {
        health: 100, trend: 0, calibrating: true,
        calibMsg: `กำลังเรียนรู้ค่า RPM ปกติ (${st.calib.length}/${CALIBRATION_N})`,
        extra: { rpm: m.rpm },
      };
    }
  }

  const base = st.baseline;
  // ความละเอียดของการนับ: พัลส์เพิ่ม/ลด 1 ครั้งเท่ากับกี่หน่วย rpm (RAW mode = 1, โหมดคำนวณจริง = 60)
  // ใช้เป็นเกณฑ์ยอมรับความคลาดเคลื่อนจากการนับเป็นจำนวนเต็ม ไม่งั้นพัลส์น้อย ๆ จะถูกมองว่ารอบตก
  const q = (m.pulses > 0 && m.rpm > 0) ? m.rpm / m.pulses : 0;
  const drop = Math.max(0, (base - m.rpm - 1.5 * q) / base);   // รอบตกจากปกติกี่ % (หักค่าคลาดเคลื่อนแล้ว)
  let trend = 0, jitter = 0;
  if (st.hist.length >= 4) {
    const mid = Math.floor(st.hist.length / 2);
    trend = (avg(st.hist.slice(mid)) - avg(st.hist.slice(0, mid))) / base;   // ติดลบ = รอบกำลังตก
    const mean = avg(st.hist);
    jitter = Math.max(0, Math.sqrt(avg(st.hist.map((x) => (x - mean) ** 2))) - q) / base;      // รอบแกว่งกี่ %
  }

  const health = 100 - drop * 250 - jitter * 400 - Math.max(0, -trend) * 200;
  const pct = (x) => (x * 100).toFixed(0);
  return {
    health,
    trend,
    highMsg: `รอบพัดลมผิดปกติ: ${m.rpm.toFixed(0)} RPM (ปกติ ~${base.toFixed(0)}, ตก ${pct(drop)}%, แกว่ง ${pct(jitter)}%) คาดว่าจะเสียหายในไม่ช้า`,
    medMsg: `รอบพัดลมเริ่มเบี่ยงจากปกติ: ${m.rpm.toFixed(0)} RPM (ปกติ ~${base.toFixed(0)})`,
    extra: { rpm: m.rpm, baselineRpm: Math.round(base) },
  };
}

function analyzeMachine(m) {
  const useRpm = typeof m.vibration !== "number" && typeof m.rpm === "number";
  const r = useRpm ? scoreByRpm(m) : scoreByVibration(m);
  const health = Math.round(Math.max(0, Math.min(100, r.health)));
  const trend = r.trend;

  let risk, message, action;
  if (m.status === "error") {
    risk = "failed";
    message = "เครื่องหยุดทำงานฉุกเฉินแล้ว";
    action = "ต้องซ่อมทันที (เสียโอกาสซ่อมเชิงป้องกัน)";
  } else if (m.status === "maintenance") {
    risk = "maintenance";
    message = "กำลังอยู่ในโหมดซ่อมบำรุง";
    action = "กดยืนยัน \"ซ่อมเสร็จแล้ว\" เมื่อดำเนินการเสร็จสิ้น";
  } else if (r.calibrating) {
    risk = "low";
    message = r.calibMsg;
    action = "ยังไม่ประเมินความเสี่ยง รอให้เรียนรู้ค่าปกติเสร็จก่อน";
  } else if (health < 40) {
    risk = "high";
    message = r.highMsg;
    action = "AI จะสั่งหยุดเครื่องเพื่อซ่อมบำรุงอัตโนมัติ";
  } else if (health < 70) {
    risk = "medium";
    message = r.medMsg;
    action = "ควรวางแผนตรวจสอบในรอบถัดไป";
  } else {
    risk = "low";
    message = "ค่าจากเซนเซอร์อยู่ในเกณฑ์ปกติ";
    action = "ไม่ต้องดำเนินการเพิ่มเติม";
  }

  return {
    id: m.id,
    name: m.name,
    healthScore: health,
    risk, // low | medium | high | failed | maintenance
    message,
    action,
    ...r.extra,
    trend: Number(trend.toFixed(3)),
    updatedAt: Date.now(),
  };
}

function avg(arr) {
  return arr.reduce((a, b) => a + b, 0) / arr.length;
}

client.on("error", (err) => console.error("❌ MQTT error:", err.message));
