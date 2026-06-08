// === Heart-rate sensor (PulseSensor Playground) ===
// Install the library via Arduino IDE -> Sketch -> Include Library
// -> Manage Libraries... -> search "PulseSensor Playground"
// USE_ARDUINO_INTERRUPTS=true tells the library to sample on Timer2,
// which doesn't conflict with our Timer1-based pump PWM.
#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>

// === Pin assignments ===
const int PUMP1_PIN     = 9;   // Timer1 OC1A
const int PUMP2_PIN     = 10;  // Timer1 OC1B
const int PRESSURE1_PIN = A0;
const int PRESSURE2_PIN = A1;
const int HEART_PIN     = A2;  // PulseSensor signal lead (purple wire)

// === Pump state (controllable from laptop) ===
enum Mode { MODE_OFF, MODE_CONTINUOUS, MODE_HEARTBEAT, MODE_ALTERNATE };
Mode currentMode = MODE_OFF;

int pump1Duty = 255;        // 0–255 — only used in CONT mode for slow-PWM
int pump2Duty = 255;
int heartBPM  = 60;
unsigned long pump2OffsetMs = 70;  // for heartbeat layering

// === Slow-PWM power control ===
// The pumps have a large deadband — below ~94% fast-PWM duty they don't run
// at all. To get fine-grained power control, CONT mode runs the pumps on a
// slow ON/OFF cycle at full power instead. With a 1-second cycle:
//   duty=255 -> 1000 ms on / 0 ms off    -> 100% time-averaged power
//   duty=128 ->  502 ms on / 498 ms off  ->  50%
//   duty=26  ->  102 ms on / 898 ms off  ->  10%
// The fluid loop's hydraulic inertia smooths this into a 1 Hz pressure
// ripple; the Python side's 2 s rolling mean averages it out for display.
//
// HEART and ALT modes always run at full 255 when their pulses are active —
// the duty value is ignored in those modes.
const unsigned long POWER_CYCLE_MS = 1000;

// === Heartbeat timing (computed from BPM) ===
unsigned long cycleMs;
unsigned long lubMs, systolicGapMs, dubMs, diastolicMs;

void recalcHeartbeat() {
  cycleMs       = 60000UL / heartBPM;
  lubMs         = cycleMs * 14 / 100;
  systolicGapMs = cycleMs * 20 / 100;
  dubMs         = cycleMs * 10 / 100;
  diastolicMs   = cycleMs - lubMs - systolicGapMs - dubMs;
}

bool isHeartPulseActive(unsigned long phase) {
  if (phase < lubMs) return true;
  unsigned long dubStart = lubMs + systolicGapMs;
  if (phase >= dubStart && phase < dubStart + dubMs) return true;
  return false;
}

// === Alternate timing ===
unsigned long alternateLastSwitch = 0;
bool alternatePump1On = true;
const unsigned long ALTERNATE_INTERVAL_MS = 2000;

// === Pressure sensor (5 PSI Flylin transducer) ===
const float V_OFFSET = 0.5;
const float V_SPAN   = 4.0;
const float MAX_PSI  = 5.0;
const float ADC_TO_V = 5.0 / 1023.0;
const int OVERSAMPLE_N = 9;

float p1_smoothed = 0.0;
float p2_smoothed = 0.0;
bool firstSample = true;
const float EMA_ALPHA = 0.30;
unsigned long lastSampleMs = 0;
const unsigned long SAMPLE_INTERVAL_MS = 5;  // 200 Hz internal sampling

// === Heart rate sensor with IBI-median filtering ===
// The PulseSensor library's getBeatsPerMinute() returns BPM from the SINGLE
// most recent inter-beat interval. One missed beat doubles IBI -> BPM
// halves; one spurious detection halves IBI -> BPM doubles. We ignore the
// library's BPM and compute our own from a 7-deep IBI buffer:
//   1. range-check each IBI to physiological 300..2000 ms,
//   2. once we have a stable baseline (>=4 accepted beats), reject IBIs
//      that deviate >40% from the running median,
//   3. report BPM = 60000 / median(buffer) only after 3+ accepted beats.
// Median is robust to occasional bad beats — a single 80->200 spike
// doesn't move the reported value.
const int HR_THRESHOLD_DEFAULT = 550;
int hrThreshold = HR_THRESHOLD_DEFAULT;
PulseSensorPlayground pulseSensor;
int measuredBPM = 0;
unsigned long lastBeatMs = 0;
const unsigned long HR_TIMEOUT_MS = 5000;

const int IBI_BUF_SIZE = 7;
unsigned long ibiBuffer[IBI_BUF_SIZE];
int ibiBufCount = 0;
int ibiBufIdx = 0;
const unsigned long IBI_MIN_MS = 300;   // 200 BPM ceiling
const unsigned long IBI_MAX_MS = 2000;  // 30 BPM floor

unsigned long ibiBufferMedian() {
  unsigned long sorted[IBI_BUF_SIZE];
  int n = ibiBufCount;
  for (int i = 0; i < n; i++) sorted[i] = ibiBuffer[i];
  for (int i = 1; i < n; i++) {
    unsigned long key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j--;
    }
    sorted[j + 1] = key;
  }
  return sorted[n / 2];
}

void resetHRState() {
  measuredBPM = 0;
  ibiBufCount = 0;
  ibiBufIdx = 0;
  lastBeatMs = 0;
}

void insertionSort(uint16_t *a, int n) {
  for (int i = 1; i < n; i++) {
    uint16_t key = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > key) {
      a[j + 1] = a[j];
      j--;
    }
    a[j + 1] = key;
  }
}

float readPressurePSI(int pin) {
  uint16_t samples[OVERSAMPLE_N];
  analogRead(pin);  // dummy read for MUX settling
  for (int i = 0; i < OVERSAMPLE_N; i++) {
    samples[i] = analogRead(pin);
  }
  insertionSort(samples, OVERSAMPLE_N);
  uint16_t median = samples[OVERSAMPLE_N / 2];
  float voltage = median * ADC_TO_V;
  float psi = (voltage - V_OFFSET) / V_SPAN * MAX_PSI;
  return psi < 0 ? 0 : psi;
}

void updateSensors() {
  unsigned long now = millis();
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = now;

  float raw1 = readPressurePSI(PRESSURE1_PIN);
  float raw2 = readPressurePSI(PRESSURE2_PIN);

  if (firstSample) {
    p1_smoothed = raw1;
    p2_smoothed = raw2;
    firstSample = false;
  } else {
    p1_smoothed = EMA_ALPHA * raw1 + (1.0 - EMA_ALPHA) * p1_smoothed;
    p2_smoothed = EMA_ALPHA * raw2 + (1.0 - EMA_ALPHA) * p2_smoothed;
  }
}

void updateHeartRate() {
  if (pulseSensor.sawStartOfBeat()) {
    unsigned long now = millis();
    if (lastBeatMs > 0) {
      unsigned long ibi = now - lastBeatMs;
      bool accept = (ibi >= IBI_MIN_MS && ibi <= IBI_MAX_MS);

      // Outlier rejection — only after a stable baseline so we can build
      // up the buffer initially.
      if (accept && ibiBufCount >= 4) {
        unsigned long median = ibiBufferMedian();
        if (ibi < (median * 6) / 10 || ibi > (median * 14) / 10) {
          accept = false;
        }
      }

      if (accept) {
        ibiBuffer[ibiBufIdx] = ibi;
        ibiBufIdx = (ibiBufIdx + 1) % IBI_BUF_SIZE;
        if (ibiBufCount < IBI_BUF_SIZE) ibiBufCount++;
        if (ibiBufCount >= 3) {
          unsigned long medianIBI = ibiBufferMedian();
          measuredBPM = (int)(60000UL / medianIBI);
        }
      }
    }
    lastBeatMs = now;
  }
  // Reset on prolonged signal loss
  if (lastBeatMs > 0 && (millis() - lastBeatMs) > HR_TIMEOUT_MS) {
    resetHRState();
  }
}

// === Serial reporting ===
unsigned long lastReportMs = 0;
const unsigned long REPORT_INTERVAL_MS = 20;  // 50 Hz

const char* modeToString(Mode m) {
  switch (m) {
    case MODE_OFF:        return "OFF";
    case MODE_CONTINUOUS: return "CONT";
    case MODE_HEARTBEAT:  return "HEART";
    case MODE_ALTERNATE:  return "ALT";
  }
  return "?";
}

void sendStatus() {
  // Format: DATA,p1,p2,mode,p1_duty,p2_duty,bpm_set,bpm_meas
  Serial.print("DATA,");
  Serial.print(p1_smoothed, 3); Serial.print(",");
  Serial.print(p2_smoothed, 3); Serial.print(",");
  Serial.print(modeToString(currentMode)); Serial.print(",");
  Serial.print(pump1Duty); Serial.print(",");
  Serial.print(pump2Duty); Serial.print(",");
  Serial.print(heartBPM); Serial.print(",");
  Serial.println(measuredBPM);
}

// === Command parsing ===
String inputBuffer = "";

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "OFF") {
    currentMode = MODE_OFF;
  } else if (cmd == "CONT") {
    currentMode = MODE_CONTINUOUS;
  } else if (cmd == "HEART") {
    currentMode = MODE_HEARTBEAT;
    recalcHeartbeat();
  } else if (cmd == "ALT") {
    currentMode = MODE_ALTERNATE;
    alternateLastSwitch = millis();
    alternatePump1On = true;
  } else if (cmd.startsWith("P1=")) {
    pump1Duty = constrain(cmd.substring(3).toInt(), 0, 255);
  } else if (cmd.startsWith("P2=")) {
    pump2Duty = constrain(cmd.substring(3).toInt(), 0, 255);
  } else if (cmd.startsWith("BPM=")) {
    heartBPM = constrain(cmd.substring(4).toInt(), 30, 180);
    recalcHeartbeat();
  } else if (cmd.startsWith("HRT=")) {
    // Runtime tuning of the PulseSensor threshold (typical 500-580).
    // Useful if the sensor is jittery or won't detect at all.
    int t = constrain(cmd.substring(4).toInt(), 300, 800);
    hrThreshold = t;
    pulseSensor.setThreshold(t);
    resetHRState();
  } else if (cmd == "STATUS") {
    sendStatus();
    return;
  } else {
    Serial.print("ERR,Unknown command: ");
    Serial.println(cmd);
    return;
  }

  Serial.print("OK,");
  Serial.println(cmd);
}

void readCommandsFromSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (inputBuffer.length() > 0) {
        handleCommand(inputBuffer);
        inputBuffer = "";
      }
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }
}

// === Pump control logic ===
void applyPumpControl() {
  switch (currentMode) {
    case MODE_OFF:
      analogWrite(PUMP1_PIN, 0);
      analogWrite(PUMP2_PIN, 0);
      break;

    case MODE_CONTINUOUS: {
      // Slow-PWM at POWER_CYCLE_MS period. Pump is FULL ON for the first
      // (duty/255 * POWER_CYCLE_MS) ms of each cycle, OFF for the rest.
      // Lets us hit average powers below the fast-PWM deadband.
      unsigned long now = millis();
      unsigned long phase = now % POWER_CYCLE_MS;
      unsigned long on1 = (POWER_CYCLE_MS * (unsigned long)pump1Duty) / 255UL;
      unsigned long on2 = (POWER_CYCLE_MS * (unsigned long)pump2Duty) / 255UL;
      analogWrite(PUMP1_PIN, phase < on1 ? 255 : 0);
      analogWrite(PUMP2_PIN, phase < on2 ? 255 : 0);
      break;
    }

    case MODE_HEARTBEAT: {
      // Pulses run at full 255 — duty ignored. Use CONT mode for power
      // control; a 140 ms lub pulse can't render intermediate amplitude.
      unsigned long now = millis();
      unsigned long phase1 = now % cycleMs;
      unsigned long phase2 = (now + cycleMs - pump2OffsetMs) % cycleMs;
      analogWrite(PUMP1_PIN, isHeartPulseActive(phase1) ? 255 : 0);
      analogWrite(PUMP2_PIN, isHeartPulseActive(phase2) ? 255 : 0);
      break;
    }

    case MODE_ALTERNATE: {
      unsigned long now = millis();
      if (now - alternateLastSwitch >= ALTERNATE_INTERVAL_MS) {
        alternatePump1On = !alternatePump1On;
        alternateLastSwitch = now;
      }
      analogWrite(PUMP1_PIN, alternatePump1On ? 255 : 0);
      analogWrite(PUMP2_PIN, alternatePump1On ? 0   : 255);
      break;
    }
  }
}

void setup() {
  pinMode(PUMP1_PIN, OUTPUT);
  pinMode(PUMP2_PIN, OUTPUT);

  // Boost Timer1 PWM frequency to ~31.25 kHz (only matters when we DO
  // analogWrite at intermediate values, which now only happens during
  // the "off" portion of slow-PWM — i.e., not at all in CONT, but full
  // 255 during pulses in HEART/ALT).
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;

  analogWrite(PUMP1_PIN, 0);
  analogWrite(PUMP2_PIN, 0);

  Serial.begin(115200);

  // Pulse sensor — Timer2 sampling, doesn't conflict with our Timer1 PWM
  pulseSensor.analogInput(HEART_PIN);
  pulseSensor.setThreshold(hrThreshold);
  pulseSensor.begin();

  recalcHeartbeat();
  Serial.println("READY");
}

void loop() {
  readCommandsFromSerial();
  applyPumpControl();
  updateSensors();
  updateHeartRate();

  unsigned long now = millis();
  if (now - lastReportMs >= REPORT_INTERVAL_MS) {
    sendStatus();
    lastReportMs = now;
  }
}
