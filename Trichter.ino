#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>

// ---------- Display ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#define OLED_ADDR 0x3C

// ---------- Pins ----------
const int PIN_FLOW    = 27;   // Flowsensor Signal (mit 10k Pull-Up auf 3.3V)
const int BUZZER_PIN  = 25;   // aktiver Buzzer (über Transistor auf 5V), HIGH = an
const int PIN_BUTTON  = 26;   // Taster nach GND, interner Pull-Up aktiv

// ---------- Zustände (ganz nach oben!) ----------
enum State { IDLE, RUNNING, SHOW_RESULT };
State state = IDLE;

// ---------- Flow-Messung ----------
volatile uint32_t pulseCount = 0;
void IRAM_ATTR onFlowPulse() {
  // Hinweis: ++ bei volatile erzeugt eine Warnung, ist hier okay.
  pulseCount += 1; // vermeidet -Wvolatile Warnung mit ++
}

// --- Flow-Kalibrierung ---
const float PULSES_PER_LITER = 450.0f;         // kalibrierbar
const float ML_PER_PULSE     = 1000.0f / PULSES_PER_LITER;

// --- Glättung (EMA) ---
float hz_raw = 0.0f;
float hz_ema = 0.0f;
const float EMA_ALPHA = 0.25f;                 // 0..1 (größer = reaktiver)

// Erkennung "Fluss vorhanden?"
const float HZ_START_THRESHOLD = 2.0f;         // Start bei > ~2 Hz
const float HZ_STOP_THRESHOLD  = 1.0f;         // Ende bei < ~1 Hz (mit Holdoff)
const uint32_t STOP_HOLDOFF_MS = 800;

// ---------- Akku (optional) ----------
int battPct = -1; bool battValid = false; // Platzhalter

// ---------- Zeit/Flow ----------
unsigned long lastSampleMs = 0;
uint32_t lastPulses = 0;
float flow_ml_s = 0.0f;

unsigned long runStartMs = 0;
unsigned long runLastBelowThreshMs = 0;
double elapsed_s = 0.0;
double total_mL  = 0.0;

// Ergebnis
double final_time_s = 0.0;
double avg_ml_s     = 0.0;

unsigned long showResultStartMs = 0;
const uint32_t RESULT_HOLD_MS   = 5000;

// ---------- Messungs-Speicher (Leaderboard) ----------
struct Measurement {
  float time_s;
  float avg_ml_s;
  float total_mL;
};
const size_t MAX_MEAS = 100;
Measurement measurements[MAX_MEAS];
size_t measCount = 0;

Preferences prefs;              // NVS
const char* NVS_NS = "biertrichter";
const char* NVS_KEY_COUNT = "count";
const char* NVS_KEY_DATA  = "data";

// ---------- Anzeige-Modi ----------
enum ScreenMode { SCREEN_CURRENT, SCREEN_LEADERBOARD };
ScreenMode screenMode = SCREEN_CURRENT;

// ---------- Button (Entprellung) ----------
bool btnLast = true;            // Pull-Up: true = nicht gedrückt
unsigned long btnLastChange = 0;
const uint32_t DEBOUNCE_MS = 40;

// ---------- Hilfsfunktionen ----------
const char* stateToName(State s) {
  switch(s){
    case IDLE:        return "IDLE";
    case RUNNING:     return "RUNNING";
    case SHOW_RESULT: return "SHOW_RESULT";
  }
  return "?";
}

void beepOnce(int onMs=100, int offMs=80){
  digitalWrite(BUZZER_PIN, HIGH); delay(onMs);
  digitalWrite(BUZZER_PIN, LOW);  delay(offMs);
}
void beepTimes(int n, int onMs=100, int offMs=80){
  for(int i=0;i<n;i++) beepOnce(onMs, offMs);
}

// --- Persistenz ---
void saveMeasurements() {
  prefs.begin(NVS_NS, false);
  prefs.putUInt(NVS_KEY_COUNT, (uint32_t)measCount);
  prefs.putBytes(NVS_KEY_DATA, measurements, sizeof(Measurement) * measCount);
  prefs.end();
}
void loadMeasurements() {
  prefs.begin(NVS_NS, true);
  uint32_t count = prefs.getUInt(NVS_KEY_COUNT, 0);
  if (count > MAX_MEAS) count = MAX_MEAS;
  size_t need = sizeof(Measurement) * count;
  if (need > 0) {
    size_t got = prefs.getBytes(NVS_KEY_DATA, measurements, need);
    if (got != need) count = 0; // inkonsistent
  }
  measCount = count;
  prefs.end();
}
void addMeasurement(float t_s, float avg_ml_s_, float total_mL_){
  if (measCount < MAX_MEAS) {
    measurements[measCount++] = {t_s, avg_ml_s_, total_mL_};
  } else {
    for (size_t i=1;i<MAX_MEAS;i++) measurements[i-1] = measurements[i];
    measurements[MAX_MEAS-1] = {t_s, avg_ml_s_, total_mL_};
  }
  saveMeasurements();
}
void sortLeaderboard() {
  for (size_t i=1; i<measCount; i++){
    Measurement key = measurements[i];
    int j = (int)i - 1;
    while (j >= 0 && measurements[j].time_s > key.time_s) {
      measurements[j+1] = measurements[j];
      j--;
    }
    measurements[j+1] = key;
  }
}

// ---------- Display ----------
void drawHeader() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Bierotrauma v1.2");
  String btxt = battValid ? (String(battPct) + "%") : String("--%");
  int16_t x1,y1; uint16_t w,h;
  display.getTextBounds(btxt, 0,0, &x1,&y1,&w,&h);
  display.setCursor(SCREEN_WIDTH - w, 0);
  display.print(btxt);
}
void drawIdle() {
  display.clearDisplay();
  drawHeader();
  display.setTextSize(1);
  display.setCursor(0, 16);
  display.print("Bereit. Warte auf Fluss...");
  display.setCursor(0, 30);
  display.print("Flow: ");
  display.print(flow_ml_s, 1);
  display.print(" mL/s");
  display.display();
}
void drawRunning() {
  display.clearDisplay();
  drawHeader();
  char tbuf[24];
  dtostrf(elapsed_s, 0, 2, tbuf);
  display.setTextSize(2);
  display.setCursor(0, 18);
  display.print(tbuf);
  display.print(" s");

  display.setTextSize(1);
  display.setCursor(0, 42);
  display.print("mL: ");
  display.print(total_mL, 1);

  display.setCursor(0, 54);
  display.print("Flow: ");
  display.print(flow_ml_s, 1);
  display.print(" mL/s");
  display.display();
}
void drawResult() {
  display.clearDisplay();
  drawHeader();
  display.setTextSize(1);
  display.setCursor(0, 14);
  display.print("Gestoppte Zeit:");
  display.setTextSize(2);
  display.setCursor(0, 26);
  display.print(final_time_s, 2);
  display.print(" s");

  display.setTextSize(1);
  display.setCursor(0, 46);
  display.print("ØFlow: ");
  display.print(avg_ml_s, 1);
  display.print(" mL/s");

  display.setCursor(72, 46);
  display.print("mL: ");
  display.print(total_mL, 0);
  display.display();
}
void drawLeaderboard() {
  display.clearDisplay();
  drawHeader();
  display.setTextSize(1);
  display.setCursor(0, 12);
  display.print("Leaderboard (Bestzeiten)");

  int y = 24;
  int shown = min((size_t)3, measCount);
  for (int i=0; i<shown; i++){
    const Measurement& m = measurements[i];
    display.setCursor(0, y);
    display.print(i+1);
    display.print(") ");
    display.print(m.time_s, 2);
    display.print("s  ");
    display.print(m.avg_ml_s, 0);
    display.print("mL/s  ");
    display.print(m.total_mL, 0);
    display.print("mL");
    y += 12;
  }
  if (measCount == 0) {
    display.setCursor(0, 28);
    display.print("Noch keine Messungen.");
  }
  display.display();
}

// ---------- Button ----------
void handleButton() {
  bool nowLevel = digitalRead(PIN_BUTTON); // HIGH=offen, LOW=gedrückt
  if (nowLevel != btnLast) {
    unsigned long t = millis();
    if (t - btnLastChange >= DEBOUNCE_MS) {
      btnLast = nowLevel;
      btnLastChange = t;
      if (nowLevel == LOW) { // Press
        if (state != RUNNING) {
          screenMode = (screenMode == SCREEN_CURRENT) ? SCREEN_LEADERBOARD : SCREEN_CURRENT;
          beepOnce(40,0);
        }
      }
    }
  }
}

// ---------- Setup & Loop ----------
void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(PIN_BUTTON, INPUT_PULLUP);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 not found @0x3C (try 0x3D or check wiring)");
    while(true){ delay(1000); }
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("Bierotrauma v1.2");
  display.display();
  delay(300);

  pinMode(PIN_FLOW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), onFlowPulse, FALLING);

  loadMeasurements();
  sortLeaderboard();

  lastSampleMs = millis();
  drawIdle();
}

void loop() {
  handleButton();
  unsigned long now = millis();

  // --- Abtastung alle 100 ms ---
  if (now - lastSampleMs >= 100) {
    uint32_t p  = pulseCount;
    uint32_t dp = p - lastPulses;
    float dt    = (now - lastSampleMs) / 1000.0f;

    hz_raw = (dt > 0) ? (dp / dt) : 0.0f;
    hz_ema = (1.0f - EMA_ALPHA) * hz_ema + EMA_ALPHA * hz_raw;

    total_mL = p * ML_PER_PULSE;
    flow_ml_s = hz_ema * ML_PER_PULSE;

    lastPulses   = p;
    lastSampleMs = now;

    switch(state) {
      case IDLE:
        // Serial Debug
        Serial.print("Hz: ");   Serial.print(hz_ema, 2);
        Serial.print(" | Flow: "); Serial.print(flow_ml_s, 1);
        Serial.print(" | State: "); Serial.println(stateToName(state));

        if (hz_ema > HZ_START_THRESHOLD) {
          runStartMs = now;
          elapsed_s  = 0.0;
          total_mL   = 0.0;
          lastPulses = p;
          beepTimes(1);
          state = RUNNING;
          screenMode = SCREEN_CURRENT;
        }
        if (screenMode == SCREEN_LEADERBOARD) drawLeaderboard();
        else drawIdle();
        break;

      case RUNNING:
        elapsed_s += dt;

        // Serial Debug
        Serial.print("Hz: ");   Serial.print(hz_ema, 2);
        Serial.print(" | Flow: "); Serial.print(flow_ml_s, 1);
        Serial.print(" | mL: ");   Serial.print(total_mL, 1);
        Serial.print(" | t: ");    Serial.print(elapsed_s, 2);
        Serial.print(" | State: "); Serial.println(stateToName(state));

        if (hz_ema < HZ_STOP_THRESHOLD) {
          if (runLastBelowThreshMs == 0) runLastBelowThreshMs = now;
          if ((now - runLastBelowThreshMs) >= STOP_HOLDOFF_MS) {
            final_time_s = elapsed_s;
            avg_ml_s = (elapsed_s > 0.0) ? (total_mL / elapsed_s) : 0.0;

            addMeasurement(final_time_s, avg_ml_s, total_mL);
            sortLeaderboard();

            beepTimes(2);
            state = SHOW_RESULT;
            runLastBelowThreshMs = 0;
            showResultStartMs = now;
          }
        } else {
          runLastBelowThreshMs = 0;
        }
        drawRunning();
        break;

      case SHOW_RESULT:
        // Serial Debug
        Serial.print("Hz: ");   Serial.print(hz_ema, 2);
        Serial.print(" | Flow: "); Serial.print(flow_ml_s, 1);
        Serial.print(" | FINAL t: "); Serial.print(final_time_s, 2);
        Serial.print(" | FINAL mL: "); Serial.print(total_mL, 1);
        Serial.print(" | AVG: "); Serial.print(avg_ml_s, 1);
        Serial.print(" | State: "); Serial.println(stateToName(state));

        if (screenMode == SCREEN_LEADERBOARD) drawLeaderboard();
        else drawResult();

        if (now - showResultStartMs >= RESULT_HOLD_MS && screenMode == SCREEN_CURRENT) {
          state = IDLE;
        }
        if (hz_ema > HZ_START_THRESHOLD) {
          runStartMs = now;
          elapsed_s  = 0.0;
          total_mL   = 0.0;
          lastPulses = pulseCount;
          beepTimes(1);
          state = RUNNING;
          screenMode = SCREEN_CURRENT;
        }
        break;
    }
  }
  delay(5);
}
