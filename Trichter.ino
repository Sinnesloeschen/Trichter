#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <Fonts/FreeSans9pt7b.h>

// ---------- Display ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// OLED Farbzonen
const int Y_HEADER = 0;     // gelb: y=0..15
const int Y_BLUE_START = 17;

// ---------- Pins ----------
const int PIN_FLOW = 27;
const int BUZZER_PIN = 25;
const int PIN_BUTTON = 26;
const int PIN_BATT = 34;

// ---------- Flow States ----------
enum State { IDLE, RUNNING, SHOW_RESULT };
State state = IDLE;

// ---------- Pages (korrekt!) ----------
enum Page {
  PAGE_MEASUREMENT,
  PAGE_LEADERBOARD
};
Page currentPage = PAGE_MEASUREMENT;

// ---------- Akku ----------
int battPct = -1;
bool battValid = false;
float battPctFiltered = -1.0f;

// ---------- Flow ----------
volatile uint32_t pulseCount = 0;
void IRAM_ATTR onFlowPulse() { pulseCount++; }

const float PULSES_PER_LITER = 450.0f;
const float ML_PER_PULSE = 1000.0f / PULSES_PER_LITER;

float hz_raw = 0, hz_ema = 0;
const float EMA_ALPHA = 0.25f;

const float HZ_START_THRESHOLD = 2.0f;
const float HZ_STOP_THRESHOLD = 1.0f;
const uint32_t STOP_HOLDOFF_MS = 800;

unsigned long lastSampleMs = 0;
uint32_t lastPulses = 0;

float flow_ml_s = 0;
double elapsed_s = 0;
double total_mL = 0;
unsigned long runLastBelowThreshMs = 0;

// Ergebnis
double final_time_s = 0;
double avg_ml_s = 0;
unsigned long showResultStartMs = 0;
const uint32_t RESULT_HOLD_MS = 5000;

// ---------- Leaderboard ----------
struct Measurement {
  float time_s;
  float avg_ml_s;
  float total_mL;
};

const size_t MAX_MEAS = 100;
Measurement measurements[MAX_MEAS];
size_t measCount = 0;

Preferences prefs;

// ---------- Button ----------
bool btnLast = true;
unsigned long btnLastChange = 0;
const uint32_t DEBOUNCE_MS = 40;

// ---------- Akku-Messung ----------
const float R_TOP = 100000.0f;
const float R_BOTTOM = 47000.0f;
const float V_CORR = 1.09f;

float readBatteryVoltage() {
  int raw = analogRead(PIN_BATT);

  float v_adc = (raw / 4095.0f) * 3.6f;
  float v_batt = v_adc * (R_TOP + R_BOTTOM) / R_BOTTOM;
  v_batt *= V_CORR;

  return v_batt;
}

void updateBattery() {
  float v = readBatteryVoltage();

  const float V_EMPTY = 3.20f;
  const float V_FULL = 4.10f;

  if (v < 2.5f || v > 4.5f) {
    battValid = false;
    return;
  }

  float pctRaw;
  if (v <= V_EMPTY) pctRaw = 0;
  else if (v >= V_FULL) pctRaw = 100;
  else pctRaw = (v - V_EMPTY) * 100.0f / (V_FULL - V_EMPTY);

  const float ALPHA = 0.2f;
  if (battPctFiltered < 0) battPctFiltered = pctRaw;
  else battPctFiltered = (1 - ALPHA) * battPctFiltered + ALPHA * pctRaw;

  int pctRounded = (int)(battPctFiltered + 0.5f);
  if (battPct >= 0 && abs(pctRounded - battPct) < 2) return;

  battPct = pctRounded;
  battValid = true;
}

// ---------- Helper ----------
const char* stateToName(State s) {
  switch (s) {
    case IDLE: return "IDLE";
    case RUNNING: return "RUNNING";
    case SHOW_RESULT: return "SHOW_RESULT";
  }
  return "?";
}

void beepOnce(int onMs = 80, int offMs = 60) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(onMs);
  digitalWrite(BUZZER_PIN, LOW);
  delay(offMs);
}
void beepTimes(int n) { for (int i=0; i<n; i++) beepOnce(); }

// ---------- Persistence ----------
void saveMeasurements() {
  prefs.begin("bier", false);
  prefs.putUInt("count", measCount);
  prefs.putBytes("data", measurements, measCount * sizeof(Measurement));
  prefs.end();
}

void loadMeasurements() {
  prefs.begin("bier", true);
  measCount = prefs.getUInt("count", 0);
  if (measCount > MAX_MEAS) measCount = MAX_MEAS;
  prefs.getBytes("data", measurements, measCount * sizeof(Measurement));
  prefs.end();
}

void addMeasurement(float t_s, float avg_s, float total) {
  if (measCount < MAX_MEAS) {
    measurements[measCount++] = { t_s, avg_s, total };
  } else {
    for (size_t i = 1; i < MAX_MEAS; i++)
      measurements[i - 1] = measurements[i];
    measurements[MAX_MEAS - 1] = { t_s, avg_s, total };
  }
  saveMeasurements();
}

void sortLeaderboard() {
  for (int i=1; i<measCount; i++) {
    Measurement key = measurements[i];
    int j = i-1;
    while (j >= 0 && measurements[j].time_s > key.time_s) {
      measurements[j+1] = measurements[j];
      j--;
    }
    measurements[j+1] = key;
  }
}

// ---------- Page Titles ----------
const char* currentPageTitle() {
  switch (currentPage) {
    case PAGE_MEASUREMENT: return "Messung";
    case PAGE_LEADERBOARD: return "Board";
  }
  return "???";
}

// ---------- Header ----------
void drawHeader() {
  // oberen 16 Pixel (gelber Bereich) löschen
  display.fillRect(0, 0, SCREEN_WIDTH, 16, SSD1306_BLACK);

  // --- Titel im Standardfont, TextSize 2 (ca. 14px hoch) ---
  display.setFont();        // Standardfont
  display.setTextSize(2);   // 2x -> passt in 0..15px Höhe
  display.setCursor(0, 0);  // oben links
  display.print(currentPageTitle());  // "Messung" / "Leaderboard"

  // --- Akkuanzeige rechts, kleiner ---
  display.setTextSize(1);
  String battStr = battValid ? (String(battPct) + "%") : "--%";

  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(battStr, 0, 0, &x1, &y1, &w, &h);

  display.setCursor(SCREEN_WIDTH - w, 0);
  display.print(battStr);
}



// ---------- Screens ----------
void drawIdle() {
  currentPage = PAGE_MEASUREMENT;

  display.clearDisplay();
  drawHeader();

  display.setFont();
  display.setCursor(0, Y_BLUE_START);
  display.print("Warte auf Fluss...");

  display.display();
}

void drawRunning() {
  currentPage = PAGE_MEASUREMENT;

  display.clearDisplay();
  drawHeader();

  display.setTextSize(2);
  display.setCursor(0, Y_BLUE_START);
  display.print(elapsed_s,2);
  display.print("s");

  display.setTextSize(1);
  display.setCursor(0, Y_BLUE_START + 20);
  display.print("mL: ");
  display.print(total_mL,1);

  display.setCursor(0, Y_BLUE_START + 32);
  display.print("Flow: ");
  display.print(flow_ml_s,1);
  display.print(" mL/s");

  display.display();
}

void drawResult() {
  currentPage = PAGE_MEASUREMENT;

  display.clearDisplay();
  drawHeader();

  display.setCursor(0, Y_BLUE_START);
  display.print("Gestoppte Zeit:");

  display.setTextSize(2);
  display.setCursor(0, Y_BLUE_START + 10);
  display.print(final_time_s,2);
  display.print("s");

  display.setTextSize(1);
  display.setCursor(0, Y_BLUE_START + 30);
  display.print("ØFlow: ");
  display.print(avg_ml_s,1);

  display.setCursor(72, Y_BLUE_START + 30);
  display.print("mL: ");
  display.print(total_mL,0);

  display.display();
}

void drawLeaderboard() {
  currentPage = PAGE_LEADERBOARD;

  display.clearDisplay();
  drawHeader();

  display.setFont();
  display.setCursor(0, Y_BLUE_START);

  int y = Y_BLUE_START + 12;
  int n = min((size_t)3, measCount);

  for (int i=0; i<n; i++) {
    auto &m = measurements[i];
    display.setCursor(0, y);
    display.print(i+1);
    display.print(") ");
    display.print(m.time_s,2);
    display.print("s  | ");

    display.print(m.avg_ml_s,0);
    display.print("ml/s | ");

    display.print(m.total_mL,0);
    display.print("mL");

    y += 12;
  }

  if (measCount == 0) {
    display.setCursor(0, Y_BLUE_START + 16);
    display.print("Keine Daten.");
  }

  display.display();
}

// ---------- Button ----------
void handleButton() {
  bool lvl = digitalRead(PIN_BUTTON);
  if (lvl != btnLast) {
    if (millis() - btnLastChange > DEBOUNCE_MS) {
      btnLast = lvl;
      btnLastChange = millis();
      if (lvl == LOW && state != RUNNING) {
        // Page toggle
        currentPage =
          (currentPage == PAGE_MEASUREMENT)
            ? PAGE_LEADERBOARD
            : PAGE_MEASUREMENT;
        beepOnce(40, 0);
      }
    }
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(22, 21);
  delay(100);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  pinMode(PIN_FLOW, INPUT_PULLUP);
  pinMode(PIN_BATT, INPUT);
  analogSetPinAttenuation(PIN_BATT, ADC_11db);

  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), onFlowPulse, FALLING);

  loadMeasurements();
  sortLeaderboard();

  drawIdle();
  lastSampleMs = millis();
}

// ---------- Loop ----------
void loop() {
  handleButton();

  static unsigned long lastBatt = 0;
  if (millis() - lastBatt >= 1000) {
    updateBattery();
    lastBatt = millis();
  }

  unsigned long now = millis();
  if (now - lastSampleMs >= 100) {
    float dt = (now - lastSampleMs) / 1000.0f;
    uint32_t p = pulseCount;
    uint32_t dp = p - lastPulses;

    hz_raw = dp / dt;
    hz_ema = (1 - EMA_ALPHA) * hz_ema + EMA_ALPHA * hz_raw;
    flow_ml_s = hz_ema * ML_PER_PULSE;

    total_mL = p * ML_PER_PULSE;
    lastPulses = p;
    lastSampleMs = now;

    if (currentPage == PAGE_LEADERBOARD) {
      drawLeaderboard();
      return;
    }

    switch (state) {
      case IDLE:
        if (hz_ema > HZ_START_THRESHOLD) {
          elapsed_s = 0;
          total_mL = 0;
          lastPulses = p;
          beepTimes(1);
          state = RUNNING;
        }
        drawIdle();
        break;

      case RUNNING:
        elapsed_s += dt;

        if (hz_ema < HZ_STOP_THRESHOLD) {
          if (runLastBelowThreshMs == 0) runLastBelowThreshMs = now;
          if (now - runLastBelowThreshMs >= STOP_HOLDOFF_MS) {
            final_time_s = elapsed_s;
            avg_ml_s = (elapsed_s > 0) ? total_mL / elapsed_s : 0;

            addMeasurement(final_time_s, avg_ml_s, total_mL);
            sortLeaderboard();

            beepTimes(2);
            state = SHOW_RESULT;
            showResultStartMs = now;
            runLastBelowThreshMs = 0;
          }
        } else {
          runLastBelowThreshMs = 0;
        }

        drawRunning();
        break;

      case SHOW_RESULT:
        drawResult();

        if (now - showResultStartMs > RESULT_HOLD_MS)
          state = IDLE;

        if (hz_ema > HZ_START_THRESHOLD) {
          elapsed_s = 0;
          total_mL = 0;
          lastPulses = pulseCount;
          beepTimes(1);
          state = RUNNING;
        }
        break;
    }
  }

  delay(5);
}
