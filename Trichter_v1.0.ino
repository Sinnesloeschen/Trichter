#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- Displaysettings ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#define OLED_ADDR 0x3C

// ---------- Pins ----------
const int PIN_FLOW   = 27;   // Flowsensor Signal (mit 10k Pull-Up auf 3.3V)
const int BUZZER_PIN = 25;   // aktiver Buzzer (über Transistor auf 5V), HIGH = an

// ---------- Flow-Messung ----------
volatile uint32_t pulseCount = 0;
void IRAM_ATTR onFlowPulse() { pulseCount++; }

// YF-S201 Richtwert: mL/s = Hz * (1000 / 450) = Hz * 2.2222
const float HZ_TO_ML_S = 1000.0f / 450.0f;

// Erkennung "Fluss vorhanden?"
const float HZ_START_THRESHOLD = 2.0f;    // > ~2 Hz = Start
const float HZ_STOP_THRESHOLD  = 1.0f;    // < ~1 Hz = (potenziell) Ende
const uint32_t STOP_HOLDOFF_MS = 800;     // so lange unter Schwelle => Ende

// ---------- Akku (optional) ----------
int battPct = -1; bool battValid = false; // Platzhalter

// ---------- Timer/Status ----------
enum State { IDLE, RUNNING, SHOW_RESULT };
State state = IDLE;

unsigned long lastSampleMs = 0;
uint32_t lastPulses = 0;
float flow_ml_s = 0.0f;

unsigned long runStartMs = 0;
unsigned long runLastBelowThreshMs = 0;
double elapsed_s = 0.0;       // laufende Zeit
double total_mL  = 0.0;       // integriertes Volumen

// Ergebnis
double final_time_s = 0.0;
double avg_ml_s     = 0.0;

void beepOnce(int onMs=120, int offMs=120){
  digitalWrite(BUZZER_PIN, HIGH); delay(onMs);
  digitalWrite(BUZZER_PIN, LOW);  delay(offMs);
}
void beepTimes(int n, int onMs=120, int offMs=120){
  for(int i=0;i<n;i++) beepOnce(onMs, offMs);
}

void drawHeader() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Bierotrauma v1.0");

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
  // Timer gross zentriert
  char tbuf[32];
  dtostrf(elapsed_s, 0, 2, tbuf);   // 2 Nachkommastellen
  display.setTextSize(2);
  display.setCursor(0, 22);
  display.print(tbuf);
  display.print(" s");

  // optional klein aktuelle Flussrate
  display.setTextSize(1);
  display.setCursor(0, 50);
  display.print("Flow: ");
  display.print(flow_ml_s, 1);
  display.print(" mL/s");
  display.display();
}

void drawResult() {
  display.clearDisplay();
  drawHeader();

  display.setTextSize(1);
  display.setCursor(0, 16);
  display.print("Gestoppte Zeit:");

  display.setTextSize(2);
  display.setCursor(0, 28);
  display.print(final_time_s, 2);
  display.print(" s");

  display.setTextSize(1);
  display.setCursor(0, 52);
  display.print("Ø Flow: ");
  display.print(avg_ml_s, 1);
  display.print(" mL/s");
  display.display();
}

void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(21, 22);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  pinMode(PIN_FLOW, INPUT_PULLUP); // zusätzlich zum externen 10k Pull-Up ok
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), onFlowPulse, FALLING);

  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("Bierotrauma v1.0");
  display.display();
  delay(400);

  lastSampleMs = millis();
  drawIdle();
}

void loop() {
  unsigned long now = millis();

  // --- Abtastung alle 100 ms für glatte Zeit ---
  if (now - lastSampleMs >= 100) {
    uint32_t p = pulseCount;
    uint32_t dp = p - lastPulses;
    float dt = (now - lastSampleMs) / 1000.0f;

    float hz = dp / dt;
    flow_ml_s = hz * HZ_TO_ML_S;

    lastPulses  = p;
    lastSampleMs = now;

    // --- Zustandsautomat ---
    switch(state) {
      case IDLE:
        if (hz > HZ_START_THRESHOLD) {
          // Start
          runStartMs = now;
          elapsed_s  = 0.0;
          total_mL   = 0.0;
          beepTimes(1);                // 1× piep
          state = RUNNING;
        }
        drawIdle();
        break;

      case RUNNING:
        elapsed_s += dt;
        total_mL  += flow_ml_s * dt;   // mL/s * s = mL

        // Ende erkennen: unter Stopp-Schwelle für STOP_HOLDOFF_MS
        if (hz < HZ_STOP_THRESHOLD) {
          if (runLastBelowThreshMs == 0) runLastBelowThreshMs = now;
          if ((now - runLastBelowThreshMs) >= STOP_HOLDOFF_MS) {
            // Stop
            final_time_s = elapsed_s;
            avg_ml_s = (elapsed_s > 0.0) ? (total_mL / elapsed_s) : 0.0;
            beepTimes(2);              // 2× piep
            state = SHOW_RESULT;
            runLastBelowThreshMs = 0;
          }
        } else {
          runLastBelowThreshMs = 0;    // wieder Fluss da -> Reset Holdoff
        }

        drawRunning();
        break;

      case SHOW_RESULT:
        drawResult();
        // Zurück in IDLE, wenn wieder Fluss startet:
        if (hz > HZ_START_THRESHOLD) {
          runStartMs = now;
          elapsed_s  = 0.0;
          total_mL   = 0.0;
          beepTimes(1);
          state = RUNNING;
        }
        break;
    }
  }

  // kleine UI-Entlastung
  delay(10);
}
