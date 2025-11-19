#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
  Serial.begin(115200);
  delay(200);

  // I2C Start
  Wire.begin(22, 21);  
  delay(100);

  Serial.println("Starting OLED...");

  // SSD1306 versuchen (0x3C oder 0x3D)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED nicht auf 0x3C gefunden. Probiere 0x3D...");
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
      Serial.println("Kein OLED gefunden! Checke Kabel / Adresse.");
      while (true) delay(1000);
    }
  }

  // Display löschen
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Testtext
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("OLED OK");

  display.setTextSize(1);
  display.setCursor(0, 30);
  display.println("I2C running");

  display.display();
}

void loop() {}
