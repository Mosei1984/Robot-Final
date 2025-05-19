#include "display.h"
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include "robot_bitmap.h" // ← Hier ist dein externes Bitmap drin!

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Initialisiert das Display
void initDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 konnte nicht gestartet werden"));
    return;
  }
  display.clearDisplay();
  display.display();
  showStartupScreen();
}

// Bitmap beim Start anzeigen
void showStartupScreen() {
  display.clearDisplay();
  drawBitmap();
  display.setCursor(0, 56);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println(F("6DOF Robot Controller"));
  display.display();
  delay(2000);
}

// Zeigt Statuszeile oben
void showStatus(const String &text) {
  display.fillRect(0, 0, SCREEN_WIDTH, 10, SSD1306_BLACK); // alte löschen
  display.setCursor(0, 0);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.println(text);
  display.display();
}

// Gemeinsame Funktion zur Anzeige der Gelenkwinkel
void showJointValues(const float jointAngles[], int jointCount) {
  display.clearDisplay();
  for (int i = 0; i < jointCount; i++) {
    display.setCursor(0, 10 + i * 10);
    display.setTextSize(1);
    display.printf("Achse %d: %.1f°\n", i + 1, jointAngles[i]);
  }
  display.display();
}

// Menüanzeige
void showMenu(const String* options, int optionCount, int selectedIndex) {
  display.clearDisplay();
  for (int i = 0; i < optionCount; i++) {
    display.setCursor(0, i * 10);
    if (i == selectedIndex) {
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.print(options[i]);
  }
  display.display();
}

// Display komplett löschen
void clearDisplay() {
  display.clearDisplay();
  display.display();
}

// Platzhalter-Bitmap aus externer Datei anzeigen
void drawBitmap() {
  display.drawBitmap(
    (SCREEN_WIDTH - 64) / 2, (SCREEN_HEIGHT - 64) / 2,
    robotArmBitmap, 64, 64, SSD1306_WHITE);
  display.display();
}
