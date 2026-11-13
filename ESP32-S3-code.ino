
#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include "pin_config.h"
#include <Wire.h>
#include <WiFi.h>

// =====================================================
// DISPLAY INITIALIZATION
// =====================================================

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS,
  LCD_SCLK,
  LCD_SDIO0,
  LCD_SDIO1,
  LCD_SDIO2,
  LCD_SDIO3
);

Arduino_GFX *gfx = new Arduino_CO5300(
  bus,
  LCD_RESET,
  0,
  LCD_WIDTH,
  LCD_HEIGHT,
  22,
  0,
  0,
  0
);

// =====================================================
// COLORS
// =====================================================

const uint16_t COLOR_BG    = 0x0000;
const uint16_t COLOR_TEXT  = 0xFFFF;
const uint16_t COLOR_CYAN  = 0x07FF;
const uint16_t COLOR_GREEN = 0x07E0;
const uint16_t COLOR_GREY  = 0x8410;
const uint16_t COLOR_RED   = 0xF800;

// =====================================================
// WIFI SETTINGS
// =====================================================

const int MAX_NETWORKS = 6;
const unsigned long SCAN_INTERVAL = 4000;
const int REMOVE_AFTER_MISSES = 5;

unsigned long lastScanStart = 0;
bool scanRunning = false;

// =====================================================
// PERSISTENT NETWORK STRUCTURE
// =====================================================

typedef struct {
  String name;
  int rssi;
  bool secure;
  int missedScans;
} WiFiNetwork;

WiFiNetwork networks[MAX_NETWORKS];

int networkCount = 0;

// =====================================================
// DISPLAY LAYOUT
// =====================================================

const int HEADER_Y = 30;
const int ROW_START_Y = 135;
const int ROW_HEIGHT = 50;
const int FOOTER_LINE_Y = 438;

// =====================================================
// DRAW HEADER
// =====================================================

void drawHeader() {

  gfx->fillRect(0, 0, LCD_WIDTH, 75, COLOR_BG);

  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_CYAN);

  const char *title = "WIFI SCANNER";

  int16_t x1, y1;
  uint16_t textWidth, textHeight;

  gfx->getTextBounds(
    title, 0, 0,
    &x1, &y1,
    &textWidth, &textHeight
  );

  int titleX = (LCD_WIDTH - textWidth) / 2;

  gfx->setCursor(titleX, HEADER_Y);
  gfx->print(title);

  gfx->drawFastHLine(
    30, 78,
    LCD_WIDTH - 60,
    COLOR_GREY
  );
}

// =====================================================
// DRAW SIGNAL BARS
// =====================================================

void drawSignalBars(int x, int y, int rssi) {

  int bars = 0;

  if (rssi >= -55) {
    bars = 4;
  }
  else if (rssi >= -67) {
    bars = 3;
  }
  else if (rssi >= -75) {
    bars = 2;
  }
  else if (rssi >= -85) {
    bars = 1;
  }

  const int barWidth = 5;
  const int gap = 3;
  const int heights[4] = {6, 11, 16, 21};

  for (int i = 0; i < 4; i++) {

    int barHeight = heights[i];

    uint16_t color =
      (i < bars) ? COLOR_CYAN : COLOR_GREY;

    gfx->fillRect(
      x + i * (barWidth + gap),
      y + 21 - barHeight,
      barWidth,
      barHeight,
      color
    );
  }
}

// =====================================================
// DRAW ONE NETWORK ROW
// =====================================================

void drawNetwork(int index) {

  int y = ROW_START_Y + index * ROW_HEIGHT;

  // Clear only this row
  gfx->fillRect(
    20, y - 5,
    LCD_WIDTH - 40,
    ROW_HEIGHT,
    COLOR_BG
  );

  // Network name
  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_TEXT);
  gfx->setCursor(30, y);

  String name = networks[index].name;

  if (name.length() > 15) {
    name = name.substring(0, 14) + "...";
  }

  gfx->print(name);

  // RSSI
  gfx->setTextSize(1);
  gfx->setTextColor(COLOR_GREY);
  gfx->setCursor(30, y + 23);

  gfx->print(networks[index].rssi);
  gfx->print(" dBm");

  // Security status
  gfx->setCursor(110, y + 23);

  if (networks[index].secure) {
    gfx->setTextColor(COLOR_GREEN);
    gfx->print("SECURED");
  }
  else {
    gfx->setTextColor(COLOR_RED);
    gfx->print("OPEN");
  }

  // Signal bars
  drawSignalBars(
    LCD_WIDTH - 65,
    y + 1,
    networks[index].rssi
  );

  // Separator
  gfx->drawFastHLine(
    30, y + 39,
    LCD_WIDTH - 60,
    COLOR_GREY
  );
}

// =====================================================
// DRAW FOOTER
// =====================================================

void drawFooter() {

  gfx->drawFastHLine(
    30,
    FOOTER_LINE_Y,
    LCD_WIDTH - 60,
    COLOR_GREY
  );

  gfx->fillRect(
    25, 448,
    LCD_WIDTH - 50,
    45,
    COLOR_BG
  );

  gfx->setTextSize(1);
  gfx->setTextColor(COLOR_GREY);

  gfx->setCursor(30, 455);

  if (scanRunning) {
    gfx->print("Updating networks...");
  }
  else {
    gfx->print("Nearby networks: ");
    gfx->print(networkCount);
  }

  gfx->setCursor(30, 475);
  gfx->print("Auto refresh: 4 seconds");
}

// =====================================================
// FIND NETWORK BY NAME
// =====================================================

int findNetwork(String name) {

  for (int i = 0; i < networkCount; i++) {

    if (networks[i].name == name) {
      return i;
    }
  }

  return -1;
}

// =====================================================
// REMOVE NETWORK
// =====================================================

void removeNetwork(int index) {

  for (int i = index; i < networkCount - 1; i++) {
    networks[i] = networks[i + 1];
  }

  networkCount--;

  // Clear the unused final row
  int y = ROW_START_Y + networkCount * ROW_HEIGHT;

  gfx->fillRect(
    20, y - 5,
    LCD_WIDTH - 40,
    ROW_HEIGHT,
    COLOR_BG
  );

  // Redraw shifted rows
  for (int i = index; i < networkCount; i++) {
    drawNetwork(i);
  }
}

// =====================================================
// PROCESS SCAN RESULTS
// =====================================================

void updateNetworkList() {

  int found = WiFi.scanComplete();

  if (found == WIFI_SCAN_RUNNING) {
    return;
  }

  scanRunning = false;

  // Keep previous list if scanning failed
  if (found == WIFI_SCAN_FAILED) {
    drawFooter();
    WiFi.scanDelete();
    return;
  }

  // Mark existing networks as missing
  for (int i = 0; i < networkCount; i++) {
    networks[i].missedScans++;
  }

  // Process detected networks
  for (int j = 0; j < found; j++) {

    String name = WiFi.SSID(j);

    // Ignore hidden networks
    if (name.length() == 0) {
      continue;
    }

    int index = findNetwork(name);

    if (index >= 0) {

      // Update existing network
      networks[index].rssi = WiFi.RSSI(j);

      networks[index].secure =
        WiFi.encryptionType(j) != WIFI_AUTH_OPEN;

      networks[index].missedScans = 0;

      drawNetwork(index);
    }

    else if (networkCount < MAX_NETWORKS) {

      // Add new network
      index = networkCount;

      networks[index].name = name;
      networks[index].rssi = WiFi.RSSI(j);

      networks[index].secure =
        WiFi.encryptionType(j) != WIFI_AUTH_OPEN;

      networks[index].missedScans = 0;

      networkCount++;

      drawNetwork(index);
    }
  }

  // Remove networks missing for five scans
  for (int i = 0; i < networkCount; ) {

    if (networks[i].missedScans >= REMOVE_AFTER_MISSES) {
      removeNetwork(i);
    }
    else {
      i++;
    }
  }

  drawFooter();

  WiFi.scanDelete();
}

// =====================================================
// START ASYNCHRONOUS SCAN
// =====================================================

void startScan() {

  if (scanRunning) {
    return;
  }

  int result = WiFi.scanNetworks(true, true);

  if (result == WIFI_SCAN_RUNNING) {

    scanRunning = true;
    lastScanStart = millis();

    drawFooter();
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  // Initialize display
  if (!gfx->begin()) {
    Serial.println("Display initialization reported failure.");
  }

  gfx->fillScreen(COLOR_BG);

  drawHeader();

  // Initial message
  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_CYAN);
  gfx->setCursor(30, 200);
  gfx->print("Searching...");

  gfx->setTextSize(1);
  gfx->setTextColor(COLOR_GREY);
  gfx->setCursor(30, 235);
  gfx->print("Looking for nearby networks");

  // Initialize Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect();

  delay(300);

  startScan();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {

  // Check whether scan has completed
  if (scanRunning) {

    if (WiFi.scanComplete() != WIFI_SCAN_RUNNING) {
      updateNetworkList();
    }
  }

  // Start another scan every four seconds
  if (!scanRunning &&
      millis() - lastScanStart >= SCAN_INTERVAL) {

    startScan();
  }

  delay(20);
}
