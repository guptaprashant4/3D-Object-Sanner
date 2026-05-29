#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <vl53l4cd_class.h>
#include "graphics.h"                     // Contains images for the OLED display
#include "network.h"                     // Contains network info of the connected Wi-Fi

// ============================================================
//  PIN DEFINITIONS
// ============================================================
#define ENM1 21         // The Two Stepper Motors are named M1 and M2
#define ENM2 20
#define DIR 2           // The direction for both the motors will remain same
#define STEPM1 3
#define STEPM2 4

// Microstepping Pins for motor M1
#define M0M1 5
#define M1M1 6
#define M2M1 7
// Microstepping Pins for Motor M2
#define M0M2 8
#define M1M2 9
#define M2M2 10

// Switch Key Pins
#define BTN_RIGHT   12
#define BTN_LEFT    11
#define BTN_UP      13
#define BTN_DOWN    14
#define BTN_SELECT  15

// ============================================================
//  CONSTANT DEFINITIONS
// ============================================================
#define DISTANCESAMPLE 10         // Data sampling rate for the ToF sensor
#define XSHUT_PIN -1    // -1 means the shutdown pin on ToF sensor is unused

// Define directions for the motors
#define CLKWISE LOW
#define ANTICLKWISE HIGH

#define TURNDIAMETER 100
#define THREADDISTANCE 1.5785

// ============================================================
//  DISPLAY SETTINGS
// ============================================================
#define SCREEN_WIDTH 128        // OLED display width, in pixels
#define SCREEN_HEIGHT 64        // OLED display height, in pixels
#define SCREEN_ADDR 0x3C
#define OLED_RESET -1           // Reset NOT AVAILABLE

// ============================================================
//  HARDWARE OBJECTS
// ============================================================
VL53L4CD tofSensor(&Wire1, XSHUT_PIN);    // Initialize ToF sensor
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);        // Initialize the OLED display

// Server Object with port Definition
WebServer server(80);

// ============================================================
//  SCREEN STATE MACHINE
// ============================================================
enum Screen {
  SCREEN_HOME,
  SCREEN_WIFI_STATUS,
  SCREEN_WIFI_NETWORKS,
  SCREEN_KEYBOARD,
  SCREEN_CONN_STATUS,
  SCREEN_PRECISION,
  SCREEN_SCANNING,
  SCREEN_NETWORK_INFO,
  SCREEN_SCAN_COMPLETE
};
Screen activeScreen = SCREEN_HOME;
int    cursorPos    = 0;

// ============================================================
//  KEYBOARD STATE
// ============================================================
String  keyboardInput = "";   // The string being built
bool    kbShift       = false;
bool    kbSymbols     = false;
int kbRow = 0;
int kbCol = 0;
const int kbRowMax[4] = { 9, 8, 8, 2 };   // Max column index per row

// ============================================================
//  SCAN STATE
// ============================================================
bool   scanRunning  = false;
bool   scanPaused   = false;
int scanStepM1 = 0;   
int scanStepM2 = 0;   
double scanY   = 0;
double scanZ   = 0;
int    scanProgress = 0;
String estTime      = "40s";
unsigned long lastScanDraw = 0;

// ============================================================
//  MOTOR
// ============================================================
int pulseWidth = 10;
int stepSize   = 50;

float stepAngleM2 = 1.8;
int numTurnsM1 = 0;
int maxTurnsM1 = 60;
int maxTurnsM2 = (int) (360 / stepAngleM2);



// ============================================================
//  BUTTON DEBOUNCE
// ============================================================
unsigned long lastBtnTime = 0;
const unsigned long DEBOUNCE_MS = 200;

// ============================================================
//  OUTPUT
// ============================================================
String pointCloud = "X Y Z\n";


void setup() {

  // Serial Monitor for TESTING PURPOSES ONLY
  Serial.begin(9600);

  // Turn on LED when powered On
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Motor driver Pins
  pinMode(ENM1, OUTPUT);  pinMode(ENM2, OUTPUT);
  pinMode(STEPM1, OUTPUT);  pinMode(STEPM2, OUTPUT);
  pinMode(DIR, OUTPUT);

  // Micro-stepping Pins
  pinMode(M0M1, OUTPUT);  pinMode(M1M1, OUTPUT);  pinMode(M2M1, OUTPUT);
  pinMode(M0M2, OUTPUT);  pinMode(M1M2, OUTPUT);  pinMode(M2M2, OUTPUT);

  // Control Keys
  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);

  // Set I2C channel for the OLED
  Wire.setSDA(16);  Wire.setSCL(17);  Wire.begin();

  // Set I2C channel for ToF sensor
  Wire1.setSDA(18); Wire1.setSCL(19); Wire1.begin();

  // Motor defaults
  turnOffMotor(ENM1); turnOffMotor(ENM2);
  setDir(ANTICLKWISE);

  // Turn off microstepping
  setMicroStep(1, 0); setMicroStep(2, 0);

  // Start connection to the display
  delay(100);         // Allows for a reset after a power cycle
  oled.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDR);

  // Initiate connection to sensor
  tofSensor.begin();

  // Try Wi-Fi connection
  attemptWifiConn();

  // Initiate the server
  server.on("/", handleRoot);
  server.begin();

  // Display the home screen
  navigateTo(SCREEN_HOME);
}

void loop() {
  handleInput();

  if (activeScreen == SCREEN_SCANNING && scanRunning) {
    stepScan();

    if (millis() - lastScanDraw > 200) {
      lastScanDraw = millis();
      displayScanProgress();
    }
  } else {
    server.handleClient();  // Only serve when scan is not running
  }
}

// ============================================================
//  BUTTON HELPERS
// ============================================================
bool btnPressed(int pin) {
  if (digitalRead(pin) == LOW) {
    unsigned long now = millis();
    if (now - lastBtnTime > DEBOUNCE_MS) {
      lastBtnTime = now;
      return true;
    }
  }
  return false;
}

// ============================================================
//  INPUT ROUTER
// ============================================================
void handleInput() {
  switch (activeScreen) {
    case SCREEN_HOME:      handleHomeInput();           break;
    case SCREEN_PRECISION: handlePrecisionInput();      break;
    case SCREEN_SCANNING:  handleScanningInput();       break;
    case SCREEN_NETWORK_INFO: handleNetworkInfoInput(); break;
    case SCREEN_CONN_STATUS: handleConnStatusInput();   break;
    case SCREEN_WIFI_NETWORKS:  handleAvailableNetworksInput(); break;
    case SCREEN_KEYBOARD: handleKeyboardInput(); break;
    case SCREEN_SCAN_COMPLETE: handleScanCompleteInput(); break;
  }
}

// ============================================================
//  SCAN STUB
// ============================================================
void startScan() {
  if (scanRunning) return;
  if (scanPaused) {           // Resume
    scanRunning = true;
    scanPaused  = false;
    return;
  }
  scanRunning = true;
  scanPaused  = false;
  scanStepM1   = 0;
  scanStepM2   = 0;
  scanY       = 0;
  scanZ       = 0;
  pointCloud  = "X Y Z\n";
  numTurnsM1  = 0;
}

void stepScan() {
  if (!scanRunning || scanPaused) return;

  int dist = getDistance();

  // Convert cylindrical → Cartesian
  double angle = (scanStepM2 * stepAngleM2 * PI) / 180.0;
  double px = dist * cos(angle);
  double py = dist * sin(angle);

  Serial.println(String(px) + " " + String(py) + " " + String(scanZ));
  pointCloud += String(px) + " " + String(py) + " " + String(scanZ) + "\n";

  moveMotor(2, 1);
  scanStepM2++;

  if (scanStepM2 >= maxTurnsM2) {
    scanStepM2 = 0;          // angle resets — no need for scanY at all
    moveMotor(1, 200);
    numTurnsM1++;
    scanZ += THREADDISTANCE;
    scanStepM1++;
    delay(100);
  }

  scanProgress = map(scanStepM1, 0, maxTurnsM1, 0, 100);

  if (scanStepM1 >= maxTurnsM1) {
    scanRunning = false;
    navigateTo(SCREEN_SCAN_COMPLETE);
  }
}

void stopScan() {
  scanRunning = false;
  scanPaused = false;

  // Turn Motors back to original position
  setDir(CLKWISE);
  setMicroStep(2, 0);
  moveMotor(1, numTurnsM1 * 200);
  delay(1000);

  turnOffMotor(ENM1);
  turnOffMotor(ENM2);
  tofSensor.VL53L4CD_StopRanging();

  pointCloud = "X Y Z\n";
  numTurnsM1 = 0;
  
  navigateTo(SCREEN_HOME);
}

void pauseScan() {

  if(!scanRunning) return;
  else {
    scanRunning = false;
    scanPaused = true;
    turnOffMotor(ENM1);
    turnOffMotor(ENM2);
    tofSensor.VL53L4CD_StopRanging();
  }
}

// ============================================================
//  TOF SENSOR
// ============================================================
int getDistance() {
  VL53L4CD_Result_t result;
  uint8_t dataReady = 0;
  int sum = 0;
  int avgDistance;

  tofSensor.VL53L4CD_StartRanging();

  // Collect 10 sample data and then calculate average for more accurate calculation
  for(int i = 0; i < DISTANCESAMPLE; i++) {
    checkScanButtons();
    dataReady = 0;

    // Wait for measurement to complete (with timeout)
    unsigned long startTime = millis();
    while(!dataReady && millis() - startTime < 1000) {  // 1 second timeout
      tofSensor.VL53L4CD_CheckForDataReady(&dataReady);
      delay(5);
    }

    if(dataReady) {
      tofSensor.VL53L4CD_GetResult(&result);
      tofSensor.VL53L4CD_ClearInterrupt();
      delay(50);
    }
    sum += (int) result.distance_mm;

    delay(100);
  }

  tofSensor.VL53L4CD_StopRanging();

  avgDistance = sum / DISTANCESAMPLE;

  return avgDistance;
}

// ============================================================
//  MOTOR HELPERS
// ============================================================
void turnOffMotor(int mtrENPin) { digitalWrite(mtrENPin, HIGH); }
void turnOnMotor(int mtrENPin) {  digitalWrite(mtrENPin, LOW);  }
void setDir(int direction) {  digitalWrite(DIR, direction); }

void setMicroStep(int motorNum, int step) {
  // Allowed Micro-stepping configurations are 1/2, 1/4, 1/8
  // Just type the denominator digit as the step
  
  if(motorNum == 1) {
    if(step == 0) {
      digitalWrite(M0M1, LOW);
      digitalWrite(M1M1, LOW);
      digitalWrite(M2M1, LOW);
    }
    else if(step == 2) {
      digitalWrite(M0M1, HIGH);
      digitalWrite(M1M1, LOW);
      digitalWrite(M2M1, LOW);
    }
    else if(step == 4) {
      digitalWrite(M0M1, LOW);
      digitalWrite(M1M1, HIGH);
      digitalWrite(M2M1, LOW);  
    }
    else if(step == 8) {
      digitalWrite(M0M1, HIGH);
      digitalWrite(M1M1, HIGH);
      digitalWrite(M2M1, LOW);
    }
    else {
      digitalWrite(M0M1, LOW);
      digitalWrite(M1M1, LOW);
      digitalWrite(M2M1, LOW);
    }
  }

  else if(motorNum == 2) {
    if(step == 0) {
      digitalWrite(M0M2, LOW);
      digitalWrite(M1M2, LOW);
      digitalWrite(M2M2, LOW);
    }
    else if(step == 2) {
      digitalWrite(M0M2, HIGH);
      digitalWrite(M1M2, LOW);
      digitalWrite(M2M2, LOW);
    }
    else if(step == 4) {
      digitalWrite(M0M2, LOW);
      digitalWrite(M1M2, HIGH);
      digitalWrite(M2M2, LOW);  
    }
    else if(step == 8) {
      digitalWrite(M0M2, HIGH);
      digitalWrite(M1M2, HIGH);
      digitalWrite(M2M2, LOW);
    }
    else {
      digitalWrite(M0M2, LOW);
      digitalWrite(M1M2, LOW);
      digitalWrite(M2M2, LOW);
    }
  }
}

void moveMotor(int motorNum, int numSteps) {
  
  if(motorNum == 1) {
    turnOnMotor(ENM1);

    for(int i = 0; i < numSteps; i++) {
      checkScanButtons();
      digitalWrite(STEPM1, HIGH);
      delayMicroseconds(pulseWidth);
      digitalWrite(STEPM1, LOW);

      delay(stepSize);
    }
    turnOffMotor(ENM1);
  }

  else if(motorNum == 2) {
    turnOnMotor(ENM2);

    for(int i = 0; i < numSteps; i++) {
      digitalWrite(STEPM2, HIGH);
      delayMicroseconds(pulseWidth);
      digitalWrite(STEPM2, LOW);

      delay(stepSize);
    }
    turnOffMotor(ENM2);    
  }
}

// ============================================================
//  WI-FI SCAN
// ============================================================
void listWiFi() {
  // Clear the array first
  for(int i = 0; i < 5; i++) {
    availNetworks[i] = "";
  }
  
  int networkNum = WiFi.scanNetworks();

  if(networkNum == 0) {
    availNetworks[0] = "No Network Available!";
  }
  else if(networkNum == -1) {
    availNetworks[0] = "Scan Failed!";
  }
  else {
    // Store up to 10 networks
    for(int i = 0; i < 5; i++) {
      availNetworks[i] = WiFi.SSID(i);
    }
  }
}

// ============================================================
//  CONNECTION HANDLERS
// ============================================================
void attemptWifiConn() {
  // Don't attempt a connection if no SSID has been configured
  if (strlen(SSID) == 0) {
    return;
  }

  WiFi.disconnect(true);
  delay(200);
  WiFi.begin(SSID, PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis()-start < 15000) {
    delay(500);
  }

  connSuccess = (WiFi.status() == WL_CONNECTED);
  if (connSuccess) {
    deviceIP  = WiFi.localIP();
    deviceMac = WiFi.macAddress();
    connMsg   = "Connected!";
    connMsg2  = WiFi.SSID();
  } 
  else {
    deviceIP  = IPAddress(0, 0, 0, 0);  // Clear stale IP
    deviceMac = "";
    connMsg  = "Connection failed!!";
    connMsg2 = "Check credentials.";
  }
}

void handleRoot() {
  if (scanRunning) {
    server.send(503, "text/plain", "Scan in progress, please wait.");
    return;
  }
  if (pointCloud == "X Y Z\n") {
    server.send(200, "text/plain", "No data available.");
    return;
  }
  server.send(200, "text/plain", pointCloud);
}

// ============================================================
//  NAVIGATE SCREENS (set active screen, reset cursor, redraw)
// ============================================================
void navigateTo(Screen s) {
  activeScreen = s;
  cursorPos    = 0;
  switch (s) {
    case SCREEN_HOME:          displayHome();                 break;
    case SCREEN_WIFI_NETWORKS: displayAvailableNetworks();    break;
    case SCREEN_KEYBOARD:      displayKeyboard();             break;
    case SCREEN_CONN_STATUS:   displayConnStatus();           break;
    case SCREEN_PRECISION:     displayPrecisionSelect();      break;
    case SCREEN_SCANNING:      displayScanProgress();         break;
    case SCREEN_NETWORK_INFO:  displayNetworkInfo();          break;
    case SCREEN_SCAN_COMPLETE: displayScanComplete();         break;
  }
}

// ============================================================
//  SCREENS
// ============================================================
void displayHome() {
  oled.clearDisplay();

  oled.setCursor(0, 0);
  oled.setTextColor(WHITE);
  oled.setTextSize(2);
  oled.print("3D Scanner");

  oled.drawLine(0, 16, 128, 16, WHITE);
  oled.drawBitmap(0, 32, startLogo, 28, 28, WHITE);
  oled.drawBitmap(32, 32, wifiLogo, 28, 28, WHITE);
  oled.drawBitmap(64, 32, infoLogo, 28, 28, WHITE);
  oled.drawBitmap(96, 32, resetLogo, 32, 32, WHITE);


  // Highlight whichever icon is selected
  oled.fillRect(cursorPos * 32, 32, 32, 32, SSD1306_INVERSE);

  oled.display();
}

void displayNetworkInfo() {
  oled.clearDisplay();

  oled.drawLine(0, 16, 128, 16, WHITE);
  oled.drawBitmap(112, 0, backLogo, 16, 16, WHITE);
  oled.setTextColor(WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 4);
  oled.print("Network Info");

  // Back button is the only interactive element (cursor 0)
  if (cursorPos == 0) oled.fillRect(112, 0, 16, 16, SSD1306_INVERSE);

  oled.setCursor(0, 22);
  oled.print("IP:  ");
  oled.println(deviceIP.toString());

  oled.setCursor(0, 33);
  oled.print("MAC: ");

  const int macIndentX  = 30; // x-position where MAC value starts
  const int charsPerRow = (128 - macIndentX) / 6;

  String mac = deviceMac;
  oled.print(mac.substring(0, charsPerRow));           // first 16 chars on same line
  if (mac.length() > charsPerRow) {
    oled.setCursor(macIndentX, 43);                    // indent to align under MAC value
    oled.print(mac.substring(charsPerRow));            // remaining char(s) on next line
  }

  oled.display();
}

void displayAvailableNetworks() {
  oled.clearDisplay();

  oled.drawLine(0, 16, 128, 16, WHITE);
  oled.drawBitmap(112, 0, backLogo, 16, 16, WHITE);
  oled.setCursor(0, 4);
  oled.setTextColor(WHITE);
  oled.setTextSize(1);
  oled.print("Available Networks");

  if (cursorPos == 0) oled.fillRect(112, 0, 16, 16, SSD1306_INVERSE);

  // Fixed y position for each row
  const int rowY[4] = { 20, 30, 40, 50 };

  for (int i = 0; i < 4; i++) {
    oled.setCursor(0, rowY[i]);

    if (availNetworks[i] == "No Network Available!" || availNetworks[i] == "Scan Failed!") {
      oled.print(availNetworks[i]);
      break;
    }

    if (availNetworks[i] != "") {
      String networkName = availNetworks[i];
      if (networkName.length() > 18) {
        networkName = networkName.substring(0, 15) + "...";
      }
      oled.print(i + 1);
      oled.print(". ");
      oled.print(networkName);

      // Highlight selected row — cursorPos 1 maps to row 0, etc.
      if (cursorPos == i + 1) {
        oled.fillRect(0, rowY[i] - 1, 110, 10, SSD1306_INVERSE);
      }
    }
  }

  oled.display();
}

void displayConnStatus() {
  oled.clearDisplay();
  oled.setTextColor(WHITE);
  oled.setTextSize(1);

  oled.drawLine(0, 16, 128, 16, WHITE);
  oled.setCursor(0, 4);
  oled.print("Connection Status");

  // Primary message
  oled.setCursor(4, 22);
  oled.print(connMsg);

  // Secondary detail
  if (connMsg2.length() > 0) {
    oled.setCursor(4, 33);
    oled.print(connMsg2);
  }

  // ADD (left) and OK (right) buttons
  oled.drawRect(4,  51, 40, 12, WHITE);
  oled.setCursor(16, 54);
  oled.print("NEW");

  oled.drawRect(84, 51, 40, 12, WHITE);
  oled.setCursor(98, 54);
  oled.print("OK");

  // Highlight: 0 = ADD, 1 = OK
  if (cursorPos == 0) oled.fillRect(4,  51, 40, 12, SSD1306_INVERSE);
  if (cursorPos == 1) oled.fillRect(84, 51, 40, 12, SSD1306_INVERSE);

  oled.display();
}

void displayPrecisionSelect() {
  oled.clearDisplay();

  oled.drawLine(0, 16, 128, 16, WHITE);
  oled.drawBitmap(112, 0, backLogo, 16, 16, WHITE);
  oled.setTextColor(WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 4);
  oled.print("Select Precision");

  if (cursorPos == 0) oled.fillRect(112, 0, 16, 16, SSD1306_INVERSE);

  const char* options[4] = {
    "Normal",
    "Precise",
    "Very Precise",
    "Extremely Precise"
  };
  const int optY[4] = { 20, 30, 40, 50 };

  for (int i = 0; i < 4; i++) {
    if (cursorPos == i+1) {
      oled.fillRect(0, optY[i]-1, 108, 10, WHITE);
      oled.setTextColor(BLACK);
    } else {
      oled.setTextColor(WHITE);
    }
    oled.setCursor(4, optY[i]);
    oled.print(options[i]);
  }

  oled.setTextColor(WHITE);
  oled.display();
}

void displayScanProgress() {
  oled.clearDisplay();

  oled.setTextColor(WHITE);
  oled.setTextSize(1.5);
  oled.setCursor(22, 0);
  oled.print("Scanning...");

  // Animation frame derived from scan progress (0-26)
  int frameIndex = map(scanProgress, 0, 100, 0, 26);
  if (frameIndex > 26) frameIndex = 26;
  oled.drawBitmap(10, 16, progressAnimation[frameIndex], 32, 32, WHITE);

  // Time remaining
  oled.setTextSize(1);
  oled.setCursor(4, 52);
  oled.print("Time: ");
  oled.print(estTime);

  // Vertical divider
  oled.drawLine(85, 16, 85, 64, WHITE);

  // Three control buttons on the right side
  const char* btnLabels[3] = {"Start","Stop ","Pause"};
  const int   btnY[3]      = { 17, 32, 47 };
  for (int i = 0; i < 3; i++) {
    oled.drawRect(88, btnY[i], 38, 12, WHITE);
    oled.setCursor(92, btnY[i]+3);
    oled.print(btnLabels[i]);
  }

  // Highlight selected button
  oled.fillRect(88, btnY[cursorPos], 38, 12, SSD1306_INVERSE);

  oled.display();
}

void displayScanComplete() {
  oled.clearDisplay();

  oled.setTextColor(WHITE);
  oled.setTextSize(1);
  oled.drawLine(0, 16, 128, 16, WHITE);
  oled.setCursor(0, 4);
  oled.print("Scan Complete!");

  // Tick icon
  oled.drawBitmap(48, 20, checkLogo, 32, 32, WHITE); 

  // OK button
  oled.drawRect(44, 53, 40, 11, WHITE);
  oled.setCursor(57, 55);
  oled.print("OK");

  oled.fillRect(44, 53, 40, 11, SSD1306_INVERSE);

  oled.display();
}

void displayConnecting() {
  for (int frame = 0; frame < 27; frame++) {
    oled.clearDisplay();

    // Title
    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(26, 0);
    oled.print("Connecting...");

    oled.drawLine(0, 10, 128, 10, WHITE);

    // Center the 48x48 bitmap horizontally, below the title
    oled.drawBitmap(40, 13, connectingAnimation[frame], 48, 48, WHITE);

    oled.display();
    delay(80);  // ~12fps — adjust to taste
  }
}

void displayKeyboard() {
  if (kbSymbols) {
    displayKeyboardSymbols();
    return;
  }

  oled.clearDisplay();

  // Header — show current input and back button
  oled.drawLine(0, 16, 128, 16, WHITE);
  oled.drawBitmap(112, 0, backLogo, 16, 16, WHITE);
  oled.setCursor(2, 4);
  oled.setTextColor(WHITE);
  oled.setTextSize(1);
  oled.print(keyboardInput);
  oled.print("_");

  // Row 1: Q W E R T Y U I O P
  const char* row1[10] = {"Q","W","E","R","T","Y","U","I","O","P"};
  for (int i = 0; i < 10; i++) {
    oled.drawRect(2 + (i * 12), 18, 11, 10, WHITE);
    oled.setCursor(4 + (i * 12), 20);
    oled.print(kbChar(row1[i]));
  }

  // Row 2: A S D F G H J K L (offset 8px to center)
  const char* row2[9] = {"A","S","D","F","G","H","J","K","L"};
  for (int i = 0; i < 9; i++) {
    oled.drawRect(8 + (i * 12), 30, 11, 10, WHITE);
    oled.setCursor(10 + (i * 12), 32);
    oled.print(kbChar(row2[i]));
  }

  // Row 3: SH(wide) Z X C V B N M BS(wide)
  const char* row3[7] = {"Z","X","C","V","B","N","M"};

  // Shift button — label reflects current state
  oled.drawRect(2, 42, 17, 10, WHITE);
  oled.setCursor(4, 44);
  if (kbShift) {
    oled.print("SH");
  } else {
    oled.print("sh");
  }

  for (int i = 0; i < 7; i++) {
    oled.drawRect(21 + (i * 12), 42, 11, 10, WHITE);
    oled.setCursor(23 + (i * 12), 44);
    oled.print(kbChar(row3[i]));
  }

  // Backspace
  oled.drawRect(106, 42, 20, 10, WHITE);
  oled.setCursor(108, 44);
  oled.print("BS");

  // Row 4: 123, Space, Enter
  oled.drawRect(2,  54, 20, 10, WHITE);
  oled.setCursor(4,  56);
  oled.print("123");

  oled.drawRect(24, 54, 60, 10, WHITE);
  oled.setCursor(45, 56);
  oled.print("SPACE");

  oled.drawRect(86, 54, 40, 10, WHITE);
  oled.setCursor(92, 56);
  oled.print("ENTER");

  // Highlight current key
  if (kbRow == 0) {
    oled.fillRect(2 + (kbCol * 12), 18, 11, 10, SSD1306_INVERSE);
  }
  else if (kbRow == 1) {
    oled.fillRect(8 + (kbCol * 12), 30, 11, 10, SSD1306_INVERSE);
  }
  else if (kbRow == 2) {
    if (kbCol == 0) {
      oled.fillRect(2, 42, 17, 10, SSD1306_INVERSE);      // SH
    } else if (kbCol == 8) {
      oled.fillRect(106, 42, 20, 10, SSD1306_INVERSE);    // BS
    } else {
      oled.fillRect(21 + ((kbCol - 1) * 12), 42, 11, 10, SSD1306_INVERSE);
    }
  }
  else if (kbRow == 3) {
    if (kbCol == 0) {
      oled.fillRect(2,  54, 20, 10, SSD1306_INVERSE);     // 123
    } else if (kbCol == 1) {
      oled.fillRect(24, 54, 60, 10, SSD1306_INVERSE);     // SPACE
    } else if (kbCol == 2) {
      oled.fillRect(86, 54, 40, 10, SSD1306_INVERSE);     // ENTER
    }
  }
  else if (kbRow == -1) {
    oled.fillRect(112, 0, 16, 16, SSD1306_INVERSE);       // Back button
  }

  oled.display();
}

void displayKeyboardSymbols() {
  oled.clearDisplay();

  // Header — show current input text
  oled.drawLine(0, 16, 128, 16, WHITE);
  oled.drawBitmap(112, 0, backLogo, 16, 16, WHITE);
  oled.setCursor(2, 4);
  oled.setTextColor(WHITE);
  oled.setTextSize(1);
  oled.print(keyboardInput);
  oled.print("_");

  // Row 1: 1 2 3 4 5 6 7 8 9 0
  const char* row1Sym[10] = {"1","2","3","4","5","6","7","8","9","0"};
  for (int i = 0; i < 10; i++) {
    oled.drawRect(2 + (i * 12), 18, 11, 10, WHITE);
    oled.setCursor(4 + (i * 12), 20);
    oled.print(row1Sym[i]);
  }

  // Row 2: ! @ # $ % ^ & * ( )
  const char* row2Sym[10] = {"!","@","#","$","%","^","&","*","(",")"}; 
  for (int i = 0; i < 10; i++) {
    oled.drawRect(2 + (i * 12), 30, 11, 10, WHITE);
    oled.setCursor(4 + (i * 12), 32);
    oled.print(row2Sym[i]);
  }

  // Row 3: - _ = + [ ] . , ? BS
  const char* row3Sym[8] = {"-","_","=","+"," [","]",".",","};
  for (int i = 0; i < 8; i++) {
    oled.drawRect(2 + (i * 12), 42, 11, 10, WHITE);
    oled.setCursor(4 + (i * 12), 44);
    oled.print(row3Sym[i]);
  }
  // Backspace
  oled.drawRect(98, 42, 28, 10, WHITE);
  oled.setCursor(100, 44);
  oled.print("BS");

  // Row 4: ABC (back to letters), Space, Enter
  oled.drawRect(2,  54, 20, 10, WHITE);
  oled.setCursor(4, 56);
  oled.print("ABC");

  oled.drawRect(24, 54, 60, 10, WHITE);
  oled.setCursor(45, 56);
  oled.print("SPACE");

  oled.drawRect(86, 54, 40, 10, WHITE);
  oled.setCursor(92, 56);
  oled.print("ENTER");

  // Highlight current key — same positions as letter layout
  if (kbRow == 0) {
    oled.fillRect(2 + (kbCol * 12), 18, 11, 10, SSD1306_INVERSE);
  }
  else if (kbRow == 1) {
    oled.fillRect(2 + (kbCol * 12), 30, 11, 10, SSD1306_INVERSE);
  }
  else if (kbRow == 2) {
    if (kbCol == 8) {
      oled.fillRect(98, 42, 28, 10, SSD1306_INVERSE);     // BS
    } else {
      oled.fillRect(2 + (kbCol * 12), 42, 11, 10, SSD1306_INVERSE);
    }
  }
  else if (kbRow == 3) {
    if (kbCol == 0) {
      oled.fillRect(2,  54, 20, 10, SSD1306_INVERSE);     // ABC
    } else if (kbCol == 1) {
      oled.fillRect(24, 54, 60, 10, SSD1306_INVERSE);     // SPACE
    } else if (kbCol == 2) {
      oled.fillRect(86, 54, 40, 10, SSD1306_INVERSE);     // ENTER
    }
  }
  else if (kbRow == -1) {
    oled.fillRect(112, 0, 16, 16, SSD1306_INVERSE);       // Back button
  }

  oled.display();
}

// ============================================================
//  INPUT HANDLERS
// ============================================================
void checkScanButtons() {
  if (btnPressed(BTN_UP))   { 
    cursorPos = max(0, cursorPos-1); 
    displayScanProgress(); 
  }
  if (btnPressed(BTN_DOWN)) {
    cursorPos = min(2, cursorPos+1); 
    displayScanProgress();
  }
  if (btnPressed(BTN_SELECT)) {
    switch (cursorPos) {
      case 0: startScan(); break;
      case 1: stopScan();  break;
      case 2: pauseScan(); break;
    }
  }
}

void handleHomeInput() {
  // 4 icons: 0=Start, 1=WiFi, 2=Info, 3=Reset
  if (btnPressed(BTN_RIGHT)) {
    cursorPos = (cursorPos + 1) % 4;
    displayHome();
  }
  else if (btnPressed(BTN_LEFT)) {
    cursorPos = (cursorPos + 3) % 4;  // +3 wraps backwards
    displayHome();
  }
  else if (btnPressed(BTN_SELECT)) {
    switch (cursorPos) {
      case 0: navigateTo(SCREEN_PRECISION);     break;  // Start scan
      case 1: navigateTo(SCREEN_CONN_STATUS); break;  // WiFi
      case 2: navigateTo(SCREEN_NETWORK_INFO);  break;  // Info
      case 3:                                           // Reset
        rp2040.reboot();                 
        break;  
    }
  }
}

void handlePrecisionInput() {
  if (btnPressed(BTN_UP)) {
    cursorPos = max(0, cursorPos - 1);
    displayPrecisionSelect();
  }
  else if (btnPressed(BTN_DOWN)) {
    cursorPos = min(4, cursorPos + 1);
    displayPrecisionSelect();
  }
  else if (btnPressed(BTN_SELECT)) {
    if (cursorPos == 0) {
      navigateTo(SCREEN_HOME);  // Back button
    } else {
      // Map cursor to step size — higher precision = smaller steps
      switch (cursorPos) {
        case 1: setMicroStep(2, 0);  maxTurnsM2 = (360 / stepAngleM2) * 1; break;  // Normal
        case 2: setMicroStep(2, 2);  maxTurnsM2 = (360 / stepAngleM2) * 2; break;  // Precise
        case 3: setMicroStep(2, 4);  maxTurnsM2 = (360 / stepAngleM2) * 4; break;  // Very Precise
        case 4: setMicroStep(2, 8);  maxTurnsM2 = (360 / stepAngleM2) * 8; break;  // Extremely Precise
      }
      navigateTo(SCREEN_SCANNING);
    }
  }
}

void handleScanningInput() {
  if (btnPressed(BTN_UP)) {
    cursorPos = max(0, cursorPos - 1);
    displayScanProgress();
  }
  else if (btnPressed(BTN_DOWN)) {
    cursorPos = min(2, cursorPos + 1);
    displayScanProgress();
  }
  else if (btnPressed(BTN_SELECT)) {
    switch (cursorPos) {
      case 0: startScan();  break;
      case 1: stopScan();   break;
      case 2: pauseScan();  break;
    }
  }
}

void handleScanCompleteInput() {
  if (btnPressed(BTN_SELECT)) {
    navigateTo(SCREEN_HOME);
  }
}

void handleNetworkInfoInput() {
  if (btnPressed(BTN_SELECT)) {
    navigateTo(SCREEN_HOME);
  }
}

void handleConnStatusInput() {
  if (btnPressed(BTN_LEFT) || btnPressed(BTN_RIGHT)) {
    if (cursorPos == 0) {
      cursorPos = 1;
    } else {
      cursorPos = 0;
    }
    displayConnStatus();
  }
  else if (btnPressed(BTN_SELECT)) {
    if (cursorPos == 0) {
      listWiFi();
      navigateTo(SCREEN_WIFI_NETWORKS);
    } else if (cursorPos == 1) {
      navigateTo(SCREEN_HOME);
    }
  }
}

void handleAvailableNetworksInput() {
  if (btnPressed(BTN_UP)) {
    cursorPos = max(0, cursorPos - 1);
    displayAvailableNetworks();
  }
  else if (btnPressed(BTN_DOWN)) {
    cursorPos = min(4, cursorPos + 1);
    displayAvailableNetworks();
  }
  else if (btnPressed(BTN_SELECT)) {
    if (cursorPos == 0) {
      navigateTo(SCREEN_HOME);
    } else {
      // Replace SSID with the selected network name
      String selected = availNetworks[cursorPos - 1];
      selected.toCharArray(SSID, sizeof(SSID));
      navigateTo(SCREEN_KEYBOARD);
    }
  }
}

void handleKeyboardInput() {
  bool up     = btnPressed(BTN_UP);
  bool down   = btnPressed(BTN_DOWN);
  bool left   = btnPressed(BTN_LEFT);
  bool right  = btnPressed(BTN_RIGHT);
  bool select = btnPressed(BTN_SELECT);

  if (up) {
    if (kbRow > 0) {
      kbRow--;
      if (kbCol > kbRowMax[kbRow]) {
        kbCol = kbRowMax[kbRow];
      }
    } else if (kbRow == 0) {
      kbRow = -1;
    }
    displayKeyboard();
  }
  else if (down) {
    if (kbRow == -1) {
      kbRow = 0;
    } else if (kbRow < 3) {
      kbRow++;
      if (kbCol > kbRowMax[kbRow]) {
        kbCol = kbRowMax[kbRow];
      }
    }
    displayKeyboard();
  }
  else if (left && kbRow != -1) {
    if (kbCol > 0) {
      kbCol--;
    } else {
      kbCol = kbRowMax[kbRow];
    }
    displayKeyboard();
  }
  else if (right && kbRow != -1) {
    if (kbCol < kbRowMax[kbRow]) {
      kbCol++;
    } else {
      kbCol = 0;
    }
    displayKeyboard();
  }
  else if (select) {
    if (kbRow == -1) {
      keyboardInput = "";
      kbRow = 0; kbCol = 0;
      kbShift = false; kbSymbols = false;
      navigateTo(SCREEN_HOME);
    } else {
      handleKeyboardSelect();
      if (activeScreen == SCREEN_KEYBOARD) {  // Only redraw if still on keyboard
        displayKeyboard();
      }
    }
  }
}

void handleKeyboardSelect() {
  const char* row1[10] = {"Q","W","E","R","T","Y","U","I","O","P"};
  const char* row2[9]  = {"A","S","D","F","G","H","J","K","L"};
  const char* row3[7]  = {"Z","X","C","V","B","N","M"};

  const char* sym1[10] = {"1","2","3","4","5","6","7","8","9","0"};
  const char* sym2[10] = {"!","@","#","$","%","^","&","*","(",")"}; 
  const char* sym3[8]  = {"-","_","=","+","[","]",".",","};

  if (kbSymbols) {
    if (kbRow == 0) {
      keyboardInput += sym1[kbCol];
    }
    else if (kbRow == 1) {
      keyboardInput += sym2[kbCol];
    }
    else if (kbRow == 2) {
      if (kbCol == 8) {
        if (keyboardInput.length() > 0) {
          keyboardInput.remove(keyboardInput.length() - 1);  // Backspace
        }
      } else {
        keyboardInput += sym3[kbCol];
      }
    }
    else if (kbRow == 3) {
      if (kbCol == 0) {
        kbSymbols = false;                  // Back to letters
      } else if (kbCol == 1) {
        keyboardInput += " ";               // Space
      } else if (kbCol == 2) {
        keyboardInput.toCharArray(PASSWORD, sizeof(PASSWORD));
        keyboardInput = "";
        kbRow = 0; kbCol = 0;
        kbShift = false; kbSymbols = false;
        attemptWifiConn();
        navigateTo(SCREEN_CONN_STATUS);
      }
    }
  }

  else {
    // Letters layout
    if (kbRow == 0) {
      keyboardInput += kbChar(row1[kbCol]);
    }
    else if (kbRow == 1) {
      keyboardInput += kbChar(row2[kbCol]);
    }
    else if (kbRow == 2) {
      if (kbCol == 0) {
        kbShift = !kbShift;
      } else if (kbCol == 8) {
        if (keyboardInput.length() > 0) {
          keyboardInput.remove(keyboardInput.length() - 1);
        }
      } else {
        keyboardInput += kbChar(row3[kbCol - 1]);
      }
    }
    else if (kbRow == 3) {
      if (kbCol == 0) {
        kbSymbols = true;
      } else if (kbCol == 1) {
        keyboardInput += " ";
      } else if (kbCol == 2) {
        keyboardInput.toCharArray(PASSWORD, sizeof(PASSWORD));
        keyboardInput = "";
        kbRow = 0; kbCol = 0;
        kbShift = false; kbSymbols = false;

        displayConnecting();
        attemptWifiConn();
        navigateTo(SCREEN_CONN_STATUS);
      }
    }
  }
}

// Returns the key label accounting for shift state
String kbChar(const char* c) {
  String s = String(c);
  if (kbShift) {
    s.toUpperCase();
  } else {
    s.toLowerCase();
  }
  return s;
}

