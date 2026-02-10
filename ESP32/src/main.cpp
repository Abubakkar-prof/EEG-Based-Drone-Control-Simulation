#include <Arduino.h>
#include <Keypad.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// -------- Wokwi Configuration --------
const char *ssid = "Wokwi-GUEST";
const char *pass = "";

// -------- PC Configuration --------
const char *PC_IP = "192.168.185.77"; // YOUR IPv4
const uint16_t PC_PORT = 5005;

WiFiUDP udp;

// -------- Keypad Configuration --------
const byte ROWS = 4;
const byte COLS = 4;

char keys[ROWS][COLS] = {{'1', '2', '3', 'A'},
                         {'4', '5', '6', 'B'},
                         {'7', '8', '9', 'C'},
                         {'*', '0', '#', 'D'}};

// Wiring (MATCHING THE DIAGRAM - SCRAMBLE FIX):
// Rows -> 27, 14, 12, 13
// Cols -> 26, 25, 33, 32
byte rowPins[ROWS] = {27, 14, 12, 13};
byte colPins[COLS] = {26, 25, 33, 32};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// -------- Command Mapping (Standard Layout) --------
String keyToCmd(char k) {
  switch (k) {
  case '2':
    return "FWD";
  case '8':
    return "BWD";
  case '4':
    return "LEFT";
  case '6':
    return "RIGHT";
  case '5':
    return "HOVER";

  case '1':
    return "UP";
  case '3':
    return "DOWN";
  case '0':
    return "RESET";

  case '7':
    return "START";

  default:
    return "";
  }
}

void sendUDP(const String &msg) {
  udp.beginPacket(PC_IP, PC_PORT);
  udp.print(msg);
  udp.endPacket();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n--- DRONE CONTROLLER STARTED (REVERTED) ---");
  Serial.println("Connecting to WiFi...");

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Target PC: ");
  Serial.println(PC_IP);

  Serial.println("--------------------------------");
  Serial.println("CONTROLS:");
  Serial.println("  2: FWD  | 8: BWD");
  Serial.println("  4: LEFT | 6: RIGHT");
  Serial.println("  1: UP   | 3: DOWN");
  Serial.println("  7: START| 5: HOVER");
  Serial.println("--------------------------------");

  udp.begin(12345);
}

void loop() {
  char key = keypad.getKey();

  if (key) {
    // Debug
    Serial.print("Key [");
    Serial.print(key);

    String cmd = keyToCmd(key);
    Serial.print("] -> Cmd: ");
    Serial.println(cmd);

    if (cmd.length() > 0) {
      sendUDP(cmd);
    }
  }
}
