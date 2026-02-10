
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

// -------- Keypad Configuration (7 Wires: Full 0-9 Keys) --------
// ACTIVE KEYS: 1,2,3, 4,5,6, 7,8,9, *,0,#
const byte ROWS = 4;
const byte COLS = 3;

char keys[ROWS][COLS] = {
    {'1', '2', '3'}, {'4', '5', '6'}, {'7', '8', '9'}, {'*', '0', '#'}};

// Wiring (7 Green Wires):
// Rows (R1, R2, R3, R4) -> 13, 12, 14, 27
// Cols (C1, C2, C3) -> 25, 33, 32
byte rowPins[ROWS] = {13, 12, 14, 27};
byte colPins[COLS] = {25, 33, 32};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// -------- Command Mapping (Standard Layout) --------
String keyToCmd(char k) {
  switch (k) {
  case '8':
    return "FWD"; // Fwd
  case '2':
    return "BWD"; // Bwd
  case '4':
    return "LEFT"; // Left
  case '6':
    return "RIGHT"; // Right
  case '5':
    return "HOVER"; // Hover

  case '1':
    return "UP"; // Up
  case '3':
    return "DOWN"; // Down

  case '7':
    return "LAND"; // Land
  case '9':
    return "RESET"; // Reset

  case '0':
    return "HOVER"; // Zero -> Hover
  case '*':
    return "RESET"; // Star -> Reset
  case '#':
    return "LAND"; // Hash -> Land

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

  Serial.println("\n--- DRONE CONTROLLER STARTED ---");
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
  Serial.println("CONTROLS (Keys 1-9):");
  Serial.println("  2: FWD   | 8: BWD");
  Serial.println("  4: LEFT  | 6: RIGHT");
  Serial.println("  1: UP    | 3: DOWN");
  Serial.println("  5: HOVER | 7: LAND");
  Serial.println("  9: RESET");
  Serial.println("--------------------------------");

  udp.begin(12345);
}

void loop() {
  char key = keypad.getKey();

  if (key) {
    String cmd = keyToCmd(key);

    Serial.print("Key [");
    Serial.print(key);
    Serial.print("] -> Cmd: ");
    Serial.println(cmd);

    if (cmd.length() > 0) {
      sendUDP(cmd);
    }
  }
}
