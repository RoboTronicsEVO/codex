#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// =================================================================
// CONFIGURATION
// =================================================================
uint8_t receiverMAC[] = {0x08, 0xa6, 0xf7, 0x6a, 0xa8, 0x2c};

// =================================================================
// PIN DEFINITIONS
// =================================================================
// Device Remote Pins
#define SW1_DEV 15  // Relay 1 control (momentary)
#define SW2_DEV 4   // Speed toggle (on/off switch)
#define SW3_DEV 16  // Relay 2 control (on/off switch)
#define SW4_DEV 17  // Emergency stop (momentary)

#define LED1_DEV 21
#define LED2_DEV 19
#define LED3_DEV 18
#define LED4_DEV 5  // INVERTED: HIGH = OFF, LOW = ON

// Robot Controller Pins
#define SW_RELAY1_ROBOT   13
#define SW_RELAY2_ROBOT   25
#define SW_SOFTSTART_ROBOT 33
#define SW_SPEED_ROBOT    34

#define CONNECT_LED 12
#define DISCONNECT_LED 14
#define BATTERY_LED1 26
#define BATTERY_LED2 27
#define BATTERY_LED3 22
#define BATTERY_LED4 23

#define VRX1 32
#define VRX2 36

// =================================================================
// TIMING CONSTANTS (ALL IN MICROSECONDS)
// =================================================================
const unsigned long DEBOUNCE_DELAY_US = 50000;        // 50ms in microseconds
const unsigned long MIN_SEND_INTERVAL_US = 100000;    // 100ms in microseconds
const unsigned long CONNECTION_TIMEOUT_US = 3000000;  // 3 seconds in microseconds
const unsigned long HEARTBEAT_INTERVAL_US = 800000;   // 800ms in microseconds
const unsigned long STATUS_UPDATE_US = 200000;        // 200ms in microseconds
const unsigned long DEVICE_CHECK_US = 10000;          // 10ms in microseconds
const unsigned long RETRY_CHECK_US = 50000;           // 50ms in microseconds
const unsigned long CONNECTION_CHECK_US = 500000;     // 500ms in microseconds
const unsigned long RETRY_INTERVAL_US = 800000;       // 800ms in microseconds
const unsigned long MAX_RETRY_TIME_US = 5000000;      // 5 seconds in microseconds
const unsigned long ROBOT_DEBOUNCE_US = 400000;       // 400ms in microseconds
const unsigned long IMMEDIATE_RETRY_US = 5000;        // 5ms between immediate retries

// =================================================================
// DATA STRUCTURES
// =================================================================
typedef struct {
  int switchNum;
  bool state;
  unsigned long timestamp;
} SwitchMessage;

typedef struct {
  int switchNum;
  bool ack;
} AckMessage;

typedef struct __attribute__((packed)) {
  uint8_t move;
  uint8_t flags;
} fast_message;

typedef struct __attribute__((packed)) {
  char relayStatus[16];
  char batteryVoltage[8];
  char speedPin[4];
} status_message;

// =================================================================
// GLOBAL VARIABLES
// =================================================================
// Device Remote Variables
bool currentSwitchState_DEV[4];
bool lastSwitchState_DEV[4] = {LOW, LOW, LOW, LOW};
unsigned long lastSwitchTime_DEV[4] = {0, 0, 0, 0};
unsigned long lastSendTime_DEV[4] = {0, 0, 0, 0};
bool deviceStates[4] = {false, false, false, false};

struct RetryInfo {
  bool pending;
  unsigned long lastTryTime;
  unsigned long firstTryTime;
  int attempts;
  bool state;
};
RetryInfo retryInfo[4] = {{false, 0, 0, 0, false}};

// Robot Controller Variables
fast_message robotDataToSend;
status_message receivedRobotStatus;
bool lastRobotSW1State = HIGH, lastRobotSW2State = HIGH, lastRobotSW3State = HIGH, lastRobotSW4State = HIGH;
bool isConnected = false;
unsigned long lastDataReceived = 0;
unsigned long lastHeartbeat = 0;
float batteryVoltage = 0.0;
bool currentSpeedHigh = false;

// Connection monitoring
unsigned long lastConnectionCheck = 0;

// Joystick variables
const int JOYSTICK_CENTER = 2000;
const int JOYSTICK_DEADZONE = 500;
uint8_t lastMove1 = 'n', lastMove2 = 'n';
uint8_t currentCombinedMove = 'n';

// Movement commands
const uint8_t MOVE_NEUTRAL = 'n', MOVE_FORWARD = 'f', MOVE_BACKWARD = 'b', MOVE_FORWARD1 = '1',
              MOVE_FORWARD2 = '2', MOVE_BACKWARD1 = '3', MOVE_BACKWARD2 = '4', MOVE_ROTATE_CW = 'r',
              MOVE_ROTATE_CCW = 'R', CMD_RELAY1 = 'B', CMD_RELAY2 = 'V', CMD_SPEED = 'P', CMD_STOP = 'S';

// =================================================================
// ESP-NOW CALLBACKS
// =================================================================
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // No status LED feedback - removed for optimization
}

void onDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  lastDataReceived = micros(); // Use microseconds for precision
  
  if (len == sizeof(AckMessage)) {
    AckMessage ackMsg;
    memcpy(&ackMsg, data, sizeof(ackMsg));
    
    if (ackMsg.switchNum >= 1 && ackMsg.switchNum <= 4) {
      int switchIndex = ackMsg.switchNum - 1;
      deviceStates[switchIndex] = ackMsg.ack;
      updateDeviceLED(ackMsg.switchNum, ackMsg.ack);
      resetRetry(switchIndex);
    }
  } 
  else if (len == sizeof(status_message)) {
    memcpy(&receivedRobotStatus, data, sizeof(receivedRobotStatus));
    
    if (strlen(receivedRobotStatus.batteryVoltage) > 0) {
      batteryVoltage = atof(receivedRobotStatus.batteryVoltage);
    }
  }
}

// =================================================================
// SETUP
// =================================================================
void setup() {
  // Pin initialization
  pinMode(SW1_DEV, INPUT); pinMode(SW2_DEV, INPUT); pinMode(SW3_DEV, INPUT); pinMode(SW4_DEV, INPUT);
  pinMode(LED1_DEV, OUTPUT); pinMode(LED2_DEV, OUTPUT); pinMode(LED3_DEV, OUTPUT); pinMode(LED4_DEV, OUTPUT);
  
  pinMode(SW_RELAY1_ROBOT, INPUT_PULLUP); pinMode(SW_RELAY2_ROBOT, INPUT_PULLUP);
  pinMode(SW_SOFTSTART_ROBOT, INPUT_PULLUP); pinMode(SW_SPEED_ROBOT, INPUT_PULLUP);
  pinMode(CONNECT_LED, OUTPUT); pinMode(DISCONNECT_LED, OUTPUT);
  pinMode(BATTERY_LED1, OUTPUT); pinMode(BATTERY_LED2, OUTPUT); pinMode(BATTERY_LED3, OUTPUT); pinMode(BATTERY_LED4, OUTPUT);
  pinMode(VRX1, INPUT); pinMode(VRX2, INPUT);

  // Initial LED states - LED4_DEV (pin 5) is INVERTED
  digitalWrite(LED1_DEV, LOW); digitalWrite(LED2_DEV, LOW); digitalWrite(LED3_DEV, LOW); 
  digitalWrite(LED4_DEV, HIGH); // INVERTED: HIGH = OFF
  digitalWrite(CONNECT_LED, LOW); digitalWrite(DISCONNECT_LED, HIGH);
  digitalWrite(BATTERY_LED1, LOW); digitalWrite(BATTERY_LED2, LOW); digitalWrite(BATTERY_LED3, LOW); digitalWrite(BATTERY_LED4, LOW);

  // WiFi and ESP-NOW initialization with maximum performance
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  // CRITICAL: Disable WiFi power saving for maximum performance
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  
  delayMicroseconds(100000); // 100ms in microseconds

  // Optimize for maximum speed and reliability
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_54M);

  esp_now_init();
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&peerInfo);
  
  // Read initial switch states
  for (int i = 0; i < 4; i++) {
    currentSwitchState_DEV[i] = digitalRead(i == 0 ? SW1_DEV : i == 1 ? SW2_DEV : i == 2 ? SW3_DEV : SW4_DEV);
    lastSwitchState_DEV[i] = currentSwitchState_DEV[i];
  }
  
  // Initialize connection timestamp
  lastDataReceived = micros();
}

// =================================================================
// MAIN LOOP - MAXIMUM EFFICIENCY WITH MICROSECOND TIMING
// =================================================================
void loop() {
  unsigned long currentTime = micros();
  
  // PRIORITY 1: Handle joystick movement (highest frequency - every loop)
  handleJoystickMovement();
  
  // PRIORITY 2: Monitor connection status
  if (currentTime - lastConnectionCheck >= CONNECTION_CHECK_US) {
    updateConnectionStatus();
    lastConnectionCheck = currentTime;
  }
  
  // PRIORITY 3: Handle robot switches (medium frequency)
  handleRobotSwitches();
  
  // PRIORITY 4: Handle device switches (controlled frequency)
  static unsigned long lastDeviceCheck = 0;
  if (currentTime - lastDeviceCheck >= DEVICE_CHECK_US) {
    handleDeviceSwitches();
    lastDeviceCheck = currentTime;
  }
  
  // PRIORITY 5: Handle retries (lower frequency)
  static unsigned long lastRetryCheck = 0;
  if (currentTime - lastRetryCheck >= RETRY_CHECK_US) {
    handleRetries();
    lastRetryCheck = currentTime;
  }
  
  // PRIORITY 6: Update battery display (lowest frequency)
  static unsigned long lastBatteryUpdate = 0;
  if (currentTime - lastBatteryUpdate >= STATUS_UPDATE_US) {
    updateBatteryDisplay();
    lastBatteryUpdate = currentTime;
  }
  
  // NO DELAY - Maximum efficiency
}

// =================================================================
// DEVICE REMOTE FUNCTIONS - MICROSECOND OPTIMIZED
// =================================================================
void handleDeviceSwitches() {
  unsigned long currentTime = micros();
  int switchPins[] = {SW1_DEV, SW2_DEV, SW3_DEV, SW4_DEV};

  for (int i = 0; i < 4; i++) {
    bool reading = digitalRead(switchPins[i]);
    
    if (reading != lastSwitchState_DEV[i]) {
      lastSwitchTime_DEV[i] = currentTime;
    }
    
    if ((currentTime - lastSwitchTime_DEV[i]) >= DEBOUNCE_DELAY_US) {
      if (reading != currentSwitchState_DEV[i]) {
        currentSwitchState_DEV[i] = reading;
        
        switch (i) {
          case 0: // Pin 15 - Relay 1 (momentary)
            if (reading == HIGH) {
              sendSwitchMessageReliable(1, true);
              lastSendTime_DEV[i] = currentTime;
            }
            break;
            
          case 1: // Pin 04 - Speed Toggle
            sendSwitchMessageReliable(2, reading);
            lastSendTime_DEV[i] = currentTime;
            break;
            
          case 2: // Pin 16 - Relay 2
            sendSwitchMessageReliable(3, reading);
            lastSendTime_DEV[i] = currentTime;
            break;
            
          case 3: // Pin 17 - Emergency Stop
            if (reading == HIGH) {
              sendSwitchMessageReliable(4, true);
              lastSendTime_DEV[i] = currentTime;
            }
            break;
        }
      }
    }
    lastSwitchState_DEV[i] = reading;
  }
}

// Enhanced message sending with microsecond timing
void sendSwitchMessageReliable(int switchNum, bool state) {
  SwitchMessage msg;
  msg.switchNum = switchNum;
  msg.state = state;
  msg.timestamp = micros();
  
  // More retries for critical commands (Relay 1 and Emergency Stop)
  int maxRetries = (switchNum == 1 || switchNum == 4) ? 5 : 3;
  
  bool sent = false;
  for (int i = 0; i < maxRetries && !sent; i++) {
    if (esp_now_send(receiverMAC, (uint8_t*)&msg, sizeof(msg)) == ESP_OK) {
      sent = true;
      int switchIndex = switchNum - 1;
      retryInfo[switchIndex] = {true, micros(), micros(), 1, state};
    } else {
      delayMicroseconds(IMMEDIATE_RETRY_US); // 5ms between immediate retries
    }
  }
}

void handleRetries() {
  unsigned long currentTime = micros();
  for (int i = 0; i < 4; i++) {
    if (retryInfo[i].pending) {
      if (currentTime - retryInfo[i].firstTryTime >= MAX_RETRY_TIME_US || retryInfo[i].attempts >= 5) {
        resetRetry(i);
      } else if (currentTime - retryInfo[i].lastTryTime >= RETRY_INTERVAL_US) {
        SwitchMessage msg = {i + 1, retryInfo[i].state, micros()};
        if (esp_now_send(receiverMAC, (uint8_t*)&msg, sizeof(msg)) == ESP_OK) {
          retryInfo[i].lastTryTime = currentTime;
          retryInfo[i].attempts++;
        }
      }
    }
  }
}

void resetRetry(int switchIndex) {
  retryInfo[switchIndex] = {false, 0, 0, 0, false};
}

// MODIFIED: LED4_DEV (pin 5) is INVERTED
void updateDeviceLED(int switchNum, bool deviceOn) {
  switch (switchNum) {
    case 1: digitalWrite(LED1_DEV, deviceOn ? HIGH : LOW); break;
    case 2: digitalWrite(LED2_DEV, deviceOn ? HIGH : LOW); break;
    case 3: digitalWrite(LED3_DEV, deviceOn ? HIGH : LOW); break;
    case 4: digitalWrite(LED4_DEV, deviceOn ? LOW : HIGH); break; // INVERTED: LOW = ON, HIGH = OFF
  }
}

// =================================================================
// ROBOT CONTROLLER FUNCTIONS - MICROSECOND OPTIMIZED
// =================================================================
void handleRobotSwitches() {
  static unsigned long lastDebounceTime[4] = {0, 0, 0, 0};
  unsigned long currentTime = micros();
  
  bool currentSW1 = digitalRead(SW_RELAY1_ROBOT);
  if (currentSW1 == LOW && lastRobotSW1State == HIGH && (currentTime - lastDebounceTime[0] >= ROBOT_DEBOUNCE_US)) {
    sendFastCommandReliable(CMD_RELAY1);
    lastDebounceTime[0] = currentTime;
  }
  lastRobotSW1State = currentSW1;

  bool currentSW2 = digitalRead(SW_RELAY2_ROBOT);
  if (currentSW2 == LOW && lastRobotSW2State == HIGH && (currentTime - lastDebounceTime[1] >= ROBOT_DEBOUNCE_US)) {
    sendFastCommandReliable(CMD_RELAY2);
    lastDebounceTime[1] = currentTime;
  }
  lastRobotSW2State = currentSW2;

  bool currentSW3 = digitalRead(SW_SOFTSTART_ROBOT);
  if (currentSW3 == LOW && lastRobotSW3State == HIGH && (currentTime - lastDebounceTime[2] >= ROBOT_DEBOUNCE_US)) {
    sendFastCommandReliable(CMD_RELAY1);
    lastDebounceTime[2] = currentTime;
  }
  lastRobotSW3State = currentSW3;

  bool currentSW4 = digitalRead(SW_SPEED_ROBOT);
  if (currentSW4 != lastRobotSW4State && (currentTime - lastDebounceTime[3] >= ROBOT_DEBOUNCE_US)) {
    currentSpeedHigh = (currentSW4 == LOW);
    sendFastCommandReliable(CMD_SPEED);
    lastDebounceTime[3] = currentTime;
  }
  lastRobotSW4State = currentSW4;
}

void handleJoystickMovement() {
  int vrx1Value = analogRead(VRX1);
  int vrx2Value = analogRead(VRX2);
  
  uint8_t move1 = getJoystickDirection(vrx1Value, false);
  uint8_t move2 = getJoystickDirection(vrx2Value, true);
  
  if (move1 != lastMove1 || move2 != lastMove2) {
    lastMove1 = move1;
    lastMove2 = move2;
    processMovementLogic(move1, move2);
  }
  
  // Send heartbeat if joysticks are neutral
  if (currentCombinedMove == MOVE_NEUTRAL && micros() - lastHeartbeat >= HEARTBEAT_INTERVAL_US) {
      lastHeartbeat = micros();
      sendFastCommand(MOVE_NEUTRAL);
  }
}

void processMovementLogic(uint8_t move1, uint8_t move2) {
    uint8_t newMove = MOVE_NEUTRAL;
    if (move1 == MOVE_FORWARD1 && move2 == MOVE_FORWARD2) newMove = MOVE_FORWARD;
    else if (move1 == MOVE_FORWARD1 && move2 == MOVE_NEUTRAL) newMove = MOVE_FORWARD1;
    else if (move1 == MOVE_NEUTRAL && move2 == MOVE_FORWARD2) newMove = MOVE_FORWARD2;
    else if (move1 == MOVE_BACKWARD1 && move2 == MOVE_BACKWARD2) newMove = MOVE_BACKWARD;
    else if (move1 == MOVE_NEUTRAL && move2 == MOVE_BACKWARD2) newMove = MOVE_BACKWARD2;
    else if (move1 == MOVE_BACKWARD1 && move2 == MOVE_NEUTRAL) newMove = MOVE_BACKWARD1;
    else if (move1 == MOVE_BACKWARD1 && move2 == MOVE_FORWARD2) newMove = MOVE_ROTATE_CW;
    else if (move1 == MOVE_FORWARD1 && move2 == MOVE_BACKWARD2) newMove = MOVE_ROTATE_CCW;

    if (newMove != currentCombinedMove) {
        currentCombinedMove = newMove;
        sendFastCommand(newMove);
        lastHeartbeat = micros();
    }
}

uint8_t getJoystickDirection(int analogValue, bool isSecondJoystick) {
  if (analogValue > JOYSTICK_CENTER + JOYSTICK_DEADZONE) return isSecondJoystick ? MOVE_FORWARD2 : MOVE_FORWARD1;
  if (analogValue < JOYSTICK_CENTER - JOYSTICK_DEADZONE) return isSecondJoystick ? MOVE_BACKWARD2 : MOVE_BACKWARD1;
  return MOVE_NEUTRAL;
}

// Enhanced fast command sending for critical commands
void sendFastCommandReliable(uint8_t command) {
  robotDataToSend.move = command;
  robotDataToSend.flags = 0;
  
  // Try multiple times for critical commands
  for (int i = 0; i < 3; i++) {
    if (esp_now_send(receiverMAC, (uint8_t *) &robotDataToSend, sizeof(robotDataToSend)) == ESP_OK) {
      break;
    }
    delayMicroseconds(2000); // 2ms between retries
  }
}

void sendFastCommand(uint8_t command) {
  robotDataToSend.move = command;
  robotDataToSend.flags = 0;
  esp_now_send(receiverMAC, (uint8_t *) &robotDataToSend, sizeof(robotDataToSend));
}

void updateConnectionStatus() {
  bool shouldBeConnected = (micros() - lastDataReceived < CONNECTION_TIMEOUT_US) && (lastDataReceived > 0);
  
  if (shouldBeConnected != isConnected) {
    isConnected = shouldBeConnected;
    digitalWrite(CONNECT_LED, isConnected);
    digitalWrite(DISCONNECT_LED, !isConnected);
    
    // If connection lost, try to re-establish
    if (!isConnected) {
      // Send emergency neutral command
      sendFastCommand(MOVE_NEUTRAL);
    }
  }
}

void updateBatteryDisplay() {
  digitalWrite(BATTERY_LED1, LOW); digitalWrite(BATTERY_LED2, LOW);
  digitalWrite(BATTERY_LED3, LOW); digitalWrite(BATTERY_LED4, LOW);

  if (batteryVoltage <= 0) return;

  if (batteryVoltage < 10.5) {
      if((micros() / 1000) % 1000 < 500) { // Convert to milliseconds for modulo
          digitalWrite(BATTERY_LED1, HIGH);
      }
  } else {
      if (batteryVoltage >= 10.5) digitalWrite(BATTERY_LED1, HIGH);
      if (batteryVoltage >= 11.1) digitalWrite(BATTERY_LED2, HIGH);
      if (batteryVoltage >= 11.8) digitalWrite(BATTERY_LED3, HIGH);
      if (batteryVoltage >= 12.4) digitalWrite(BATTERY_LED4, HIGH);
  }
}
