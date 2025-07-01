#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// =================================================================
// CONFIGURATION
// =================================================================
uint8_t senderMACAddress[] = {0xcc, 0xdb, 0xA7, 0x30, 0x09, 0xe8};

// =================================================================
// PIN DEFINITIONS
// =================================================================
// Robot Controller Pins
#define EN1 21
#define RPWM1 23
#define LPWM1 22
#define EN2 14
#define RPWM2 13
#define LPWM2 12
#define RELAY1 18
#define RELAY2 5
#define BatteryPin 32
#define SOFT_START_PWM_PIN 19
#define Right_Front_Sensor 34
#define Left_Front_Sensor 39
#define Right_Rear_Sensor 35
#define Left_Rear_Sensor 36

// Device Controller Pins
#define DEVICE2_PIN 26

// =================================================================
// TIMING CONSTANTS (ALL IN MICROSECONDS)
// =================================================================
const unsigned long HEARTBEAT_TIMEOUT_US = 2500000;    // 2.5 seconds in microseconds
const unsigned long HEARTBEAT_CHECK_US = 100000;       // 100ms in microseconds
const unsigned long STATUS_SEND_US = 600000;           // 600ms in microseconds
const unsigned long RELAY_SETTLE_US = 10000;           // 10ms relay settle time
const unsigned long EMI_SETTLE_US = 30000;             // 30ms EMI settle time
const unsigned long MOVEMENT_STOP_US = 20000;          // 20ms movement stop time
const unsigned long RELAY_OPERATION_DEFER_US = 100000; // 100ms relay operation defer
const unsigned long RELAY_FORCE_US = 500000;           // 500ms force relay operation
const unsigned long SOFT_START_STEP_US = 4000;         // 4ms per soft start step (1/10 of original 40ms)
const unsigned long BATTERY_SAMPLE_US = 300;           // 300µs between battery samples
const unsigned long ACK_RETRY_US = 2000;               // 2ms between ACK retries
const unsigned long STATUS_RETRY_US = 5000;            // 5ms between status retries

// =================================================================
// PWM CONFIGURATION
// =================================================================
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;

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
} ultra_fast_message;

typedef struct __attribute__((packed)) {
  char relayStatus[16];
  char batteryVoltage[8];
  char speedPin[4];
} status_message;

// =================================================================
// GLOBAL VARIABLES
// =================================================================
bool deviceStates[4] = {false, false, false, false};
const float R1 = 3.3, R2 = 1.5;
const int numReadings = 15;
bool relay1On = false, relay2On = false, SpeedStatus = false;
int pwmPercentage = 60;
float batteryVoltageAverage = 0;
uint8_t currentMove = 'n';
bool isMoving = false;
bool sensorsEnabled = false;
bool invertSensorLogic = false;

// Connection monitoring with microsecond precision
unsigned long lastHeartbeat = 0;
bool connectionLost = false;

// EMI protection - delay relay switching during movement
bool relayOperationPending = false;
unsigned long relayOperationTime = 0;
int pendingRelayNum = 0;
bool pendingRelayState = false;

const uint8_t CMD_NEUTRAL = 'n', CMD_FORWARD = 'f', CMD_BACKWARD = 'b', CMD_FORWARD1 = '1',
              CMD_FORWARD2 = '2', CMD_BACKWARD1 = '3', CMD_BACKWARD2 = '4', CMD_ROTATE_CW = 'r',
              CMD_ROTATE_CCW = 'R', CMD_RELAY1 = 'B', CMD_RELAY2 = 'V', CMD_SPEED = 'P', CMD_STOP = 'S';

ultra_fast_message receivedFastData;
status_message dataToSend;

// =================================================================
// SETUP
// =================================================================
void setup() {
  // Pin initialization
  pinMode(DEVICE2_PIN, OUTPUT); digitalWrite(DEVICE2_PIN, LOW);
  
  setupMotorPins();
  pinMode(RELAY1, OUTPUT); digitalWrite(RELAY1, LOW);
  pinMode(RELAY2, OUTPUT); digitalWrite(RELAY2, HIGH);
  setupSoftStartPWM();
  
  pinMode(Right_Front_Sensor, INPUT); pinMode(Left_Front_Sensor, INPUT);
  pinMode(Right_Rear_Sensor, INPUT); pinMode(Left_Rear_Sensor, INPUT);
  noMovement();

  // WiFi & ESP-NOW initialization with maximum performance
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  // CRITICAL: Disable WiFi power saving for maximum performance
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  
  delayMicroseconds(100000); // 100ms in microseconds

  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_now_init();
  esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_54M);
  esp_now_register_recv_cb(onDataRecv_merged);
  esp_now_register_send_cb(onDataSent_merged);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, senderMACAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&peerInfo);

  // Initialize connection timestamp
  lastHeartbeat = micros();
}

void setupMotorPins() {
  pinMode(EN1, OUTPUT); pinMode(RPWM1, OUTPUT); pinMode(LPWM1, OUTPUT);
  pinMode(EN2, OUTPUT); pinMode(RPWM2, OUTPUT); pinMode(LPWM2, OUTPUT);
}

void setupSoftStartPWM() {
  ledcAttach(SOFT_START_PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(SOFT_START_PWM_PIN, 0);
}

// =================================================================
// MAIN LOOP - MAXIMUM EFFICIENCY WITH MICROSECOND TIMING
// =================================================================
void loop() {
  unsigned long currentTime = micros();
  static unsigned long lastStatusSendTime = 0;
  static unsigned long lastHeartbeatCheckTime = 0;

  // PRIORITY 1: Connection monitoring (highest priority)
  if (currentTime - lastHeartbeatCheckTime >= HEARTBEAT_CHECK_US) {
    checkConnection();
    lastHeartbeatCheckTime = currentTime;
  }

  // PRIORITY 2: Safety check (critical for motor control systems)
  if (sensorsEnabled && !checkSensorSafety()) {
    if (isMoving) {
      emergencyStop();
    }
  }

  // PRIORITY 3: Handle pending relay operations (to avoid EMI during movement)
  if (relayOperationPending && currentTime - relayOperationTime >= RELAY_OPERATION_DEFER_US) {
    if (!isMoving || currentTime - relayOperationTime >= RELAY_FORCE_US) {
      executeRelayOperation();
    }
  }

  // PRIORITY 4: Send status data (optimized frequency)
  if (currentTime - lastStatusSendTime >= STATUS_SEND_US) {
    lastStatusSendTime = currentTime;
    
    // Ultra-fast battery reading with microsecond timing
    float total = 0;
    for (int i = 0; i < numReadings; i++) {
      total += (analogRead(BatteryPin) * (12.3 / 4096) * (R1 + R2)) / R2;
      delayMicroseconds(BATTERY_SAMPLE_US); // 300µs between samples
    }
    batteryVoltageAverage = total / numReadings;
    sendStatusData();
  }

  // NO DELAY - Maximum efficiency for movement commands
}

// =================================================================
// CONNECTION MONITORING WITH MICROSECOND PRECISION
// =================================================================
void checkConnection() {
  bool wasConnected = !connectionLost;
  connectionLost = (micros() - lastHeartbeat >= HEARTBEAT_TIMEOUT_US);
  
  if (connectionLost && wasConnected) {
    // Connection just lost - emergency stop
    emergencyStop();
  }
}

// =================================================================
// ESP-NOW CALLBACKS - ULTRA-OPTIMIZED
// =================================================================
void onDataRecv_merged(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  lastHeartbeat = micros(); // Update heartbeat with microsecond precision
  connectionLost = false;   // Reset connection lost flag
  
  if (len == sizeof(SwitchMessage)) {
    handleSwitchMessage(info, data, len);
  } else if (len == sizeof(ultra_fast_message)) {
    handleRobotMessage(info, data, len);
  }
}

void onDataSent_merged(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // No status LED feedback - removed for optimization
}

// =================================================================
// MESSAGE HANDLERS - WITH MICROSECOND EMI PROTECTION
// =================================================================
void handleSwitchMessage(const esp_now_recv_info *recv_info, const uint8_t *data, int len) {
  SwitchMessage msg;
  memcpy(&msg, data, sizeof(msg));

  if (msg.switchNum < 1 || msg.switchNum > 4) return;
  
  int switchIndex = msg.switchNum - 1;
  
  // Handle relay operations with microsecond EMI protection
  if (msg.switchNum == 1 && msg.state) { // Relay 1 operation
    if (isMoving) {
      // Defer relay operation to avoid EMI during movement
      relayOperationPending = true;
      relayOperationTime = micros();
      pendingRelayNum = msg.switchNum;
      pendingRelayState = msg.state;
    } else {
      controlDevice(msg.switchNum, msg.state);
    }
  } else {
    controlDevice(msg.switchNum, msg.state);
  }
  
  bool ackState = deviceStates[switchIndex];
  sendAck(msg.switchNum, ackState);
}

void handleRobotMessage(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  memcpy(&receivedFastData, data, sizeof(receivedFastData));
  handleMovement();
}

void executeRelayOperation() {
  controlDevice(pendingRelayNum, pendingRelayState);
  relayOperationPending = false;
}

void sendAck(int switchNum, bool ackStatus) {
  AckMessage ackMsg;
  ackMsg.switchNum = switchNum;
  ackMsg.ack = ackStatus;
  
  // Try multiple times for ACK reliability with microsecond timing
  for (int i = 0; i < 3; i++) {
    if (esp_now_send(senderMACAddress, (uint8_t*)&ackMsg, sizeof(ackMsg)) == ESP_OK) {
      break;
    }
    delayMicroseconds(ACK_RETRY_US); // 2ms between retries
  }
}

// =================================================================
// DEVICE CONTROL - WITH MICROSECOND EMI PROTECTION
// =================================================================
void controlDevice(int switchNum, bool turnOn) {
  int switchIndex = switchNum - 1;

  switch (switchNum) {
    case 1: { // Relay 1 + Soft Start - FIXED: Added braces for proper scoping
      if (turnOn) {
        // Stop movement temporarily to reduce EMI with microsecond timing
        bool wasMoving = isMoving;
        if (wasMoving) {
          noMovement();
          delayMicroseconds(MOVEMENT_STOP_US); // 20ms movement stop
        }
        
        relay1On = !relay1On;
        deviceStates[switchIndex] = relay1On;
        
        if (relay1On) {
          digitalWrite(RELAY1, HIGH);
          delayMicroseconds(RELAY_SETTLE_US); // 10ms relay settle
          softStart();
        } else {
          stopSoftStart();
          digitalWrite(RELAY1, LOW);
        }
        
        delayMicroseconds(EMI_SETTLE_US); // 30ms EMI settle
      }
      break;
    }

    case 2: // Speed Toggle
      SpeedStatus = !SpeedStatus;
      pwmPercentage = SpeedStatus ? 100 : 60;
      deviceStates[switchIndex] = SpeedStatus;
      break;

    case 3: // Relay 2
      relay2On = !relay2On;
      digitalWrite(RELAY2, relay2On ? LOW : HIGH);
      deviceStates[switchIndex] = relay2On;
      break;

    case 4: // Emergency Stop
      if (turnOn) {
        emergencyStop();
        deviceStates[switchIndex] = false;
      }
      break;
  }
}

// =================================================================
// MOVEMENT FUNCTIONS - ULTRA-OPTIMIZED
// =================================================================
void setMotor(int enPin, int rpwmPin, int lpwmPin, int pwmValue, bool forward) {
  int mappedPWM = map(pwmValue, 0, 100, 0, 255);
  analogWrite(enPin, mappedPWM);
  digitalWrite(rpwmPin, forward ? HIGH : LOW);
  digitalWrite(lpwmPin, forward ? LOW : HIGH);
}

void moveMotor(bool forward) {
  setMotor(EN1, RPWM1, LPWM1, pwmPercentage, forward);
  setMotor(EN2, RPWM2, LPWM2, pwmPercentage, forward);
  isMoving = true;
}

void noMovement() {
  setMotor(EN1, RPWM1, LPWM1, 0, true);
  setMotor(EN2, RPWM2, LPWM2, 0, true);
  isMoving = false;
}

void handleMovement() {
  currentMove = receivedFastData.move;
  
  switch (currentMove) {
    case CMD_FORWARD: moveMotor(true); break;
    case CMD_BACKWARD: moveMotor(false); break;
    case CMD_FORWARD1: setMotor(EN1, RPWM1, LPWM1, pwmPercentage, true); setMotor(EN2, RPWM2, LPWM2, 0, true); isMoving = true; break;
    case CMD_FORWARD2: setMotor(EN1, RPWM1, LPWM1, 0, true); setMotor(EN2, RPWM2, LPWM2, pwmPercentage, true); isMoving = true; break;
    case CMD_BACKWARD1: setMotor(EN1, RPWM1, LPWM1, pwmPercentage, false); setMotor(EN2, RPWM2, LPWM2, 0, true); isMoving = true; break;
    case CMD_BACKWARD2: setMotor(EN1, RPWM1, LPWM1, 0, true); setMotor(EN2, RPWM2, LPWM2, pwmPercentage, false); isMoving = true; break;
    case CMD_ROTATE_CW: setMotor(EN1, RPWM1, LPWM1, pwmPercentage, true); setMotor(EN2, RPWM2, LPWM2, pwmPercentage, false); isMoving = true; break;
    case CMD_ROTATE_CCW: setMotor(EN1, RPWM1, LPWM1, pwmPercentage, false); setMotor(EN2, RPWM2, LPWM2, pwmPercentage, true); isMoving = true; break;
    case CMD_NEUTRAL: noMovement(); break;
    default: handleButtonCommand(); break;
  }
}

// FIXED: Proper scoping for switch-case statement
void handleButtonCommand() {
  switch (currentMove) {
    case CMD_RELAY1: { // FIXED: Added braces for proper scoping
      // Stop movement temporarily to reduce EMI with microsecond timing
      bool wasMoving = isMoving;
      if (wasMoving) {
        noMovement();
        delayMicroseconds(MOVEMENT_STOP_US); // 20ms
      }
      
      relay1On = !relay1On;
      if (relay1On) {
        digitalWrite(RELAY1, HIGH);
        delayMicroseconds(RELAY_SETTLE_US); // 10ms
        softStart();
      } else {
        stopSoftStart();
        digitalWrite(RELAY1, LOW);
      }
      
      delayMicroseconds(EMI_SETTLE_US); // 30ms EMI settle
      break;
    }
      
    case CMD_RELAY2:
      relay2On = !relay2On;
      digitalWrite(RELAY2, relay2On ? LOW : HIGH);
      break;
      
    case CMD_SPEED:
      SpeedStatus = !SpeedStatus;
      pwmPercentage = SpeedStatus ? 100 : 60;
      break;
      
    case CMD_STOP:
      emergencyStop();
      break;
  }
}

void sendStatusData() {
  snprintf(dataToSend.relayStatus, sizeof(dataToSend.relayStatus), "R1:%s R2:%s", relay1On ? "ON" : "OFF", relay2On ? "ON" : "OFF");
  snprintf(dataToSend.speedPin, sizeof(dataToSend.speedPin), SpeedStatus ? "H" : "L");
  snprintf(dataToSend.batteryVoltage, sizeof(dataToSend.batteryVoltage), "%.2f", batteryVoltageAverage);
  
  // Try multiple times for status reliability with microsecond timing
  for (int i = 0; i < 2; i++) {
    if (esp_now_send(senderMACAddress, (uint8_t *)&dataToSend, sizeof(dataToSend)) == ESP_OK) {
      break;
    }
    delayMicroseconds(STATUS_RETRY_US); // 5ms between retries
  }
}

// MODIFIED: Ultra-fast soft start with 1/10 delay (4ms per step instead of 40ms)
void softStart() {
  digitalWrite(RELAY1, HIGH);
  for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle += 5) {
    ledcWrite(SOFT_START_PWM_PIN, dutyCycle);
    delayMicroseconds(SOFT_START_STEP_US); // 4ms per step (1/10 of original 40ms)
  }
  ledcWrite(SOFT_START_PWM_PIN, 255);
}

void stopSoftStart() {
  ledcWrite(SOFT_START_PWM_PIN, 0);
}

void emergencyStop() {
  noMovement();
  stopSoftStart();
  digitalWrite(RELAY1, LOW);
  relay1On = false;
  relay2On = false;
  digitalWrite(RELAY2, HIGH);
  SpeedStatus = false;
  pwmPercentage = 60;
  
  // Clear any pending relay operations
  relayOperationPending = false;
}

bool checkSensorSafety() {
  bool rf_raw = digitalRead(Right_Front_Sensor) == HIGH;
  bool lf_raw = digitalRead(Left_Front_Sensor) == HIGH;
  bool rr_raw = digitalRead(Right_Rear_Sensor) == HIGH;
  bool lr_raw = digitalRead(Left_Rear_Sensor) == HIGH;
  
  bool rf_clear = invertSensorLogic ? !rf_raw : rf_raw;
  bool lf_clear = invertSensorLogic ? !lf_raw : lf_raw;
  bool rr_clear = invertSensorLogic ? !rr_raw : rr_raw;
  bool lr_clear = invertSensorLogic ? !lr_raw : lr_raw;
  
  bool front_safe = !(rf_clear && lf_clear);
  bool rear_safe = !(rr_clear && lr_clear);
  
  return front_safe && rear_safe;
}
