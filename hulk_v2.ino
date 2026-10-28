#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

// =====================================================
// RECEIVER PINS
// =====================================================
#define CH1_PIN 16  // Steer
#define CH2_PIN 17  // Throttle
#define CH3_PIN 18  // Speed Knob (Flysky Channel 5 - VRA)

// =====================================================
// MODE SWITCH
// LOW  = Receiver
// HIGH = Bluetooth
// =====================================================
#define MODE_PIN 4

// =====================================================
// LEFT BTS7960 PINS
// =====================================================
#define L_LPWM 27
#define L_RPWM 14
#define L_LEN 25
#define L_REN 26

// =====================================================
// RIGHT BTS7960 PINS
// =====================================================
#define R_LPWM 13
#define R_RPWM 12
#define R_LEN 33
#define R_REN 32

// =====================================================
// PWM SETTINGS - ESP32 CORE 3.x
// =====================================================
const int pwmFreq = 20000;
const int pwmResolution = 8;   // 0-255

// =====================================================
// RC SETTINGS
// =====================================================
const int RC_MIN = 1000;
const int RC_MAX = 2000;
const int DEADZONE = 30;

const unsigned long FAILSAFE_US = 25000;

// =====================================================
// RC PULSE VARIABLES
// =====================================================
volatile uint32_t ch1Rise = 0;
volatile uint32_t ch2Rise = 0;
volatile uint32_t ch3Rise = 0;

volatile uint32_t ch1Pulse = 1500;
volatile uint32_t ch2Pulse = 1500;
volatile uint32_t ch3Pulse = 1500;

volatile uint32_t ch1Last = 0;
volatile uint32_t ch2Last = 0;
volatile uint32_t ch3Last = 0;

// =====================================================
// BLUETOOTH
// =====================================================
char btCmd = 'S';
unsigned long btLast = 0;
const unsigned long BT_TIMEOUT = 500;


// =====================================================
// CH1 INTERRUPT (STEER)
// =====================================================
void IRAM_ATTR isrCH1() {
  if (digitalRead(CH1_PIN)) {
    ch1Rise = micros();
  } else {
    ch1Pulse = micros() - ch1Rise;
    ch1Last = micros();
  }
}


// =====================================================
// CH2 INTERRUPT (THROTTLE)
// =====================================================
void IRAM_ATTR isrCH2() {
  if (digitalRead(CH2_PIN)) {
    ch2Rise = micros();
  } else {
    ch2Pulse = micros() - ch2Rise;
    ch2Last = micros();
  }
}


// =====================================================
// CH3 INTERRUPT (SPEED KNOB - VRA / CH5)
// =====================================================
void IRAM_ATTR isrCH3() {
  if (digitalRead(CH3_PIN)) {
    ch3Rise = micros();
  } else {
    ch3Pulse = micros() - ch3Rise;
    ch3Last = micros();
  }
}


// =====================================================
// RC VALUE MAPPING
// 1000us = -255
// 1500us = 0
// 2000us = +255
// =====================================================
int mapRC(int us) {
  us = constrain(us, RC_MIN, RC_MAX);
  return map(us, RC_MIN, RC_MAX, -255, 255);
}


// =====================================================
// DEADZONE
// =====================================================
int deadband(int value, int dz) {
  if (abs(value) < dz)
    return 0;

  return value;
}


// =====================================================
// STOP ALL MOTORS
// =====================================================
void stopAll() {
  ledcWrite(L_LPWM, 0);
  ledcWrite(L_RPWM, 0);

  ledcWrite(R_LPWM, 0);
  ledcWrite(R_RPWM, 0);
}


// =====================================================
// DUAL MOTOR STARTUP TONE FUNCTION
// =====================================================
void playMotorStartupSound() {
  // Tone 1: Low frequency on both left and right sides
  ledcAttach(L_LPWM, 1500, pwmResolution);
  ledcAttach(R_LPWM, 1500, pwmResolution);
  ledcWrite(L_LPWM, 10);
  ledcWrite(R_LPWM, 10);
  delay(100);

  // Tone 2: Mid frequency
  ledcAttach(L_LPWM, 2500, pwmResolution);
  ledcAttach(R_LPWM, 2500, pwmResolution);
  ledcWrite(L_LPWM, 12);
  ledcWrite(R_LPWM, 12);
  delay(100);

  // Tone 3: High frequency
  ledcAttach(L_LPWM, 3800, pwmResolution);
  ledcAttach(R_LPWM, 3800, pwmResolution);
  ledcWrite(L_LPWM, 14);
  ledcWrite(R_LPWM, 14);
  delay(150);

  // Re-attach pins back to normal motor PWM frequency (1000Hz)
  ledcAttach(L_LPWM, pwmFreq, pwmResolution);
  ledcAttach(R_LPWM, pwmFreq, pwmResolution);
  
  // Turn off all motor PWM signals completely
  stopAll();
  delay(50);
}


// =====================================================
// LEFT MOTOR
// =====================================================
void setLeftMotor(int speedVal) {
  speedVal = constrain(speedVal, -255, 255);

  if (speedVal > 0) {
    // Forward
    ledcWrite(L_LPWM, speedVal);
    ledcWrite(L_RPWM, 0);
  } else if (speedVal < 0) {
    // Backward
    ledcWrite(L_LPWM, 0);
    ledcWrite(L_RPWM, -speedVal);
  } else {
    // Stop
    ledcWrite(L_LPWM, 0);
    ledcWrite(L_RPWM, 0);
  }
}


// =====================================================
// RIGHT MOTOR
// RIGHT SIDE IS PHYSICALLY INVERTED
// =====================================================
void setRightMotor(int speedVal) {
  speedVal = constrain(speedVal, -255, 255);

  // Invert right side direction
  speedVal = -speedVal;

  if (speedVal > 0) {
    // Forward
    ledcWrite(R_LPWM, speedVal);
    ledcWrite(R_RPWM, 0);
  } else if (speedVal < 0) {
    // Backward
    ledcWrite(R_LPWM, 0);
    ledcWrite(R_RPWM, -speedVal);
  } else {
    // Stop
    ledcWrite(R_LPWM, 0);
    ledcWrite(R_RPWM, 0);
  }
}


// =====================================================
// DRIVE MOTORS
// =====================================================
void driveMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);
}


// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);

  // Bluetooth
  SerialBT.begin("ESP32_RC_BOT");

  // Mode switch
  pinMode(MODE_PIN, INPUT_PULLUP);

  // Receiver pins
  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH3_PIN, INPUT);

  // Receiver interrupts
  attachInterrupt(digitalPinToInterrupt(CH1_PIN), isrCH1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH2_PIN), isrCH2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH3_PIN), isrCH3, CHANGE);


  // ===================================================
  // BTS7960 ENABLE PINS
  // ===================================================
  pinMode(L_LEN, OUTPUT);
  pinMode(L_REN, OUTPUT);

  pinMode(R_LEN, OUTPUT);
  pinMode(R_REN, OUTPUT);

  // Enable BTS7960
  digitalWrite(L_LEN, HIGH);
  digitalWrite(L_REN, HIGH);

  digitalWrite(R_LEN, HIGH);
  digitalWrite(R_REN, HIGH);


  // ===================================================
  // ESP32 CORE 3.x PWM
  // ===================================================
  ledcAttach(L_LPWM, pwmFreq, pwmResolution);
  ledcAttach(L_RPWM, pwmFreq, pwmResolution);
  ledcAttach(R_LPWM, pwmFreq, pwmResolution);
  ledcAttach(R_RPWM, pwmFreq, pwmResolution);

  // Make sure motors are stopped
  stopAll();

  // PLAY MOTOR STARTUP SOUND TONE ON BOTH SIDES
  playMotorStartupSound();

  btLast = millis();

  Serial.println("=================================");
  Serial.println(" HULK BOT STARTED");
  Serial.println(" ESP32 CORE 3.x");
  Serial.println(" RC MODE WITH VRA SPEED CONTROL");
  Serial.println(" DUAL MOTOR SOUND TONE ACTIVE");
  Serial.println("=================================");
}


// =====================================================
// MAIN LOOP
// =====================================================
void loop() {

  bool btMode = false;

  // ===================================================
  // BLUETOOTH MODE
  // ===================================================
  if (btMode) {
    while (SerialBT.available()) {
      char c = SerialBT.read();

      if (c == '\n' || c == '\r')
        continue;

      btCmd = c;
      btLast = millis();
    }

    // Bluetooth failsafe
    if (millis() - btLast > BT_TIMEOUT)
      btCmd = 'S';

    int left = 0;
    int right = 0;

    switch (btCmd) {
      case 'F': left = 180; right = 180; break;
      case 'B': left = -180; right = -180; break;
      case 'L': left = -150; right = 150; break;
      case 'R': left = 150; right = -150; break;
      case 'S':
      default: left = 0; right = 0; break;
    }

    driveMotors(left, right);
    delay(20);
    return;
  }


  // ===================================================
  // READ RECEIVER
  // ===================================================
  unsigned long now = micros();

  bool ch1Ok = (now - ch1Last) < FAILSAFE_US;
  bool ch2Ok = (now - ch2Last) < FAILSAFE_US;


  // ===================================================
  // FAILSAFE
  // ===================================================
  if (!ch1Ok || !ch2Ok) {
    stopAll();
    Serial.println("RC FAILSAFE");
    delay(20);
    return;
  }


  // ===================================================
  // VRA SPEED LIMIT CALCULATION (CH5)
  // ===================================================
  int maxSpeed = map(ch3Pulse, 1000, 2000, 60, 255); // Minimum speed 60, Max 255
  maxSpeed = constrain(maxSpeed, 60, 255);


  // ===================================================
  // READ CHANNELS
  // ===================================================
  int steer = mapRC(ch1Pulse);
  int throttle = mapRC(ch2Pulse);


  // ===================================================
  // DEADZONE
  // ===================================================
  steer = deadband(steer, DEADZONE);
  throttle = deadband(throttle, DEADZONE);


  // ===================================================
  // DIFFERENTIAL MIXING
  // ===================================================
  int left = throttle + steer;
  int right = throttle - steer;


  // ===================================================
  // DYNAMIC SPEED LIMIT ACCORDING TO VRA
  // ===================================================
  left = constrain(left, -maxSpeed, maxSpeed);
  right = constrain(right, -maxSpeed, maxSpeed);


  // ===================================================
  // STOP WHEN STICK IS CENTER
  // ===================================================
  if (abs(throttle) < 20 && abs(steer) < 20) {
    stopAll();
  } else {
    driveMotors(left, right);
  }


  // ===================================================
  // SERIAL MONITOR
  // ===================================================
  Serial.print("CH1 = ");
  Serial.print(ch1Pulse);

  Serial.print(" | CH2 = ");
  Serial.print(ch2Pulse);

  Serial.print(" | VRA Limit = ");
  Serial.print(maxSpeed);

  Serial.print(" | LEFT = ");
  Serial.print(left);

  Serial.print(" | RIGHT = ");
  Serial.println(right);

  delay(20);
}