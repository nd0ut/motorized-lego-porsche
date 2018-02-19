#include <ArduinoJson.h>
#include <RF24.h>
#include <SPI.h>
#include <Servo.h>
#include <nRF24L01.h>

// motor driver
#define MOTOR_CS A6
#define MOTOR_EN A7
#define MOTOR_INA 4
#define MOTOR_INB 5
#define MOTOR_PWM 6

int maxSpeed = 50;
int currentSpeed = 0;
int targetSpeed = 0;
int motorDir = 1;
int motorCurrent = 0;
bool motorStuck = false;
int stuckTime = 0;

// servo
#define SERVO_PWM 10
int servoAngle = 0;
Servo servo;

// radio
#define RADIO_CE 7
#define RADIO_CS 6

RF24 radio(RADIO_CE, RADIO_CS);
int rcData[2];

// time
int deltaTime = 0;
float damping = 0.5;

// logging
const size_t bufferSize = JSON_OBJECT_SIZE(200);
DynamicJsonBuffer jsonBuffer(bufferSize);

void setupMotor() {
  pinMode(MOTOR_INA, OUTPUT);
  pinMode(MOTOR_INB, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);

  digitalWrite(MOTOR_EN, HIGH);
}

void setupRadio() {
  radio.begin();
  radio.setChannel(5);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openReadingPipe(1, 0x1234567890LL);
  radio.startListening();
}

void setupServo() { servo.attach(SERVO_PWM); }

void setup() {
  Serial.begin(9600);

  setupMotor();
  setupServo();
  setupRadio();

  calcDeltaTime();
}

void logState() {
  JsonObject &root = jsonBuffer.createObject();
  root["currentSpeed"] = currentSpeed;
  root["targetSpeed"] = targetSpeed;
  root["motorCurrent"] = motorCurrent;
  root["motorDir"] = motorDir;
  root["motorStuck"] = motorStuck;

  root["servoAngle"] = servoAngle;
  root["deltaTime"] = deltaTime;
  root["stuckTime"] = stuckTime;

  root.printTo(Serial);
}

float lerp(float v0, float v1, float t) { return v0 * (1 - t) + v1 * t; }

void calcDeltaTime() { deltaTime = millis() - deltaTime; }

void setDirection(int dir) {
  if (dir > 0) {
    digitalWrite(MOTOR_INA, LOW);
    digitalWrite(MOTOR_INB, HIGH);
  } else if (dir < 0) {
    digitalWrite(MOTOR_INA, HIGH);
    digitalWrite(MOTOR_INB, LOW);
  } else {
    digitalWrite(MOTOR_INA, LOW);
    digitalWrite(MOTOR_INB, LOW);
  }
}

float getMotorCurrent() {
  int sensorValue = analogRead(MOTOR_CS);
  float voltage = sensorValue * (5.0 / 1023.0);
  float amperage = voltage / 0.13;

  return amperage;
}

bool stuckDetection() {
  motorCurrent = getMotorCurrent();

  if (motorCurrent > 20) {
    stuckTime += deltaTime;
  } else {
    stuckTime = 0;
  }

  if (stuckTime > 1000) {
    currentSpeed = 0;
    targetSpeed = 0;
    setDirection(0);
    analogWrite(MOTOR_PWM, 0);
    return true;
  }

  return false;
}

void updateSpeed(int value) {
  value = constrain(value, 0, 1023);

  int idle = 1023 / 2;
  int dir = value < idle ? -1 : 1;
  int amount = abs(value - idle);
  int speed = map(amount, 0, idle, 0, maxSpeed);

  motorStuck = stuckDetection();

  if (!motorStuck) {
    targetSpeed = speed;
    currentSpeed = lerp(currentSpeed, targetSpeed, damping * deltaTime);

    setDirection(dir);
    analogWrite(MOTOR_PWM, currentSpeed);
  }
}

void updateAngle(int value) {
  value = constrain(value, 0, 1023);
  int angle = map(value, 0, 1023, 0, 180);
  myservo.write(angle);
}

void readRadio() {
  if (radio.available()) {
    radio.read(&rcData, sizeof(rcData));

    int angleValue = rcData[0];
    int speedValue = rcData[1];

    updateSpeed(angleValue);
    updateAngle(speedValue);
  }
}

void loop() {
  readRadio();
  logState();

  calcDeltaTime();
}
