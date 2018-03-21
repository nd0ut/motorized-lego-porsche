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

int maxSpeed = 125;
float currentSpeed = 0;
float targetSpeed = 0;
int motorDir = 1;
float motorCurrent = 0;
bool motorStuck = false;
int stuckTime = 0;

// servo
#define SERVO_PWM 8
int servoAngle = 0;
Servo servo;

// radio
#define RADIO_CE 9
#define RADIO_CS 10

RF24 radio(RADIO_CE, RADIO_CS);
int rcData[2];

// time
float deltaTime = 0;
float lastTime = 0;
float damping = 1.5;

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
  Serial.begin(115200);

  setupMotor();
  setupServo();
  setupRadio();

  calcDeltaTime();
}

void plotState() {
  plot("currentSpeed", currentSpeed);
  plot("targetSpeed", targetSpeed);
  plot("motorCurrent", motorCurrent);
  plot("motorDir", motorDir);
  plot("motorStuck", motorStuck);

  plot("servoAngle", servoAngle);
  plot("deltaTime", deltaTime);
  plot("stuckTime", stuckTime);
}

void plot(String name, float value) {
  String time = String(millis());
  Serial.println("PLOT[" + time + ";" + name + "=" + value + "]");
}

float lerp(float v0, float v1, float t) { return v0 * (1 - t) + v1 * t; }

void calcDeltaTime() { 
  int time = millis();
  deltaTime = (time - lastTime);
  lastTime = time;
}

void setDirection(int dir) {
  motorDir = dir;
  
  if (dir < 0) {
    digitalWrite(MOTOR_INA, LOW);
    digitalWrite(MOTOR_INB, HIGH);
  } else if (dir > 0) {
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
  if (motorCurrent > 30) {
    stuckTime += deltaTime;
  } else {
    stuckTime = 0;
  }

  if (stuckTime > 3000) {
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
  
  motorCurrent = getMotorCurrent();
  motorStuck = stuckDetection();

  if(motorStuck) {
    currentSpeed = 0;
    targetSpeed = 0;
    setDirection(0);
    analogWrite(MOTOR_PWM, 0);
  } else {
    targetSpeed = speed;
    currentSpeed = speed;
    // currentSpeed = lerp(currentSpeed, targetSpeed, (1 / damping) * (deltaTime / 1000));
    currentSpeed = constrain(currentSpeed, 0, maxSpeed);

    setDirection(dir);
    analogWrite(MOTOR_PWM, currentSpeed);
  }
}

void updateAngle(int value) {
  value = constrain(value, 0, 1023);
  int angle = map(value, 0, 1023, 0, 180);
  servo.write(angle);
}

void readRadio() {
  if (radio.available()) {
    radio.read(&rcData, sizeof(rcData));

    int speedValue = rcData[0];
    int angleValue = rcData[1];

    updateAngle(speedValue);
    updateSpeed(angleValue);
  }
}

void readWired() {
  int speedValue = analogRead(1);

  updateSpeed(speedValue);
  updateAngle(speedValue);
}

void loop() {
  readRadio();
  plotState();

  calcDeltaTime();
}
