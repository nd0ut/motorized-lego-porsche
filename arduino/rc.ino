#include <RF24.h>
#include <SPI.h>
#include <nRF24L01.h>

#define RADIO_CE 9
#define RADIO_CS 10

#define JOY_ANGLE A0
#define JOY_SPEED A1

RF24 radio(RADIO_CE, RADIO_CS);
int rcData[2];

void setup() {
  radio.begin();
  radio.setChannel(5);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openWritingPipe(0x1234567890LL);
}

void loop() {
  rcData[0] = analogRead(JOY_ANGLE);
  rcData[1] = analogRead(JOY_SPEED);
  radio.write(&rcData, sizeof(rcData));
}
