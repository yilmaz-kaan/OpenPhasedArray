#include <stdint.h>
#include <stdbool.h>
#include <SPI.h>

#define SERIALBUFSIZE 8
#define CSPin 10
#define ADDR 0x00

uint8_t serialBuffer[SERIALBUFSIZE] = {'\0'};
uint8_t serialOutput;
uint8_t phaseShifter;
uint8_t attenuator;

int waitForEnter() {
  int bytesRead = Serial.readBytesUntil('\n', serialBuffer, SERIALBUFSIZE);
  serialBuffer[bytesRead] = '\0';  // Null-terminate the string just in case

  Serial.print("Received Command: ");
  Serial.println(serialBuffer);

  return (uint8_t)atoi(serialBuffer);  // Convert to uint8
}

void SPITransmit(uint8_t *SPIBuf, int bufLength) {
  digitalWrite(CSPin, LOW);
  SPI.transfer(SPIBuf, bufLength);
  delay(1); // wait 1ms
  digitalWrite(CSPin, LOW);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600)
  SPI.begin()
  pinMode(CSPin, OUTPUT);
  digitalWrite(CSPin, HIGH); // set high initially
}

void loop() {
  // put your main code here, to run repeatedly:
  phaseShifter = 0;
  attenuator = 0;

  Serial.print("Input 'P' for phase shifter, 'A' for attenuator");
  serialOutput = waitForEnter();
  if (serialOutput == 'P') {
    SPI.beginTransaction(SPISettings(1e6, MSBFIRST, SPI_MODE0));
    phaseShifter = 1;
  } else if (serialOutput == 'A') {
    SPI.beginTransaction(SPISettings(1e6, LSBFIRST, SPI_MODE0)); 
    attenuator = 1;
  } else {continue;} // try again if invalid input

  Serial.print("Input SPI Command for DUT as integer 0-255: ");
  serialOutput = waitForEnter();
  if (phaseShifter) {
    uint8_t spiCommand[] = {serialOutput};
    SPITransmit(spiCommand, 1);
    }
  else if (attenuator) {
    uint8_t spiCommand[] = {serialOutput, ADDR};
    SPITransmit(spiCommand, 2);
  else (continue;)

  Serial.print("Command Sent");
  delay(100);
  }





}
