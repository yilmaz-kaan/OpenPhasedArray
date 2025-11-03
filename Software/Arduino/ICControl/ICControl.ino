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

uint8_t customAddress;

int waitForChar() {
  // Wait indefinitely until at least one character is available
  while (Serial.available() == 0) {
    // optionally, you could add a small delay to avoid busy-waiting
    delay(1);
  }

  // Now read until newline or buffer full
  int bytesRead = Serial.readBytesUntil('\n', serialBuffer, SERIALBUFSIZE - 1);

  if (bytesRead <= 0) return 0;

  serialBuffer[bytesRead] = '\0';  // Null-terminate for safety
  Serial.print("Received Command: ");

  return (uint8_t)(serialBuffer[0]);
}


uint8_t waitForUInt() {
  // Wait indefinitely until at least one character is available
  while (Serial.available() == 0) {
    delay(1);  // avoid busy-waiting
  }

  // Read the string until newline
  int bytesRead = Serial.readBytesUntil('\n', serialBuffer, SERIALBUFSIZE - 1);

  if (bytesRead <= 0) return 0;

  serialBuffer[bytesRead] = '\0';  // Null-terminate

  Serial.print("Received Command: \r\n");

  // Convert ASCII string to integer and return as uint8_t
  return (uint8_t)atoi(serialBuffer);
}


void SPITransmit(uint8_t *SPIBuf, int bufLength) {
  digitalWrite(CSPin, LOW);
  SPI.transfer(SPIBuf, bufLength);
  delay(1); // wait 1ms
  digitalWrite(CSPin, HIGH);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SPI.begin();
  pinMode(CSPin, OUTPUT);
  digitalWrite(CSPin, HIGH); // set high initially
}

void loop() {
  // put your main code here, to run repeatedly:
  while (1) {
    phaseShifter = 0;
    attenuator = 0;
    while (Serial.available()) Serial.read();

    Serial.print("Input 'P' for phase shifter, 'A' for attenuator \r\n");
    serialOutput = waitForChar();
    Serial.print("Received Char. \r\n");
    if (serialOutput == 'P') {
      SPI.beginTransaction(SPISettings(1e6, MSBFIRST, SPI_MODE0));
      phaseShifter = 1;

    } else if (serialOutput == 'A') {
      SPI.beginTransaction(SPISettings(1e6, LSBFIRST, SPI_MODE0)); 
      attenuator = 1;
      Serial.print("Input attuator address 0-7: \r\n");
      customAddress = waitForUInt();
      
    } else {continue;} // try again if invalid input

    Serial.print("Input SPI Command for DUT as integer 0-255: \r\n");
    serialOutput = waitForUInt();
    if (phaseShifter) {
      uint8_t spiCommand[] = {serialOutput};
      SPITransmit(spiCommand, 1);
      }
    else if (attenuator) {
      uint8_t spiCommand[] = {serialOutput, customAddress};
      SPITransmit(spiCommand, 2);
    }
    else {continue;}

    Serial.print("Command Sent \r\n");
    delay(100);
  }
}
