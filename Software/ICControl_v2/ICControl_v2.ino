#include <stdint.h>
#include <stdbool.h>
#include <SPI.h>

#define SERIALBUFSIZE 8

// Decoder Pin Definitions
#define G1Pin 10 // Controls G1 (Enable). Active HIGH to drive Y outputs LOW.
#define PinA 9   // Select Input A (LSB)
#define PinB 8   // Select Input B
#define PinC 7   // Select Input C (MSB)

uint8_t serialBuffer[SERIALBUFSIZE] = { '\0' };
uint8_t serialOutput;
uint8_t deviceID; // Variable to store selected device (1-4)
bool isPhaseShifter = false;
bool isAttenuator = false;

int waitForChar() {
  // Wait indefinitely until at least one character is available
  while (Serial.available() == 0) {
    delay(1); // [cite_start]// avoid busy-waiting [cite: 2]
  }

  // Now read until newline or buffer full
  int bytesRead = Serial.readBytesUntil('\n', serialBuffer, SERIALBUFSIZE - 1);
  if (bytesRead <= 0) return 0; // [cite: 4]

  serialBuffer[bytesRead] = '\0';  // Null-terminate for safety
  Serial.print("Received Command: ");

  return (uint8_t)(serialBuffer[0]);
}


uint8_t waitForUInt() {
  // Wait indefinitely until at least one character is available
  while (Serial.available() == 0) {
    delay(1);
  }

  // Read the string until newline
  int bytesRead = Serial.readBytesUntil('\n', serialBuffer, SERIALBUFSIZE - 1);
  if (bytesRead <= 0) return 0; 

  serialBuffer[bytesRead] = '\0';  // Null-terminate

  Serial.print("Received Command: \r\n");
  return (uint8_t)atoi(serialBuffer); 
}

// Function to set the decoder inputs (A, B, C)
void setDecoderLines(uint8_t targetY) {
  // targetY is the decoder output index (0-7) we want to select
  // A is LSB (bit 0), B is bit 1, C is MSB (bit 2)
  digitalWrite(PinA, (targetY & 0x01));
  digitalWrite(PinB, (targetY & 0x02) >> 1);
  digitalWrite(PinC, (targetY & 0x04) >> 2);
}

void SPITransmit(uint8_t *SPIBuf, int bufLength) {
  // MODIFIED FOR DECODER LOGIC:
  // To assert Chip Select (Active Low) on the device, we must set G1 HIGH.
  // G1 = H -> Selected Y output = L (Active)
  // G1 = L -> All Y outputs = H (Inactive)
  
  digitalWrite(G1Pin, HIGH); // Enable the decoder (Active State)
  SPI.transfer(SPIBuf, bufLength);
  delay(1);  // wait 1ms
  digitalWrite(G1Pin, LOW);  // Disable the decoder (Idle State)
}

void setup() {
  Serial.begin(9600);
  SPI.begin();
  
  // Initialize Decoder Pins
  pinMode(G1Pin, OUTPUT);
  pinMode(PinA, OUTPUT);
  pinMode(PinB, OUTPUT);
  pinMode(PinC, OUTPUT);
  
  // Set G1 Low initially so all decoder outputs are High (Inactive)
  digitalWrite(G1Pin, LOW); 
}

void loop() {
  while (1) {
    isPhaseShifter = false;
    isAttenuator = false;
    deviceID = 0;

    while (Serial.available()) Serial.read();

    Serial.print("Input 'P' for phase shifter, 'A' for attenuator \r\n");
    char modeChar = waitForChar();
    
    if (modeChar == 'P') {
      isPhaseShifter = true;
    } else if (modeChar == 'A') {
      isAttenuator = true;
    } else {
      Serial.print("Invalid Input. \r\n");
      continue;
    }

    Serial.print("Input Device Number (1-4): \r\n");
    deviceID = waitForUInt();
    
    // Validate device range
    if (deviceID < 1 || deviceID > 4) {
      Serial.print("Invalid Device Number. Must be 1-4.\r\n");
      continue;
    }

    // Determine target Decoder Output (Y0-Y7)
    uint8_t targetY = 0;
    
    if (isAttenuator) {
      // Attenuators use LSBFIRST [cite: 14]
      SPI.beginTransaction(SPISettings(1e6, LSBFIRST, SPI_MODE0));
      
      // Mapping from User Prompt:
      // Y0 -> AT_LE4, Y1 -> AT_LE3, Y2 -> AT_LE2, Y3 -> AT_LE1
      // Device 1 -> Y3, Device 4 -> Y0
      targetY = 4 - deviceID; 
      
    } else if (isPhaseShifter) {
      // Phase Shifters use MSBFIRST [cite: 13]
      SPI.beginTransaction(SPISettings(1e6, MSBFIRST, SPI_MODE0));
      
      // Mapping from User Prompt:
      // Y4 -> PS_LE4, Y5 -> PS_LE3, Y6 -> PS_LE2, Y7 -> PS_LE1
      // Device 1 -> Y7, Device 4 -> Y4
      targetY = 8 - deviceID;
    }

    // Configure the select lines A, B, C
    setDecoderLines(targetY);

    Serial.print("Input SPI Command for DUT as integer 0-255: \r\n");
    serialOutput = waitForUInt();
    
    if (isPhaseShifter) {
      uint8_t spiCommand[] = { serialOutput };
      SPITransmit(spiCommand, 1);
    } else if (isAttenuator) {
      // Address byte is now always 0 for attenuators
      uint8_t spiCommand[] = { serialOutput, 0x00 }; 
      SPITransmit(spiCommand, 2);
    } 

    Serial.print("Command Sent \r\n");
    SPI.endTransaction(); // Good practice to end transaction
    delay(100);
  }
}