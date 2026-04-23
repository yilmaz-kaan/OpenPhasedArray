#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/*
 * Pinout Configuration for nRF24L01 Breakout:
 * nRF24L01 Pin | Arduino Pin
 * -------------|------------
 * VCC          | 3.3V (or 5V if using Adapter)
 * GND          | GND
 * CE           | 9
 * CSN          | 8
 * SCK          | 13
 * MOSI         | 11
 * MISO         | 12
 * IRQ          | Not Connected
 */

// CE on pin 9, CSN on pin 8
RF24 radio(9, 8); 

const byte address[6] = "00001"; // Must match transmitter
const int ledPin = 2;            // LED connected to Digital Pin 2

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  
  if (!radio.begin()) {
    Serial.println("Radio hardware not responding!");
    while (1); // Halt if hardware is not connected
  }
  
  radio.setPALevel(RF24_PA_LOW);    // Match transmitter power level
  radio.setChannel(108);           // Match transmitter channel
  radio.setDataRate(RF24_250KBPS); // Match transmitter data rate
  radio.openReadingPipe(1, address); // Set receive address (using pipe 1)
  radio.startListening();        // Put into receiver mode
  
  Serial.println("Receiver Ready. Waiting for data...");
}

void loop() {
  if (radio.available()) {
    char text[32] = {0};
    radio.read(&text, sizeof(text)); // Read the incoming message
    
    Serial.print("Received: ");
    Serial.println(text);  
    
    if (strcmp(text, "FREAKBEAR") == 0) {
      digitalWrite(ledPin, HIGH);    // Turn LED ON
      delay(500);                    // Brief visual feedback
      digitalWrite(ledPin, LOW);     // Turn LED OFF
    }
  }
}