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

// Create an RF24 object: CE pin 9, CSN pin 8
RF24 radio(9, 8); 

// Communication address
const byte address[6] = "00001"; 

void setup() {
  Serial.begin(9600);
  if (!radio.begin()) {
    Serial.println("Radio hardware not responding!");
    while (1); // Halt if hardware is not connected
  }
  
  radio.setPALevel(RF24_PA_LOW);    // Low power for testing to prevent saturation
  radio.setChannel(108);           // High frequency to avoid WiFi interference
  radio.setDataRate(RF24_250KBPS); // Lower data rate for better range/reliability
  radio.openWritingPipe(address);  // Set destination address
  radio.stopListening();           // Set module as transmitter
  
  Serial.println("Transmitter Initialized.");
}

void loop() {
  const char text[] = "FREAKBEAR"; 
  bool success = radio.write(&text, sizeof(text)); // Send data
  
  if (success) {
    Serial.println("Sent: FREAKBEAR");
  } else {
    Serial.println("Send failed.");
  }
  
  delay(1000); // Wait 1 second between transmissions
}