#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Create an RF24 object: CE pin 9, CSN pin 8 [cite: 67]
RF24 radio(9, 8); 

// Communication address [cite: 69]
const byte address[6] = "00001"; 

void setup() {
  radio.begin();
  radio.setPALevel(0x00); // Low power for testing [cite: 73]
  radio.setChannel(108); // Above most WiFi frequencies [cite: 74, 76]
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(address); // Set destination address [cite: 79]
  radio.stopListening(); // Set module as transmitter [cite: 81]
}

void loop() {
  const char text[] = "FREAKBEAR"; 
  radio.write(&text, sizeof(text)); // Send data [cite: 85, 88]
  delay(1);
}