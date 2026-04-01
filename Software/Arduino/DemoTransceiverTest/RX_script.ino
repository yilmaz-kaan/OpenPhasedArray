#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// CE on pin 9, CSN on pin 8 [cite: 67]
RF24 radio(9, 8); 

const byte address[6] = "00001"; // Must match transmitter [cite: 69]
const int ledPin = 2;            // LED connected to Digital Pin 2
const char msg[32] = "FREAKBEAR";
void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);       // Set D2 as output
  
  radio.begin();                 // Initialize the module [cite: 104]
  radio.setPALevel(RF24_PA_LOW); // Low power for desk testing [cite: 105]
  radio.setChannel(108);         // Set frequency [cite: 106]
  radio.setDataRate(RF24_250KBPS); // Set data rate [cite: 110]
  radio.openReadingPipe(0, address); // Set receive address [cite: 112]
  radio.startListening();        // Put into receiver mode [cite: 114]
  
  Serial.println("Receiver Ready. Waiting for data...");
}

void loop() {
  // Check if data is available in the buffer 
  if (radio.available()) {
    const char text[32] = {0};
    radio.read(&text, sizeof(text)); // Read the incoming message [cite: 119]
    Serial.println(text);  
    if (strcmp(text, "FREAKBEAR") == 0){
    digitalWrite(ledPin, HIGH);      // Turn LED ON when data arrives
          // Print message to monitor [cite: 120]
    
    // Optional: add a tiny delay so the LED blink is visible to the eye
    delay(500);
    }
  } else {
    digitalWrite(ledPin, LOW);       // Turn LED OFF if no data
  }
}