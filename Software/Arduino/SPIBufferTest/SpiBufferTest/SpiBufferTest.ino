#include <stdint.h>
#include <SPI.h>


#define REF_ATTN 0x00
#define MAX_ATTN 0xFF
#define ADDR 0x00

#define CSPin 10

uint8_t ref_cmd[] = {REF_ATTN, ADDR};
uint8_t max_cmd[] = {MAX_ATTN, ADDR};

uint8_t test_cmd[] = {MAX_ATTN, ADDR, REF_ATTN, ADDR};

void waitForSerial() {
  // gates execution until a character is sent over VCOM
  while (Serial.available() == 0) {
    delay(10); // wait 10ms
  }
  return;
}


void setup() {
  // put your setup code here, to run once:

  // enable SPI bus. Configure for 1 MHz. RFSA3713 follows SPI mode 0.
  SPI.begin();
  SPI.beginTransaction(SPISettings(1e6, LSBFIRST, SPI_MODE0)); 
  // enable GPIO for controllable LE
  pinMode(CSPin, OUTPUT);
  digitalWrite(CSPin, HIGH); // set high initially
  // enable UART/VCOM for test control
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  // Wait for input to begin test
  waitForSerial();
  Serial.print("Beginning Test Sequence. Writing 0 attenuation command.");

  // Push ref_cmd into SPI bus and latch.
  digitalWrite(CSPin, LOW); // pull LE low
  SPI.transfer(ref_cmd, sizeof(ref_cmd));
  delay(1);
  digitalWrite(CSPin, HIGH); // pull LE high
  
  // Wait for input to continue test (time to verify ~0 attenuation)
  waitForSerial();
  Serial.print("0 attenuation verified. Writing max attenuation command");

  // Push max_cmd into SPI bus and latch.
  digitalWrite(CSPin, LOW); // pull LE low
  SPI.transfer(max_cmd, sizeof(max_cmd));
  delay(1);
  digitalWrite(CSPin, HIGH); // pull LE high
  
  // Wait for input to continue test (time to verify ~31.5 dB attenuation)
  waitForSerial();
  Serial.print("Max attenuation verified. Performing buffer test");
  // Push test_cmd intp SPI bus and latch. Desired behavior --> back to ~0 attenuation.

  digitalWrite(CSPin, LOW); // pull LE low
  SPI.transfer(test_cmd, sizeof(test_cmd));
  delay(1);
  digitalWrite(CSPin, HIGH); // pull LE high

  waitForSerial();
  Serial.print("Buffer test complete.");

  while (1) {delay(10);}
}
