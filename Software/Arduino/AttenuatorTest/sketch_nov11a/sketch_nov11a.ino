#include <SPI.h>

// --- Configuration ---
#define CSPin 10         // Chip Select pin for SPI
#define SERIALBUFSIZE 8  // Buffer for reading serial input

// --- Global Variables ---
char serialBuffer[SERIALBUFSIZE]; // Buffer for serial commands
uint8_t customAddress;            // Stores the attenuator address set at start
int currentStep = 0;              // Current attenuator step (0-127)

/**
 * @brief Reads an integer value from the Serial monitor.
 * Waits for a newline character ('\n') before processing.
 * @return The integer entered, converted to uint8_t.
 */
uint8_t getAddressFromSerial() {
  // Wait indefinitely until at least one character is available
  while (Serial.available() == 0) {
    delay(1);  // avoid busy-waiting
  }

  // Read the string until newline
  int bytesRead = Serial.readBytesUntil('\n', serialBuffer, SERIALBUFSIZE - 1);

  if (bytesRead <= 0) return 0; // Return 0 if no bytes read

  serialBuffer[bytesRead] = '\0';  // Null-terminate the string

  // Convert ASCII string to integer and return as uint8_t
  return (uint8_t)atoi(serialBuffer);
}

/**
 * @brief Waits for the user to press the Enter key.
 * Ignores all other characters.
 */
void waitForEnter() {
  bool receivedEnter = false;
  while (!receivedEnter) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      // Check for newline or carriage return
      if (c == '\n' || c == '\r') {
        receivedEnter = true;
      }
      // Ignore all other characters
    }
    delay(1); // Small delay to prevent busy-waiting
  }
  
  // Clear any other characters left in the serial buffer (e.g., if \r\n was sent)
  while(Serial.available() > 0) {
    Serial.read();
  }
}

/**
 * @brief Prints the current step and calculated attenuation.
 * @param step The current attenuator step (0-127).
 */
void printStepInfo(int step) {
  // Calculate attenuation. 32.5dB range over 128 steps (0-127).
  // (float)step * (32.5 / 128.0)
  // Using 0.25390625 which is 32.5 / 128.0
  float attenuation = (float)step * 0.25390625; 
  
  Serial.print("Step: ");
  Serial.print(step);
  Serial.print("\t / \tAttenuation: ~");
  Serial.print(attenuation, 3); // Print with 3 decimal places
  Serial.println(" dB");
}

/**
 * @brief Sends the attenuation step and address via SPI.
 * Assumes LSBFIRST based on the example for the attenuator.
 * @param stepValue The attenuation step (0-127).
 * @param address The device address.
 */
void sendSPIStep(uint8_t stepValue, uint8_t address) {
  // The example sent {value, address}, so we follow that format.
  uint8_t spiCommand[] = {stepValue, address};
  
  digitalWrite(CSPin, LOW);     // Select SPI device
  SPI.transfer(spiCommand, 2);  // Send the 2-byte command
  delay(1);                     // Wait 1ms (from example)
  digitalWrite(CSPin, HIGH);    // Deselect SPI device
}

/**
 * @brief Setup runs once on boot.
 */
void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for Serial Monitor to open (for boards like Leonardo)

  // --- Initialize SPI ---
  SPI.begin();
  pinMode(CSPin, OUTPUT);
  digitalWrite(CSPin, HIGH); // Set high initially (device not selected)
  
  // Configure SPI settings for the attenuator (LSBFIRST from example)
  SPI.beginTransaction(SPISettings(1e6, LSBFIRST, SPI_MODE0));

  // --- Get Address from User ---
  Serial.println("--- Attenuator Step Test ---");
  Serial.println("Please enter the attenuator address (0-7) and press Enter:");
  
  customAddress = getAddressFromSerial();
  
  Serial.print("Using address: ");
  Serial.println(customAddress);
  Serial.println("\nPress ENTER to advance to the next step.");
  Serial.println("----------------------------------------");

  // --- Send Initial Step (Step 0) ---
  printStepInfo(currentStep);
  sendSPIStep((uint8_t)currentStep, customAddress);
}

/**
 * @brief Main loop runs repeatedly.
 */
void loop() {
  // 1. Wait for user to press Enter
  waitForEnter();

  // 2. Increment step and handle wrap-around
  currentStep++;
  if (currentStep > 127) {
    currentStep = 0; // Reset to 0 after step 127
    Serial.println("--- Resetting to Step 0 ---");
  }

  // 3. Print new info
  printStepInfo(currentStep);

  // 4. Send SPI command for the new step
  sendSPIStep((uint8_t)currentStep, customAddress);
}