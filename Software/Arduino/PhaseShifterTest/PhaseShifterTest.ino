#include <SPI.h>

// --- Configuration ---
#define CSPin 10         // Chip Select pin for SPI
#define SERIALBUFSIZE 8  // Buffer for reading serial input

// --- Global Variables ---
char serialBuffer[SERIALBUFSIZE]; // Buffer for serial commands (used by waitForEnter)
int currentStep = 0;              // Current phase shifter step (0-63)

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
 * @brief Prints the current step and calculated phase.
 * @param step The current phase shifter step (0-63).
 */
void printStepInfo(int step) {
  // Calculate phase. 360 degree range over 64 steps (0-63).
  // 360.0 / 64.0 = 5.625 degrees per step.
  float phase = (float)step * 5.625; 
  
  Serial.print("Step: ");
  Serial.print(step);
  Serial.print("\t / \tPhase: ~");
  Serial.print(phase, 3); // Print with 3 decimal places
  Serial.println(" degrees");
}

/**
 * @brief Sends the phase step value via SPI.
 * Assumes MSBFIRST and a 1-byte command based on the example.
 * @param stepValue The phase step (0-63).
 */
void sendSPIStep(uint8_t stepValue) {
  // The example sent a single byte for the phase shifter.
  uint8_t spiCommand[] = {stepValue};
  
  digitalWrite(CSPin, LOW);     // Select SPI device
  SPI.transfer(spiCommand, 1);  // Send the 1-byte command
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
  
  // Configure SPI settings for the PHASE SHIFTER (MSBFIRST from example)
  SPI.beginTransaction(SPISettings(1e6, MSBFIRST, SPI_MODE0));

  // --- Welcome Message ---
  Serial.println("--- Phase Shifter Step Test ---");
  Serial.println("Press ENTER to advance to the next step.");
  Serial.println("----------------------------------------");

  // --- Send Initial Step (Step 0) ---
  printStepInfo(currentStep);
  sendSPIStep((uint8_t)currentStep);
}

/**
 * @brief Main loop runs repeatedly.
 */
void loop() {
  // 1. Wait for user to press Enter
  waitForEnter();

  // 2. Increment step and handle wrap-around
  currentStep++;
  if (currentStep > 63) {
    currentStep = 0; // Reset to 0 after step 63
    Serial.println("--- Resetting to Step 0 ---");
  }

  // 3. Print new info
  printStepInfo(currentStep);

  // 4. Send SPI command for the new step
  sendSPIStep((uint8_t)currentStep);
}