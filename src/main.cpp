#include <Arduino.h>

#define LED_BUILTIN 2
 
void setup()
{
  // Initialize LED pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}
 
void loop()
{
  // Set the LED HIGH
  digitalWrite(LED_BUILTIN, HIGH);
 
  // Wait for a second
  delay(1000);
 
  // Set the LED LOW
  digitalWrite(LED_BUILTIN, LOW);
 
   // Wait for a second
  delay(1000);
}