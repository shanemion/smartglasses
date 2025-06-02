/*
  Simple Sine Wave Toggle from ESP32S3 Signal
  Using working pin configuration with simple audio
*/

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// Simple audio objects
AudioSynthWaveform       sine1;          // Single sine wave
AudioOutputI2S           i2s1;           // I2S output
AudioControlSGTL5000     sgtl5000_1;     // Audio shield control

// Simple audio connections
AudioConnection patchCord1(sine1, 0, i2s1, 0);  // Left channel
AudioConnection patchCord2(sine1, 0, i2s1, 1);  // Right channel

// Digital input variables - USING WORKING PIN 2
const int TRIG_PIN = 2;  // Pin 2 works (not used by audio shield)
volatile bool fireAudio = false;  // ISR flag

// Audio state
bool audioOn = false;
unsigned long lastTriggerTime = 0;
const unsigned long triggerCooldown = 600;  // Prevent rapid retriggering

// ISR - keep it minimal!
void isrTrigger() { 
  fireAudio = true;  // Just set flag and exit
}

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Set up digital input pin - USING WORKING CONFIGURATION
  pinMode(TRIG_PIN, INPUT_PULLUP);  // Internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(TRIG_PIN), isrTrigger, FALLING);  // Active-low trigger
  
  // Initialize Audio Library
  AudioMemory(20);
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.6);
  
  // Set up sine wave
  sine1.begin(WAVEFORM_SINE);
  sine1.frequency(440);    // 440 Hz (A note)
  sine1.amplitude(0.0);    // Start silent
  
  Serial.println("Simple sine wave ready on PIN 2!");
  Serial.println("Connect ESP32S3 D0 to Teensy Pin 2");
  Serial.println("Connect ESP32S3 GND to Teensy GND");
  Serial.println("Will toggle 440Hz sine wave ON/OFF");
}

void loop() {
  // Check the ISR flag (NOT calling audio functions in ISR)
  if (fireAudio) {
    fireAudio = false;  // Clear flag
    
    unsigned long now = millis();
    
    // Apply cooldown logic
    if (now - lastTriggerTime > triggerCooldown) {
      lastTriggerTime = now;
      
      // Toggle audio on/off
      audioOn = !audioOn;
      
      if (audioOn) {
        sine1.amplitude(0.7);  // Turn on sine wave
        Serial.println("[ESP32S3 SIGNAL] → Sine wave ON (440Hz)");
      } else {
        sine1.amplitude(0.0);  // Turn off sine wave
        Serial.println("[ESP32S3 SIGNAL] → Sine wave OFF");
      }
    }
  }
  
  delay(1);  // Small delay for stability
}
