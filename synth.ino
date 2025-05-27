#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "AudioSampleSaber.h"  // WAV converted to header via wav2sketch

// === AUDIO OBJECTS ===
AudioSynthWaveform sine1, sine2, sine3;
AudioSynthWaveform arpeggiator1, arpeggiator2;
AudioSynthNoiseWhite noise;
AudioFilterStateVariable noiseFilter;
AudioInputI2S micInput;  // ✅ SGTL5000 mic input
AudioPlayMemory saberSound;
AudioSynthWaveform saberBass1, saberBass2;
AudioEffectReverb reverbL, reverbR;
AudioEffectDelay delay1;
AudioFilterStateVariable micFilter;


AudioMixer4 sineMixL, sineMixR;
AudioMixer4 arpMixL, arpMixR;
AudioMixer4 sfxMixL, sfxMixR;
AudioMixer4 micMixL, micMixR;
AudioMixer4 finalMixL, finalMixR;
AudioOutputI2S i2s1;
AudioControlSGTL5000 sgtl5000_1;

// === AUDIO ROUTING ===
AudioConnection patchSine1L(sine1, 0, sineMixL, 0);
AudioConnection patchSine2L(sine2, 0, sineMixL, 1);
AudioConnection patchSine3L(sine3, 0, sineMixL, 2);
AudioConnection patchSine1R(sine1, 0, sineMixR, 0);
AudioConnection patchSine2R(sine2, 0, sineMixR, 1);
AudioConnection patchSine3R(sine3, 0, sineMixR, 2);

AudioConnection patchArp1L(arpeggiator1, 0, arpMixL, 0);
AudioConnection patchArp2L(arpeggiator2, 0, arpMixL, 1);
AudioConnection patchNoiseL(noiseFilter, 0, arpMixL, 2);
AudioConnection patchArp1R(arpeggiator1, 0, arpMixR, 0);
AudioConnection patchArp2R(arpeggiator2, 0, arpMixR, 1);
AudioConnection patchNoiseR(noiseFilter, 0, arpMixR, 2);

// Mic input routing - only dry path initially 
// (effects will be enabled via connections created dynamically)
AudioConnection patchMicToDryL(micInput, 0, micMixL, 0); // Mic dry path
AudioConnection patchMicToDryR(micInput, 0, micMixR, 0); // Mic dry path

// We'll use pointers to dynamically create/delete effect connections
AudioConnection *reverbConnectionL = NULL;
AudioConnection *reverbConnectionR = NULL;
AudioConnection *delayConnectionIn = NULL;
AudioConnection *delayConnectionOutL = NULL;
AudioConnection *delayConnectionOutR = NULL;

// Mic mixer to final mix
AudioConnection patchMicL(micMixL, 0, finalMixL, 2);
AudioConnection patchMicR(micMixR, 0, finalMixR, 2);

// SFX sounds
AudioConnection patchSaberL(saberSound, 0, sfxMixL, 0);
AudioConnection patchSaberR(saberSound, 0, sfxMixR, 0);
AudioConnection patchBass1L(saberBass1, 0, sfxMixL, 1);
AudioConnection patchBass1R(saberBass1, 0, sfxMixR, 1);
AudioConnection patchBass2L(saberBass2, 0, sfxMixL, 2);
AudioConnection patchBass2R(saberBass2, 0, sfxMixR, 2);

// Final mix inputs
AudioConnection patchFinalL1(sineMixL, 0, finalMixL, 0);
AudioConnection patchFinalL2(arpMixL, 0, finalMixL, 1);
AudioConnection patchFinalL3(sfxMixL, 0, finalMixL, 3); // Adding SFX to final mix
AudioConnection patchFinalR1(sineMixR, 0, finalMixR, 0);
AudioConnection patchFinalR2(arpMixR, 0, finalMixR, 1);
AudioConnection patchFinalR3(sfxMixR, 0, finalMixR, 3); // Adding SFX to final mix

// Output
AudioConnection patchOutL(finalMixL, 0, i2s1, 0);
AudioConnection patchOutR(finalMixR, 0, i2s1, 1);

// === MPU6050 OBJECT ===
Adafruit_MPU6050 mpu;

// === ARPEGGIATOR ===
float scale[] = {220.0, 246.94, 261.63, 293.66, 329.63, 349.23, 392.00};
unsigned long lastNoteTime = 0;
const int arpInterval = 400;
float filteredX = 0.0;
const float smoothingFactor = 0.9;
bool useBass1 = true;
unsigned long lastTriggerTime = 0;
const unsigned long triggerCooldown = 600;

// === EFFECT TOGGLE PINS AND STATES ===
const int reverbButtonPin = 15;  // Updated to pin 15
const int delayButtonPin = 16;   // Updated to pin 16
bool reverbEnabled = false;
bool delayEnabled = false;
bool lastReverbState = HIGH;
bool lastDelayState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;  // Debounce time in ms

// === THIN POT ===
float heldGain = 0.0;  // Keeps last gain when not touching
float smoothedGain = 0.0;
const float potSmoothingFactor = 0.9;  // Higher = smoother but slower


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initializing...");

  // Set button pins as inputs with pull-up resistors
  pinMode(reverbButtonPin, INPUT_PULLUP);
  pinMode(delayButtonPin, INPUT_PULLUP);
  Serial.println("Button pins configured:");
  Serial.print("- Reverb Button: Pin ");
  Serial.println(reverbButtonPin);
  Serial.print("- Delay Button: Pin ");
  Serial.println(delayButtonPin);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found. Halting.");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.println("MPU6050 ready.");

  AudioMemory(120); // Increased memory for dynamic connections
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.6);
  sgtl5000_1.inputSelect(AUDIO_INPUT_MIC);
  sgtl5000_1.micGain(30);
  Serial.println("SGTL5000 initialized with mic gain 30");

  // Mixer gains
  sineMixL.gain(0, 0.3); sineMixL.gain(1, 0.3); sineMixL.gain(2, 0.3);
  sineMixR.gain(0, 0.3); sineMixR.gain(1, 0.3); sineMixR.gain(2, 0.3);

  arpMixL.gain(0, 0.25); arpMixL.gain(1, 0.25); arpMixL.gain(2, 0.15);
  arpMixR.gain(0, 0.25); arpMixR.gain(1, 0.25); arpMixR.gain(2, 0.15);

  sfxMixL.gain(0, 0.0); sfxMixL.gain(1, 0.5); sfxMixL.gain(2, 0.5); sfxMixL.gain(3, 0.5);
  sfxMixR.gain(0, 0.0); sfxMixR.gain(1, 0.5); sfxMixR.gain(2, 0.5); sfxMixR.gain(3, 0.5);

  // Set up mic mixer gains - only dry signal initially
  micMixL.gain(0, 0.2);  // Dry signal
  micMixL.gain(1, 0.0);  // Reverb (off initially)
  micMixL.gain(2, 0.0);  // Delay (off initially)

  micMixR.gain(0, 0.2);  // Dry signal
  micMixR.gain(1, 0.0);  // Reverb (off initially)
  micMixR.gain(2, 0.0);  // Delay (off initially)

  finalMixL.gain(0, 0.7); finalMixL.gain(1, 0.7); finalMixL.gain(2, 0.9); finalMixL.gain(3, 0.7);
  finalMixR.gain(0, 0.7); finalMixR.gain(1, 0.7); finalMixR.gain(2, 0.9); finalMixR.gain(3, 0.7);

  // Configure delay effect with feedback
  delay1.delay(0, 300); // 300ms delay
  
  // Base pad
  sine1.begin(WAVEFORM_SINE); sine1.frequency(220.0); sine1.amplitude(0.15);
  sine2.begin(WAVEFORM_SINE); sine2.frequency(261.6); sine2.amplitude(0.12);
  sine3.begin(WAVEFORM_SINE); sine3.frequency(329.6); sine3.amplitude(0.12);

  // Arpeggio + texture
  noise.amplitude(0.15);
  noiseFilter.frequency(800);
  noiseFilter.resonance(1.5);
  noiseFilter.octaveControl(3.0);

  arpeggiator1.begin(WAVEFORM_SINE);
  arpeggiator1.frequency(0);
  arpeggiator1.amplitude(0.15);

  arpeggiator2.begin(WAVEFORM_SINE);
  arpeggiator2.frequency(0);
  arpeggiator2.amplitude(0.15);

  saberBass1.begin(WAVEFORM_TRIANGLE);
  saberBass1.frequency(60);
  saberBass1.amplitude(0.0);

  saberBass2.begin(WAVEFORM_TRIANGLE);
  saberBass2.frequency(60);
  saberBass2.amplitude(0.0);

  // Initialize button states
  lastReverbState = digitalRead(reverbButtonPin);
  lastDelayState = digitalRead(delayButtonPin);

  Serial.println("Audio initialized.");
  Serial.println("EFFECT STATUS: Reverb=OFF, Delay=OFF");
  
  // Print initial processor and memory usage
  printEffectStatus();
}

void loop() {
  // Check effect toggle buttons
  checkEffectButtons();


  // ACCELEROMETER 
  // Original arpeggiator and motion sensing code
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  float tilt = constrain(a.acceleration.y / 9.81, -1.0, 1.0);
  int mappedInterval = map(tilt * 1000, -1000, 1000, 250, 1200);
  mappedInterval = constrain(mappedInterval, 200, 1500);

  unsigned long now = millis();
  if (now - lastNoteTime > mappedInterval) {
    int i = random(0, sizeof(scale) / sizeof(scale[0]));
    int j = random(0, sizeof(scale) / sizeof(scale[0]));
    arpeggiator1.frequency(scale[i]);
    arpeggiator2.frequency(scale[j] * -1.5);  // Slight harmonic offset
    lastNoteTime = now;
  }

  float x = a.acceleration.x / 9.81;
  filteredX = (smoothingFactor * filteredX) + ((1.0 - smoothingFactor) * x);
  float dx = abs(x - filteredX);

  if (dx > 0.15 && now - lastTriggerTime > triggerCooldown) {
    lastTriggerTime = now;
    Serial.println("[HEAD TURN] → Triggering bass!");
    // saberSound.play(AudioSampleSaber);
    AudioSynthWaveform &bass = useBass1 ? saberBass1 : saberBass2;
    useBass1 = !useBass1;
    bass.amplitude(0.3);
    for (int i = 0; i <= 30; i++) {
      float amp = 0.3 * (1.0 - (i / 30.0));
      bass.amplitude(amp);
      delay(100);
    }
    bass.amplitude(0.0);
  }



  // THIN POT

  int rawPotValue = analogRead(16);
  float mappedValue;

  // Only update if touched
  if (rawPotValue < 1010) {
    // Map rawPotValue (510–1010) to 0.0–1.0 range
    mappedValue = (rawPotValue - 510.0) / (1010.0 - 510.0);
    mappedValue = constrain(mappedValue, 0.0, 1.0);
    heldGain = mappedValue * 0.3;  // scale to max 0.3
  }

  // Smooth toward the last held gain
  smoothedGain = (potSmoothingFactor * smoothedGain) + 
                 ((1.0 - potSmoothingFactor) * heldGain);

  // Apply smoothed gain to sine mixers
  for (int i = 0; i < 3; i++) {
    sineMixL.gain(i, smoothedGain);
    sineMixR.gain(i, smoothedGain);
  }

  // Optional debug log
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.print("Raw: "); Serial.print(rawPotValue);
    Serial.print(" | Held Gain: "); Serial.print(heldGain, 3);
    Serial.print(" | Smoothed Gain: "); Serial.println(smoothedGain, 3);
    lastPrint = millis();
  }



  

  delay(10);
}

void checkEffectButtons() {
  // Read current button states (LOW when pressed, HIGH when released due to pull-up)
  bool reverbState = digitalRead(reverbButtonPin);
  bool delayState = digitalRead(delayButtonPin);
  unsigned long now = millis();
  
  // Handle the reverb button
  if (reverbState != lastReverbState && (now - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = now;
    
    // If button state is now pressed (LOW)
    if (reverbState == LOW) {
      // Toggle reverb state
      reverbEnabled = !reverbEnabled;
      
      if (reverbEnabled) {
        // Enable reverb by creating new connections
        if (reverbConnectionL == NULL) {
          reverbConnectionL = new AudioConnection(micInput, 0, reverbL, 0);
        }
        if (reverbConnectionR == NULL) {
          reverbConnectionR = new AudioConnection(reverbL, 0, micMixL, 1);
          // Both L and R outputs connect to the same reverb
          new AudioConnection(reverbL, 0, micMixR, 1);
        }
        // Set reverb gain
        micMixL.gain(1, 0.7);
        micMixR.gain(1, 0.7);
      } else {
        // Disable reverb by setting gain to 0
        micMixL.gain(1, 0.0);
        micMixR.gain(1, 0.0);
      }
      
      // Debug logging
      Serial.print("REVERB BUTTON PRESSED - Now ");
      Serial.println(reverbEnabled ? "ON" : "OFF");
      printEffectStatus();
    }
    lastReverbState = reverbState;
  }
}

// Function to print current effect status for debugging
void printEffectStatus() {
  Serial.print("EFFECT STATUS: Reverb=");
  Serial.print(reverbEnabled ? "ON" : "OFF");
  
  Serial.print("Processor Usage: ");
  Serial.print(AudioProcessorUsage());
  Serial.print("%, Memory: ");
  Serial.print(AudioMemoryUsage());
  Serial.print("/");
  Serial.print(AudioMemoryUsageMax());
  Serial.println();
}
