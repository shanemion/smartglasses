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

// === NEW: Feedback Delay Objects ===
AudioEffectDelay delay1;
AudioMixer4 delayMixer;  // For feedback loop

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
AudioConnection patchMicToDryL(micInput, 0, micMixL, 0);  // Dry to left
AudioConnection patchMicToDryR(micInput, 0, micMixR, 0);  // Dry to right

// We'll use pointers to dynamically create/delete effect connections
AudioConnection *reverbConnectionL = NULL;
AudioConnection *reverbConnectionR = NULL;
AudioConnection *delayConnectionInput = NULL;
AudioConnection *delayConnectionOutput = NULL;
AudioConnection *delayConnectionFeedback = NULL;
AudioConnection *delayConnectionToMixL = NULL;
AudioConnection *delayConnectionToMixR = NULL;

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
AudioConnection patchFinalL3(sfxMixL, 0, finalMixL, 3);
AudioConnection patchFinalR1(sineMixR, 0, finalMixR, 0);
AudioConnection patchFinalR2(arpMixR, 0, finalMixR, 1);
AudioConnection patchFinalR3(sfxMixR, 0, finalMixR, 3);

// Output
AudioConnection patchOutL(finalMixL, 0, i2s1, 0);
AudioConnection patchOutR(finalMixR, 0, i2s1, 1);

// Connect mic mixers to final mix
AudioConnection patchMicLToFinal(micMixL, 0, finalMixL, 2);
AudioConnection patchMicRToFinal(micMixR, 0, finalMixR, 2);

// === MPU6050 OBJECT ===
Adafruit_MPU6050 mpu;

// === DELAY TOGGLE PIN + DEBOUNCE ===
const int delayButtonPin = 16;
bool delayEnabled = false;
bool lastDelayState = HIGH;
unsigned long lastDelayDebounce = 0;
const unsigned long debounceDelay = 50;  // ms

// === ARPEGGIATOR ===
float scale[] = { 220.0, 246.94, 261.63, 293.66, 329.63, 349.23, 392.00 };
unsigned long lastNoteTime = 0;
const int arpInterval = 400;
float filteredX = 0.0;
const float smoothingFactor = 0.9;
bool useBass1 = true;
unsigned long lastTriggerTime = 0;
const unsigned long triggerCooldown = 600;

// === EFFECT TOGGLE PINS AND STATES ===
const int reverbButtonPin = 15;
const int thinPotPin = 17;
const int micMutePin = 14;  // New mic mute button
bool reverbEnabled = false;
bool lastReverbState = HIGH;
bool micMuted = false;
bool lastMuteState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long lastMuteDebounce = 0;

// === THIN POT ===
float heldGain = 0.0;
float smoothedGain = 0.0;
const float potSmoothingFactor = 0.9;

// 

const int buttonPin = 22;
elapsedMillis timeSinceLastCycle;

int delayTimes[] = {50, 100, 200, 400};  // ms values
const int numDelayTimes = 4;
int currentDelayIndex = 0;
bool lastButtonState = HIGH;
bool changeRequested = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initializing...");

  pinMode(buttonPin, INPUT_PULLUP);
  delay1.delay(0, delayTimes[currentDelayIndex]);  // Start with first delay time

  // Button setup
  pinMode(delayButtonPin, INPUT_PULLUP);
  pinMode(reverbButtonPin, INPUT_PULLUP);
  pinMode(micMutePin, INPUT_PULLUP);
  lastDelayState = digitalRead(delayButtonPin);
  lastReverbState = digitalRead(reverbButtonPin);
  lastMuteState = digitalRead(micMutePin);

  Serial.println("Button pins configured:");
  Serial.print("- Reverb Button: Pin ");
  Serial.println(reverbButtonPin);
  Serial.print("- Delay Button: Pin ");
  Serial.println(delayButtonPin);
  Serial.print("- Mic Mute Button: Pin ");
  Serial.println(micMutePin);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found. Halting.");
    while (1)
      ;
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.println("MPU6050 ready.");

  AudioMemory(220);
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.6);
  sgtl5000_1.inputSelect(AUDIO_INPUT_MIC);
  sgtl5000_1.micGain(30);
  Serial.println("SGTL5000 initialized with mic gain 30");

  // Initialize delay settings
  delay1.delay(0, 400);  // 400ms delay

  // Setup delay mixer for feedback loop
  delayMixer.gain(0, 0.5);  // Fresh signal from mic
  delayMixer.gain(1, 0.0);  // Feedback signal (will be set to 0.7 when enabled)
  delayMixer.gain(2, 0.0);  // Unused
  delayMixer.gain(3, 0.0);  // Unused

  // Mixer gains
  sineMixL.gain(0, 0.3);
  sineMixL.gain(1, 0.3);
  sineMixL.gain(2, 0.3);
  sineMixR.gain(0, 0.3);
  sineMixR.gain(1, 0.3);
  sineMixR.gain(2, 0.3);

  arpMixL.gain(0, 0.25);
  arpMixL.gain(1, 0.25);
  arpMixL.gain(2, 0.15);
  arpMixR.gain(0, 0.25);
  arpMixR.gain(1, 0.25);
  arpMixR.gain(2, 0.15);

  sfxMixL.gain(0, 0.0);
  sfxMixL.gain(1, 0.5);
  sfxMixL.gain(2, 0.5);
  sfxMixL.gain(3, 0.5);
  sfxMixR.gain(0, 0.0);
  sfxMixR.gain(1, 0.5);
  sfxMixR.gain(2, 0.5);
  sfxMixR.gain(3, 0.5);

  // Set up mic mixer gains
  micMixL.gain(0, 0.2);  // Dry signal
  micMixL.gain(1, 0.0);  // Reverb (off initially)
  micMixL.gain(2, 0.0);  // Delay (off initially)
  micMixL.gain(3, 0.0);  // Unused

  micMixR.gain(0, 0.2);  // Dry signal
  micMixR.gain(1, 0.0);  // Reverb (off initially)
  micMixR.gain(2, 0.0);  // Delay (off initially)
  micMixR.gain(3, 0.0);  // Unused

  finalMixL.gain(0, 0.7);
  finalMixL.gain(1, 0.7);
  finalMixL.gain(2, 0.9);
  finalMixL.gain(3, 0.7);
  finalMixR.gain(0, 0.7);
  finalMixR.gain(1, 0.7);
  finalMixR.gain(2, 0.9);
  finalMixR.gain(3, 0.7);

  // Base pad
  sine1.begin(WAVEFORM_SINE);
  sine1.frequency(220.0);
  sine1.amplitude(0.15);
  sine2.begin(WAVEFORM_SINE);
  sine2.frequency(261.6);
  sine2.amplitude(0.12);
  sine3.begin(WAVEFORM_SINE);
  sine3.frequency(329.6);
  sine3.amplitude(0.12);

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

  Serial.println("Audio initialized.");
  Serial.println("EFFECT STATUS: Reverb=OFF, Delay=OFF, Mic=ACTIVE");
  printEffectStatus();
}

void loop() {
  // Check effect toggle buttons
  checkEffectButtons();

  bool buttonState = digitalRead(buttonPin);

  // Rising edge detection
  if (lastButtonState == HIGH && buttonState == LOW) {
    changeRequested = true;
  }
  lastButtonState = buttonState;

  // Check if it's time to cycle delay in sync
  if (timeSinceLastCycle >= delayTimes[currentDelayIndex]) {
    timeSinceLastCycle = 0;

    if (changeRequested) {
      currentDelayIndex = (currentDelayIndex + 1) % numDelayTimes;
      delay1.delay(0, delayTimes[currentDelayIndex]);
      changeRequested = false;
    }
  }


  // === Sensor Poll ===
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  // === Accel for Tempo ===
  float forwardAccel = a.acceleration.y;
  if (abs(forwardAccel) < 0.15) forwardAccel = 0;

  static float smoothedAccel = 0;
  float accelAlpha = (forwardAccel < smoothedAccel) ? 0.3 : 0.02;
  smoothedAccel = (1 - accelAlpha) * smoothedAccel + accelAlpha * forwardAccel;
  smoothedAccel *= -1;
  smoothedAccel = constrain(smoothedAccel, -2.5, 2.5);

  const int baseInterval = 500;
  const int minInterval = 120;
  const int maxInterval = 1250;
  int adjustedInterval = baseInterval - smoothedAccel * 160;
  adjustedInterval = constrain(adjustedInterval, minInterval, maxInterval);

  delayTimes[0] = adjustedInterval / 4;
  delayTimes[1] = adjustedInterval / 2;
  delayTimes[2] = adjustedInterval;
  delayTimes[3] = adjustedInterval * 2;



  // === Gyro for Detune (Only update on trigger) ===
  float rawZ = abs(g.gyro.z);
  static float smoothedGyroZ = 0;
  float gyroAlpha = 0.5;
  smoothedGyroZ = (1 - gyroAlpha) * smoothedGyroZ + gyroAlpha * rawZ;

  int newDetuneCents = smoothedGyroZ * 250;
  if (newDetuneCents <= 8) {
    newDetuneCents = 0;
  }
  newDetuneCents = constrain(newDetuneCents, 0, 100);


  // === Arpeggiator Trigger ===
  unsigned long now = millis();

  if (now - lastNoteTime > adjustedInterval) {
    int i = random(0, sizeof(scale) / sizeof(scale[0]));
    float basePitch = scale[i];
    float detunedPitch = basePitch * pow(2, newDetuneCents / 1200.0);

    // Apply gyro-based detune to sine wave pad
    float detuneRatio = pow(2, -newDetuneCents / 1200.0);  // detune multiplier

    sine1.frequency(220.0 * detuneRatio);
    sine2.frequency(261.6 * detuneRatio);
    sine3.frequency(329.6 * detuneRatio);

    arpeggiator1.frequency(basePitch);
    arpeggiator2.frequency(basePitch);


    lastNoteTime = now;

    // Optional print
    Serial.print("Note: ");
    Serial.print(basePitch);
    Serial.print(" Hz  Detuned: ");
    Serial.print(detunedPitch);
    Serial.print(" Hz (");
    Serial.print(newDetuneCents);
    Serial.println(" cents)");
  }

  delay(25);  // Give CPU room for audio processing





  // THIN POT
  // int rawPotValue = analogRead(thinPotPin);
  int rawPotValue = 1009;
  float mappedValue;

  if (rawPotValue < 1010) {
    mappedValue = (rawPotValue - 510.0) / (1010.0 - 510.0);
    mappedValue = constrain(mappedValue, 0.0, 1.0);
    heldGain = mappedValue * 0.3;
  }



  smoothedGain = (potSmoothingFactor * smoothedGain) + ((1.0 - potSmoothingFactor) * heldGain);

  for (int i = 0; i < 3; i++) {
    sineMixL.gain(i, smoothedGain);
    sineMixR.gain(i, smoothedGain);
  }

  delay(10);
}

void checkEffectButtons() {
  bool reverbState = digitalRead(reverbButtonPin);
  bool delayState = digitalRead(delayButtonPin);
  bool muteState = digitalRead(micMutePin);
  unsigned long now = millis();

  // Handle reverb button
  if (reverbState != lastReverbState && (now - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = now;
    if (reverbState == LOW) {
      reverbEnabled = !reverbEnabled;
      if (reverbEnabled) {
        if (reverbConnectionL == NULL) {
          reverbConnectionL = new AudioConnection(micInput, 0, reverbL, 0);
        }
        if (reverbConnectionR == NULL) {
          reverbConnectionR = new AudioConnection(reverbL, 0, micMixL, 1);
          new AudioConnection(reverbL, 0, micMixR, 1);
        }
        if (!micMuted) {
          micMixL.gain(1, 0.7);
          micMixR.gain(1, 0.7);
        }
      } else {
        micMixL.gain(1, 0.0);
        micMixR.gain(1, 0.0);
      }
      Serial.print("REVERB BUTTON PRESSED - Now ");
      Serial.println(reverbEnabled ? "ON" : "OFF");
      printEffectStatus();
    }
    lastReverbState = reverbState;
  }

  // Handle delay button
  if (delayState != lastDelayState && (now - lastDelayDebounce) > debounceDelay) {
    lastDelayDebounce = now;
    if (delayState == LOW) {
      delayEnabled = !delayEnabled;
      if (delayEnabled) {
        // Create feedback delay connections
        if (delayConnectionInput == NULL) {
          delayConnectionInput = new AudioConnection(micInput, 0, delayMixer, 0);
        }
        if (delayConnectionOutput == NULL) {
          delayConnectionOutput = new AudioConnection(delayMixer, 0, delay1, 0);
        }
        if (delayConnectionFeedback == NULL) {
          delayConnectionFeedback = new AudioConnection(delay1, 0, delayMixer, 1);
        }
        if (delayConnectionToMixL == NULL) {
          delayConnectionToMixL = new AudioConnection(delay1, 0, micMixL, 2);
        }
        if (delayConnectionToMixR == NULL) {
          delayConnectionToMixR = new AudioConnection(delay1, 0, micMixR, 2);
        }
        // Set feedback gain
        delayMixer.gain(1, 0.7);
        // Set delay output gain in mic mixers (only if not muted)
        if (!micMuted) {
          micMixL.gain(2, 0.5);
          micMixR.gain(2, 0.5);
        }
      } else {
        // Disable delay by setting gains to 0
        delayMixer.gain(1, 0.0);
        micMixL.gain(2, 0.0);
        micMixR.gain(2, 0.0);
      }
      Serial.print("DELAY BUTTON PRESSED - Now ");
      Serial.println(delayEnabled ? "ON" : "OFF");
      printEffectStatus();
    }
    lastDelayState = delayState;
  }

  // Handle mic mute button
  if (muteState != lastMuteState && (now - lastMuteDebounce) > debounceDelay) {
    lastMuteDebounce = now;
    if (muteState == LOW) {
      micMuted = !micMuted;
      if (micMuted) {
        // Mute all mic channels
        micMixL.gain(0, 0.0);  // Dry
        micMixL.gain(1, 0.0);  // Reverb
        micMixL.gain(2, 0.0);  // Delay
        micMixR.gain(0, 0.0);  // Dry
        micMixR.gain(1, 0.0);  // Reverb
        micMixR.gain(2, 0.0);  // Delay
      } else {
        // Unmute - restore appropriate gains based on effect states
        micMixL.gain(0, 0.2);  // Dry signal
        micMixR.gain(0, 0.2);  // Dry signal

        if (reverbEnabled) {
          micMixL.gain(1, 0.7);
          micMixR.gain(1, 0.7);
        }

        if (delayEnabled) {
          micMixL.gain(2, 0.5);
          micMixR.gain(2, 0.5);
        }
      }
      Serial.print("MIC MUTE BUTTON PRESSED - Mic is now ");
      Serial.println(micMuted ? "MUTED" : "UNMUTED");
      printEffectStatus();
    }
    lastMuteState = muteState;
  }
}

void printEffectStatus() {
  Serial.print("EFFECT STATUS: Reverb=");
  Serial.print(reverbEnabled ? "ON" : "OFF");
  Serial.print(", Delay=");
  Serial.print(delayEnabled ? "ON" : "OFF");
  Serial.print(", Mic=");
  Serial.print(micMuted ? "MUTED" : "ACTIVE");
  Serial.print(" | Processor Usage: ");
  Serial.print(AudioProcessorUsage());
  Serial.print("%, Memory: ");
  Serial.print(AudioMemoryUsage());
  Serial.print("/");
  Serial.print(AudioMemoryUsageMax());
  Serial.println();
}
