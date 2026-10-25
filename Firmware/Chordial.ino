/*
  CHORDIAL - Rotary Chord Synth
  Board: Seeed Studio XIAO ESP32-S3

  PCB wiring (XIAO labels -> ESP32-S3 GPIO):
  EC11 CLK D0/GPIO1, DT D1/GPIO2, push D2/GPIO3 (active LOW)
  Duration key D3/GPIO4 (active LOW)
  OLED SDA D4/GPIO5, SCL D5/GPIO6, address 0x3C
  MAX98357A BCLK D8/GPIO7, LRCLK D9/GPIO8, DIN D10/GPIO9
*/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP_I2S.h>
#include <math.h>

constexpr int PIN_ENC_A = 1;
constexpr int PIN_ENC_B = 2;
constexpr int PIN_ENC_PUSH = 3;
constexpr int PIN_LENGTH_KEY = 4;
constexpr int PIN_OLED_SDA = 5;
constexpr int PIN_OLED_SCL = 6;
constexpr int PIN_I2S_BCLK = 7;
constexpr int PIN_I2S_LRCLK = 8;
constexpr int PIN_I2S_DATA = 9;

constexpr uint8_t SCREEN_WIDTH = 128;
constexpr uint8_t SCREEN_HEIGHT = 64;
constexpr uint8_t OLED_ADDR = 0x3C;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
I2SClass I2S;
bool oledReady = false;

constexpr uint32_t SAMPLE_RATE = 22050;
constexpr size_t FRAMES_PER_BUFFER = 128;
int16_t stereoBuffer[FRAMES_PER_BUFFER * 2];
constexpr float PI2 = 6.2831853071795864769f;
constexpr float MASTER_GAIN = 6500.0f;

const char* NOTE_NAMES[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
int chordIndex = 0;
const uint16_t DURATIONS_MS[] = {500, 1000, 2000, 4000};
const char* DURATION_LABELS[] = {"0.5s", "1.0s", "2.0s", "4.0s"};
uint8_t durationIndex = 2;

float phase[3] = {0, 0, 0};
float phaseIncrement[3] = {0, 0, 0};
uint32_t samplesElapsed = 0;
uint32_t noteSamples = 0;
bool playing = false;
bool uiDirty = true;
uint32_t lastUiRefresh = 0;

// Debounced controls (using INPUT_PULLUP)
struct Button {
  uint8_t pin;
  bool stable = HIGH;
  bool lastRaw = HIGH;
  uint32_t changedAt = 0;

  bool pressedEvent() {
    const bool raw = digitalRead(pin);
    const uint32_t now = millis();
    if (raw != lastRaw) {
      lastRaw = raw;
      changedAt = now;
    }
    if (raw != stable && now - changedAt >= 25) {
      stable = raw;
      if (stable == LOW) return true;
    }
    return false;
  }
};
Button encoderButton{PIN_ENC_PUSH};
Button lengthButton{PIN_LENGTH_KEY};

// Quadrature decoding: one click generally produces four valid transitions.
const int8_t QUAD_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};
uint8_t previousAB = 0;
int8_t accumulatedSteps = 0;
constexpr int STEPS_PER_DETENT = 4; // change to 2 if a detent needs two clicks
constexpr int ENCODER_DIRECTION = 1; // change to -1 if rotation direction is reversed

void readEncoder() {
  const uint8_t ab = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  const uint8_t transition = (previousAB << 2) | ab;
  previousAB = ab;
  accumulatedSteps += QUAD_TABLE[transition];
  if (accumulatedSteps >= STEPS_PER_DETENT || accumulatedSteps <= -STEPS_PER_DETENT) {
    const int direction = (accumulatedSteps > 0 ? 1 : -1) * ENCODER_DIRECTION;
    accumulatedSteps = 0;
    chordIndex = (chordIndex + direction + 24) % 24;
    uiDirty = true;
  }
}

float noteFrequency(uint8_t midiNote) {
  return 440.0f * powf(2.0f, (int(midiNote) - 69) / 12.0f);
}

void selectedMidiNotes(uint8_t out[3]) {
  const bool minor = chordIndex >= 12;
  const int root = chordIndex % 12;
  out[0] = 60 + root;
  out[1] = 60 + root + (minor ? 3 : 4);
  out[2] = 60 + root + 7;
}

void midiToName(uint8_t midi, char* out, size_t length) {
  snprintf(out, length, "%s%d", NOTE_NAMES[midi % 12], (midi / 12) - 1);
}

void startChord() {
  uint8_t notes[3];
  selectedMidiNotes(notes);
  for (int i = 0; i < 3; ++i) {
    phase[i] = 0.0f;
    phaseIncrement[i] = PI2 * noteFrequency(notes[i]) / SAMPLE_RATE;
  }
  samplesElapsed = 0;
  noteSamples = (SAMPLE_RATE * DURATIONS_MS[durationIndex]) / 1000;
  playing = true;
  uiDirty = true;
}


float envelope(uint32_t sampleNumber) {
  constexpr uint32_t ATTACK_SAMPLES = SAMPLE_RATE / 45; // ~22ms
  constexpr uint32_t RELEASE_SAMPLES = SAMPLE_RATE / 7; // ~143ms
  float amp = 1.0f;
  if (sampleNumber < ATTACK_SAMPLES)
    amp = float(sampleNumber) / float(ATTACK_SAMPLES);
  if (sampleNumber + RELEASE_SAMPLES >= noteSamples) {
    float remain = float(noteSamples - sampleNumber) / float(RELEASE_SAMPLES);
    if (remain < amp) amp = remain;
  }
  // Smoothstep for natural, click-free fade.
  amp = constrain(amp, 0.0f, 1.0f);
  return amp * amp * (3.0f - 2.0f * amp);
}

void fillAudioBuffer() {
  for (size_t i = 0; i < FRAMES_PER_BUFFER; ++i) {
    float sample = 0.0f;
    if (playing && samplesElapsed < noteSamples) {
      for (int j = 0; j < 3; ++j) {
        // Warm, soft chord: fundamental plus subtle 2nd/3rd harmonics.
        const float p = phase[j];
        sample += (sinf(p) + 0.16f * sinf(2.0f * p) + 0.05f * sinf(3.0f * p)) / 3.63f;
        phase[j] += phaseIncrement[j];
        if (phase[j] >= PI2) phase[j] -= PI2;
      }
      sample *= envelope(samplesElapsed);
      ++samplesElapsed;
    } else if (playing) {
      playing = false;
      uiDirty = true;
    }
    const int16_t pcm = (int16_t)(constrain(sample, -1.0f, 1.0f) * MASTER_GAIN);
    // Standard stereo I2S; both channels are identical for mono MAX98357A.
    stereoBuffer[2 * i] = pcm;
    stereoBuffer[2 * i + 1] = pcm;
  }
}

void drawUI() {
  if (!oledReady) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("CHORDIAL"));
  display.setCursor(79, 0);
  display.print(DURATION_LABELS[durationIndex]);
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(0, 16);
  display.print(NOTE_NAMES[chordIndex % 12]);
  display.print(chordIndex < 12 ? F(" Maj") : F(" min"));

  uint8_t notes[3];
  selectedMidiNotes(notes);
  char n1[5], n2[5], n3[5];
  midiToName(notes[0], n1, sizeof(n1));
  midiToName(notes[1], n2, sizeof(n2));
  midiToName(notes[2], n3, sizeof(n3));
  display.setTextSize(1);
  display.setCursor(0, 39);
  display.print(n1); display.print(F("  "));
  display.print(n2); display.print(F("  "));
  display.print(n3);
  display.setCursor(0, 52);
  display.print(playing ? F("PLAYING") : F("Press to play"));
  if (playing && noteSamples > 0) {
    const int w = int(43UL * samplesElapsed / noteSamples);
    display.drawRect(83, 53, 45, 8, SSD1306_WHITE);
    display.fillRect(84, 54, constrain(w, 0, 43), 6, SSD1306_WHITE);
  }
  display.display();
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_PUSH, INPUT_PULLUP);
  pinMode(PIN_LENGTH_KEY, INPUT_PULLUP);
  previousAB = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);

  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  Wire.setClock(400000);
  oledReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!oledReady) Serial.println("OLED not detected at 0x3C");
  else {
    display.clearDisplay();
    display.display();
  }

  I2S.setPins(PIN_I2S_BCLK, PIN_I2S_LRCLK, PIN_I2S_DATA);
  if (!I2S.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO)) {
    Serial.println("ERROR: I2S could not start. Check ESP32 board package 3.x");
    if (oledReady) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.print("I2S START FAILED");
      display.display();
    }
    while (true) delay(1000);
  }
  uiDirty = true;
  drawUI();
  Serial.println("Chordial ready: rotate=chord, encoder push=play, key=length");
}

void loop() {
  readEncoder();
  if (encoderButton.pressedEvent()) startChord();
  if (lengthButton.pressedEvent()) {
    durationIndex = (durationIndex + 1) % 4;
    uiDirty = true;
  }

  fillAudioBuffer();
  I2S.write((const uint8_t*)stereoBuffer, sizeof(stereoBuffer));

  // Keep OLED transfers infrequent so audio playback remains stable.
  const uint32_t now = millis();
  if (uiDirty && now - lastUiRefresh > 60) {
    drawUI();
    uiDirty = false;
    lastUiRefresh = now;
  } else if (playing && now - lastUiRefresh > 120) {
    drawUI();
    lastUiRefresh = now;
  }
}
