/*

  MIDI Ring Knob Light Ring LED Circus v2.8.11
  Copyright 2015 Johan Nordberg & Giosue Russo

*/

#include <Adafruit_NeoPixel.h>
#include <MIDI.h>
#include <math.h>

#define RING_BUS_PIN 5
#define RING_NUM_PIXELS 24
#define BUTTONLED_BUS_PIN 6
#define BUTTONLED_NUM_PIXELS 5

#define NUM_BUTTONS 5
#define BUTTON_RED_PIN 8
#define BUTTON_YELLOW_PIN 9
#define BUTTON_GREEN_PIN 10
#define BUTTON_BLUE_PIN 11
#define BUTTON_WHITE_PIN 12

#define ENCODER_STEP_RESOLUTION 800

Adafruit_NeoPixel ring_pixels = Adafruit_NeoPixel(
  RING_NUM_PIXELS, RING_BUS_PIN, NEO_GRB + NEO_KHZ800
);

Adafruit_NeoPixel button_pixels = Adafruit_NeoPixel(
  BUTTONLED_NUM_PIXELS, BUTTONLED_BUS_PIN, NEO_RGB + NEO_KHZ800
);

MIDI_CREATE_DEFAULT_INSTANCE();

static uint8_t buttonColors[5 * 3] = {
  255, 0, 0,
  255, 255, 0,
  0, 255, 0,
  0, 0, 255,
  200, 200, 200
};

static byte ccLookup[32] = {
  0, 3, 9, 14, 15, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 85,
  86, 87, 88, 89, 90, 102, 103, 104, 105, 106, 107, 108, 109, 110
};

// red - yellow - green - blue - white
byte buttonState = B00001;
byte lastButtonState = B00001;

int activeColors = 0;
uint8_t colorLookup[NUM_BUTTONS];

float controlState[32] = {0.0};
byte ccState[32] = {0};

int A_SIG=0, B_SIG=1;

long pulses;
long lastPulses = 0;
long index = 0;
long lastIndex = 0;
bool needsUpdate = false;

bool control_changed = false;

long encoder_pulses = 0;
long encoder_last_pulses = 0;
bool encoder_changed = false;
bool encoder_a = true;
bool encoder_b = false;

void setup(){
  // setup interrupts for encoder
  attachInterrupt(0, encoder_a_rise, RISING);
  attachInterrupt(1, encoder_b_rise, RISING);

  // setup input mode for buttons
  pinMode(BUTTON_RED_PIN, INPUT_PULLUP);
  pinMode(BUTTON_YELLOW_PIN, INPUT_PULLUP);
  pinMode(BUTTON_GREEN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_BLUE_PIN, INPUT_PULLUP);
  pinMode(BUTTON_WHITE_PIN, INPUT_PULLUP);

  // start midi, fucks up the serial
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.setHandleControlChange(controlChange);

  // start serial, only if not using midi
  // Serial.begin(115400);
  // Serial.println("Hello");

  // start the neopixels
  ring_pixels.begin();
  button_pixels.begin();
}

void loop() {
  readButtons();

  if (encoder_changed) {
    // encoder changed position, update current control state and reset pulses
    long delta_pulses = encoder_pulses - encoder_last_pulses;
    float delta = (float)delta_pulses / ENCODER_STEP_RESOLUTION;
    controlState[buttonState] += delta;
    if (controlState[buttonState] > 1.0) controlState[buttonState] = 1.0;
    if (controlState[buttonState] < 0.0) controlState[buttonState] = 0.0;
    encoder_last_pulses = encoder_pulses;
    sendControlChange();
    updateRing();
  }

  MIDI.read();
  //delay(10);
}

void controlChange(byte channel, byte number, byte value) {

  byte n = 0;
  for (int i = 0; i < 32; i++) {
    if (ccLookup[i] == number) {
      n = i;
    }
  }

  controlState[n] = (float)value / 127;
  ccState[n] = value;
  updateRing();
}

void readButtons() {
  // if (millis() - releaseTime > 100) {
  //   //Serial.println("blocked");
  //   return;
  // }

  byte newState = B00000;
  bool releasing = false;

  if (!digitalRead(BUTTON_RED_PIN))    newState |= 1 << 0;
  if (!digitalRead(BUTTON_YELLOW_PIN)) newState |= 1 << 1;
  if (!digitalRead(BUTTON_GREEN_PIN))  newState |= 1 << 2;
  if (!digitalRead(BUTTON_BLUE_PIN))   newState |= 1 << 3;
  if (!digitalRead(BUTTON_WHITE_PIN))  newState |= 1 << 4;

  // for (int buttonIdx = 0; buttonIdx < NUM_BUTTONS; buttonIdx++) {
  //   if ((buttonState & (1 << buttonIdx)) == 1 && (newState & (1 << buttonIdx)) == 0) {
  //     releaseTime = millis();
  //     buttonState = lastButtonState;
  //     needsUpdate = true;
  //   }
  // }

  if (newState != B00000 && buttonState != newState) {
    buttonState = newState;
    needsUpdate = true;
  }

  activeColors = 0;
  uint8_t colorIdx, r, g, b;
  float lum;

  for (int buttonIdx = 0; buttonIdx < NUM_BUTTONS; buttonIdx++) {
    r = buttonColors[buttonIdx * 3];
    g = buttonColors[buttonIdx * 3 + 1];
    b = buttonColors[buttonIdx * 3 + 2];
    lum = 0.1;

    if (buttonState & (1 << buttonIdx)) {
      colorLookup[activeColors++] = buttonIdx * 3;
      lum = 1.0;
    }

    r *= lum;
    g *= lum;
    b *= lum;

    button_pixels.setPixelColor(buttonIdx, button_pixels.Color(r, g, b));
  }

  button_pixels.show();
  lastButtonState = newState;

  // set button led colors



}

void sendControlChange() {
  byte cc = ccLookup[buttonState];
  float val = controlState[buttonState];
  byte lastVal = ccState[buttonState];
  byte newVal = (byte)(val * 127);

  if (lastVal != newVal) {
    MIDI.sendControlChange(cc, newVal, 1);
    ccState[buttonState] = newVal;
  }
}

void updateRing() {


  float val = controlState[buttonState];

  // update LEDs

  uint8_t colorIdx, r, g, b;
  float pxelValue;
  for (int pixelIdx = 0; pixelIdx < RING_NUM_PIXELS; pixelIdx++) {
    colorIdx = colorLookup[pixelIdx % activeColors];

    float lum = (1 - (pixelIdx / (float)(RING_NUM_PIXELS / 2))) + (val * 2) - 1;
    lum *= 20;
    if (lum < 0.1) lum = 0.1;
    if (lum > 1.0) lum = 1.0;

    r = buttonColors[colorIdx] * lum;
    g = buttonColors[colorIdx + 1] * lum;
    b = buttonColors[colorIdx + 2] * lum;

    uint32_t color = ring_pixels.Color(r, g, b);
    ring_pixels.setPixelColor(pixelIdx, color);
  }

  ring_pixels.show();
}

void encoder_a_rise() {
  detachInterrupt(0);

  if (encoder_b) {
    encoder_pulses--;
  } else {
    encoder_pulses++;
  }

  encoder_a = true;
  encoder_changed = true;

  attachInterrupt(0, encoder_a_fall, FALLING);
}

void encoder_a_fall() {
  detachInterrupt(0);

  if (encoder_b) {
    encoder_pulses++;
  } else {
    encoder_pulses--;
  }

  encoder_a = false;
  encoder_changed = true;

  attachInterrupt(0, encoder_a_rise, RISING);
}

void encoder_b_rise() {
  detachInterrupt(1);

  if (encoder_a) {
    encoder_pulses++;
  } else {
    encoder_pulses--;
  }

  encoder_b = true;
  encoder_changed = true;

  attachInterrupt(1, encoder_b_fall, FALLING);
}

void encoder_b_fall() {
  detachInterrupt(1);

  if (encoder_a) {
    encoder_pulses--;
  } else {
    encoder_pulses++;
  }

  encoder_b = false;
  encoder_changed = true;

  attachInterrupt(1, encoder_b_rise, RISING);
}
