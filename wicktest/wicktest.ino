#include <FastLED.h>
#include <SoftwareSerial.h>

#define WICK_LED_PIN    10
#define BARREL_LED_PIN  11
#define NUM_WICK_LEDS   60
#define NUM_BARREL_LEDS 60
#define BRIGHTNESS      50
#define LED_TYPE        WS2811
#define COLOR_ORDER     GRB
CRGB wickLeds[NUM_WICK_LEDS];
CRGB barrelLeds[NUM_BARREL_LEDS];

#define UPDATES_PER_SECOND 100

#define MOTION_SENSOR_PIN 4
static bool motionSensorValue = false;
static bool motionState = false;

#define INTERNAL_LED_PIN 13

#define AUDIO_RX_PIN 6  // Arduino Pin connected to the TX of the Serial MP3 Player module
#define AUDIO_TX_PIN 7  // Arduino Pin connected to the RX of the Serial MP3 Player module
#define CMD_SEL_DEV 0x09
#define DEV_TF 0x02
#define CMD_PLAY 0x0D

SoftwareSerial mp3(AUDIO_RX_PIN, AUDIO_TX_PIN);

void setup() {
    delay( 3000 ); // power-up safety delay

    // LED Setup
    FastLED.addLeds<LED_TYPE, WICK_LED_PIN, COLOR_ORDER>(wickLeds, NUM_WICK_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.addLeds<LED_TYPE, BARREL_LED_PIN, COLOR_ORDER>(barrelLeds, NUM_BARREL_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness(  BRIGHTNESS );

    // Motion Sensor Setup
    pinMode(MOTION_SENSOR_PIN, INPUT);
    pinMode(INTERNAL_LED_PIN, OUTPUT);

    // Audio Setup
    Serial.begin(9600);
    mp3.begin(9600);
    delay(500);  // wait chip initialization is complete
    mp3_command(CMD_SEL_DEV, DEV_TF);  // select the TF card

    clearWickLEDsUpToIndex(NUM_WICK_LEDS);

    Serial.print("Setup Complete\n");
}

void loop()
{    
    // Read out the pirPin and store as val:
  motionSensorValue = digitalRead(MOTION_SENSOR_PIN);

  // If motion is detected (pirPin = HIGH), do the following:
  if (motionSensorValue == HIGH) {
    digitalWrite(INTERNAL_LED_PIN, HIGH); // Turn on the on-board LED.

    // Change the motion state to true (motion detected):
    long currentTime = millis();
    static long timeOfLastRun = -10000;
    long timeDelta = currentTime - timeOfLastRun;
    static long minTimeBetweenRuns = 1000;
    Serial.print("Motion sensor high. Time since last run: ");
    Serial.print(timeDelta);
    Serial.print("\n");
    if (motionState == false && timeDelta > minTimeBetweenRuns) {
      Serial.print("Lighting Wick\n");
      motionState = true;      
      // Kick off sound
      mp3_command(CMD_PLAY, 0x0000);
      Serial.print("Played sound\n");
      // Light wick
      lightWick();
      Serial.print("Wick Done\n");
      // Smoke!
      // TODO: smokeOn();

      // Delay for synchronization 
      FastLED.delay(750);
      // Flash barrel
      flashBarrel();
      // Turn off wick
      clearWickLEDsUpToIndex(NUM_WICK_LEDS);
      // Fade barrel:
      fadeBarrel();
      // TODO: smokeOff();      

      timeOfLastRun = millis();
      Serial.print("Run complete at ");
      Serial.print(timeOfLastRun);
      Serial.print("\n");
    }
    else {
      Serial.print("Not running - not enough elapsed time\n");
    }
  }
  // If no motion is detected (pirPin = LOW), do the following:
  else {
    digitalWrite(INTERNAL_LED_PIN, LOW); // Turn off the on-board LED.

    // Change the motion state to false (no motion):
    if (motionState == true) {
      motionState = false;
    }
  }    
}

// How many MS between wick flickers
static const int wickFlickerSpeedMS = 1000 / UPDATES_PER_SECOND;
// Amount of time between the wick "burning down" in ms
static const int wickMovementDelayMS = 500;
static const int flickerIterations = wickMovementDelayMS / flickerUpdateDelayMS;
void lightWick() {
    uint8_t wickLength = 10;
    int lastWickTick = -1; // start at a time we can't match
    uint8_t wickIndex = wickLength-1;

    while (1) {
        // Current "wick tick" is the milliseconds from launch div by how often we care to move the wick, mod a reasonable number so we don't overflow.
        int wickTick = (millis() / wickMovementDelayMS) % 1000;
        
        // Progress wick index once per time chunk
        if (wickTick != lastWickTick) {
            lastWickTick = wickTick;   
            wickIndex = wickIndex - 1;
            // Serial.print("Decrementing wick index\n");  
        }

        // Serial.print("Flickering at index ");
        // Serial.print(wickIndex);
        // Serial.print("\n");
        flickerWickStartingAtIndex(wickIndex, wickLength);        

        if (wickIndex == 0) {
          break;
        }
    }
}

void clearWickLEDsUpToIndex(uint8_t idx) {
      // Clear all wick LEDs, add extra to accomodate for flame height
      // Serial.print("Clearing wick LEDS: ");
    for (uint8_t i = 0; i < idx; i++) {
        // Serial.print(i);
        // Serial.print(" ");
        wickLeds[i] = CRGB::Black;
    }
    // Serial.print("\n");
    FastLED.show();
}

void flickerWickStartingAtIndex(uint8_t idx, uint8_t wickLength) {

    // Clear all wick LEDs, add extra to accomodate for flame height
    clearWickLEDsUpToIndex(idx);

    // Set "burnt" wick LEDs to dim red
    for (uint8_t i = idx + 2; i < wickLength+1; i++) {
        wickLeds[i].setRGB(8, 0, 0);
    }
    
    for (int i = 0; i < flickerIterations; i++) {
      // Fraction of a second mod 255 to fit into uint8
      uint8_t quantizedTimestamp = millis() % 128;
      static uint8_t lastTimestamp = 255;

      if (lastTimestamp != quantizedTimestamp) {
          lastTimestamp = quantizedTimestamp;
          if (quantizedTimestamp % 2 == 0) {
              wickLeds[idx+1].setRGB(145, 88, 0);
              wickLeds[idx].setRGB(94, 53, 0);
          } else {
              wickLeds[idx+1] = CRGB::Black;
              wickLeds[idx].setRGB(94, 53, 0);
          }
          
      }
      
      FastLED.show();
      FastLED.delay(wickFlickerSpeedMS);
    }
}

void flashBarrel() {
  for (uint8_t i = 0; i < NUM_BARREL_LEDS; i++) {
    barrelLeds[i].setRGB(255, 128, 128);
  } 
  FastLED.show();
}

void fadeBarrel() {
  for (uint8_t step = 0; step < 255; step++) {
    for (uint8_t i = 0; i < NUM_BARREL_LEDS; i++) {
      barrelLeds[i].fadeToBlackBy(2);
    }  
    FastLED.show();
    FastLED.delay(1000 / UPDATES_PER_SECOND);
    Serial.print("Fade step");
    Serial.print(step);
    Serial.print("\n");
  }
}

void mp3_command(int8_t command, int16_t dat) {
  int8_t frame[8] = { 0 };
  frame[0] = 0x7e;                // starting byte
  frame[1] = 0xff;                // version
  frame[2] = 0x06;                // the number of bytes of the command without starting byte and ending byte
  frame[3] = command;             //
  frame[4] = 0x00;                // 0x00 = no feedback, 0x01 = feedback
  frame[5] = (int8_t)(dat >> 8);  // data high byte
  frame[6] = (int8_t)(dat);       // data low byte
  frame[7] = 0xef;                // ending byte
  for (uint8_t i = 0; i < 8; i++) {
    mp3.write(frame[i]);
  }
}

