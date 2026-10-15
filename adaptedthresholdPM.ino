/*
** Calculate heart rate from pulse sensor
** Checkpoint 5 - Adaptive Threshold
*/

#include "switch.h"

// Pins
#define LED_PIN 32
#define SWITCH_PIN 33
#define SENSOR_PIN 25
// printing output
uint8_t print_counter = 0;
#define PRINT_DECIMATE 5
// Scheduler timing
unsigned long lastTickTime;
#define TICK_20MSEC 20000

// Switch debounce
#define DEBOUNCE_CNT 10
Switch switch1(1, DEBOUNCE_CNT);

// Latching switch
bool latching_1 = true;


//**************** PULSE SENSOR ****************

uint16_t sensor_reading;

// Adaptive threshold variables
uint16_t min_signal = 4095;
uint16_t max_signal = 0;
uint16_t pulse_threshold = 1400;

// 100 samples × 20 ms = 2 seconds
uint16_t threshold_count = 0;
#define THRESHOLD_WINDOW 100


//**************** PULSE TRAIN ****************

bool old_PT_state = false;
bool current_PT_state = false;


//**************** HEART RATE ****************

unsigned long last_transition = 0;
unsigned long new_transition = 0;
unsigned long pulse_period = 0;

float heart_rate = 0;


void setup() {

  pinMode(LED_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);
  pinMode(SENSOR_PIN, INPUT);

  Serial.begin(115200);
  delay(1000);

  lastTickTime = micros();
}


void loop() {

  if ((micros() - lastTickTime) >= TICK_20MSEC) {

    lastTickTime = micros();


    //================================================
    // LATCHING SWITCH
    //================================================

    if (switch1.update(digitalRead(SWITCH_PIN))) {

      if (switch1.state()) {
        latching_1 = !latching_1;
      }
    }


    //================================================
    // READ SENSOR
    //================================================

    sensor_reading = analogRead(SENSOR_PIN);


    //================================================
    // ADAPTIVE THRESHOLD
    //================================================

    if (sensor_reading < min_signal) {
      min_signal = sensor_reading;
    }

    if (sensor_reading > max_signal) {
      max_signal = sensor_reading;
    }

    threshold_count++;


    // Recalculate threshold every 2 seconds
    if (threshold_count >= THRESHOLD_WINDOW) {

      // Set threshold 70% of the way
      // from minimum to maximum
      pulse_threshold =
          min_signal +
          (uint16_t)(0.7 * (max_signal - min_signal));

      // Reset min/max for next window
      min_signal = 4095;
      max_signal = 0;
      threshold_count = 0;
    }


    //================================================
    // CREATE PULSE TRAIN
    //================================================

    old_PT_state = current_PT_state;

    if (sensor_reading > pulse_threshold) {

      current_PT_state = true;
      digitalWrite(LED_PIN, HIGH);

    } else {

      current_PT_state = false;
      digitalWrite(LED_PIN, LOW);
    }


    //================================================
    // RISING EDGE DETECTION
    //================================================

    if ((old_PT_state == false) &&
        (current_PT_state == true)) {

      new_transition = millis();


      // First detected heartbeat
      if (last_transition == 0) {

        last_transition = new_transition;

      } else {

        unsigned long time_since_last =
            new_transition - last_transition;


        // Ignore false detections that are too fast
        if (time_since_last >= 300) {

          pulse_period = time_since_last;

          heart_rate =
              60000.0 / (float)pulse_period;

          last_transition = new_transition;
        }
      }
    }


    //================================================
    // OUTPUT
    //================================================
    print_counter++;
    if (print_counter >= PRINT_DECIMATE) {
      print_counter = 0;
      if (latching_1) {


        Serial.printf(
          "Raw:%d Threshold:%d HeartRate:%.1f\n",
          sensor_reading,
          pulse_threshold,
          heart_rate
      );
    }
    }
  }

}