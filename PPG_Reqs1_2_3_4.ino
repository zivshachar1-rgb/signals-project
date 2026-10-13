#include "switch.h"

#define SWITCH_PIN 33
#define SENSOR_PIN 25

#define TICK_20MSEC 20000      // REQ 1: 20 ms sampling period (in us)
#define DEBOUNCE_CNT 5         // REQ 3: debounce count of 5
#define WINDOW_TICKS 150       // REQ 19: threshold re-estimated every 3 s
#define MIN_AMPLITUDE 100      // ignores 'false readings' (if no finger is present on the sensor just ignore it as a no reading)
#define MIN_PERIOD_MS 300      // ignore beats faster than 200 BPM

Switch switch1(1, DEBOUNCE_CNT);   // REQ 3: debounce implemented on the microcontroller

unsigned long lastTickTime;
uint16_t win_min = 4095, win_max = 0, win_count = 0;
int threshold = 2000;
bool valid_signal = false, old_PT_state = false;
unsigned long last_transition = 0;
float heart_rate = 0.0;

void setup() {
  pinMode(SWITCH_PIN, INPUT);
  pinMode(SENSOR_PIN, INPUT);
  Serial.begin(115200);
  lastTickTime = micros();
}

void loop() {
  if ((micros() - lastTickTime) < TICK_20MSEC) return;
  lastTickTime += TICK_20MSEC;     // REQ 1

  // REQ 1 + 3: switch sampled every 20 ms and debounced (5 counts)
  switch1.update(digitalRead(SWITCH_PIN));

  // REQ 1: sensor sampled every 20 ms
  uint16_t sensor_reading = analogRead(SENSOR_PIN);

  // REQ 19: adaptive threshold from the last window's min and max
  if (sensor_reading < win_min) win_min = sensor_reading;
  if (sensor_reading > win_max) win_max = sensor_reading;
  if (++win_count >= WINDOW_TICKS) {
    valid_signal = (win_max - win_min) > MIN_AMPLITUDE;
    if (valid_signal) threshold = win_min + 0.6 * (win_max - win_min);
    else { heart_rate = 0.0; last_transition = 0; }
    win_min = 4095; win_max = 0; win_count = 0;
  }

  // REQ 2 - calculating the BPM from threshold crossings
  bool pt_state = valid_signal && (sensor_reading > threshold);
  if (pt_state && !old_PT_state) {
    unsigned long now = millis();
    if (last_transition == 0) last_transition = now;
    else if (now - last_transition > MIN_PERIOD_MS) {
      heart_rate = 60000.0 / (now - last_transition);
      last_transition = now;
    }
  }
  old_PT_state = pt_state;

  // REQ 4: waveform + threshold printed for the Arduino Serial Plotter
  Serial.printf("%u,%d\n", sensor_reading, threshold);
}