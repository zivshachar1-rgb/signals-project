// CURRENTLY THE BEST WORKING CODE FOR BBPM ACQUIRE --> GUI INTERFACE
// FUTURE ADAPTATIONS USE THIS AND IMPLEMENT MORE METHODS OF CALCULATING HEART RATE / FURTHER SIGNAL PROCESSING

#include "switch.h"

// ---- pins ----
#define SENSOR_PIN       25
#define BTN2_PIN         33   // Button 2: silence alarm
#define LED_HIGH_PIN     27   // LED 2: high HR alarm
#define LED_LOW_PIN      32   // LED 3: low HR alarm

#define TICK_20MSEC      20000UL   // REQ 1: 20 ms sampling period (in us)
#define DEBOUNCE_CNT     5         // REQ 3: debounce count of 5

// ---- signal processing ----
#define WINDOW_TICKS     100    // REQ 19: 2 s window for the adaptive threshold
#define THRESHOLD_UPDATE 5      // recalculate every 100 ms so it catches up quickly
#define MIN_AMPLITUDE    50     // below this peak-to-trough range = no pulse (try 20 if never valid)

// ---- beat timing ----
#define MIN_PERIOD_MS    300    // ignore beats faster than 200 BPM
#define MAX_PERIOD_MS    2000   // gap longer than this restarts the estimate
#define BEAT_TIMEOUT_MS  3000   // no beat for this long = BPM back to 0

// ---- alarm limits (currently hard coded, when gui complete need to obtain a state from this rather than hard values (e.g. gui sends alarm_high or alarm_low)) ----
#define LOW_BPM_LIMIT    70
#define HIGH_BPM_LIMIT   120

enum AlarmState { ALARM_NONE, ALARM_HIGH, ALARM_LOW };
AlarmState alarm_state = ALARM_NONE;
bool alarm_silenced = false;

Switch switch2(2, DEBOUNCE_CNT);   // REQ 3: debounce on the microcontroller

unsigned long lastTickTime = 0;
unsigned long last_beat_time = 0;
uint32_t ticks = 0;

// REQ 19: filtering and adaptive threshold
float smooth = 0, baseline = 0;
bool filters_ready = false;
int16_t samples[WINDOW_TICKS];
uint16_t sample_index = 0, samples_seen = 0;
bool valid_signal = false, beat_armed = true;
int threshold = 0, reset_threshold = 0;

// recent beat-to-beat intervals
#define INTERVAL_COUNT 5
unsigned long intervals[INTERVAL_COUNT];
uint8_t interval_index = 0, interval_count = 0;

float heart_rate = 0.0;

void clearIntervals() {
  interval_index = 0;
  interval_count = 0;
}

float medianInterval() {
  if (interval_count == 0) return 0.0;
  unsigned long sorted[INTERVAL_COUNT];
  for (uint8_t i = 0; i < interval_count; i++) sorted[i] = intervals[i];
  for (uint8_t i = 1; i < interval_count; i++) {          // insertion sort
    unsigned long key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key) { sorted[j + 1] = sorted[j]; j--; }
    sorted[j + 1] = key;
  }
  if (interval_count % 2 == 1) return sorted[interval_count / 2];
  return (sorted[interval_count / 2 - 1] + sorted[interval_count / 2]) / 2.0;
}

// REQ 2: BPM from the median of recent beat intervals
void registerBeat(unsigned long now) {
  if (last_beat_time == 0) { last_beat_time = now; return; }

  unsigned long period = now - last_beat_time;
  if (period < MIN_PERIOD_MS) return;                      // too fast, ignore

  if (period > MAX_PERIOD_MS) {                            // too slow, restart estimate
    last_beat_time = now;
    heart_rate = 0.0;
    clearIntervals();
    return;
  }

  last_beat_time = now;
  intervals[interval_index] = period;
  interval_index = (interval_index + 1) % INTERVAL_COUNT;
  if (interval_count < INTERVAL_COUNT) interval_count++;

  float median_ms = medianInterval();
  if (median_ms > 0.0) heart_rate = 60000.0 / median_ms;
}

void setup() {
  pinMode(SENSOR_PIN, INPUT);
  pinMode(BTN2_PIN, INPUT);
  pinMode(LED_HIGH_PIN, OUTPUT);
  pinMode(LED_LOW_PIN, OUTPUT);
  Serial.begin(115200);
  lastTickTime = micros();
}

void loop() {
  if ((unsigned long)(micros() - lastTickTime) < TICK_20MSEC) return;
  lastTickTime += TICK_20MSEC;     // REQ 1: fixed 20 ms schedule
  ticks++;

  // REQ 1 + 3: switch sampled every 20 ms and debounced
  static bool prev2 = false;
  switch2.update(digitalRead(BTN2_PIN));
  bool s2 = switch2.state();
  if (s2 && !prev2 && alarm_state != ALARM_NONE) alarm_silenced = true;
  prev2 = s2;

  // REQ 1: sensor sampled every 20 ms
  uint16_t raw = analogRead(SENSOR_PIN);

  // REQ 19: low-pass removes noise, slow baseline removes drift
  if (!filters_ready) { smooth = baseline = raw; filters_ready = true; }
  smooth   += 0.4  * (raw - smooth);
  baseline += 0.01 * (smooth - baseline);
  int sensor_reading = (int)(smooth - baseline);           // pulse signal centred on 0

  // rolling sample window
  samples[sample_index] = sensor_reading;
  sample_index = (sample_index + 1) % WINDOW_TICKS;
  if (samples_seen < WINDOW_TICKS) samples_seen++;

  // REQ 19: adaptive threshold, updated every 100 ms once the window is full
  if (samples_seen == WINDOW_TICKS && ticks % THRESHOLD_UPDATE == 0) {
    int16_t win_min = 32767, win_max = -32768;
    for (uint16_t i = 0; i < WINDOW_TICKS; i++) {
      if (samples[i] < win_min) win_min = samples[i];
      if (samples[i] > win_max) win_max = samples[i];
    }
    int amplitude = win_max - win_min;
    valid_signal = (amplitude >= MIN_AMPLITUDE);
    if (valid_signal) {
      threshold       = win_min + 0.60 * amplitude;
      reset_threshold = win_min + 0.40 * amplitude;
    }
  }

  // hysteresis: must fall below the lower level before the next beat can count
  if (valid_signal && sensor_reading < reset_threshold) beat_armed = true;
  if (valid_signal && beat_armed && sensor_reading > threshold) {
    beat_armed = false;
    registerBeat(millis());
  }

  // expire BPM if no new beat has arrived
  unsigned long now = millis();
  if (last_beat_time != 0 && now - last_beat_time >= BEAT_TIMEOUT_MS) {
    heart_rate = 0.0;
    last_beat_time = 0;
    clearIntervals();
  }

  // REQ 5: alarm state from BPM
  if (heart_rate <= 0.0)               alarm_state = ALARM_NONE;
  else if (heart_rate > HIGH_BPM_LIMIT) alarm_state = ALARM_HIGH;
  else if (heart_rate < LOW_BPM_LIMIT)  alarm_state = ALARM_LOW;
  else                                  alarm_state = ALARM_NONE;

  if (alarm_state == ALARM_NONE) alarm_silenced = false;   // re-arms when BPM is normal

  // REQ 5: alarm LEDs (high = fast flash, low = slow flash)
  bool slow = (ticks / 25) % 2, fast = (ticks / 5) % 2;
  digitalWrite(LED_HIGH_PIN, alarm_state == ALARM_HIGH && !alarm_silenced && fast);
  digitalWrite(LED_LOW_PIN,  alarm_state == ALARM_LOW  && !alarm_silenced && slow);

  // REQ 4: waveform + threshold (+ BPM) for the Serial Plotter and the GUI
  Serial.printf("Raw:%d Threshold:%d HeartRate:%.1f\n", sensor_reading, threshold, heart_rate);
}