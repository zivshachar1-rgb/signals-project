#include "switch.h"
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

// ---- pins ----
#define SENSOR_PIN       25
#define BUTTON_PIN       33
#define LED_STATUS_PIN   27   // connection / recording status
#define LED_ALARM_PIN    32   // heart-rate alarm

#define TICK_20MSEC      20000UL   // REQ 1: 20 ms sampling period (in us)
#define DEBOUNCE_CNT     5         // REQ 3: debounce count of 5

// ---- signal processing ----
#define WINDOW_TICKS     100       // REQ 19: 2 s window for the adaptive threshold
#define THRESHOLD_UPDATE 5         // recalculate every 100 ms
#define MIN_AMPLITUDE    50        // below this peak-to-trough range = no pulse

// ---- heart-rate timing ----
#define MIN_PERIOD_MS    300       // ignore beats faster than 200 BPM
#define MAX_PERIOD_MS    2000      // gap longer than this restarts the estimate
#define BEAT_TIMEOUT_MS  3000      // no beat for this long = BPM back to 0
#define INTERVAL_COUNT   5

// ---- alarm limits (fixed for now) ----
#define LOW_BPM_LIMIT    70
#define HIGH_BPM_LIMIT   120

// ---- button timing ----
#define HOLD_MS          2000UL
#define DOUBLE_MS        500UL

// ---- Bluetooth and packet (REQ 15, 18) ----
#define BT_NAME          "ESP32AK"   // name the host pairs with
#define PKT_SAMPLES      50          // 50 x 20 ms = 1 s of data per packet

enum AlarmState { ALARM_NONE, ALARM_HIGH, ALARM_LOW };
enum LinkState  { LINK_OFF, LINK_SEARCHING, LINK_CONNECTED };

AlarmState alarm_state = ALARM_NONE;
LinkState  link_state = LINK_OFF;
bool alarm_silenced = false;
bool recording = false;
bool bt_enabled = true;            // button hold turns host connection off / on

BluetoothSerial SerialBT;
Switch button(2, DEBOUNCE_CNT);    // REQ 3: debounce on the microcontroller

unsigned long lastTickTime = 0;
uint32_t ticks = 0;

// button gesture state
bool button_prev = false, hold_handled = false, pending_click = false, second_press = false;
unsigned long pressed_at = 0, released_at = 0;

// packet state
int pkt_raw[PKT_SAMPLES];
int pkt_count = 0;
unsigned long pkt_seq = 0;
uint8_t pending_cmd = 0;           // 0 none, 1 record start, 2 record stop, 3 marker, 4 alarm silenced

// PPG filtering and adaptive threshold (REQ 19)
float smooth = 0.0f, baseline = 0.0f;
bool filters_ready = false;
int16_t samples[WINDOW_TICKS];
uint16_t sample_index = 0, samples_seen = 0;
bool valid_signal = false, beat_armed = true;
int threshold = 0, reset_threshold = 0;

// heart-rate estimation (REQ 2)
unsigned long last_beat_time = 0;
unsigned long intervals[INTERVAL_COUNT];
uint8_t interval_index = 0, interval_count = 0;
float heart_rate = 0.0f;

void clearIntervals() {
  interval_index = 0;
  interval_count = 0;
}

float medianInterval() {
  if (interval_count == 0) return 0.0f;
  unsigned long sorted[INTERVAL_COUNT];
  for (uint8_t i = 0; i < interval_count; i++) sorted[i] = intervals[i];
  for (uint8_t i = 1; i < interval_count; i++) {          // insertion sort
    unsigned long key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key) { sorted[j + 1] = sorted[j]; j--; }
    sorted[j + 1] = key;
  }
  if (interval_count % 2 == 1) return sorted[interval_count / 2];
  return (sorted[interval_count / 2 - 1] + sorted[interval_count / 2]) / 2.0f;
}

void registerBeat(unsigned long now) {
  if (last_beat_time == 0) { last_beat_time = now; return; }

  unsigned long period = now - last_beat_time;
  if (period < MIN_PERIOD_MS) return;                      // too fast, ignore

  if (period > MAX_PERIOD_MS) {                            // too slow, restart estimate
    last_beat_time = now;
    heart_rate = 0.0f;
    clearIntervals();
    return;
  }

  last_beat_time = now;
  intervals[interval_index] = period;
  interval_index = (interval_index + 1) % INTERVAL_COUNT;
  if (interval_count < INTERVAL_COUNT) interval_count++;

  float median_ms = medianInterval();
  if (median_ms > 0.0f) heart_rate = 60000.0f / median_ms;
}

// REQ 18: fixed-length packet (264 bytes), sent once per second
// format: seq,bpm,s0,...,s49,cmd\n   e.g. 00123,078.9,1930,...,1944,3
void sendPacket() {
  char line[300];
  float bpm = heart_rate > 999.9f ? 999.9f : heart_rate;
  int len = snprintf(line, sizeof(line), "%05lu,%05.1f", pkt_seq, bpm);
  for (int i = 0; i < PKT_SAMPLES; i++)
    len += snprintf(line + len, sizeof(line) - len, ",%04d", pkt_raw[i]);
  len += snprintf(line + len, sizeof(line) - len, ",%u\n", (unsigned)pending_cmd);

  if (link_state == LINK_CONNECTED) SerialBT.write((const uint8_t *)line, len);
  pkt_seq = (pkt_seq + 1) % 100000;      // keeps counting even with no host, so gaps are visible
  pending_cmd = 0;
}

// REQ 15 + 17 + 5, status LED: off = host disabled, slow blink = waiting for host,
// double-blink = connected (not recording), solid = connected and recording
void connectionUpdate() {
  if (!bt_enabled && SerialBT.hasClient()) SerialBT.disconnect();   // user chose to disconnect

  if (!bt_enabled)               link_state = LINK_OFF;
  else if (SerialBT.hasClient()) link_state = LINK_CONNECTED;
  else                           link_state = LINK_SEARCHING;

  if (link_state != LINK_CONNECTED) recording = false;

  bool slow_blink = ((ticks / 25) % 2) != 0;
  uint8_t p = ticks % 50;
  bool dbl = (p < 5) || (p >= 10 && p < 15);

  bool led_on = false;
  if (link_state == LINK_SEARCHING) led_on = slow_blink;
  else if (link_state == LINK_CONNECTED) led_on = recording ? true : dbl;
  digitalWrite(LED_STATUS_PIN, led_on ? HIGH : LOW);
}

// REQ 5, one button: single = silence alarm (else toggle recording),
// double = event marker, hold = host connection off / on
void buttonUpdate(unsigned long now) {
  bool pressed = button.state();

  if (pressed && !button_prev) {                           // press
    pressed_at = now;
    hold_handled = false;
    if (pending_click && now - released_at <= DOUBLE_MS) { // 2nd press arrived in time
      second_press = true;
      pending_click = false;
    }
  }

  if (pressed && !hold_handled && now - pressed_at >= HOLD_MS) {   // hold
    hold_handled = true;
    pending_click = false;
    second_press = false;
    bt_enabled = !bt_enabled;
  }

  if (!pressed && button_prev && !hold_handled) {          // release
    if (second_press) {
      second_press = false;                                // double click
      pending_cmd = 3;
    } else {
      pending_click = true;
      released_at = now;
    }
  }

  if (pending_click && !pressed && now - released_at >= DOUBLE_MS) {   // single click
    pending_click = false;
    if (alarm_state != ALARM_NONE && !alarm_silenced) {
      alarm_silenced = true;
      pending_cmd = 4;
    } else if (link_state == LINK_CONNECTED) {
      recording = !recording;
      pending_cmd = recording ? 1 : 2;
    }
  }

  button_prev = pressed;
}

// REQ 5, alarm LED: fast flash = high, slow flash = low, off when silenced
void alarmLedUpdate() {
  bool slow_blink = ((ticks / 25) % 2) != 0;
  bool fast_blink = ((ticks / 5) % 2) != 0;

  if (alarm_state == ALARM_NONE) alarm_silenced = false;   // re-arms when BPM is normal

  bool led_on = false;
  if (!alarm_silenced) {
    if (alarm_state == ALARM_HIGH) led_on = fast_blink;
    else if (alarm_state == ALARM_LOW) led_on = slow_blink;
  }
  digitalWrite(LED_ALARM_PIN, led_on ? HIGH : LOW);
}

void setup() {
  pinMode(SENSOR_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);
  pinMode(LED_ALARM_PIN, OUTPUT);
  digitalWrite(LED_STATUS_PIN, LOW);
  digitalWrite(LED_ALARM_PIN, LOW);
  Serial.begin(115200);
  if (!SerialBT.begin(BT_NAME)) Serial.println("Bluetooth init failed");   // REQ 15, called once only
  lastTickTime = micros();
}

void loop() {
  if ((unsigned long)(micros() - lastTickTime) < TICK_20MSEC) return;
  lastTickTime += TICK_20MSEC;     // REQ 1: fixed 20 ms schedule
  ticks++;

  unsigned long now = millis();

  // REQ 1 + 3: button sampled every 20 ms and debounced
  button.update(digitalRead(BUTTON_PIN));
  buttonUpdate(now);
  connectionUpdate();

  // REQ 1: sensor sampled every 20 ms
  uint16_t raw = analogRead(SENSOR_PIN);

  // REQ 18: collect 50 raw samples, then send one packet per second
  pkt_raw[pkt_count++] = raw;
  if (pkt_count >= PKT_SAMPLES) { pkt_count = 0; sendPacket(); }

  // REQ 19: low-pass removes noise, slow baseline removes drift
  if (!filters_ready) { smooth = raw; baseline = raw; filters_ready = true; }
  smooth   += 0.4f  * (raw - smooth);
  baseline += 0.01f * (smooth - baseline);
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
      threshold       = win_min + (int)(0.60f * amplitude);
      reset_threshold = win_min + (int)(0.40f * amplitude);
    }
  }

  // REQ 2: beat detection with hysteresis
  if (valid_signal && sensor_reading < reset_threshold) beat_armed = true;
  if (valid_signal && beat_armed && sensor_reading > threshold) {
    beat_armed = false;
    registerBeat(now);
  }

  // expire BPM if no new beat has arrived
  if (last_beat_time != 0 && now - last_beat_time >= BEAT_TIMEOUT_MS) {
    heart_rate = 0.0f;
    last_beat_time = 0;
    clearIntervals();
  }

  // REQ 5: alarm state from BPM
  if (heart_rate <= 0.0f)                alarm_state = ALARM_NONE;
  else if (heart_rate > HIGH_BPM_LIMIT)  alarm_state = ALARM_HIGH;
  else if (heart_rate < LOW_BPM_LIMIT)   alarm_state = ALARM_LOW;
  else                                   alarm_state = ALARM_NONE;

  alarmLedUpdate();

  // REQ 4: waveform + threshold (+ BPM) over USB for the Arduino Serial Plotter
  Serial.printf("Raw:%d Threshold:%d HeartRate:%.1f\n", sensor_reading, threshold, heart_rate);
}