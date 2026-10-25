/*
 * ============================================================================
 *  ESP32 WEARABLE PPG PULSE MONITOR  -  final sketch
 * ============================================================================
 *  WHAT IT DOES
 *    Samples a pulse (PPG) sensor every 20 ms, detects heartbeats, calculates
 *    heart rate (BPM) and sends the data to a host computer over Bluetooth
 *    once per second. One button and two LEDs give the user local control.
 *    The host (GUI) sends back the alarm limits chosen by the user, so the
 *    alarm LED follows the same limits as the GUI (two-way alarm).
 *
 *  DATA FLOW (inside loop(), every 20 ms)
 *    button -> sensor -> filter -> adaptive threshold -> beat detection
 *    -> BPM -> alarm state (using the GUI's limits) -> LEDs
 *    -> one packet per second over Bluetooth
 *
 *  REQUIREMENT MAP
 *    Req 1   20 ms sampling of sensor and button ..... loop() scheduler
 *    Req 2   BPM calculated on the microcontroller ... registerBeat(), loop()
 *    Req 3   debounce count of 5 ..................... Switch class, DEBOUNCE_CNT
 *    Req 4   waveform + threshold for Serial Plotter . last line of loop()
 *    Req 5   LED and button behaviour ................ buttonUpdate(),
 *                                                      connectionUpdate(),
 *                                                      alarmLedUpdate(),
 *                                                      limitsUpdate()
 *    Req 15  Bluetooth SPP ........................... SerialBT, setup()
 *    Req 16  reconnect ............................... ESP32 keeps advertising
 *                                                      after a drop; the host retries
 *    Req 17  connect within 10 s ..................... advertising from power-up
 *    Req 18  fixed-length packet every second ........ sendPacket()
 *    Req 19  low-frequency drift ..................... baseline removal and
 *                                                      adaptive threshold in loop()
 * ============================================================================
 */

#include "switch.h"            // Switch class: debounces the button (Req 3)
#include "BluetoothSerial.h"   // classic Bluetooth serial port, SPP (Req 15)

// Stop compilation early if the board package has Bluetooth switched off
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

// ---- pins ----
#define SENSOR_PIN       25   // PPG sensor analog input
#define BUTTON_PIN       33   // the single user button
#define LED_STATUS_PIN   27   // connection / recording status
#define LED_ALARM_PIN    32   // heart-rate alarm

#define TICK_20MSEC      20000UL   // REQ 1: 20 ms sampling period (in us) = 50 samples/s
#define DEBOUNCE_CNT     5         // REQ 3: debounce count of 5 (button must disagree for 5 samples)

// ---- signal processing ----
#define WINDOW_TICKS     100       // REQ 19: 2 s window (100 x 20 ms) for the adaptive threshold
#define THRESHOLD_UPDATE 5         // recalculate threshold every 5 ticks = 100 ms
#define MIN_AMPLITUDE    50        // below this peak-to-trough range = no pulse (no finger)

// ---- heart-rate timing ----
#define MIN_PERIOD_MS    300       // ignore beats faster than 200 BPM (noise / double triggers)
#define MAX_PERIOD_MS    2000      // gap longer than this restarts the estimate (below 30 BPM)
#define BEAT_TIMEOUT_MS  3000      // no beat for this long = BPM back to 0
#define INTERVAL_COUNT   5         // number of recent beat intervals used for the median

// ---- alarm limits: defaults used until the GUI sends its own ----
#define DEFAULT_LOW_BPM  40
#define DEFAULT_HIGH_BPM 120
#define LIMIT_MIN_BPM    20        // limits outside 20-250 BPM are rejected as invalid
#define LIMIT_MAX_BPM    250

// ---- button timing ----
#define HOLD_MS          2000UL    // press longer than this = "hold"
#define DOUBLE_MS        500UL     // second press within this time = "double click"

// ---- Bluetooth and packet (REQ 15, 18) ----
#define BT_NAME          "ESP32AK"   // name the host pairs with
#define PKT_SAMPLES      50          // 50 x 20 ms = 1 s of data per packet

enum AlarmState { ALARM_NONE, ALARM_HIGH, ALARM_LOW };
enum LinkState  { LINK_OFF, LINK_SEARCHING, LINK_CONNECTED };

// ---- overall system state ----
AlarmState alarm_state = ALARM_NONE;
LinkState  link_state = LINK_OFF;
bool alarm_silenced = false;       // user silenced the current alarm
bool recording = false;            // host is recording (toggled by button)
bool bt_enabled = true;            // button hold turns host connection off / on
int low_bpm_limit = DEFAULT_LOW_BPM;     // alarm limits, set by the GUI sliders (two-way alarm)
int high_bpm_limit = DEFAULT_HIGH_BPM;

BluetoothSerial SerialBT;          // the Bluetooth serial port object
Switch button(2, DEBOUNCE_CNT);    // REQ 3: debounce on the microcontroller

// ---- scheduler ----
unsigned long lastTickTime = 0;    // when the last 20 ms tick started (us)
uint32_t ticks = 0;                // number of ticks since power-up (used for LED blink timing)

// ---- button gesture state ----
bool button_prev = false;          // debounced button state on the previous tick
bool hold_handled = false;         // true once a hold was acted on (so release is not a click)
bool pending_click = false;        // a click finished and we are waiting to see if a second follows
bool second_press = false;         // the second press of a double click has started
unsigned long pressed_at = 0, released_at = 0;   // timestamps of the last press / release (ms)

// ---- packet state ----
int pkt_raw[PKT_SAMPLES];          // raw ADC readings collected since the last packet
int pkt_count = 0;                 // how many are collected so far
unsigned long pkt_seq = 0;         // packet sequence number
uint8_t pending_cmd = 0;           // 0 none, 1 record start, 2 record stop, 3 marker, 4 alarm silenced

// ---- PPG filtering and adaptive threshold (REQ 19) ----
float smooth = 0.0f, baseline = 0.0f;   // low-pass output and slow baseline estimate
bool filters_ready = false;             // filters are initialised on the first sample
int16_t samples[WINDOW_TICKS];          // rolling window of filtered samples (ring buffer)
uint16_t sample_index = 0, samples_seen = 0;
bool valid_signal = false;              // true when the signal is big enough to be a pulse
bool beat_armed = true;                 // hysteresis flag: allowed to detect a new beat
int threshold = 0, reset_threshold = 0; // upper (detect) and lower (re-arm) levels

// ---- heart-rate estimation (REQ 2) ----
unsigned long last_beat_time = 0;                // time of the previous beat (ms), 0 = none yet
unsigned long intervals[INTERVAL_COUNT];         // recent beat-to-beat intervals (ms)
uint8_t interval_index = 0, interval_count = 0;  // ring buffer write position and fill level
float heart_rate = 0.0f;                         // latest BPM (0 = no valid reading)

/*
 * clearIntervals()
 * ----------------------------------------------------------------------------
 * Purpose : Forget the recent beat intervals.
 * Effect  : empties the interval ring buffer (the data is overwritten later).
 * Why     : used when the pulse is lost, so an old BPM never mixes with new beats.
 */
void clearIntervals() {
  interval_index = 0;
  interval_count = 0;
}

/*
 * medianInterval()
 * ----------------------------------------------------------------------------
 * Purpose : Return the median of the stored beat intervals, in ms.
 * Input   : the intervals[] array (up to 5 values)
 * Output  : median in ms, or 0 if no intervals yet
 * Why     : a median ignores a single false or missed beat, whereas an average
 *           would jump. Sorting a copy keeps the ring buffer order intact.
 */
float medianInterval() {
  if (interval_count == 0) return 0.0f;
  unsigned long sorted[INTERVAL_COUNT];
  for (uint8_t i = 0; i < interval_count; i++) sorted[i] = intervals[i];   // copy
  for (uint8_t i = 1; i < interval_count; i++) {          // insertion sort (tiny array)
    unsigned long key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key) { sorted[j + 1] = sorted[j]; j--; }
    sorted[j + 1] = key;
  }
  if (interval_count % 2 == 1) return sorted[interval_count / 2];                      // odd: middle value
  return (sorted[interval_count / 2 - 1] + sorted[interval_count / 2]) / 2.0f;        // even: mean of middle two
}

/*
 * registerBeat()
 * ----------------------------------------------------------------------------
 * Purpose : Turn one detected heartbeat into an updated heart rate.
 * Req     : 2 (BPM calculated on the microcontroller)
 * Input   : now = time of the beat in ms
 * Effect  : updates heart_rate, the interval history and last_beat_time
 * Why     : BPM = 60000 / median interval. Intervals outside 300-2000 ms are
 *           rejected (200 BPM max, 30 BPM min) so noise cannot give nonsense.
 */
void registerBeat(unsigned long now) {
  if (last_beat_time == 0) { last_beat_time = now; return; }   // first beat: just start the clock

  unsigned long period = now - last_beat_time;   // time since the previous beat (ms)
  if (period < MIN_PERIOD_MS) return;                      // too fast, ignore

  if (period > MAX_PERIOD_MS) {                            // too slow, restart estimate
    last_beat_time = now;
    heart_rate = 0.0f;
    clearIntervals();
    return;
  }

  last_beat_time = now;                                    // accept this beat
  intervals[interval_index] = period;                      // store the interval in the ring buffer
  interval_index = (interval_index + 1) % INTERVAL_COUNT;
  if (interval_count < INTERVAL_COUNT) interval_count++;

  float median_ms = medianInterval();
  if (median_ms > 0.0f) heart_rate = 60000.0f / median_ms; // ms per beat -> beats per minute
}

/*
 * sendPacket()
 * ----------------------------------------------------------------------------
 * Purpose : Build and send one fixed-length data packet to the host.
 * Req     : 18 (packet every second: BPM, 50 raw samples, sequence number)
 * Format  : seq,bpm,s0,...,s49,cmd\n    e.g. 00123,078.9,1930,...,1944,3
 *             seq : 5 digits, wraps at 99999
 *             bpm : 5 characters, one decimal place (e.g. 078.9)
 *             s0..s49 : 50 raw ADC values, 4 digits each (0000-4095)
 *             cmd : 1 digit, last button action (see pending_cmd)
 *           Length is always 263 characters + newline = 264 bytes.
 * Effect  : writes to Bluetooth (only if a host is connected), increments the
 *           sequence number and clears the pending button command.
 * Why     : zero-padded fields make every packet the same length (Req 18),
 *           and plain text is easy to debug in a serial monitor. The sequence
 *           number keeps counting even with no host, so the host can see gaps.
 */
void sendPacket() {
  char line[300];                                          // big enough for 264 bytes
  float bpm = heart_rate > 999.9f ? 999.9f : heart_rate;   // clamp so the field never exceeds 5 characters
  int len = snprintf(line, sizeof(line), "%05lu,%05.1f", pkt_seq, bpm);   // header: seq,bpm
  for (int i = 0; i < PKT_SAMPLES; i++)
    len += snprintf(line + len, sizeof(line) - len, ",%04d", pkt_raw[i]); // 50 samples, 4 digits each
  len += snprintf(line + len, sizeof(line) - len, ",%u\n", (unsigned)pending_cmd);   // command + end of packet

  if (link_state == LINK_CONNECTED) SerialBT.write((const uint8_t *)line, len);      // send only if a host is connected
  pkt_seq = (pkt_seq + 1) % 100000;      // keeps counting even with no host, so gaps are visible
  pending_cmd = 0;                       // command has been delivered (or dropped), clear it
}

/*
 * limitsUpdate()
 * ----------------------------------------------------------------------------
 * Purpose : Receive the alarm limits chosen on the GUI sliders (two-way alarm).
 * Req     : 5 and 11 (the alarm follows the limits the user sets on the host)
 * Message : "LIM,low,high\n"   e.g. LIM,055,120
 *           The GUI sends it whenever a slider moves, after every (re)connect
 *           and every 5 s as a refresh.
 * Effect  : updates low_bpm_limit and high_bpm_limit if the message is valid.
 * Why     : the ESP32 keeps its own copy of the limits and compares its own
 *           BPM to them, so the alarm LED still works if Bluetooth drops. The
 *           5 s refresh means a lost message, or an ESP32 reboot, corrects
 *           itself. Invalid values (outside 20-250, or low >= high) are
 *           ignored. Characters are read one at a time without waiting, so the
 *           20 ms sampling is never delayed.
 */
void limitsUpdate() {
  static char buf[24];                                     // one incoming line
  static uint8_t len = 0;

  while (SerialBT.available() > 0) {                       // only reads what has already arrived
    char c = SerialBT.read();
    if (c == '\r') continue;

    if (c == '\n') {                                       // end of line: try to decode it
      buf[len] = '\0';
      int lo, hi;
      if (sscanf(buf, "LIM,%d,%d", &lo, &hi) == 2 &&
          lo >= LIMIT_MIN_BPM && hi <= LIMIT_MAX_BPM && lo < hi) {
        low_bpm_limit = lo;
        high_bpm_limit = hi;
      }
      len = 0;
    }
    else if (len < sizeof(buf) - 1) buf[len++] = c;        // build up the line
    else len = 0;                                          // discard an overlong line
  }
}

/*
 * connectionUpdate()
 * ----------------------------------------------------------------------------
 * Purpose : Work out the Bluetooth link state and drive the status LED.
 * Req     : 15 / 17 (Bluetooth link, connect quickly), 5 (LED behaviour)
 * Effect  : sets link_state, clears recording when not connected, writes LED 1.
 * LED 1   : off          = host connection disabled by the user
 *           slow blink   = waiting for the host
 *           double-blink = connected, not recording
 *           solid        = connected and recording
 * Why     : the state comes straight from SerialBT.hasClient(), so the LED
 *           always shows the real link and not what we hope is happening.
 */
void connectionUpdate() {
  if (!bt_enabled && SerialBT.hasClient()) SerialBT.disconnect();   // user chose to disconnect

  if (!bt_enabled)               link_state = LINK_OFF;
  else if (SerialBT.hasClient()) link_state = LINK_CONNECTED;
  else                           link_state = LINK_SEARCHING;

  if (link_state != LINK_CONNECTED) recording = false;     // cannot record without a host

  bool slow_blink = ((ticks / 25) % 2) != 0;               // toggles every 25 ticks = 500 ms
  uint8_t p = ticks % 50;                                  // position in a 1 s cycle (50 ticks)
  bool dbl = (p < 5) || (p >= 10 && p < 15);               // two 100 ms flashes per second

  bool led_on = false;
  if (link_state == LINK_SEARCHING) led_on = slow_blink;
  else if (link_state == LINK_CONNECTED) led_on = recording ? true : dbl;
  digitalWrite(LED_STATUS_PIN, led_on ? HIGH : LOW);
}

/*
 * buttonUpdate()
 * ----------------------------------------------------------------------------
 * Purpose : Turn the debounced button state into single / double / hold gestures.
 * Req     : 5 (button behaviour; the command also goes to the host, Req 18)
 * Input   : now = current time in ms
 * Gestures: single click = silence an active alarm, otherwise start/stop recording
 *           double click = event marker
 *           hold 2 s     = turn the host connection off / on
 * Effect  : sets pending_cmd (sent in the next packet), alarm_silenced,
 *           recording and bt_enabled.
 * Why     : it is a small state machine that runs every 20 ms and never blocks.
 *           A single click is only confirmed 500 ms after release, because
 *           until then a second press could still turn it into a double click.
 */
void buttonUpdate(unsigned long now) {
  bool pressed = button.state();                           // debounced state (true = pressed)

  if (pressed && !button_prev) {                           // press
    pressed_at = now;
    hold_handled = false;
    if (pending_click && now - released_at <= DOUBLE_MS) { // 2nd press arrived in time
      second_press = true;
      pending_click = false;
    }
  }

  if (pressed && !hold_handled && now - pressed_at >= HOLD_MS) {   // hold
    hold_handled = true;                                   // stops the release being read as a click
    pending_click = false;
    second_press = false;
    bt_enabled = !bt_enabled;
  }

  if (!pressed && button_prev && !hold_handled) {          // release
    if (second_press) {
      second_press = false;                                // double click
      pending_cmd = 3;
    } else {
      pending_click = true;                                // maybe a single click: wait for a 2nd press
      released_at = now;
    }
  }

  if (pending_click && !pressed && now - released_at >= DOUBLE_MS) {   // single click
    pending_click = false;
    if (alarm_state != ALARM_NONE && !alarm_silenced) {    // alarm active: silence it
      alarm_silenced = true;
      pending_cmd = 4;
    } else if (link_state == LINK_CONNECTED) {             // otherwise toggle recording (needs a host)
      recording = !recording;
      pending_cmd = recording ? 1 : 2;
    }
  }

  button_prev = pressed;                                   // remember for edge detection next tick
}

/*
 * alarmLedUpdate()
 * ----------------------------------------------------------------------------
 * Purpose : Drive the heart-rate alarm LED.
 * Req     : 5 (LED behaviour)
 * Effect  : writes LED 2 and re-arms the alarm when BPM is back in range.
 * LED 2   : fast flash (5 Hz) = high heart rate
 *           slow flash (1 Hz) = low heart rate
 *           off               = normal, or alarm silenced
 * Why     : a silenced alarm stays silent until BPM returns to normal, so a
 *           new excursion triggers a fresh alarm instead of staying muted.
 */
void alarmLedUpdate() {
  bool slow_blink = ((ticks / 25) % 2) != 0;               // 500 ms on / 500 ms off
  bool fast_blink = ((ticks / 5) % 2) != 0;                // 100 ms on / 100 ms off

  if (alarm_state == ALARM_NONE) alarm_silenced = false;   // re-arms when BPM is normal

  bool led_on = false;
  if (!alarm_silenced) {
    if (alarm_state == ALARM_HIGH) led_on = fast_blink;
    else if (alarm_state == ALARM_LOW) led_on = slow_blink;
  }
  digitalWrite(LED_ALARM_PIN, led_on ? HIGH : LOW);
}

/*
 * setup()
 * ----------------------------------------------------------------------------
 * Purpose : One-time start-up: configure pins, USB serial and Bluetooth.
 * Req     : 15 (Bluetooth SPP started once, so the board is discoverable
 *           from power-up, which helps Req 17).
 */
void setup() {
  pinMode(SENSOR_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);
  pinMode(LED_ALARM_PIN, OUTPUT);
  digitalWrite(LED_STATUS_PIN, LOW);                       // both LEDs start off
  digitalWrite(LED_ALARM_PIN, LOW);
  Serial.begin(115200);                                    // USB serial for the Serial Plotter (Req 4)
  if (!SerialBT.begin(BT_NAME)) Serial.println("Bluetooth init failed");   // REQ 15, called once only
  lastTickTime = micros();                                 // start the 20 ms scheduler
}

/*
 * loop()
 * ----------------------------------------------------------------------------
 * Purpose : The whole real-time pipeline, run once every 20 ms.
 * Req     : 1 (20 ms schedule), 2, 4, 5, 18, 19
 * Why     : loop() runs continuously but only does work when 20 ms have
 *           passed. No delay() is used, so nothing blocks the sampling.
 *
 * Steps each tick:
 *   1. Schedule      wait until 20 ms have passed
 *   2. Inputs        read and debounce the button, update connection state,
 *                    receive any new alarm limits from the GUI
 *   3. Sample        read the PPG sensor, store it for the packet
 *   4. Filter        low-pass + baseline removal (Req 19)
 *   5. Threshold     adaptive threshold from the last 2 s (Req 19)
 *   6. Beat detect   hysteresis crossing -> registerBeat() (Req 2)
 *   7. Alarm         BPM vs the GUI's limits -> alarm state -> alarm LED (Req 5)
 *   8. Output        Serial Plotter line (Req 4)
 */
void loop() {
  // ---- 1. Schedule ----
  if ((unsigned long)(micros() - lastTickTime) < TICK_20MSEC) return;   // not time yet (unsigned maths survives micros() overflow)
  lastTickTime += TICK_20MSEC;     // REQ 1: fixed 20 ms schedule (advance by exactly 20 ms, so no drift)
  ticks++;

  unsigned long now = millis();    // timestamp used by the button and beat logic

  // ---- 2. Inputs ----
  // REQ 1 + 3: button sampled every 20 ms and debounced
  button.update(digitalRead(BUTTON_PIN));
  buttonUpdate(now);
  connectionUpdate();
  limitsUpdate();                  // two-way alarm: pick up new limits from the GUI

  // ---- 3. Sample ----
  // REQ 1: sensor sampled every 20 ms
  uint16_t raw = analogRead(SENSOR_PIN);                   // 12-bit reading, 0-4095

  // REQ 18: collect 50 raw samples, then send one packet per second
  pkt_raw[pkt_count++] = raw;
  if (pkt_count >= PKT_SAMPLES) { pkt_count = 0; sendPacket(); }   // 50 x 20 ms = exactly 1 s

  // ---- 4. Filter ----
  // REQ 19: low-pass removes noise, slow baseline removes drift
  if (!filters_ready) { smooth = raw; baseline = raw; filters_ready = true; }   // start from the first reading
  smooth   += 0.4f  * (raw - smooth);                      // moving average: low-pass, roughly 4 Hz cutoff
  baseline += 0.01f * (smooth - baseline);                 // very slow average (about 2 s): follows drift
  int sensor_reading = (int)(smooth - baseline);           // pulse signal centred on 0

  // rolling sample window (the last 2 s of filtered samples)
  samples[sample_index] = sensor_reading;
  sample_index = (sample_index + 1) % WINDOW_TICKS;
  if (samples_seen < WINDOW_TICKS) samples_seen++;

  // ---- 5. Adaptive threshold ----
  // REQ 19: adaptive threshold, updated every 100 ms once the window is full
  if (samples_seen == WINDOW_TICKS && ticks % THRESHOLD_UPDATE == 0) {
    int16_t win_min = 32767, win_max = -32768;
    for (uint16_t i = 0; i < WINDOW_TICKS; i++) {          // find the lowest and highest sample in the window
      if (samples[i] < win_min) win_min = samples[i];
      if (samples[i] > win_max) win_max = samples[i];
    }
    int amplitude = win_max - win_min;                     // peak-to-trough size of the pulse
    valid_signal = (amplitude >= MIN_AMPLITUDE);           // too small = no finger on the sensor
    if (valid_signal) {
      threshold       = win_min + (int)(0.60f * amplitude);   // detect level: 60% up the pulse
      reset_threshold = win_min + (int)(0.40f * amplitude);   // re-arm level: 40% up the pulse
    }
  }

  // ---- 6. Beat detection ----
  // REQ 2: beat detection with hysteresis (two levels stop one noisy edge counting twice)
  if (valid_signal && sensor_reading < reset_threshold) beat_armed = true;         // signal fell low: ready for the next beat
  if (valid_signal && beat_armed && sensor_reading > threshold) {                  // rising edge through the upper level
    beat_armed = false;
    registerBeat(now);
  }

  // expire BPM if no new beat has arrived
  if (last_beat_time != 0 && now - last_beat_time >= BEAT_TIMEOUT_MS) {
    heart_rate = 0.0f;
    last_beat_time = 0;
    clearIntervals();
  }

  // ---- 7. Alarm ----
  // REQ 5: alarm state from BPM, using the limits set on the GUI sliders
  if (heart_rate <= 0.0f)                    alarm_state = ALARM_NONE;    // no reading = no alarm
  else if (heart_rate > high_bpm_limit)      alarm_state = ALARM_HIGH;
  else if (heart_rate < low_bpm_limit)       alarm_state = ALARM_LOW;
  else                                       alarm_state = ALARM_NONE;

  alarmLedUpdate();

  // ---- 8. Output ----
  // REQ 4: waveform + threshold (+ BPM) over USB for the Arduino Serial Plotter
  Serial.printf("Raw:%d Threshold:%d HeartRate:%.1f\n", sensor_reading, threshold, heart_rate);
}