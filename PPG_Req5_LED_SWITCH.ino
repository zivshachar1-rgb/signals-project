// REQ 5: LED and button behaviour (separate tab)

#define LED_STATUS_PIN 32   // LED 1: connection status
#define LED_HIGH_PIN   18   // LED 2: high HR alarm   (placeholder pin) REMEMBER TO CHANGE ALL THESE PLEASE PELASE PLEASE DONT BE STUPID AND FORGET!!!
#define LED_LOW_PIN    19   // LED 3: low HR alarm    (placeholder pin)
#define BTN2_PIN       26   // Button 2: silence alarm (placeholder pin)
#define BTN3_PIN       27   // Button 3: change preset (placeholder pin)
// Button 1 is switch1 on SWITCH_PIN in the main file

#define HOLD_MS   2000
#define DOUBLE_MS 400

enum LinkState  { LINK_OFF, LINK_SEARCHING, LINK_CONNECTED };
enum AlarmState { ALARM_NONE, ALARM_HIGH, ALARM_LOW };
enum Command    { CMD_NONE, CMD_RECORD, CMD_MARKER, CMD_CONNECT, CMD_SILENCE, CMD_PRESET };

// set by the comms code
LinkState  link_state  = LINK_OFF;
AlarmState alarm_state = ALARM_NONE;
// read and cleared by the packet code (goes in the Req 18 packet)
Command command = CMD_NONE;
bool alarm_silenced = false;

Switch switch2(2, DEBOUNCE_CNT);   // REQ 3 for buttons 2 and 3
Switch switch3(3, DEBOUNCE_CNT);

void ledsButtonsSetup() {
  pinMode(LED_STATUS_PIN, OUTPUT);
  pinMode(LED_HIGH_PIN, OUTPUT);
  pinMode(LED_LOW_PIN, OUTPUT);
  pinMode(BTN2_PIN, INPUT);
  pinMode(BTN3_PIN, INPUT);
}

// Button 1: single = record, double = marker, hold = connect/disconnect
void button1Update(unsigned long now) {
  static bool prev = false, held = false, pending = false;
  static unsigned long pressed_at = 0, released_at = 0;
  bool s = switch1.state();

  if (s && !prev) { pressed_at = now; held = false; }                // press
  if (!s && prev && !held) {                                         // release
    if (pending) { command = CMD_MARKER; pending = false; }          // 2nd click
    else { pending = true; released_at = now; }                      // 1st click
  }
  if (s && !held && now - pressed_at >= HOLD_MS) {                   // hold
    held = true; pending = false; command = CMD_CONNECT;
  }
  if (pending && !s && now - released_at >= DOUBLE_MS) {             // single
    pending = false; command = CMD_RECORD;
  }
  prev = s;
}

// Buttons 2 and 3: single press only
void button23Update() {
  static bool prev2 = false, prev3 = false;
  switch2.update(digitalRead(BTN2_PIN));
  switch3.update(digitalRead(BTN3_PIN));
  bool s2 = switch2.state(), s3 = switch3.state();

  if (s2 && !prev2) { alarm_silenced = true; command = CMD_SILENCE; }
  if (s3 && !prev3) command = CMD_PRESET;
  prev2 = s2; prev3 = s3;
}

void ledsUpdate() {
  static uint32_t ticks = 0;
  ticks++;
  bool slow = (ticks / 25) % 2;   // 500 ms on/off
  bool fast = (ticks / 5) % 2;    // 100 ms on/off

  if (alarm_state == ALARM_NONE) alarm_silenced = false;   // re-arms after returning to normal

  digitalWrite(LED_STATUS_PIN, link_state == LINK_CONNECTED ? HIGH : (link_state == LINK_SEARCHING ? slow : LOW));
  digitalWrite(LED_HIGH_PIN, alarm_state == ALARM_HIGH && !alarm_silenced && fast);
  digitalWrite(LED_LOW_PIN,  alarm_state == ALARM_LOW  && !alarm_silenced && slow);
}

// call once per 20 ms tick, after switch1.update()
void ledsButtonsUpdate() {
  button1Update(millis());
  button23Update();
  ledsUpdate();
}