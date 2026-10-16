// TESTING FOR THE CONNECTION LED:

// FOR SIMON (if you wanted to test over weekend otherwise ignore):
// what we want to see is this but remember this uses synthetic data so once gui finished but for now just want to check the code is good and will turn led on?
/*
Hold 2 s: it prints SEARCHING and the LED blinks slowly. After 3 s it prints CONNECTED and the LED double-blinks.
Single click: RECORD START and the LED goes solid. Another single click prints RECORD STOP and it goes back to the double-blink.
Double click: EVENT MARKER.
Hold again: DISCONNECTED, the LED goes off, and recording stops.
Single click while disconnected: nothing happens. 
*/

#include "switch.h"

#define BTN1_PIN       26   // PLACEHOLDER: change to your Button 1 pin
#define LED_STATUS_PIN 18   // PLACEHOLDER: change to your LED 1 pin (not 34-39)

#define TICK_20MSEC     20000UL
#define DEBOUNCE_CNT    5       // REQ 3
#define HOLD_MS         2000
#define DOUBLE_MS       400
#define TEST_CONNECT_MS 3000    // TEST ONLY: fake link comes up 3 s after searching starts

enum LinkState { LINK_OFF, LINK_SEARCHING, LINK_CONNECTED };
enum Command   { CMD_NONE, CMD_RECORD, CMD_MARKER, CMD_CONNECT };

LinkState link_state = LINK_OFF;   // later set by the comms code
Command command = CMD_NONE;        // later sent in the Req 18 packet, then cleared
bool recording = false;

Switch switch1(1, DEBOUNCE_CNT);
unsigned long lastTickTime = 0;
uint32_t ticks = 0;

// REQ 5, Button 1: single = record, double = event marker, hold = connect/disconnect
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

// TEST ONLY: stands in for the comms code. Delete when real comms exist.
void testApplyCommand() {
  static unsigned long searching_since = 0;
  if (command == CMD_RECORD && link_state == LINK_CONNECTED) {
    recording = !recording;
    Serial.println(recording ? "RECORD START" : "RECORD STOP");
  } else if (command == CMD_MARKER) {
    Serial.println("EVENT MARKER");
  } else if (command == CMD_CONNECT) {
    link_state = (link_state == LINK_OFF) ? LINK_SEARCHING : LINK_OFF;
    if (link_state == LINK_OFF) recording = false;   // disconnecting stops recording
    searching_since = millis();
    Serial.println(link_state == LINK_OFF ? "DISCONNECTED" : "SEARCHING");
  }
  command = CMD_NONE;

  if (link_state == LINK_SEARCHING && millis() - searching_since >= TEST_CONNECT_MS) {
    link_state = LINK_CONNECTED;
    Serial.println("CONNECTED");
  }
}

// REQ 5, LED 1: off = disconnected, slow blink = searching,
// double-blink = connected (not recording), solid = connected and recording
void ledStatusUpdate() {
  bool slow = (ticks / 25) % 2;                          // 500 ms on/off
  uint8_t p = ticks % 50;                                // 1 s cycle
  bool dbl = (p < 5) || (p >= 10 && p < 15);             // two quick flashes per second

  bool out = LOW;
  if (link_state == LINK_SEARCHING) out = slow;
  else if (link_state == LINK_CONNECTED) out = recording ? HIGH : dbl;
  digitalWrite(LED_STATUS_PIN, out);
}

void setup() {
  pinMode(BTN1_PIN, INPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);
  Serial.begin(115200);
  lastTickTime = micros();
}

void loop() {
  if ((unsigned long)(micros() - lastTickTime) < TICK_20MSEC) return;
  lastTickTime += TICK_20MSEC;
  ticks++;

  switch1.update(digitalRead(BTN1_PIN));   // REQ 1 + 3
  button1Update(millis());
  testApplyCommand();
  ledStatusUpdate();
}