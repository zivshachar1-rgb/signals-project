// Pulse monitor - ESP32 Bluetooth SPP transmitter
// Adapted from the Lab 5 Bluetooth sketch.
//
// Packet sent once per second (ASCII line):
//     seq,bpm,s0,s1,...,s49\n
//   seq  : packet sequence number (increments every packet)
//   bpm  : latest pulse rate calculated here (0 = no valid reading)
//   s0..s49 : 50 raw ADC integers (0-4095), sampled every 20 ms

#include "BluetoothSerial.h"
#include <Esp.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

String btName = "ESP32AK";                 // NEW_RQ15
BluetoothSerial SerialBT;

#define PPG_PIN 34                         // ADC1 pin (works with Bluetooth on)

const unsigned long SAMPLE_PERIOD_MS = 20;     // 50 samples/s
const int SAMPLES_PER_PACKET = 50;             // 1 s of data
const unsigned long MIN_BEAT_GAP_MS = 300;     // ignore double triggers
const unsigned long NO_BEAT_MS = 3000;         // no beat this long -> bpm = 0

int samples[SAMPLES_PER_PACKET];               // raw readings since last packet
int sampleCount = 0;
uint32_t seqNum = 0;                           // NEW_RQ15: sequence number
unsigned long lastSample = 0;

// --- pulse detection state ---
float sigMax = 0.5, sigMin = 0.5;
bool wasAbove = false;
unsigned long lastBeat = 0;
unsigned long beatGaps[4] = {0, 0, 0, 0};
int gapIdx = 0, gapCount = 0;
int currentBpm = 0;                            // latest calculation

void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT)  Serial.println("Client connected");
  if (event == ESP_SPP_CLOSE_EVT)     Serial.println("Client disconnected");
}

void setup() {
  Serial.begin(115200);
  SerialBT.register_callback(btCallback);
  // NEW_RQ15: begin() called once only
  if (!SerialBT.begin(btName)) Serial.println("Bluetooth init failed");
  else Serial.println("Bluetooth ready: " + btName);
  lastSample = millis();
}

// Updates beat detection with one sample (normalised 0..1), returns BPM
int processSample(float v) {
  if (v > sigMax) sigMax = v; else sigMax -= (sigMax - v) * 0.005f;
  if (v < sigMin) sigMin = v; else sigMin += (v - sigMin) * 0.005f;

  float range = sigMax - sigMin;
  float thr = sigMin + 0.6f * range;
  unsigned long now = millis();

  bool above = (v > thr) && (range > 0.05f);   // tiny range = no finger
  if (above && !wasAbove && (now - lastBeat) > MIN_BEAT_GAP_MS) {
    if (lastBeat != 0 && (now - lastBeat) < 2000) {
      beatGaps[gapIdx] = now - lastBeat;
      gapIdx = (gapIdx + 1) % 4;
      if (gapCount < 4) gapCount++;
    }
    lastBeat = now;
  }
  wasAbove = above;

  if (gapCount == 0 || (now - lastBeat) > NO_BEAT_MS) {
    gapCount = 0;
    return 0;
  }
  unsigned long sum = 0;
  for (int i = 0; i < gapCount; i++) sum += beatGaps[i];
  return (int)(60000.0f / (sum / (float)gapCount));
}

// NEW_RQ15: build and send one packet "seq,bpm,s0..s49\n"
void sendPacket() {
  char line[360];                              // 50 x 5 chars + header, with margin
  int len = snprintf(line, sizeof(line), "%lu,%d", (unsigned long)seqNum, currentBpm);
  for (int i = 0; i < SAMPLES_PER_PACKET; i++) {
    len += snprintf(line + len, sizeof(line) - len, ",%d", samples[i]);
  }
  len += snprintf(line + len, sizeof(line) - len, "\n");

  if (SerialBT.hasClient()) SerialBT.write((const uint8_t *)line, len);
  Serial.write((const uint8_t *)line, len);    // USB echo for debugging
  seqNum++;                                    // increments even if no client, so gaps are visible
}

void loop() {
  // Non-blocking 20 ms sampling (no delay())
  if (millis() - lastSample >= SAMPLE_PERIOD_MS) {
    lastSample += SAMPLE_PERIOD_MS;

    int raw = analogRead(PPG_PIN);             // 0..4095 integer
    samples[sampleCount++] = raw;
    currentBpm = processSample(raw / 4095.0f);

    // 50 samples collected = 1 s -> send packet
    if (sampleCount >= SAMPLES_PER_PACKET) {
      sendPacket();
      sampleCount = 0;
    }
  }
}
