// Lamp controller -- Waveshare ESP32-C6-Pico.
//
// Arduino IDE:  Board = "ESP32C6 Dev Module", USB CDC On Boot = Enabled.
// arduino-cli:  --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc
//
// USB CDC On Boot MUST be enabled. That board defaults it to Disabled, which binds
// Serial to the UART0 GPIO pins instead of USB: the lamp runs perfectly but prints
// nothing over USB, while the bootloader still does -- an easy hour to lose.
//
// Own MAC ADDRESS: e4:b3:23:a2:d0:74

#include "DFRobot_GP8403.h"
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>


//////////
// Config
//////////

// I2C address of the DAC (assume DIP = 0,0,0 -> 0x58)
#define DAC_I2C_ADDR 0x58
const int DAC_SDA_PIN = 6;
const int DAC_SCL_PIN = 7;

// Floor of the usable output window, in millivolts on the DAC's 0-10V range:
// 800 mV = 8% of the power supply's input range. The LED power supply does not
// light up below that share of its range, so anything under it is a dead zone.
// 1..100% of user intensity is therefore remapped onto [DAC_MIN_MV .. DAC_MAX_MV],
// and only 0% stays fully off.
const int DAC_MIN_MV = 800;
const int DAC_MAX_MV = 9999;  // top of the range; the DAC takes millivolts, not counts

// Potentiometer input
const int POT_PIN = A0;
const int NUM_SAMPLES = 14;
const int TRIM_COUNT = 2;
const int POT_MAX_COUNTS = 3500;  // ADC counts at this knob's top stop, calibrated on the lamp

// Loop pacing
const int POT_SAMPLE_PERIOD_MS = 200;             // ONBOARD: how often the knob is sampled
const int REMOTE_POLL_PERIOD_MS = 10;             // REMOTE: low latency without spinning the core
const unsigned long POT_CHECK_INTERVAL_MS = 500;  // REMOTE: how often we look for an onboard takeover

// Handover between the remote override and the onboard knob.
const int POT_TAKEOVER_THRESHOLD = 5;  // % knob move that grabs control back from the remote
const int RAMP_SNAP_HYSTERESIS = 2;    // % upward move that ends a descent ramp (ADC noise guard)


/////////
// Types
/////////

// NOTE: Modes, the CMD_* verbs, espnow_msg_t, readFilteredPot() and readPotPercent() are
// duplicated verbatim in remote/remote.ino. Sharing them would need a library under
// soft/libraries/, which is out of scope here.
enum Modes {
  ONBOARD = 1,
  REMOTE = 2
};

enum {
  CMD_MODE = 1,
  CMD_INTENSITY = 2,
  CMD_POWER = 3
};

typedef struct __attribute__((packed)) {
  uint8_t verb;
  int16_t value;
} espnow_msg_t;


/////////
// State
/////////

// Create dac object
DFRobot_GP8403 dac(&Wire, DAC_I2C_ADDR);

Modes mode = Modes::ONBOARD;

int lastAppliedPercent = 0;  // what is currently on the DAC
int lastPotPercent = 0;      // last pot reading taken in ONBOARD mode, the takeover baseline

// Written by the ESP-NOW receive callback (WiFi task), pushed to the DAC by loop().
volatile int remotePercentage = 0;

// REMOTE: when the onboard knob was last polled for a takeover.
unsigned long lastPotCheckMillis = 0;

// Descent ramp: the knob was turned down while the remote held control, so the output resumes
// from the overridden value and is scaled down to 0 across the knob's remaining travel.
bool rampActive = false;
int rampStartPot = 0;      // knob position when the remote took over
int rampStartPercent = 0;  // output the remote had imposed
int rampLowestPot = 0;     // lowest knob reading seen since the ramp started


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Init in mode " + String(modeToString(mode)));

  pinMode(POT_PIN, INPUT);

  initWifi();

  initDAC();
}

void loop() {
  switch (mode) {
    case Modes::ONBOARD:
      setFromOnboardPotentiometer();
      delay(POT_SAMPLE_PERIOD_MS);
      break;
    case Modes::REMOTE:
      applyIntensity(remotePercentage);
      checkForSignificantOnboardChange();
      delay(REMOTE_POLL_PERIOD_MS);
      break;
  }
}

///////
// DAC
///////
void initDAC() {
  Serial.println("Start DAC init …");
  // Initialising DAC module
  Wire.begin(DAC_SDA_PIN, DAC_SCL_PIN);
  while (dac.begin() != 0) {
    Serial.println("DAC init error, retrying …");
    delay(1000);
  }
  // Choose 10V Output
  dac.setDACOutRange(dac.eOutputRange10V);
  Serial.println("DAC init succeeded");
  // Set Channel O to 0V
  dac.setDACOutVoltage(0, 0);

  // dac.store() is deliberately NOT called here. The library bit-bangs the EEPROM store
  // sequence on its own _sda/_scl, which default to the board's SDA/SCL macros (GPIO 22/23
  // on the XIAO ESP32-C6) rather than the pins used above, so it never reached the chip --
  // and to do it, it destroyed and rebuilt the Wire object. The DAC's power-on level is
  // therefore whatever is already in its EEPROM.

  Serial.println("DAC initialized.");
}

////////////
// WIFI
////////////
void initWifi() {
  Serial.println("Start WiFi init …");
  // Start wifi in station mode
  WiFi.mode(WIFI_STA);

  // Booting ESP NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialisation error.");
    return;
  }
  // Registering received message callback. The signature below is the one ESP32 core 3.x/4.x
  // expects, so no cast is needed -- and a future signature change will fail to compile
  // instead of silently passing the wrong pointer.
  esp_now_register_recv_cb(messageFromWifi);
  Serial.println("WiFi initialized.");
}

void messageFromWifi(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != (int)sizeof(espnow_msg_t)) {
    Serial.println("Ignored packet of unexpected size " + String(len));
    return;
  }

  espnow_msg_t msg;
  memcpy(&msg, data, sizeof(msg));

  switch (msg.verb) {
    case CMD_MODE: {
      Modes newMode;
      if (parseMode(msg.value, &newMode)) {
        Serial.println("Mode = " + String(msg.value));
        setMode(newMode);
      } else {
        Serial.println("Invalid mode received");
      }
      break;
    }
    case CMD_INTENSITY:
      if (msg.value >= 0 && msg.value <= 100) {
        Serial.println("Intensity = " + String(msg.value));
        // Only store it: loop() owns the DAC, so no I2C from the WiFi task.
        remotePercentage = msg.value;
      } else {
        Serial.println("Invalid intensity received");
      }
      break;
    case CMD_POWER:
      // Not implemented yet; kept so the verb stays in sync with remote/remote.ino.
      Serial.println("Power " + String(msg.value));
      break;
    default:
      Serial.println("Unknown command " + String(msg.verb));
      break;
  }
}


//////////////
// ONBOARD
//////////////
int readFilteredPot()
{
  int samples[NUM_SAMPLES];

  // Collect samples
  for (int i = 0; i < NUM_SAMPLES; i++) {
    //value from 0 to 4095
    samples[i] = analogRead(POT_PIN);
    delayMicroseconds(10);   // Small delay helps ADC stability
  }

  // Simple bubble sort (NUM_SAMPLES is small)
  for (int i = 0; i < NUM_SAMPLES - 1; i++) {
    for (int j = i + 1; j < NUM_SAMPLES; j++) {
      if (samples[j] < samples[i]) {
        int temp = samples[i];
        samples[i] = samples[j];
        samples[j] = temp;
      }
    }
  }

  // Average the middle values (exclude TRIM_COUNT extrema)
  long sum = 0;
  for (int i = TRIM_COUNT; i < NUM_SAMPLES - TRIM_COUNT; i++) {
    sum += samples[i];
  }

  return sum / (NUM_SAMPLES - 2 * TRIM_COUNT);
}

int readPotPercent()
{
  // map() overshoots 100 above POT_MAX_COUNTS, so clamp.
  return constrain(map(readFilteredPot(), 0, POT_MAX_COUNTS, 0, 100), 0, 100);
}

void setFromOnboardPotentiometer() {
  int potPercent = readPotPercent();
  lastPotPercent = potPercent;

  int target = potPercent;
  if (rampActive) {
    if (potPercent < rampLowestPot) {
      rampLowestPot = potPercent;
    }
    if (potPercent > rampLowestPot + RAMP_SNAP_HYSTERESIS || potPercent == 0) {
      rampActive = false;  // knob went back up, or reached 0: raw value from now on
    } else {
      target = (rampStartPercent * potPercent + rampStartPot / 2) / rampStartPot;
    }
  }

  applyIntensity(target);
}

void checkForSignificantOnboardChange() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastPotCheckMillis < POT_CHECK_INTERVAL_MS) {
    return;
  }
  lastPotCheckMillis = currentMillis;

  int potNow = readPotPercent();
  if (abs(potNow - lastPotPercent) <= POT_TAKEOVER_THRESHOLD) {
    return;
  }

  // The knob moved, so it takes control back. Turned down, the output picks up where the
  // remote left off and slides to 0 over the knob's remaining travel; turned up, the raw
  // knob value applies straight away. Getting here with potNow < lastPotPercent implies
  // lastPotPercent > POT_TAKEOVER_THRESHOLD, so rampStartPot is never 0 below.
  if (potNow < lastPotPercent) {
    Serial.println("Onboard takeover, ramping down from " + String(lastAppliedPercent) + "%");
    rampActive = true;
    rampStartPot = lastPotPercent;
    rampStartPercent = lastAppliedPercent;
    rampLowestPot = potNow;
  } else {
    Serial.println("Onboard takeover, following the knob");
    rampActive = false;
  }
  setMode(ONBOARD);
}


//////////
// Tools
//////////

void setMode(Modes newMode) {
  if (newMode == Modes::REMOTE) {
    rampActive = false;
  }
  mode = newMode;
}

// Single point of contact with the DAC, shared by the remote and onboard paths.
void applyIntensity(int percent) {
  if (percent == lastAppliedPercent) {
    return;
  }
  dac.setDACOutVoltage(percentToMillivolts(percent), 0);
  lastAppliedPercent = percent;
  Serial.println("Output = " + String(percent) + "%");
}

// Map a 0..100% intensity onto the usable output range of the power supply.
uint16_t percentToMillivolts(int percent) {
  if (percent <= 0) {
    return 0;
  }
  if (percent > 100) {
    percent = 100;
  }
  return DAC_MIN_MV + (uint32_t)(DAC_MAX_MV - DAC_MIN_MV) * percent / 100;
}

const char* modeToString(Modes mode) {
  switch (mode) {
    case ONBOARD: return "ONBOARD";
    case REMOTE:  return "REMOTE";
    default:      return "UNKNOWN";
  }
}

bool parseMode(int16_t value, Modes *outMode) {
  switch (value) {
    case ONBOARD:
    case REMOTE:
      *outMode = (Modes)value;
      return true;
    default:
      return false;  // invalid value
  }
}
