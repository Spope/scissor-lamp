#include "DFRobot_GP8403.h"
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>

// MAC ADRESS : 
// e4:b3:23:a2:d0:74


// Adresse I2C (assume DIP = 0,0,0 → 0x58)
#define DAC_I2C_ADDR 0x58

// Create dac object
DFRobot_GP8403 dac(&Wire, DAC_I2C_ADDR);

// The LED power supply does not light up below this share of its input range.
// Anything under it is a dead zone, so 1..100% of user intensity is remapped
// onto [DAC_MIN_PERCENT .. 100] and only 0% stays fully off.
const float DAC_MIN_PERCENT = 8.0;
const int DAC_FULL_SCALE = 9999;

// Potentiometer input
const int POT_PIN = A0;
const int NUM_SAMPLES = 14;
const int TRIM_COUNT = 2;
const float SAMPLE_HZ = 5.0;

// When on remote mode, timer to check for onboard change
unsigned long previousMillis = 0;
const unsigned long checkChangeTimer = 500;  // ms

// Handover between the remote override and the onboard knob.
const int POT_TAKEOVER_THRESHOLD = 5;  // % knob move that grabs control back from the remote
const int RAMP_SNAP_HYSTERESIS = 2;    // % upward move that ends a descent ramp (ADC noise guard)

int lastAppliedPercent = 0;  // what is currently on the DAC
int lastPotPercent = 0;      // last pot reading taken in ONBOARD mode, the takeover baseline

// Descent ramp: the knob was turned down while the remote held control, so the output resumes
// from the overridden value and is scaled down to 0 across the knob's remaining travel.
bool rampActive = false;
int rampStartPot = 0;      // knob position when the remote took over
int rampStartPercent = 0;  // output the remote had imposed
int rampLowestPot = 0;     // lowest knob reading seen since the ramp started

typedef enum Modes {
  ONBOARD = 1,
  REMOTE = 2
};

const bool debug = true;
Modes mode = Modes::ONBOARD;

void setup() {
  if (debug) {
    Serial.begin(115200);
    delay(1000);
    conditionnalPrint("Init in mode " + String(modeToString(mode)));
  }

  pinMode(POT_PIN, INPUT);

  initWifi();

  initDAC();
}

void loop() {
  
  switch (mode) {
    case Modes::ONBOARD:
    setFromOnboardPotentiometer();
    break;
    case Modes::REMOTE:
    setFromRemote();
    checkForSignificantOnbardChange();
    break;
  }
  
}

void initDAC() {
  conditionnalPrint("Start DAC init …");
  // Initialising DAC module
  Wire.begin(6, 7);
  while (dac.begin() != 0) {
    conditionnalPrint("DAC init error, retrying …");
    delay(1000);
  }
  // Choose 10V Output
  dac.setDACOutRange(dac.eOutputRange10V);
  conditionnalPrint("DAC init succeed");
  // Set Channel O to 0V
  dac.setDACOutVoltage(0, 0);
  lastAppliedPercent = 0;

  // Saving value for reboot
  dac.store();
  conditionnalPrint("DAC initialized.");
}

void setMode(Modes newMode) {
  if (newMode == Modes::REMOTE) {
    rampActive = false;
  }
  mode = newMode;
}

int remotePercentage = 0;
void saveFromRemote(int newRemotePercentage) {
  remotePercentage = newRemotePercentage;
  if (mode == Modes::REMOTE) {
    setFromRemote();
  } 
}

void setFromRemote() {
  applyIntensity(remotePercentage);
}

////////////
// WIFI
////////////
void initWifi() {
  conditionnalPrint("Start WiFi init …");
  // Start wifi in station mode
  WiFi.mode(WIFI_STA);

  // Booting ESP NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialisation error.");
    return;
  }
  // Registering received message callback
  esp_now_register_recv_cb((esp_now_recv_cb_t)messageFromWifi);
  conditionnalPrint("WiFi initialized.");
}

enum {
    CMD_MODE = 1,
    CMD_INTENSITY = 2,
    CMD_POWER = 3
};

typedef struct __attribute__((packed)) {
    uint8_t verb;     
    int16_t value;
} espnow_msg_t;

void messageFromWifi(const uint8_t * mac, const uint8_t *data, int len) {

  espnow_msg_t msg;
  memcpy(&msg, data, sizeof(msg));

  switch (msg.verb) {
      case CMD_MODE:
          Modes newMode;
          if (parseMode(msg.value, &newMode)) {
            conditionnalPrint("Mode = " + (String) msg.value);
            setMode(newMode);
          } else {
              conditionnalPrint("Invalid mode received");
          }
          break;
      case CMD_INTENSITY:
          if ((int) msg.value >= 0 && (int) msg.value <= 100) {
            conditionnalPrint("Intensity = " + (String) msg.value);
            saveFromRemote((int) msg.value);
          } else {
              conditionnalPrint("Invalid intensity received");
          }
          break;
      case CMD_POWER:
          conditionnalPrint("Power " + (String) msg.value);
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
  // map() overshoots 100 above 3500 counts, so clamp.
  return constrain(map(readFilteredPot(), 0, 3500, 0, 100), 0, 100);
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
  delay( (int)(1000.0 / SAMPLE_HZ) );
}

void checkForSignificantOnbardChange() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis < checkChangeTimer) {
    return;
  }
  previousMillis = currentMillis;

  int potNow = readPotPercent();
  if (abs(potNow - lastPotPercent) <= POT_TAKEOVER_THRESHOLD) {
    return;
  }

  // The knob moved, so it takes control back. Turned down, the output picks up where the
  // remote left off and slides to 0 over the knob's remaining travel; turned up, the raw
  // knob value applies straight away.
  if (potNow < lastPotPercent && lastPotPercent > 0) {
    conditionnalPrint("Onboard takeover, ramping down from " + String(lastAppliedPercent) + "%");
    rampActive = true;
    rampStartPot = lastPotPercent;
    rampStartPercent = lastAppliedPercent;
    rampLowestPot = potNow;
  } else {
    conditionnalPrint("Onboard takeover, following the knob");
    rampActive = false;
  }
  setMode(ONBOARD);
}




//////////
// Tools
//////////

// Single point of contact with the DAC, shared by the remote and onboard paths.
void applyIntensity(int percent) {
  if (percent == lastAppliedPercent) {
    return;
  }
  dac.setDACOutVoltage(percentToDac(percent), 0);
  lastAppliedPercent = percent;
  conditionnalPrint("Output = " + String(percent) + "%");
}

// Map a 0..100% intensity onto the usable output range of the power supply.
uint16_t percentToDac(int percent) {
  if (percent <= 0) {
    return 0;
  }
  if (percent > 100) {
    percent = 100;
  }
  float compressed = DAC_MIN_PERCENT + (100.0 - DAC_MIN_PERCENT) * percent / 100.0;
  return (uint16_t)(compressed * DAC_FULL_SCALE / 100.0 + 0.5);
}

void conditionnalPrint(String text)
{
  if (debug) {
    Serial.println(text);
  }
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
            return false; // invalid value
    }
}

/*
const char* commandToString(Modes mode) {
  switch (mode) {
    case CMD_MODE:      return "CMD_MODE";
    case CMD_INTENSITY: return "CMD_INTENSITY";
    case CMD_POWER:     return "CMD_POWER";
    default:            return "UNKNOWN";
  }
}
*/