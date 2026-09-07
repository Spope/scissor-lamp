// Référence technique: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html

// Inclure les librairies
#include <esp_now.h>
#include <WiFi.h>


//////////
// Config
//////////

const bool debug = true;

// Lamp MAC ADDRESS
uint8_t lampMacAdress[] = {0xe4, 0xb3, 0x23, 0xa2, 0xd0, 0x74};

// Potentiometer input
const int POT_PIN = A0;
const int NUM_SAMPLES = 14;
const int TRIM_COUNT = 2;
// ADC counts at this knob's top stop. Deliberately different from the lamp's own
// POT_MAX_COUNTS: these are two different physical potentiometers, so each value is a
// calibration rather than a shared constant.
const int POT_MAX_COUNTS = 3310;
const int POT_DEADBAND_COUNTS = 20;  // ignore knob jitter smaller than this
const int SAMPLE_PERIOD_MS = 200;    // how often the knob is sampled


/////////
// Types
/////////

// NOTE: Modes, the CMD_* verbs, espnow_msg_t, readFilteredPot() and conditionalPrint() are
// duplicated verbatim in soft/soft.ino. Sharing them would need a library under
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

// Une variable qui servira à stocker les réglages concernant le récepteur
esp_now_peer_info_t lampInfos;

int oldValue = 0;
byte potPercentage = 0;
byte oldPercentage = 0;


void setup() {
  // Always open the port; only the printing is gated on `debug`.
  Serial.begin(115200);

  // Init led
  pinMode(LED_BUILTIN, OUTPUT);

  // Init pot
  pinMode(POT_PIN, INPUT);

  if (!initWifi()) {
    conditionalPrint("WiFi init failed, the lamp will not be switched to remote mode.");
    return;
  }

  espnow_msg_t msg;
  msg.verb = CMD_MODE;
  msg.value = Modes::REMOTE;
  // Send the message
  esp_now_send(lampMacAdress, (uint8_t*)&msg, sizeof(msg));
  blink();
}

void loop() {
  setFromRemotePotentiometer();
  delay(SAMPLE_PERIOD_MS);
}

//////////////
// WIFI
//////////////
bool initWifi() {
  // Start wifi in station mode
  WiFi.mode(WIFI_STA);

  // Booting ESP NOW
  if (esp_now_init() != ESP_OK) {
    conditionalPrint("ESP-NOW initialisation error.");
    return false;
  }

  // Registering send message callback. The signature below is the one ESP32 core 3.x/4.x
  // expects, so no cast is needed.
  esp_now_register_send_cb(messageSentCallback);

  // Pairing configuration with lamp
  memcpy(lampInfos.peer_addr, lampMacAdress, 6);

  // default chanel
  lampInfos.channel = 0;

  // No encryption
  lampInfos.encrypt = false;

  // Pairing
  if (esp_now_add_peer(&lampInfos) != ESP_OK) {
    conditionalPrint("Pairing failed");
    return false;
  }

  return true;
}

// La fonction de rappel qui nous assurera de la bonne livraison du message
void messageSentCallback(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    conditionalPrint("Last message sended status : Success");
  } else {
    conditionalPrint("Last message sended status : Failure");
  }
}

////////////////////
// POT
////////////////////
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

void setFromRemotePotentiometer() {
  int filtered = readFilteredPot();

  // add some deadband
  if (filtered < (oldValue - POT_DEADBAND_COUNTS) || filtered > (oldValue + POT_DEADBAND_COUNTS)) {
    oldValue = filtered;

    // Convert to percentage. map() overshoots 100 above POT_MAX_COUNTS -- the lamp rejects
    // anything over 100, so without this clamp the top of the knob's travel did nothing.
    potPercentage = constrain(map(oldValue, 0, POT_MAX_COUNTS, 0, 100), 0, 100);

    if (oldPercentage != potPercentage) {
      conditionalPrint("Pot percentage is: " + String(potPercentage) + "%");
      sendIntensity(potPercentage);
      oldPercentage = potPercentage;
    }
  }
}

void sendIntensity(int percentage)
{
  espnow_msg_t msg;
  msg.verb = CMD_INTENSITY;
  msg.value = percentage;
  // Send the message
  esp_err_t result = esp_now_send(lampMacAdress, (uint8_t*)&msg, sizeof(msg));
  blink();

  if (result == ESP_OK) {
    conditionalPrint("Message sent.");
  } else {
    conditionalPrint("Message failed");
  }
}

/////////////
// Tools
/////////////
void conditionalPrint(String text)
{
  if (debug) {
    Serial.println(text);
  }
}

void blink() {
  digitalWrite(LED_BUILTIN, LOW);
  delay(50);
  digitalWrite(LED_BUILTIN, HIGH);
}
