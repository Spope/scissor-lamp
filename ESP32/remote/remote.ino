// Remote control -- Seeed XIAO ESP32-C6.
//
// Arduino IDE:  Board = "XIAO_ESP32C6" (USB CDC On Boot defaults to Enabled -- leave it).
// arduino-cli:  --fqbn esp32:esp32:XIAO_ESP32C6
//
// Référence technique: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html

// Inclure les librairies
#include <esp_now.h>
#include <WiFi.h>


//////////
// Config
//////////

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
const int POT_SAMPLE_PERIOD_MS = 200;  // how often the knob is sampled


/////////
// Types
/////////

// NOTE: Modes, the CMD_* verbs, espnow_msg_t, readFilteredPot() and readPotPercent() are
// duplicated verbatim in lamp/lamp.ino. Sharing them would need a library under
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

int lastSentPercent = 0;  // last percentage pushed to the lamp


void setup() {
  Serial.begin(115200);

  // Init led
  pinMode(LED_BUILTIN, OUTPUT);

  // Init pot
  pinMode(POT_PIN, INPUT);

  if (!initWifi()) {
    Serial.println("WiFi init failed, the lamp will not be switched to remote mode.");
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
  delay(POT_SAMPLE_PERIOD_MS);
}

//////////////
// WIFI
//////////////
bool initWifi() {
  // Start wifi in station mode
  WiFi.mode(WIFI_STA);

  // Booting ESP NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialisation error.");
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
    Serial.println("Pairing failed");
    return false;
  }

  return true;
}

// La fonction de rappel qui nous assurera de la bonne livraison du message
void messageSentCallback(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Last message sended status : Success");
  } else {
    Serial.println("Last message sended status : Failure");
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

int readPotPercent()
{
  // map() overshoots 100 above POT_MAX_COUNTS, so clamp. The lamp rejects anything over 100,
  // so without this clamp the top of the knob's travel did nothing.
  return constrain(map(readFilteredPot(), 0, POT_MAX_COUNTS, 0, 100), 0, 100);
}

// The trimmed median above is the whole noise filter: quantising to a percentage is coarser
// than the filter's residual jitter (one percent is ~33 ADC counts here), so a raw-counts
// deadband on top of it bought nothing. Same approach as the lamp's applyIntensity().
void setFromRemotePotentiometer() {
  int potPercent = readPotPercent();
  if (potPercent == lastSentPercent) {
    return;
  }
  Serial.println("Pot percentage is: " + String(potPercent) + "%");
  sendIntensity(potPercent);
  lastSentPercent = potPercent;
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
    Serial.println("Message sent.");
  } else {
    Serial.println("Message failed");
  }
}

/////////////
// Tools
/////////////
void blink() {
  digitalWrite(LED_BUILTIN, LOW);
  delay(50);
  digitalWrite(LED_BUILTIN, HIGH);
}
