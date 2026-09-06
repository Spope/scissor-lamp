// Référence technique: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html

// Inclure les librairies
#include <esp_now.h>
#include <WiFi.h>

const bool debug = true;

// Lamp MAC Adress
uint8_t lampMacAdress[] = {0xe4, 0xb3, 0x23, 0xa2, 0xd0, 0x74};

// La fonction de rappel qui nous assurera de la bonne livraison du message
void messageSentCallback(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    conditionnalPrint("Last message sended status : Success");
  } else {
    conditionnalPrint("Last message sended status : Failure");
  }
}

// Une variable qui servira à stocker les réglages concernant le récepteur
esp_now_peer_info_t lampInfos;

// Potentiometer input
const int POT_PIN = A0;
const int NUM_SAMPLES = 14;
const int TRIM_COUNT = 2;
const float SAMPLE_HZ = 5.0;
int oldValue;
byte potPercentage;
byte oldPercentage;

typedef enum Modes {
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

void setup() {
  Serial.begin(115200);
 
  initWifi();

  // Init led
  pinMode(LED_BUILTIN,OUTPUT);

  // Init pot
  pinMode(POT_PIN, INPUT);

  espnow_msg_t msg;
  msg.verb = CMD_MODE;
  msg.value = Modes::REMOTE;
  // Send the message
  esp_now_send(lampMacAdress, (uint8_t*)&msg, sizeof(msg));
  blink();
}
 
void loop() {
  setFromRemotePotentiometer();
}

//////////////
// WIFI
//////////////
void initWifi() {
  // Start wifi in station mode
  WiFi.mode(WIFI_STA);

  // Booting ESP NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialisation error.");
    return;
  }

  // Registering send message callback
  esp_now_register_send_cb((esp_now_send_cb_t)messageSentCallback);
  
  // Pairing configuration with lamp
  memcpy(lampInfos.peer_addr, lampMacAdress, 6);
  
  // default chanel
  lampInfos.channel = 0;  

  // No encryption
  lampInfos.encrypt = false;
  
  // Pairing        
  if (esp_now_add_peer(&lampInfos) != ESP_OK){
    Serial.println("Pairing failed");
    return;
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
  if (filtered < (oldValue - 20) || filtered > (oldValue + 20)) {
    oldValue = filtered;
    //conditionnalPrint(String(filtered));

    // convert to percentage
    potPercentage = map(oldValue, 0, 3310, 0, 100);

    if (oldPercentage != potPercentage) {
      conditionnalPrint("Pot percentage is: " + String(potPercentage) + "%");
      sendIntensity(potPercentage);
      oldPercentage = potPercentage;
    }
  }
  delay( (int)(1000.0 / SAMPLE_HZ) );
}

void sendIntensity(int percentage)
{
  espnow_msg_t msg2;
  msg2.verb = CMD_INTENSITY;
  msg2.value = percentage;
  // Send the message
  int result = esp_now_send(lampMacAdress, (uint8_t*)&msg2, sizeof(msg2));
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
void conditionnalPrint(String text)
{
  if (debug) {
    Serial.println(text);
  }
}

void blink() {
  digitalWrite(LED_BUILTIN,LOW);
  delay(50);
  digitalWrite(LED_BUILTIN,HIGH);
}