#include <esp_now.h>
#include <WiFi.h>

float receivedValue;

void receiveCallback(const uint8_t * mac, const uint8_t *data_reception, int taille) {
  memcpy(&receivedValue, data_reception, sizeof(receivedValue));
  Serial.print("Bytes received : ");
  Serial.println(taille);
  Serial.print("Value received : ");
  Serial.println(receivedValue);
  Serial.println();

  blink();
}

void blink() {
  digitalWrite(LED_BUILTIN,LOW);
  delay(50);
  digitalWrite(LED_BUILTIN,HIGH);
}
 
void setup() {
  Serial.begin(115200);
 
  // Start wifi in station mode
  WiFi.mode(WIFI_STA);

  // Booting ESP NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialisation error.");
    return;
  }

  // Registering received message callback
  esp_now_register_recv_cb((esp_now_recv_cb_t)receiveCallback);
  
  // Init led
  pinMode(LED_BUILTIN,OUTPUT);
}
 
void loop() {
  
}