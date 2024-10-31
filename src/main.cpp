#include <Arduino.h>

#include "ble.h"
#include <WiFi.h>
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
#include "DHT.h"
#include "NoiseSensor/NoiseSensor.h"

bool deviceConnected = false;

Preferences preferences;

uint32_t value = 0;

Ble ble = Ble(preferences);

NoiseSensor noiseSensor;

const char* TZ_INFO    = "BRST+3BRDT+2,M10.3.0,M2.3.0";  
#define DHTPIN 4    
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

String ssid;
String password;
long deviceId;
int noiseThreshold;
boolean active = false;

#define MQTT_HOST "200.239.66.45" 
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "01_device_01" // cada dispositivo deve ter um id diferente 
#define MQTT_TOPIC_ROOT "ESP32PhcnTeste" 
#define MQTT_MESSAGE_LEN 128
#define SENSORS_TOPIC MQTT_TOPIC_ROOT "/sensors"
#define NOISE_TOPIC MQTT_TOPIC_ROOT "/noises"

DHT dht(DHTPIN, DHTTYPE);

StaticJsonDocument<MQTT_MESSAGE_LEN> doc;
StaticJsonDocument<64> docNoise;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
TimerHandle_t sensorsTimer;
TimerHandle_t noiseTimer;

bool setWifiCredentials(){

  preferences.begin("credentials", false);
  String s = preferences.getString("ssid", "");
  String p = preferences.getString("password", "");
  preferences.end();

  if(s == "" || p == "") return false;

  ssid = s;
  password = p;

  return true;
}

void setDeviceId(){
  preferences.begin("info", false);
  deviceId = preferences.getLong("deviceId", 0);
  preferences.end();
}

void setNoiseThreshold(){
  preferences.begin("info", false);
  noiseThreshold = preferences.getInt("noiseThreshold", 2048); // default = 4096/2
  preferences.end();
}

void connectToWifi() {
  Serial.println("Conectando ao Wi-Fi...");
  preferences.begin("credentials", false);
  String ssid = preferences.getString("ssid", "");
  String password = preferences.getString("password", "");
  preferences.end();

  Serial.println(ssid);
  Serial.println(password);
  WiFi.disconnect();
  vTaskDelay(1000);
  WiFi.begin(ssid.c_str(), password.c_str());
  WiFi.setSleep(false);
  vTaskDelay(3000);
}

void connectToMqtt() {
  Serial.println("Conectando ao servidor MQTT...");
  mqttClient.connect();
}

void activeReadSensors(){

  Serial.println("Iniciando leitura dos sensores");
  active = true;
  xTimerStart(noiseTimer, 0);
  xTimerStop(sensorsTimer, 0);

}

void deactiveReadSensors(){

  Serial.println("Interrompendo leitura dos sensores");
  active = false;
  xTimerStop(noiseTimer, 0); // desativa as leituras caso o Wi-Fi esteja desconectado

}


void noiseMonitoring(){

    uint32_t noise = noiseSensor.read();
    setNoiseThreshold();

    if(noise > noiseThreshold){

      setDeviceId();
      //char output[35];
      //sprintf(output, "{\"deviceId\":%d,\"timestamp\":%u}", deviceId, time(nullptr));
      docNoise["deviceId"] = deviceId;
      docNoise["timestamp"] = time(nullptr);

      char output[64];
      serializeJson(docNoise, output);
      Serial.print("noise up! ");
      Serial.println(output);
      mqttClient.publish(NOISE_TOPIC, 0, true, output);
      xTimerStart(noiseTimer, 0);

    }

    xTimerStart(noiseTimer, 0);
}

void publishWeatherRead(){

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
  } else {
    doc["deviceId"] = deviceId;
    doc["humidity"] = round(h);
    doc["temperature"] = round(t);
    doc["timestamp"] = time(nullptr);

    char output[MQTT_MESSAGE_LEN];
    serializeJson(doc, output);
    Serial.println(output);
    mqttClient.publish(SENSORS_TOPIC, 0, true, output);
  }


}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();

        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        WiFi.disconnect(true);
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
        active = false; // desativa as leituras caso o Wi-Fi esteja desconectado
        xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe("test/lol", 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
  mqttClient.publish("test/lol", 0, true, "test 1");
  Serial.println("Publishing at QoS 0");
  uint16_t packetIdPub1 = mqttClient.publish("test/lol", 1, true, "test 2");
  Serial.print("Publishing at QoS 1, packetId: ");
  Serial.println(packetIdPub1);
  uint16_t packetIdPub2 = mqttClient.publish("test/lol", 2, true, "test 3");
  Serial.print("Publishing at QoS 2, packetId: ");
  Serial.println(packetIdPub2);
  mqttClient.subscribe("phtest/lol", 2);

  xTimerStart(sensorsTimer, 0);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  Serial.println("Motivo:");
  deactiveReadSensors();

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {

  Serial.begin(115200);
  Serial.println();
  Serial.println();

  configTime (0, 0, "pool.ntp.org", "time.nist.gov");
  setenv("TZ", TZ_INFO, 1);

  dht.begin();
  noiseSensor.begin();
     // delete old config
  WiFi.disconnect(true);
  delay(2000);

  WiFi.onEvent(WiFiEvent);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(3000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  sensorsTimer = xTimerCreate("sensorsTimer", pdMS_TO_TICKS(10000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(activeReadSensors));
  noiseTimer = xTimerCreate("noiseTimer", pdMS_TO_TICKS(1000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(noiseMonitoring));



  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setClientId(MQTT_CLIENT_ID);

  setDeviceId();
  setWifiCredentials();
  connectToWifi(); 
  
  ble.start();

  delay(1000);

}

void loop() {
  if(active){
    publishWeatherRead();
    vTaskDelay(600000); // 10 * 1min
  }
}
