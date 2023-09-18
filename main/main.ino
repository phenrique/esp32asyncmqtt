#include "ble.h"
#include <WiFi.h>
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
#include "DHT.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>

bool deviceConnected = false;

Preferences preferences;

#define DEVICE_NAME "LAAI02ESP"

uint32_t value = 0;

Ble ble = Ble(DEVICE_NAME, preferences);

esp_adc_cal_characteristics_t adc_cal; //Estrutura que contem as informacoes para calibracao


const char* TZ_INFO    = "BRST+3BRDT+2,M10.3.0,M2.3.0";  // enter your time zone (https://remotemonitoringsystems.ca/time-zone-abbreviations.php)

#define GPIO34 ADC1_CHANNEL_6

#define DHTPIN 4    

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

//#define WIFI_SSID "HOME"
//#define WIFI_PASSWORD "@M11g06c96#"
//#define WIFI_SSID "phgalaxy"
//#define WIFI_PASSWORD "paulohenrique"

String ssid;
String password;
long deviceId;
int noiseThreshold;
boolean active = false;

//#define MQTT_HOST IPAddress(192, 168, 1, 10)
#define MQTT_HOST "broker.hivemq.com"
#define MQTT_PORT 1883

#define MQTT_CLIENT_ID "ESP32PhcnTeste"
#define MQTT_MESSAGE_LEN 128
#define SENSORS_TOPIC MQTT_CLIENT_ID "/sensors"
#define NOISE_TOPIC MQTT_CLIENT_ID "/noises"


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

bool setDeviceId(){

  preferences.begin("info", false);
  long id = preferences.getLong("deviceId", 0);
  preferences.end();

  if(id == 0) return false;
  
  deviceId = id;
}

void setNoiseThreshold(){
  preferences.begin("info", false);
  long value = preferences.getInt("noiseThreshold", 2048); // default = 4096/2
  preferences.end();
  noiseThreshold = value;
}

void saveDeviceName(){
  preferences.begin("info", false);
  preferences.putString("deviceName", DEVICE_NAME);
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
  WiFi.begin(ssid.c_str(), password.c_str());
  //WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
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

void configureNoiseSensor(){
    setNoiseThreshold();
    adc1_config_width(ADC_WIDTH_BIT_12);
    
    // full voltage range
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    
    esp_adc_cal_value_t adc_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_cal);//Inicializa a estrutura de calibracao

}

uint32_t readNoiseSensor(){
  uint32_t voltage = adc1_get_raw(ADC1_CHANNEL_6);//Converte e calibra o valor lido (RAW) para mV
  return esp_adc_cal_raw_to_voltage(voltage, &adc_cal);//Converte e calibra o valor lido (RAW) para mV
}

void noiseMonitoring(){

    uint32_t noise = readNoiseSensor();
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
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();


        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
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

  saveDeviceName();

  Serial.begin(115200);
  Serial.println();
  Serial.println();

  configTime (0, 0, "pool.ntp.org", "time.nist.gov");
  setenv("TZ", TZ_INFO, 1);

  dht.begin();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  sensorsTimer = xTimerCreate("sensorsTimer", pdMS_TO_TICKS(10000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(activeReadSensors));
  noiseTimer = xTimerCreate("noiseTimer", pdMS_TO_TICKS(1000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(noiseMonitoring));

  WiFi.onEvent(WiFiEvent);

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

  configureNoiseSensor();

}


void loop() {

  if(active){
    publishWeatherRead();
    vTaskDelay(600000); // 10 * 1min
  }

}
