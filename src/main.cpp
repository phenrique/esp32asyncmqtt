#include <Arduino.h>

#include "BleManager/Ble.h"
#include "WifiManager/WifiManager.h"
#include "MqttManager/MqttManager.h"
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <ArduinoJson.h>
#include "DHT.h"
#include "NoiseSensor/NoiseSensor.h"

#define MQTT_MESSAGE_LEN 128

#define MQTT_HOST "200.239.66.45"
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "testEsp32Device-01" // cada dispositivo deve ter um id diferente
#define MQTT_TOPIC_ROOT "ESP32PhcnTeste"
#define SENSORS_TOPIC MQTT_TOPIC_ROOT "/sensors"
#define NOISE_TOPIC MQTT_TOPIC_ROOT "/noises"

bool deviceConnected = false;

Preferences preferences;

uint32_t value = 0;

WifiManager wifiManager;
MqttManager mqttManager(MQTT_CLIENT_ID, MQTT_HOST, MQTT_PORT);

Ble ble = Ble(preferences);

NoiseSensor noiseSensor;

// const char* TZ_INFO    = "BRST+3BRDT+2,M10.3.0,M2.3.0";  // com fuso horário
const char *TZ_INFO = "BRT3";

#define DHTPIN 4
#define DHTTYPE DHT22 // DHT 22  (AM2302), AM2321

String ssid;
String password;
long deviceId;
int noiseThreshold;
boolean active = false;

DHT dht(DHTPIN, DHTTYPE);

StaticJsonDocument<MQTT_MESSAGE_LEN> doc;
StaticJsonDocument<64> docNoise;

TimerHandle_t sensorsTimer;
TimerHandle_t noiseTimer;

void setDeviceId()
{
  preferences.begin("info", false);
  deviceId = preferences.getLong("deviceId", 0);
  preferences.end();
}

void setNoiseThreshold()
{
  preferences.begin("info", false);
  noiseThreshold = preferences.getInt("noiseThreshold", 2048); // default = 4096/2
  preferences.end();
}

void activeReadSensors()
{
  Serial.println("Iniciando leitura dos sensores");
  active = true;
  xTimerStart(noiseTimer, 0);
  xTimerStop(sensorsTimer, 0);
}

void deactiveReadSensors()
{
  Serial.println("Interrompendo leitura dos sensores");
  active = false;
  xTimerStop(noiseTimer, 0); // desativa as leituras caso o Wi-Fi esteja desconectado
}

void noiseMonitoring()
{

  uint32_t noise = noiseSensor.readSmooth();
  setNoiseThreshold();

  if (noise > noiseThreshold)
  {

    setDeviceId();
    // char output[35];
    // sprintf(output, "{\"deviceId\":%d,\"timestamp\":%u}", deviceId, time(nullptr));
    docNoise["deviceId"] = deviceId;
    docNoise["timestamp"] = time(nullptr);

    char output[64];
    serializeJson(docNoise, output);
    Serial.print("noise up! ");
    Serial.println(output);
    mqttManager.publish(NOISE_TOPIC, 0, false, output);
    xTimerStart(noiseTimer, 0);
  }

  xTimerStart(noiseTimer, 0);
}

void publishWeatherRead()
{

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t))
  {
    Serial.println(F("Failed to read from DHT sensor!"));
  }
  else
  {
    doc["deviceId"] = deviceId;
    doc["humidity"] = round(h);
    doc["temperature"] = round(t);
    doc["timestamp"] = time(nullptr);

    char output[MQTT_MESSAGE_LEN];
    serializeJson(doc, output);
    Serial.println(output);
    mqttManager.publish(SENSORS_TOPIC, 0, false, output);
  }
}

void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
  case ARDUINO_EVENT_WIFI_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    mqttManager.connect();

    break;
  case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    wifiManager.disconnect();          // Precisamos limpar a conexão antes de tentar reconectar
    mqttManager.disableReconnection(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    active = false;                    // desativa as leituras caso o Wi-Fi esteja desconectado
    wifiManager.reconnect();           // Espera o timer definido e depois tenta reconectar
    break;
  }
}

void onMqttConnect(bool sessionPresent)
{
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub = mqttManager.subscribe("test/lol", 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
  mqttManager.publish("test/lol", 0, true, "test 1");
  Serial.println("Publishing at QoS 0");
  uint16_t packetIdPub1 = mqttManager.publish("test/lol", 1, true, "test 2");
  Serial.print("Publishing at QoS 1, packetId: ");
  Serial.println(packetIdPub1);
  uint16_t packetIdPub2 = mqttManager.publish("test/lol", 2, true, "test 3");
  Serial.print("Publishing at QoS 2, packetId: ");
  Serial.println(packetIdPub2);

  xTimerStart(sensorsTimer, 0);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Disconnected from MQTT.");
  Serial.println("Motivo:");
  deactiveReadSensors();

  if (wifiManager.isConnected())
  {
    mqttManager.reconnect();
  }
}

void setup()
{

  Serial.begin(115200);
  Serial.println();
  Serial.println();

  setenv("TZ", TZ_INFO, 1);
  tzset();

  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  dht.begin();
  noiseSensor.begin();
  noiseSensor.beginSmoothing();

  mqttManager.begin(onMqttConnect, onMqttDisconnect);
  wifiManager.begin(WiFiEvent);

  sensorsTimer = xTimerCreate("sensorsTimer", pdMS_TO_TICKS(10000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(activeReadSensors));
  noiseTimer = xTimerCreate("noiseTimer", pdMS_TO_TICKS(1000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(noiseMonitoring));

  setDeviceId();

  ble.start();

  delay(1000);
}

void loop()
{
  if (active)
  {
    publishWeatherRead();
    vTaskDelay(600000); // 10 * 1min
  }
}
