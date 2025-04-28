#include <Arduino.h>

#include "Ble.h"
#include "WifiManager.h"
#include "MqttManager.h"
#include "NoiseSensor.h"
#include "DHT.h"
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

#define DEVICE_NAME "LAAI-ESP32-01"

#define MQTT_MESSAGE_SENSORS_LEN 128
#define MQTT_MESSAGE_NOISE_LEN 64
#define SENSORS_JSON "{\"deviceId\":%ld,\"humidity\":%.0f,\"temperature\":%.0f,\"timestamp\":%ld}"
#define NOISE_SENSOR "{\"deviceId\":%d,\"timestamp\":%lu}"

#define MQTT_HOST_DEFAULT "test.mosquitto.org"
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID DEVICE_NAME // cada dispositivo deve ter um id diferente
#define MQTT_TOPIC_ROOT "ESP32PhcnTeste"
#define SENSORS_TOPIC MQTT_TOPIC_ROOT "/sensors"
#define NOISE_TOPIC MQTT_TOPIC_ROOT "/noises"

Preferences preferences;

bool deviceConnected = false;

NoiseSensor noiseSensor;

// const char* TZ_INFO    = "BRST+3BRDT+2,M10.3.0,M2.3.0";  // com fuso horário
const char *TZ_INFO = "BRT3";

#define DHTPIN 4
#define DHTTYPE DHT22 // DHT 22  (AM2302), AM2321

char mqttHostBuffer[32];

long deviceId;
int noiseThreshold;
boolean active = false;

DHT dht(DHTPIN, DHTTYPE);

TimerHandle_t sensorsTimer;
TimerHandle_t noiseTimer;

WifiManager wifiManager;

MqttManager mqttManager;

Ble ble = Ble(DEVICE_NAME, preferences);

void setMqttHost(void)
{
  preferences.begin("credentials", false);
  String host = preferences.getString("mqttHost", "");
  preferences.end();

  strlcpy(mqttHostBuffer, host.c_str(), sizeof(mqttHostBuffer));
}

const char * getMqttHost(void)
{
  return mqttHostBuffer[0] == '\0' ? MQTT_HOST_DEFAULT : mqttHostBuffer;
}

void configuraNTP(){
  setenv("TZ", TZ_INFO, 1);
  tzset();
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  time(nullptr);
}

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
    time_t now = time(nullptr);
    char output[MQTT_MESSAGE_NOISE_LEN];
    snprintf(output, MQTT_MESSAGE_NOISE_LEN, NOISE_SENSOR, deviceId, now);

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
  delay(1);
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t))
  {
    Serial.println(F("Failed to read from DHT sensor!"));
  }
  else
  {
    time_t now = time(nullptr);
    char output[MQTT_MESSAGE_SENSORS_LEN];
    snprintf(output, MQTT_MESSAGE_SENSORS_LEN, SENSORS_JSON, deviceId, round(h), round(t), now);
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
    Serial.println("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
    configuraNTP();
    delay(10);
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
  Serial.printf("Connected to MQTT. HOST: %s\n", mqttManager.getHost());
  Serial.printf("Session present: %d\n", sessionPresent);
  uint16_t packetIdSub = mqttManager.subscribe("test/lol", 2);
  Serial.printf("Subscribing at QoS 2, packetId: %d\n", packetIdSub);
  mqttManager.publish("test/lol", 0, true, "test 1");
  Serial.println("Publishing at QoS 0");
  uint16_t packetIdPub1 = mqttManager.publish("test/lol", 1, true, "test 2");
  Serial.printf("Publishing at QoS 1, packetId: %d\n", packetIdPub1);
  uint16_t packetIdPub2 = mqttManager.publish("test/lol", 2, true, "test 3");
  Serial.printf("Publishing at QoS 2, packetId: %d\n", packetIdPub2);

  xTimerStart(sensorsTimer, 0);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Disconnected from MQTT.");
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
  delay(1000);

  dht.begin();
  noiseSensor.begin();
  noiseSensor.beginSmoothing();

  sensorsTimer = xTimerCreate("sensorsTimer", pdMS_TO_TICKS(5000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(activeReadSensors));
  noiseTimer = xTimerCreate("noiseTimer", pdMS_TO_TICKS(1000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(noiseMonitoring));

  setMqttHost();
  mqttManager.begin(MQTT_CLIENT_ID, getMqttHost(), MQTT_PORT, onMqttConnect, onMqttDisconnect);
  wifiManager.begin(WiFiEvent);

  setDeviceId();

  ble.start();
    
  Serial.println("Iniciando Monitoramento em 3 segundos");
  delay(3000);

}

void loop()
{
  if (active)
  {
    publishWeatherRead();
    vTaskDelay(600000); // 10min
  }
}
