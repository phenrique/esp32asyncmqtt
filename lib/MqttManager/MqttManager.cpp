#include "MqttManager.h"

TimerHandle_t mqttReconnectTimer;
AsyncMqttClient mqttClient;

void connectToMqtt();
void onMqttPublish(uint16_t packetId);
void onMqttSubscribe(uint16_t packetId, uint8_t qos);
void onMqttUnsubscribe(uint16_t packetId);
void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);

void connectToMqtt()
{
    Serial.println("Conectando ao servidor MQTT...");
    mqttClient.connect();
}

MqttManager::MqttManager()
    : clientId(nullptr), host(nullptr), port(0) // Inicializa os membros com valores padrão
{
}

MqttManager::MqttManager(const char *clientId, const char *host, uint16_t port)
    : clientId(clientId), host(host), port(port)
{
}

void MqttManager::begin(AsyncMqttClientInternals::OnConnectUserCallback onMqttConnect, AsyncMqttClientInternals::OnDisconnectUserCallback onMqttDisconnect)
{
    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));

    mqttClient.setServer(host, port);
    mqttClient.setClientId(clientId);
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onSubscribe(onMqttSubscribe);
    mqttClient.onUnsubscribe(onMqttUnsubscribe);
    mqttClient.onMessage(onMqttMessage);
    mqttClient.onPublish(onMqttPublish);
}

void MqttManager::begin(const char *clientId, const char *host, uint16_t port, AsyncMqttClientInternals::OnConnectUserCallback onMqttConnect, AsyncMqttClientInternals::OnDisconnectUserCallback onMqttDisconnect)
{
    this->clientId = clientId;
    this->host = host;
    this->port = port;

    begin(onMqttConnect, onMqttDisconnect);
}

void MqttManager::connect()
{
    connectToMqtt();
}

void MqttManager::reconnect()
{
    xTimerStart(mqttReconnectTimer, 0);
}

void MqttManager::disableReconnection()
{
    xTimerStop(mqttReconnectTimer, 0);
}

uint16_t MqttManager::publish(const char *topic, uint8_t qos, bool retain, const char *payload)
{
    return mqttClient.publish(topic, qos, retain, payload);
}

uint16_t MqttManager::subscribe(const char *topic, uint8_t qos)
{
    return mqttClient.subscribe(topic, qos);
}

const char *MqttManager::getHost() const
{
    return host;
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
    Serial.println("Inscrição reconhecida.");
    Serial.print("  packetId: ");
    Serial.println(packetId);
    Serial.print("  qos: ");
    Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId)
{
    Serial.println("Inscrição cancelada.");
    Serial.print("  packetId: ");
    Serial.println(packetId);
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
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

void onMqttPublish(uint16_t packetId)
{
    Serial.println("Publish acknowledged.");
    Serial.print("  packetId: ");
    Serial.println(packetId);
}