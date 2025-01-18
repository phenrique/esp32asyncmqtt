#ifndef MQTTMANAGER_H
#define MQTTMANAGER_H

#include <Arduino.h>
#include <AsyncMqttClient.h>

#define RETAIN true

class MqttManager
{
public:
    MqttManager(const char *clientId, const char *host, uint16_t port);
    void begin(AsyncMqttClientInternals::OnConnectUserCallback onMqttConnect, AsyncMqttClientInternals::OnDisconnectUserCallback onMqttDisconnect);
    void connect();
    void reconnect();
    void disableReconnection();
    uint16_t publish(const char *topic, uint8_t qos, bool retain, const char *payload);
    uint16_t subscribe(const char *topic, uint8_t qos);

private:
    const char *clientId;
    const char *host;
    uint16_t port;
};

#endif // MQTTMANAGER_H