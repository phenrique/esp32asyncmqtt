#ifndef WIFIMANAGER_H
#define WIFIMANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

class WifiManager
{
public:
    WifiManager();
    void begin(void metodo(arduino_event_id_t event));
    bool isConnected();
    void disconnect();
    void reconnect();

private:
    TimerHandle_t wifiReconnectTimer;
};

#endif // WIFIMANAGER_H