#include "WifiManager.h"

#define WIFI_DELAY_INICIALIZACAO 2000
#define WIFI_TIMER_PERIOD 5000
#define PREFS_DELAY 1000

TimerHandle_t wifiReconnectTimer;
Preferences prefs;
String nomeRede;
String senha;

void connect(TimerHandle_t xTime);

WifiManager::WifiManager()
{
    WiFi.disconnect(true);
    vTaskDelay(pdMS_TO_TICKS(WIFI_DELAY_INICIALIZACAO));
}

void connect(TimerHandle_t xTime)
{
    Serial.println("Conectando ao Wi-Fi...");
    Serial.println(nomeRede);
    Serial.println(senha);
    WiFi.begin(nomeRede.c_str(), senha.c_str());
    WiFi.setSleep(false);
}

void WifiManager::disconnect()
{
    WiFi.disconnect(true);
}

void WifiManager::begin(void metodo(arduino_event_id_t event))
{
    Serial.println("Iniciando WifiManager...");
    prefs.begin("credentials", true);
    vTaskDelay(pdMS_TO_TICKS(PREFS_DELAY));
    nomeRede = prefs.getString("ssid", "");
    senha = prefs.getString("password", "");
    prefs.end();
    WiFi.onEvent(metodo);
    wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(WIFI_TIMER_PERIOD), pdFALSE, NULL, connect);
    connect(wifiReconnectTimer);
}

void WifiManager::reconnect()
{
    xTimerStart(wifiReconnectTimer, 0);
}

bool WifiManager::isConnected()
{
    return WiFi.isConnected();
}