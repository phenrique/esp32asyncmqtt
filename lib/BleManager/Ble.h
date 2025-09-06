#include "Blecallbacks.h"
#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CREDENDIAL_CHRTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define INFO_CHRTIC_UUID "c1b751df-ad8b-45bc-b6db-068589694ed1"
#define LOG_CHRTIC_UUID "94f72b92-b899-448f-a9ed-7fcfb6ec611d"
#define DEFAUT_DEVICE_NAME "LAAI-ESP32"

class Ble
{
private:
    const char *deviceName;
    Preferences &preferences;

    // Ponteiros para os objetos BLE
    BLEServer *pServer;
    BLEService *pService;
    BLECharacteristic *pCredentialsCharacteristic;
    BLECharacteristic *pDevInfoCharacteristic;
    BLECharacteristic *pLogCharacteristic;
    bool deviceConnected;

public:
    Ble(const char *deviceName, Preferences &preferences);
    void start();
    void logar(const char *message);
    bool isDeviceConnected() const;
};
