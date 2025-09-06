#include "Ble.h"

Ble::Ble(const char *deviceName, Preferences &preferences) : preferences(preferences), deviceName(deviceName),
                                                             pServer(nullptr), pService(nullptr),
                                                             pCredentialsCharacteristic(nullptr),
                                                             pDevInfoCharacteristic(nullptr),
                                                             pLogCharacteristic(nullptr),
                                                             deviceConnected(false)
{
}

void Ble::start()
{
    uint8_t controller = 0;

    // Create the BLE Device
    BLEDevice::init(deviceName);

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks(deviceConnected));

    // Create the BLE Service
    pService = pServer->createService(SERVICE_UUID);

    // Create BLE Characteristics
    pCredentialsCharacteristic = pService->createCharacteristic(
        CREDENDIAL_CHRTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

    pDevInfoCharacteristic = pService->createCharacteristic(
        INFO_CHRTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

    pLogCharacteristic = pService->createCharacteristic(
        LOG_CHRTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY);

    // Set Callbacks for Characteristics
    pCredentialsCharacteristic->setCallbacks(new CredentialsCallbacks(preferences, controller));
    pDevInfoCharacteristic->setCallbacks(new DevInfoCallbacks(preferences));

    // Add Descriptors to Characteristics
    pCredentialsCharacteristic->addDescriptor(new BLE2902());
    pDevInfoCharacteristic->addDescriptor(new BLE2902());
    pLogCharacteristic->addDescriptor(new BLE2902());

    // Start the service
    pService->start();

    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    BLEDevice::startAdvertising();
}

void Ble::logar(const char *message)
{

  if(pLogCharacteristic == nullptr ) // Verifica se a característica foi inicializada
  {
    Serial.println("Lembre de instanciar o BLE");
    return;
  }

    // Verifica se o dispositivo está conectado e se a característica foi inicializada
  BLE2902* p2902 = (BLE2902*)pLogCharacteristic->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));

    if (deviceConnected && p2902->getNotifications()) // Verifica se a característica foi inicializada
    {
        pLogCharacteristic->setValue(message); // Define o valor da característica
        pLogCharacteristic->notify();         // Envia a notificação para o cliente BLE
        Serial.print("Log enviado via BLE: ");
        Serial.println(message);
    }
}

bool Ble::isDeviceConnected() const
{
    return deviceConnected; // Retorna o estado de conexão
}
