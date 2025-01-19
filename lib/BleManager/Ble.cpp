#include "Ble.h"

Ble::Ble(const char *deviceName, Preferences &preferences) : preferences(preferences), deviceName(deviceName)
{
  this->preferences = preferences;
  this->deviceName = deviceName;
}

void Ble::start()
{

  bool deviceConnected;
  uint8_t controller = 0;

  // Create the BLE Device
  BLEDevice::init(deviceName);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks(deviceConnected));

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  BLECharacteristic *pCredentialsCharacteristic =
      pService->createCharacteristic(CREDENDIAL_CHRTIC_UUID,
                                     BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE);

  BLECharacteristic *pDevInfoCharacteristic = pService->createCharacteristic(
      INFO_CHRTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  pCredentialsCharacteristic->setCallbacks(new CredentialsCallbacks(preferences, controller));
  pCredentialsCharacteristic->addDescriptor(new BLE2902());

  pDevInfoCharacteristic->setCallbacks(new DevInfoCallbacks(preferences));
  pDevInfoCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();
}
