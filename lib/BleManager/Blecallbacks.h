#include <Preferences.h>
#include <BLEUtils.h>

class MyServerCallbacks: public BLEServerCallbacks {

    bool deviceConnected;

    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      pServer->startAdvertising(); // Reiniciar o anúncio BLE após a desconexão
    }

    public: 

      MyServerCallbacks(bool& deviceConnected){
        deviceConnected = deviceConnected;
      }
};


class CredentialsCallbacks : public BLECharacteristicCallbacks {

  Preferences preferences;
  uint8_t controller;

  void onWrite(BLECharacteristic *pCharacteristic) override {
    String value = pCharacteristic->getValue();

    // Parse the command and value (format: "cmd:value")
    int separatorIndex = value.indexOf(':');
    if (separatorIndex == -1) {
      Serial.println("Invalid format");
      return;
    }

    int cmd = value.substring(0, separatorIndex).toInt();
    String data = value.substring(separatorIndex + 1);

    preferences.begin("credentials", false);

    switch (cmd) {
      case 1:
        preferences.putString("ssid", data.c_str());
        Serial.print("SSID salva: ");
        Serial.println(data);
        break;

      case 2:
        preferences.putString("password", data.c_str());
        Serial.print("Password salva: ");
        Serial.println(data);
        break;

      default:
        Serial.println("Comando inválido");
        break;
    }

    preferences.end();
    pCharacteristic->setValue(""); // Clear the characteristic value
  }

  void onRead(BLECharacteristic *pCharacteristic) {

      pCharacteristic->setValue("");
      preferences.begin("credentials", false);

      String credentials; 
      credentials.reserve(64);
      credentials += preferences.getString("ssid", "vazio");
      credentials += "\n";
      credentials += preferences.getString("password", "vazio");
      preferences.end();

      pCharacteristic->setValue(credentials.c_str());

      Serial.print("Leu usando preferencias: ");
      Serial.println(credentials);

  }

  public:

    CredentialsCallbacks(Preferences& preferences, uint8_t& controller){
      preferences = preferences;
      controller = controller;
    }
  

};

class DevInfoCallbacks : public BLECharacteristicCallbacks {

  Preferences preferences;
  uint8_t controller;
  
  void onWrite(BLECharacteristic *pCharacteristic) {

    String value = pCharacteristic->getValue();

    long deviceId = std::strtoul(value.c_str(), NULL, 0);

    if(controller == 1){
      if(deviceId > 0){
        preferences.begin("info", false);
        preferences.putLong("deviceId", deviceId);

        Serial.print("deviceId received: ");
        Serial.println(deviceId);
        Serial.println("deviceId Saved using Preferences");

        preferences.end();
        pCharacteristic->setValue("");
      }
      controller = 0;
    }
    if(controller == 2){
      preferences.begin("info", false);
      preferences.putInt("noiseThreshold", std::strtoul(value.c_str(), NULL, 0));

      Serial.print("noiseThreshold received: ");
      Serial.println(value.c_str());
      Serial.println("noiseThreshold Saved using Preferences");

      preferences.end();
      pCharacteristic->setValue("");
      controller = 0;
    }
    if(controller == 3){
      preferences.begin("info", false);
      preferences.putString("deviceName", value.c_str());

      Serial.print("deviceName received: ");
      Serial.println(value.c_str());
      Serial.println("deviceName Saved using Preferences");

      preferences.end();
      pCharacteristic->setValue("");
      controller = 0;
    }

    controller = std::strtoul(value.c_str(), NULL, 0);
    Serial.print("valor do controlador");
    Serial.println(controller);
  }

  void onRead(BLECharacteristic *pCharacteristic) {

      pCharacteristic->setValue("");
      preferences.begin("info", false);

      String info; 
      info.reserve(64);
      info += preferences.getString("deviceName", "vazio");
      info += ";";
      info += preferences.getLong("deviceId", 0);
      info += ";";
      info += preferences.getInt("noiseThreshold", 0);
      preferences.end();

      pCharacteristic->setValue(info.c_str());

      Serial.print("Leu usando preferencias: ");
      Serial.println(info);
  }

  public:

    DevInfoCallbacks(Preferences& preferences){
      preferences = preferences;
    }

};
