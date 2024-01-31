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

  void onWrite(BLECharacteristic *pCharacteristic) {

    std::string value = pCharacteristic->getValue();

    if(controller == 1){
      preferences.begin("credentials", false);
      preferences.putString("ssid", value.c_str());

      Serial.print("SSID received: ");
      Serial.println(value.c_str());
      Serial.println("SSID Saved using Preferences");

      preferences.end();
      pCharacteristic->setValue("");
      controller = 0;
    }

    if(controller == 2){
      preferences.begin("credentials", false);
      preferences.putString("password", value.c_str());

      Serial.print("password received: ");
      Serial.println(value.c_str());
      Serial.println("password Saved using Preferences");

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

    std::string value = pCharacteristic->getValue();

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
