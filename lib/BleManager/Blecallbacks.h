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

    Serial.print("Recebido BLE: ");
    Serial.println(value);

    int separatorIndex = value.indexOf(':');
    if (separatorIndex == -1) {
      Serial.println("Formato do payload ble inválido");
      return;
    }

    int cmd = value.substring(0, separatorIndex).toInt();
    String data = value.substring(separatorIndex + 1);
    data.trim();

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
      case 3:
        preferences.putString("mqttHost", data.c_str());
        Serial.print("mqttHost salva: ");
        Serial.println(data);
        break;
      case 4:
        if(data == "RST") {
          Serial.println("Resetando ESP");
          ESP.restart();
        } 
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
      credentials += "\n";
      credentials += preferences.getString("mqttHost", "vazio");
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
  
  void onWrite(BLECharacteristic *pCharacteristic) override {
    String value = pCharacteristic->getValue();

    Serial.print("Recebido BLE: ");
    Serial.println(value);

    // Verifica o formato cmd:data
    int separatorIndex = value.indexOf(':');
    if (separatorIndex == -1) {
        Serial.println("Formato do payload BLE inválido");
        return;
    }

    // Extrai o comando (cmd) e os dados (data)
    int cmd = value.substring(0, separatorIndex).toInt();
    String data = value.substring(separatorIndex + 1);
    data.trim();

    preferences.begin("info", false);

    switch (cmd) {
        case 1: // Salvar deviceId
        {
            long deviceId = std::strtoul(data.c_str(), NULL, 0);
            if (deviceId > 0) {
                preferences.putLong("deviceId", deviceId);
                Serial.print("deviceId recebido e salvo: ");
                Serial.println(deviceId);
            } else {
                Serial.println("deviceId inválido");
            }
            break;
        }
        case 2: // Salvar noiseThreshold
        {
            int noiseThreshold = std::strtoul(data.c_str(), NULL, 0);
            preferences.putInt("noiseThreshold", noiseThreshold);
            Serial.print("noiseThreshold recebido e salvo: ");
            Serial.println(noiseThreshold);
            break;
        }
        case 3: // Salvar deviceName
        {
            preferences.putString("deviceName", data.c_str());
            Serial.print("deviceName recebido e salvo: ");
            Serial.println(data);
            break;
        }
        default:
            Serial.println("Comando inválido");
            break;
    }

    preferences.end();
    pCharacteristic->setValue(""); // Limpa o valor da característica
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
