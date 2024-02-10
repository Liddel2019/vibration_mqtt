#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Инициализация переменных для WiFi и MQTT
const char* ssid = "<Your SSID Here>";
const char* password = "<Your WiFi Password Here>";
const char* mqttServer = "<MQTT Server IP Address Here>";
const int mqttPort = 1883; // Default MQTT port, adjust if necessary
const char* mqttUser = "<Your MQTT Username Here>";
const char* mqttPassword = "<Your MQTT Password Here>";

unsigned long previousMillis = 0; // Хранит время последней отправки данных
const long interval = 2000; // Интервал между отправками данных в миллисекундах (например, 2000 мс = 2 секунды)
int connectionAttempts = 0;

WiFiClient espClient;
PubSubClient client(espClient);

MPU6050 mpu;
const int ledPin = 2; // LED pin for indicating status

struct SensorData {
  long axMin, axMax, ayMin, ayMax, azMin, azMax, gxMin, gxMax, gyMin, gyMax, gzMin, gzMax;
  long axSum, aySum, azSum, gxSum, gySum, gzSum;
  long count;
} sensorData;

void resetSensorData() {
  sensorData = {INT16_MAX, INT16_MIN, INT16_MAX, INT16_MIN, INT16_MAX, INT16_MIN,
                INT16_MAX, INT16_MIN, INT16_MAX, INT16_MIN, INT16_MAX, INT16_MIN,
                0, 0, 0, 0, 0, 0, 0};
}

void publishSensorData() {
  if (sensorData.count == 0) return; // Проверяем, есть ли данные для отправки
  
  // Отправляем данные акселерометра и гироскопа в отдельные топики
  client.publish("sensor/Ax/min", String(sensorData.axMin).c_str());
  client.publish("sensor/Ax/max", String(sensorData.axMax).c_str());
  client.publish("sensor/Ax/avg", String(sensorData.axSum / sensorData.count).c_str());

  client.publish("sensor/Ay/min", String(sensorData.ayMin).c_str());
  client.publish("sensor/Ay/max", String(sensorData.ayMax).c_str());
  client.publish("sensor/Ay/avg", String(sensorData.aySum / sensorData.count).c_str());

  client.publish("sensor/Az/min", String(sensorData.azMin).c_str());
  client.publish("sensor/Az/max", String(sensorData.azMax).c_str());
  client.publish("sensor/Az/avg", String(sensorData.azSum / sensorData.count).c_str());

  client.publish("sensor/Gx/min", String(sensorData.gxMin).c_str());
  client.publish("sensor/Gx/max", String(sensorData.gxMax).c_str());
  client.publish("sensor/Gx/avg", String(sensorData.gxSum / sensorData.count).c_str());

  client.publish("sensor/Gy/min", String(sensorData.gyMin).c_str());
  client.publish("sensor/Gy/max", String(sensorData.gyMax).c_str());
  client.publish("sensor/Gy/avg", String(sensorData.gySum / sensorData.count).c_str());

  client.publish("sensor/Gz/min", String(sensorData.gzMin).c_str());
  client.publish("sensor/Gz/max", String(sensorData.gzMax).c_str());
  client.publish("sensor/Gz/avg", String(sensorData.gzSum / sensorData.count).c_str());
}

int getSignalQuality(long rssi) {
  int quality = 0;
  if (rssi <= -100) {
    quality = 0;
  } else if (rssi >= -50) {
    quality = 100;
  } else {
    quality = 2 * (rssi + 100);
  }
  return quality;
}

String getSensorDataJson() {
  StaticJsonDocument<512> doc; // Создайте JSON документ. Размер может быть адаптирован в зависимости от ваших нужд
  
  if (sensorData.count == 0) {
    doc["error"] = "No data to calculate averages";
    String result;
    serializeJson(doc, result);
    return result;
  }
  
  // Добавляем данные акселерометра и гироскопа в JSON документ
  doc["Ax"]["min"] = sensorData.axMin;
  doc["Ax"]["max"] = sensorData.axMax;
  doc["Ax"]["avg"] = sensorData.axSum / sensorData.count;
  doc["Ay"]["min"] = sensorData.ayMin;
  doc["Ay"]["max"] = sensorData.ayMax;
  doc["Ay"]["avg"] = sensorData.aySum / sensorData.count;
  doc["Az"]["min"] = sensorData.azMin;
  doc["Az"]["max"] = sensorData.azMax;
  doc["Az"]["avg"] = sensorData.azSum / sensorData.count;
  doc["Gx"]["min"] = sensorData.gxMin;
  doc["Gx"]["max"] = sensorData.gxMax;
  doc["Gx"]["avg"] = sensorData.gxSum / sensorData.count;
  doc["Gy"]["min"] = sensorData.gyMin;
  doc["Gy"]["max"] = sensorData.gyMax;
  doc["Gy"]["avg"] = sensorData.gySum / sensorData.count;
  doc["Gz"]["min"] = sensorData.gzMin;
  doc["Gz"]["max"] = sensorData.gzMax;
  doc["Gz"]["avg"] = sensorData.gzSum / sensorData.count;
  
  String result;
  serializeJson(doc, result); // Сериализуем JSON документ в строку
  return result;
}



void updateSensorData(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {
  sensorData.axMin = min(sensorData.axMin, (long)ax);
  sensorData.axMax = max(sensorData.axMax, (long)ax);
  sensorData.ayMin = min(sensorData.ayMin, (long)ay);
  sensorData.ayMax = max(sensorData.ayMax, (long)ay);
  sensorData.azMin = min(sensorData.azMin, (long)az);
  sensorData.azMax = max(sensorData.azMax, (long)az);
  sensorData.gxMin = min(sensorData.gxMin, (long)gx);
  sensorData.gxMax = max(sensorData.gxMax, (long)gx);
  sensorData.gyMin = min(sensorData.gyMin, (long)gy);
  sensorData.gyMax = max(sensorData.gyMax, (long)gy);
  sensorData.gzMin = min(sensorData.gzMin, (long)gz);
  sensorData.gzMax = max(sensorData.gzMax, (long)gz);
  
  sensorData.axSum += ax;
  sensorData.aySum += ay;
  sensorData.azSum += az;
  sensorData.gxSum += gx;
  sensorData.gySum += gy;
  sensorData.gzSum += gz;
  sensorData.count++;
}

String getSensorDataString() {
  if (sensorData.count == 0) {
    return "No data to calculate averages";
  }
  return "Ax: " + String(sensorData.axMin) + "/" + String(sensorData.axSum / sensorData.count) + "/" + String(sensorData.axMax) +
         " Ay: " + String(sensorData.ayMin) + "/" + String(sensorData.aySum / sensorData.count) + "/" + String(sensorData.ayMax) +
         " Az: " + String(sensorData.azMin) + "/" + String(sensorData.azSum / sensorData.count) + "/" + String(sensorData.azMax) +
         " Gx: " + String(sensorData.gxMin) + "/" + String(sensorData.gxSum / sensorData.count) + "/" + String(sensorData.gxMax) +
         " Gy: " + String(sensorData.gyMin) + "/" + String(sensorData.gySum / sensorData.count) + "/" + String(sensorData.gyMax) +
         " Gz: " + String(sensorData.gzMin) + "/" + String(sensorData.gzSum / sensorData.count) + "/" + String(sensorData.gzMax);
}

void setAccelRange(int range) {
  switch(range) {
    case 16:
      mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
      Serial.println("Accel range set to ±16g");
      break;
    case 8:
      mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
      Serial.println("Accel range set to ±8g");
      break;
    case 4:
      mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
      Serial.println("Accel range set to ±4g");
      break;
    case 2:
      mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
      Serial.println("Accel range set to ±2g");
      break;
    default:
      Serial.println("Invalid accel range. Setting to default ±2g");
      mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  }
}

void connectToWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password); // Начинаем подключение к WiFi

  while (WiFi.status() != WL_CONNECTED) { // Проверяем, подключились ли мы к WiFi
    delay(500);
    Serial.print(".");
    connectionAttempts++; // Увеличиваем счетчик попыток подключения
    if (connectionAttempts > 20) { // Если попыток слишком много, делаем перезагрузку
      Serial.println("Failed to connect to WiFi. Please check your credentials");
      ESP.restart(); // Перезагружаем устройство
    }
  }

  Serial.println("\nConnected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP()); // Выводим IP-адрес устройства

  // Выводим дополнительную информацию о WiFi
  long rssi = WiFi.RSSI(); // Получаем RSSI (Received Signal Strength Indication)
  Serial.print("Signal Strength (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");
  int quality = getSignalQuality(rssi); // Преобразуем RSSI в качество сигнала
  Serial.print("Signal Quality: ");
  Serial.print(quality);
  Serial.println("%");
}

void publishWiFiInfo() {
  if (!client.connected()) {
    Serial.println("MQTT client is not connected, cannot publish WiFi info.");
    return;
  }

  long rssi = WiFi.RSSI();
  int quality = getSignalQuality(rssi);
  String ip = WiFi.localIP().toString();
  String ssid = WiFi.SSID();
  String bssid = WiFi.BSSIDstr();
  int channel = WiFi.channel();
  String mac = WiFi.macAddress();

  // Отправляем информацию о силе сигнала
  client.publish("wifi/RSSI", String(rssi).c_str());
  client.publish("wifi/SignalQuality", String(quality).c_str());

  // Отправляем информацию о сети
  client.publish("wifi/IP", ip.c_str());
  client.publish("wifi/SSID", ssid.c_str());
  client.publish("wifi/BSSID", bssid.c_str());
  client.publish("wifi/Channel", String(channel).c_str());

  // Отправляем информацию о устройстве
  client.publish("wifi/MAC", mac.c_str());

  Serial.println("WiFi info sent to MQTT.");
}

void setup() {
  Serial.begin(115200);
  // Настройка WiFi
  setAccelRange(2); //2,4,8,16
  connectToWiFi();
  // Настройка клиента MQTT
  client.setServer(mqttServer, mqttPort);
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
      Serial.println("Connected to MQTT");
    } else {
      Serial.print("MQTT connection failed, state: ");
      Serial.println(client.state());
      
    }
  }

  pinMode(ledPin, OUTPUT);
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
  } else {
    Serial.println("MPU6050 connection successful");
  }
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);

  // Calibrate sensor
  calibrateSensor();

  resetSensorData();
}

void loop() {
   unsigned long currentMillis = millis();

  if (client.connected()) {
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      // Отправляем данные в формате JSON
      String sensorDataJson = getSensorDataJson();
      if (client.publish("sensor/data/json", sensorDataJson.c_str())) {
        Serial.println("JSON data sent to MQTT: " + sensorDataJson);
      } else {
        Serial.println("Failed to send JSON data to MQTT.");
      }

      // Отправляем разделенные данные для каждой подкатегории
      publishSensorData();

      publishWiFiInfo();
    }
  } else {
    Serial.println("Lost connection to MQTT. Attempting to reconnect...");
    setup(); // Переподключение к MQTT, если соединение потеряно
  }

  if (mpu.testConnection()) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);
    updateSensorData(ax, ay, az, gx, gy, gz);

    if (sensorData.count % 100 == 0) { // Adjust this value to control how often the data is printed
      Serial.println(getSensorDataString());
      resetSensorData();
    }
  } else {
    Serial.println("MPU6050 connection failed");
  }
}

void calibrateSensor() {
  Serial.println("Calibrating MPU6050, please leave the device still...");

  // Инициализация переменных для хранения предыдущих смещений
  long prevAccelOffset[3] = {0, 0, 0};
  long prevGyroOffset[3] = {0, 0, 0};
  bool isStable = false;
  int iteration = 0;
  const int maxIterations = 100; // Максимальное количество итераций
  const long stabilityThreshold = 100; // Порог стабильности для изменений смещений

  while (!isStable && iteration < maxIterations) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    isStable = true;

    // Проверка стабильности калибровки
    for (int i = 0; i < 3; i++) {
      long currentAccelOffset = mpu.getXAccelOffset() + mpu.getYAccelOffset() + mpu.getZAccelOffset();
      long currentGyroOffset = mpu.getXGyroOffset() + mpu.getYGyroOffset() + mpu.getZGyroOffset();

      if (abs(currentAccelOffset - prevAccelOffset[i]) > stabilityThreshold || 
          abs(currentGyroOffset - prevGyroOffset[i]) > stabilityThreshold) {
        isStable = false;
      }

      // Обновление предыдущих смещений для следующей итерации
      prevAccelOffset[i] = currentAccelOffset;
      prevGyroOffset[i] = currentGyroOffset;
    }

    if (!isStable) {
      Serial.println("Stability not achieved, recalibrating...");
    }

    iteration++;
  }

  mpu.PrintActiveOffsets();
  Serial.println("Calibration Done.");
}

void blinkLed(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
  }
}