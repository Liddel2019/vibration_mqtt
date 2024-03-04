// Version 1.3
#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "FS.h"
#include "SD.h"
#include "SPI.h"

#include "GyverEncoder.h"

// ПИНы энкодера

#define CLK 15
#define DT 4
#define SW 16

Encoder enc1(CLK, DT, SW);

#define DEBUG

bool writeSD = true;

// Инициализация переменных для WiFi и MQTT
const char* ssid = "ssid";
const char* password = "password";

const char* mqttServer = "mqttServer";
const int mqttPort = 1883;
const char* mqttUser = "mqttUser";
const char* mqttPassword = "mqttPassword";

unsigned long previousMillis = 0; // Хранит время последней отправки данных
unsigned long previousMillis2 = 0; // Хранит время последней записи данных на SD
const long interval = 1250; // Интервал между отправками данных в миллисекундах (например, 2000 мс = 2 секунды)
int connectionAttempts = 0;

unsigned int pubSubClientBufferSize = 1024; // Начальный размер буфера

bool bufferFull = false;
unsigned long lastBufferStatusTime = 0;
const unsigned long bufferStatusInterval = 1000; // Проверяем каждые 10 секунд

WiFiClient espClient;
PubSubClient client(espClient);

MPU6050 mpu;
const int ledPin = 2; // LED pin for indicating status

struct SensorData {
  long axMin, axMax, ayMin, ayMax, azMin, azMax, gxMin, gxMax, gyMin, gyMax, gzMin, gzMax;
  long axSum, aySum, azSum, gxSum, gySum, gzSum;
  long count;

  // Конструктор для инициализации данных
  SensorData() : axMin(INT16_MAX), axMax(INT16_MIN), ayMin(INT16_MAX), ayMax(INT16_MIN),
    azMin(INT16_MAX), azMax(INT16_MIN), gxMin(INT16_MAX), gxMax(INT16_MIN),
    gyMin(INT16_MAX), gyMax(INT16_MIN), gzMin(INT16_MAX), gzMax(INT16_MIN),
    axSum(0), aySum(0), azSum(0), gxSum(0), gySum(0), gzSum(0),
    count(0) {}
};

SensorData sensorData;

////////////////////////////////////////////////////////////

void initSD() {

  if (!SD.begin(5)) {
#ifdef DEBUG
    Serial.println("Card Mount Failed");
#endif
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
#ifdef DEBUG
    Serial.println("No SD card attached");
#endif
    return;
  }
#ifdef DEBUG
  Serial.print("SD Card Type: ");
#endif

  if (cardType == CARD_MMC) {
#ifdef DEBUG
    Serial.println("MMC");
#endif
  } else if (cardType == CARD_SD) {
#ifdef DEBUG
    Serial.println("SDSC");
#endif
  } else if (cardType == CARD_SDHC) {
#ifdef DEBUG
    Serial.println("SDHC");
#endif
  } else {
#ifdef DEBUG
    Serial.println("UNKNOWN");
#endif
  }

#ifdef DEBUG
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
#endif

}

void writeFile(fs::FS &fs, const char * path, const char * message) {

#ifdef DEBUG
  Serial.printf("Writing file: %s\n", path);
#endif

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
#ifdef DEBUG
    Serial.println("Failed to open file for writing");
#endif
    return;
  }
  if (file.print(message)) {
#ifdef DEBUG
    Serial.println("File written");
#endif
  } else {
#ifdef DEBUG
    Serial.println("Write failed");
#endif
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message) {

#ifdef DEBUG
  Serial.printf("Appending to file: %s\n", path);
#endif

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
#ifdef DEBUG
    Serial.println("Failed to open file for appending");
#endif
    return;
  }
  if (file.print(message)) {
#ifdef DEBUG
    Serial.println("Message appended");
#endif
  } else {
#ifdef DEBUG
    Serial.println("Append failed");
#endif
  }
  file.close();
}


////////////////////////////////////////////////////////////

void blinkLed(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
  }
}

void resetSensorData() {
  // Просто создаем новый экземпляр SensorData, который будет автоматически инициализирован
  sensorData = SensorData();
}

void publishSensorData() {
  if (sensorData.count == 0) return; // Проверяем, есть ли данные для отправки

  // Отправляем данные акселерометра и гироскопа в отдельные топики
  client.publish("TEST/sensor/Ax/min", String(sensorData.axMin).c_str());
  client.publish("TEST/sensor/Ax/max", String(sensorData.axMax).c_str());
  client.publish("TEST/sensor/Ax/avg", String(sensorData.axSum / sensorData.count).c_str());

  client.publish("TEST/sensor/Ay/min", String(sensorData.ayMin).c_str());
  client.publish("TEST/sensor/Ay/max", String(sensorData.ayMax).c_str());
  client.publish("TEST/sensor/Ay/avg", String(sensorData.aySum / sensorData.count).c_str());

  client.publish("TEST/sensor/Az/min", String(sensorData.azMin).c_str());
  client.publish("TEST/sensor/Az/max", String(sensorData.azMax).c_str());
  client.publish("TEST/sensor/Az/avg", String(sensorData.azSum / sensorData.count).c_str());

  client.publish("TEST/sensor/Gx/min", String(sensorData.gxMin).c_str());
  client.publish("TEST/sensor/Gx/max", String(sensorData.gxMax).c_str());
  client.publish("TEST/sensor/Gx/avg", String(sensorData.gxSum / sensorData.count).c_str());

  client.publish("TEST/sensor/Gy/min", String(sensorData.gyMin).c_str());
  client.publish("TEST/sensor/Gy/max", String(sensorData.gyMax).c_str());
  client.publish("TEST/sensor/Gy/avg", String(sensorData.gySum / sensorData.count).c_str());

  client.publish("TEST/sensor/Gz/min", String(sensorData.gzMin).c_str());
  client.publish("TEST/sensor/Gz/max", String(sensorData.gzMax).c_str());
  client.publish("TEST/sensor/Gz/avg", String(sensorData.gzSum / sensorData.count).c_str());

  client.publish("TEST/sensor/Ax/diff", String(sensorData.axMax - sensorData.axMin).c_str());
  client.publish("TEST/sensor/Ay/diff", String(sensorData.ayMax - sensorData.ayMin).c_str());
  client.publish("TEST/sensor/Az/diff", String(sensorData.azMax - sensorData.azMin).c_str());
  client.publish("TEST/sensor/Gx/diff", String(sensorData.gxMax - sensorData.gxMin).c_str());
  client.publish("TEST/sensor/Gy/diff", String(sensorData.gyMax - sensorData.gyMin).c_str());
  client.publish("TEST/sensor/Gz/diff", String(sensorData.gzMax - sensorData.gzMin).c_str());
}

int getSignalQuality(int rssi) {
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
  switch (range) {
    case 16:
      mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
      Serial.println("Accel range set to ±16g");
      break;
    case 8:
      mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
      Serial.println("Accel range set to ±8g");
      break;
    case 4:
      mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
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
  int rssi = WiFi.RSSI(); // Получаем RSSI (Received Signal Strength Indication)
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

  int rssi = WiFi.RSSI();
  int quality = getSignalQuality(rssi);
  String ip = WiFi.localIP().toString();
  String ssid = WiFi.SSID();
  String bssid = WiFi.BSSIDstr();
  int channel = WiFi.channel();
  String mac = WiFi.macAddress();

  // Отправляем информацию о силе сигнала
  client.publish("TEST/wifi/RSSI", String(rssi).c_str());
  client.publish("TEST/wifi/SignalQuality", String(quality).c_str());

  // Отправляем информацию о сети
  client.publish("TEST/wifi/IP", ip.c_str());
  client.publish("TEST/wifi/SSID", ssid.c_str());
  client.publish("TEST/wifi/BSSID", bssid.c_str());
  client.publish("TEST/wifi/Channel", String(channel).c_str());

  // Отправляем информацию о устройстве
  client.publish("TEST/wifi/MAC", mac.c_str());

  Serial.println("WiFi info sent to MQTT.");

  if (!client.publish("topic", "message")) {
    bufferFull = true;
    client.publish("TEST/warning/PubSubClient_buffer_status", "buffer full");
  }
}

void reconnectWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Attempting to reconnect...");
    WiFi.disconnect();
    WiFi.reconnect();
    connectionAttempts = 0; // Сброс счетчика попыток подключения
    while (WiFi.status() != WL_CONNECTED && connectionAttempts < 20) {
      delay(500);
      Serial.print(".");
      connectionAttempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nReconnected to WiFi");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("\nFailed to reconnect to WiFi. Check your credentials or WiFi signal");
    }
  }
}

void reconnectMQTT() {
  // Проверяем, подключены ли мы к MQTT
  if (!client.connected()) {
    Serial.println("Attempting MQTT reconnection...");
    // Пытаемся подключиться
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("Reconnected to MQTT");
      // Здесь можно также повторно подписаться на нужные топики
    } else {
      Serial.print("Failed to reconnect to MQTT, state: ");
      Serial.println(client.state());
      // Можно добавить задержку перед следующей попыткой
    }


  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");

  String messageTemp;

  if (String(topic) == "TEST/command/setBufferSize") {
    unsigned int newSize = messageTemp.toInt();
    if (newSize > 0) {
      setPubSubClientBufferSize(newSize);
    } else {
      Serial.println("Invalid buffer size received");
    }
  }


  for (int i = 0; i < length; i++) {
    messageTemp += (char)payload[i];
  }
  Serial.println(messageTemp);

  if (String(topic) == "TEST/command/calibrate") {
    if (messageTemp == "start") {
      Serial.println("Initiating calibration...");
      calibrateSensor();
    }
  }
}

void checkAndPublishBufferStatus() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastBufferStatusTime >= bufferStatusInterval) {
    lastBufferStatusTime = currentMillis;
    if (bufferFull) {
      client.publish("TEST/warning/PubSubClient_buffer_status", "OK");
      bufferFull = false;
    }
    // Отправляем среднее значение загрузки буфера (примерное, для демонстрации)
    client.publish("TEST/warning/PubSubClient_buffer_raw", String(pubSubClientBufferSize / 2).c_str()); // Пример
  }
}

void setPubSubClientBufferSize(unsigned int newSize) {
  client.setBufferSize(newSize);
  pubSubClientBufferSize = newSize;
  Serial.print("New PubSubClient buffer size: ");
  Serial.println(newSize);
}

void setup() {
  Serial.begin(115200);

  initSD();

  // Настройка WiFi
  setAccelRange(2); //2,4,8,16
  blinkLed(3);
  connectToWiFi();
  // Настройка клиента MQTT

  int iMQTT = 10;

  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttCallback); // Установка функции-обработчика сообщений
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
      Serial.println("Connected to MQTT");
      client.subscribe("TEST/command/calibrate"); // Подписка на топик команды калибровки
    } else {
      Serial.print("MQTT connection failed, state: ");
      Serial.println(client.state());
    }

    iMQTT--;
    if (iMQTT == 0) {
      break;
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
  //calibrateSensor();

  resetSensorData();

  enc1.setType(TYPE2);

  blinkLed(10);



}

void loop() {

  enc1.tick();
  if (enc1.isSingle()) {
    writeSD = !writeSD;
#ifdef DEBUG
    Serial.println("Encoder pressed");
#endif
  }

  reconnectWiFi();

  unsigned long currentMillis = millis();
  unsigned long currentMillis2 = millis();

  if (writeSD) {

    if (currentMillis2 - previousMillis2 >= interval) {

      previousMillis2 = currentMillis2;

      String stSD = "";
      stSD = getSensorDataString();
      stSD = ": " + stSD + " \n";
      stSD = String(millis()) + stSD;

      appendFile(SD, "/data.txt", stSD.c_str());
      digitalWrite(ledPin, LOW);

    }
  } else {
    digitalWrite(ledPin, HIGH);
  }


  if (client.connected()) {
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      // Отправляем данные в формате JSON
      String sensorDataJson = getSensorDataJson();


      if (client.publish("TEST/sensor/data/json", sensorDataJson.c_str())) {
        Serial.println("JSON data sent to MQTT: " + sensorDataJson);
      } else {
        Serial.println("Failed to send JSON data to MQTT.");
      }

      // Отправляем разделенные данные для каждой подкатегории
      publishSensorData();

      checkAndPublishBufferStatus();

      publishWiFiInfo();
    }
  } else {
    blinkLed(2);
    Serial.println("Lost connection to MQTT. Attempting to reconnect...");
    reconnectMQTT(); // Переподключение к MQTT, если соединение потеряно
  }
  client.loop();

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
    //Serial.println("MPU6050 connection failed");
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
