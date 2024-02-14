### Version 1.2

## Описание проекта

Проект представляет собой систему для измерения вибраций, анализа данных и поиска закономерностей изменений значений вибраций для предварительного обнаружения потенциальных проблем в механизмах. Реализован на базе платы ESP32 DevKit V1 и использует датчик MPU6050 для сбора данных о вибрации, а также модули WiFi и MQTT для передачи данных на сервер или в облако для дальнейшего анализа.

## Компоненты проекта

- **ESP32 DevKit V1** - основа системы, мощный микроконтроллер с поддержкой WiFi.
- **MPU6050** - датчик для измерения ускорения и угловой скорости, ключевой элемент для определения вибраций.
- **WiFi** - модуль для подключения к сети интернет и передачи данных.
- **MQTT** - протокол для обмена сообщениями между устройством и сервером/облаком.
- **ArduinoJson** - библиотека для работы с JSON-форматом, используется для удобной сериализации данных датчиков.

## Функционал

Программа способна измерять параметры вибрации с помощью датчика MPU6050 и отправлять полученные данные на сервер через интернет с использованием протокола MQTT. Данные отправляются в формате JSON, что облегчает их дальнейшую обработку и анализ. Программа также оснащена функциями калибровки датчика, отслеживания качества подключения к WiFi и реагирования на изменения в сетевом соединении.

### Основные возможности:
- Измерение минимальных, максимальных и средних значений параметров вибраций.
- Автоматическая калибровка датчика при запуске.
- Передача данных в формате JSON через MQTT.
- Отслеживание качества сигнала WiFi и его параметров.
- Переподключение к MQTT серверу при потере связи.
- Индикация работы и статуса подключения с помощью встроенного светодиода.

## Распиновка

- **GPIO 21 (SDA)** и **GPIO 22 (SCL)** - подключение датчика MPU6050 по интерфейсу I2C.
- **GPIO 2** - пин для индикации статуса работы/подключения светодиодом.

### Установка

Чтобы запустить этот проект, выполните следующие шаги:

1. **Настройка Arduino IDE**

   Убедитесь, что Arduino IDE установлена на вашем ПК. Если нет, загрузите ее с [официального сайта Arduino](https://www.arduino.cc/en/Main/Software) и следуйте инструкциям по установке.

2. **Установка необходимых библиотек**

   В Arduino IDE перейдите в **Скетч** > **Подключить библиотеку** > **Управление библиотеками...** и установите следующие библиотеки:
   - `MPU6050` для работы с датчиком MPU6050.
   - `WiFi` для управления WiFi соединениями.
   - `PubSubClient` для подключения и работы с MQTT.
   - `ArduinoJson` для создания и парсинга JSON объектов.

3. **Конфигурация подключения и сервера MQTT**

   В коде проекта измените следующие переменные на свои значения:
   - `const char* ssid = "yourSSID";` - имя вашей сети WiFi.
   - `const char* password = "yourWiFiPassword";` - пароль вашей сети WiFi.
   - `const char* mqttServer = "yourMQTTServerIP";` - IP-адрес вашего сервера MQTT.
   - `const int mqttPort = 1883;` - порт для подключения к MQTT серверу (по умолчанию 1883).
   - `const char* mqttUser = "yourMQTTUsername";` - имя пользователя для MQTT сервера.
   - `const char* mqttPassword = "yourMQTTPassword";` - пароль для MQTT сервера.

4. **Загрузка скетча на устройство**

   Подключите ваше устройство к компьютеру через USB. В Arduino IDE выберите тип платы и порт, к которому подключено устройство, а затем загрузите скетч.

5. **Проверка работы**

   Откройте монитор порта в Arduino IDE (`Инструменты` > `Монитор порта`), чтобы увидеть логи работы устройства: подключение к WiFi, подключение к серверу MQTT и отправку данных.

### Тестирование

Для тестирования функционала проекта следуйте этим шагам:

- **Проверка подключения к WiFi и MQTT:** Убедитесь, что в логах монитора порта появляются сообщения о успешном подключении к WiFi сети и серверу MQTT.
- **Отправка и получение данных:** Используйте MQTT клиента, например, [MQTT Explorer](http://mqtt-explorer.com/), для подписки на топики, на которые отправляются данные с вашего устройства. Проверьте, корректно ли приходят данные.
- **Тестирование датчика MPU6050:** Поверните или встряхните устройство, чтобы убедиться, что данные об изменении положения корректно отправляются и отображаются в вашем MQTT клиенте.

Эти шаги помогут вам убедиться, что система работает корректно и готова к дальнейшей разработке или развертыванию.

## Возможные проблемы и их решения

- **Проблема с подключением к WiFi**: убедитесь, что SSID и пароль указаны корректно. Проверьте мощность сигнала и удаленность от точки доступа.
- **Неуспешное подключение к MQTT серверу**: проверьте правильность адреса сервера, порта, имени пользователя и пар

оля. Убедитесь, что сервер доступен и работает.
- **Некорректные показания с датчика**: выполните повторную калибровку датчика, убедитесь, что датчик надежно закреплен и не подвергается внешним вибрациям во время калибровки.
- **Переполнение памяти при работе с JSON**: следите за размером данных, передаваемых в JSON. При необходимости увеличьте размер буфера `StaticJsonDocument`.

## Список будущих изменений для проекта измерения вибраций на ESP32 DevKit V1

В планах развития проекта предусмотрен ряд значимых улучшений и нововведений, направленных на повышение функциональности, надежности и удобства использования системы. Вот основные направления разработки:

### 🚀 Добавление LED индикации
- **Важность**: Средняя 🌟🌟
- **Описание**: Реализация многофункциональной индикации состояний работы устройства, подключения к WiFi и MQTT, а также индикация ошибок. Использование встроенного светодиода на плате для визуализации различных режимов работы и предупреждений.
- **Прогресс**: 25% Добавил индикацию!
  
### 🔋 Интеграция питания от аккумулятора
- **Важность**: Высокая 🌟🌟🌟
- **Описание**: Разработка системы автономного питания на аккумуляторных батареях для обеспечения мобильности и независимости устройства от стационарных источников питания. Будет реализована функция мониторинга уровня заряда, что критически важно для длительной автономной работы.

### 📜 Логирование ошибок
- **Важность**: Средняя 🌟🌟
- **Описание**: Создание системы логирования, которая будет фиксировать все критические ошибки и предупреждения, возникающие в процессе работы устройства. Данные об ошибках будут отправляться на сервер для анализа без сохранения в память устройства, что позволит оптимизировать использование ресурсов и обеспечить быстрое реагирование на возникающие проблемы.
- **Прогресс**: 25% Добавлена возможность управления размером буфера

### 🛠️ Самодиагностика системы
- **Важность**: Средняя 🌟🌟
- **Описание**: Введение механизма самодиагностики для автоматической проверки состояния ключевых компонентов системы, включая датчик, подключение к WiFi и состояние MQTT соединения. Это позволит своевременно обнаруживать и устранять возможные неполадки, повышая надежность системы.
- **Прогресс**: 25% Добавлена отправка данных о переполнености буфера.

### ⚙️ Улучшение стабильности работы
- **Важность**: Критическая 🌟🌟🌟🌟
- **Описание**: Разработка и внедрение мер по устранению проблем с неожиданными перезапусками ESP32, анализ возможных причин и оптимизация кода для обеспечения максимальной стабильности работы.
- **Прогресс**: 75% Исправил перезагрузку!
  
### 🎛️ Добавление дополнительных органов управления
- **Важность**: Низкая 🌟
- **Описание**: Рассмотрение возможности интеграции дополнительных элементов управления, таких как кнопки или переключатели, для расширения интерактивных возможностей устройства. Это даст пользователям больше гибкости в настройке параметров измерения и работы устройства без необходимости подключения к ПК.
- **Прогресс**: 25% 

### 📊 Расширенная отправка данных
- **Важность**: Высокая 🌟🌟🌟
- **Описание**: Увеличение объема передаваемых на сервер данных о работе ESP32, качестве соединения, уровне заряда аккумулятора, напряжении и других важных параметрах для обеспечения полного контроля над состоянием системы.
- **Прогресс**: 50% 
  
В процессе дальнейшей разработки и реализации предложенных улучшений будет уделено особое внимание тестированию нововведений для гарантии стабильности и надежности системы. Каждое из упомянутых новшеств не только повысит функциональность и удобство использования проекта, но и сделает его более привлекательным для широкого круга задач, связанных с мониторингом вибраций и диагностикой оборудования.
