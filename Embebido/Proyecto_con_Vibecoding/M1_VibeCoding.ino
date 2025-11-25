/*
 * Bicicleta Inteligente ESP32 + Ubidots
 *
 * Librerías requeridas:
 *  - WiFi.h
 *  - PubSubClient.h
 *  - ArduinoJson.h
 *  - ESP32Servo.h
 *  - rgb_lcd.h
 *
 * Arquitectura:
 *  - FreeRTOS con tareas:
 *      * sensorTask: LDR, ultrasonido, cálculo de velocidad
 *      * switchTask: lectura del switch de encendido/apagado
 *      * commsTask: WiFi + MQTT (Ubidots) + publicación periódica
 *      * fsmTask: consume eventos de cola y ejecuta la máquina de estados
 *
 * Máquina de estados:
 *  - OFF
 *  - STOPPED
 *  - RIDING
 *  - BRAKING
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <rgb_lcd.h>

// ======================= CONFIGURACIÓN WIFI / MQTT =========================

// Rellena con tus credenciales
const char* WIFI_SSID     = "TU_SSID";
const char* WIFI_PASSWORD = "TU_PASSWORD";

// Ubidots MQTT
const char* MQTT_BROKER   = "industrial.api.ubidots.com";
const uint16_t MQTT_PORT  = 1883;
const char* MQTT_USER     = "TU_USER";      // p.ej. "Ubidots"
const char* MQTT_PASS     = "TU_TOKEN";     // Token de Ubidots

// Topics Ubidots
const char* TOPIC_CONTROL = "/v1.6/devices/bici/control/lv"; // 1.0 encender, 0.0 apagar
const char* TOPIC_TRIPS   = "/v1.6/devices/bici/trips/lv";   // 1.0 iniciar viaje, 0.0 terminar viaje
const char* TOPIC_DATA    = "/v1.6/devices/bici";            // {"data": <wheel_turn_count>}

// ======================= HARDWARE: PINES ===================================

// Sensores
const int PIN_TRIG   = 5;   // HC-SR04 TRIG
const int PIN_ECHO   = 18;  // HC-SR04 ECHO
const int PIN_HALL   = 4;   // Hall effect (interrupción)
const int PIN_LDR    = 32;  // ADC LDR
const int PIN_SWITCH = 26;  // Switch encendido/apagado (INPUT_PULLUP)

// Actuadores
const int PIN_SERVO  = 23;  // Servo de freno
const int PIN_LED    = 12;  // Luz (LED)
const int PIN_BUZZER = 19;  // Buzzer PWM

// ======================= CONSTANTES LÓGICAS ================================

// Rueda
const float WHEEL_DIAMETER_M        = 0.71f;
const float WHEEL_CIRCUMFERENCE_M   = 3.1415926f * WHEEL_DIAMETER_M;

// Umbrales de distancia (cm)
const float DIST_FAR_CM             = 30.0f;
const float DIST_MEDIUM_MIN_CM      = 20.0f;
const float DIST_MEDIUM_MAX_CM      = 30.0f;
const float DIST_NEAR_MIN_CM        = 10.0f;
const float DIST_NEAR_MAX_CM        = 20.0f;
const float DIST_VERY_NEAR_CM       = 10.0f;

// Umbrales de luz (ADC)
const int LDR_LOW_THRESHOLD         = 1500;  // poca luz -> prender LED
const int LDR_HIGH_THRESHOLD        = 2000;  // mucha luz -> apagar LED

// Buzzer (Hz según distancia)
const int BUZZER_FREQ_FAR           = 0;
const int BUZZER_FREQ_MEDIUM        = 500;
const int BUZZER_FREQ_NEAR          = 1000;
const int BUZZER_FREQ_VERY_NEAR     = 2000;

// Servo freno (grados)
const float SERVO_RELEASE_ANGLE     = 0.0f;   // freno liberado
const float SERVO_MAX_ANGLE         = 180.0f; // freno máximo
// Interpolación lineal entre 20 cm y 10 cm

// Timeout de velocidad (ms)
const unsigned long SPEED_TIMEOUT_MS = 5000;

// Publicación Ubidots (ms)
const unsigned long PUBLISH_INTERVAL_MS = 3000;

// Buzzer PWM (LEDC)
const int BUZZER_CHANNEL    = 0;
const int BUZZER_RESOLUTION = 8;   // bits

// ======================= OBJETOS GLOBALES ==================================

WiFiClient espClient;
PubSubClient mqttClient(espClient);
rgb_lcd lcd;
Servo brakeServo;

// ======================= FREE RTOS =========================================

TaskHandle_t sensorTaskHandle = nullptr;
TaskHandle_t switchTaskHandle = nullptr;
TaskHandle_t commsTaskHandle  = nullptr;
TaskHandle_t fsmTaskHandle    = nullptr;

QueueHandle_t eventQueue;

// ======================= TIPOS DE EVENTOS ==================================

enum BikeState {
  STATE_OFF,
  STATE_STOPPED,
  STATE_RIDING,
  STATE_BRAKING
};

enum EventType {
  EVT_NONE = 0,
  EVT_SWITCH_ON,
  EVT_SWITCH_OFF,
  EVT_MQTT_POWER_ON,
  EVT_MQTT_POWER_OFF,
  EVT_TRIP_START,
  EVT_TRIP_STOP,
  EVT_SPEED_UPDATE,
  EVT_SPEED_TIMEOUT,
  EVT_DISTANCE_UPDATE
};

struct Event {
  EventType type;
  float value;   // uso genérico (velocidad, distancia, etc.)
};

// ======================= ESTADO GLOBAL =====================================

volatile long hallPulses = 0;                 // Pulsos para cálculo de velocidad
volatile unsigned long wheelTurnCount = 0;    // Contador acumulado de vueltas
volatile unsigned long lastPulseTime = 0;     // Último pulso Hall (ms)

portMUX_TYPE hallMux = portMUX_INITIALIZER_UNLOCKED;

BikeState currentState = STATE_OFF;

// Estado de encendido lógico
bool physicalOn = false;  // switch físico
bool mqttOn     = false;  // comando MQTT
bool tripActive = false;  // viaje activo (para publicar datos)

// Estado auxiliar
bool ultrasonicEnabled = false;
float currentSpeedKmh  = 0.0f;
float currentBrakeAngle = 0.0f;
int currentBuzzerFreq   = 0;

// ======================= PROTOTIPOS ========================================

// ISR
void IRAM_ATTR hallISR();

// Tareas
void sensorTask(void *pvParameters);
void switchTask(void *pvParameters);
void commsTask(void *pvParameters);
void fsmTask(void *pvParameters);

// Funciones auxiliares
void connectWiFi();
void setupMqtt();
void mqttCallback(char* topic, byte* payload, unsigned int length);

float readUltrasonicCM();
void setBuzzerFrequency(int freq);
void setBrakeAngle(float angle);
void updateSpeedOnLCD(float speed);

void enterOffState();
void enterStoppedState();
void enterRidingState();
void enterBrakingState();

void processEvent(const Event &evt);

// ======================= ISR HALL ==========================================

// Interrupción para contar vueltas de rueda con antirrebote
void IRAM_ATTR hallISR() {
  static unsigned long lastInterruptTime = 0;
  unsigned long now = millis();
  if (now - lastInterruptTime > 20) {  // antirrebote ~20 ms
    portENTER_CRITICAL_ISR(&hallMux);
    hallPulses++;
    wheelTurnCount++;
    lastPulseTime = now;
    portEXIT_CRITICAL_ISR(&hallMux);
    lastInterruptTime = now;
  }
}

// ======================= SETUP =============================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Iniciando Bicicleta Inteligente...");

  // Pines
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  pinMode(PIN_HALL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), hallISR, FALLING);

  pinMode(PIN_LDR, INPUT);
  pinMode(PIN_SWITCH, INPUT_PULLUP);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // Buzzer PWM
  ledcSetup(BUZZER_CHANNEL, 2000, BUZZER_RESOLUTION);
  ledcAttachPin(PIN_BUZZER, BUZZER_CHANNEL);
  setBuzzerFrequency(0);

  // Servo de freno
  brakeServo.attach(PIN_SERVO);
  setBrakeAngle(SERVO_RELEASE_ANGLE);

  // LCD
  lcd.begin(16, 2);
  lcd.setRGB(0, 0, 0);
  lcd.noDisplay();

  // WiFi + MQTT
  WiFi.mode(WIFI_STA);
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  // Cola de eventos
  eventQueue = xQueueCreate(20, sizeof(Event));
  if (eventQueue == NULL) {
    Serial.println("Error creando cola de eventos");
  }

  // Estado inicial según switch físico
  physicalOn = (digitalRead(PIN_SWITCH) == LOW);
  mqttOn     = false;
  tripActive = false;

  if (physicalOn) {
    enterStoppedState();
  } else {
    enterOffState();
  }

  // Crear tareas FreeRTOS
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 1, &sensorTaskHandle, 1);
  xTaskCreatePinnedToCore(switchTask, "SwitchTask", 2048, NULL, 1, &switchTaskHandle, 1);
  xTaskCreatePinnedToCore(commsTask,  "CommsTask",  4096, NULL, 1, &commsTaskHandle,  0);
  xTaskCreatePinnedToCore(fsmTask,    "FSMTask",    4096, NULL, 2, &fsmTaskHandle,    1);
}

void loop() {
  // El trabajo real lo hacen las tareas FreeRTOS
  delay(10);
}

// ======================= TAREA: SENSORES ===================================

void sensorTask(void *pvParameters) {
  unsigned long lastUltrasonicMillis = 0;
  unsigned long lastSpeedCalcMillis  = 0;

  for (;;) {
    // Control de luz mediante LDR (solo si no está OFF)
    if (currentState != STATE_OFF) {
      int ldrValue = analogRead(PIN_LDR);
      // Umbrales con pequeña histéresis natural
      if (ldrValue < LDR_LOW_THRESHOLD) {
        digitalWrite(PIN_LED, HIGH);
      } else if (ldrValue > LDR_HIGH_THRESHOLD) {
        digitalWrite(PIN_LED, LOW);
      }
    } else {
      digitalWrite(PIN_LED, LOW);
    }

    unsigned long now = millis();

    // Lectura de ultrasonido cada 200 ms cuando está habilitado
    if (ultrasonicEnabled && (now - lastUltrasonicMillis >= 200)) {
      float distance = readUltrasonicCM();
      if (distance > 0) {
        Event evt;
        evt.type = EVT_DISTANCE_UPDATE;
        evt.value = distance;
        xQueueSend(eventQueue, &evt, 0);
      }
      lastUltrasonicMillis = now;
    }

    // Cálculo de velocidad cada 500 ms
    if (now - lastSpeedCalcMillis >= 500) {
      long pulses;
      unsigned long lastPulseCopy;

      portENTER_CRITICAL(&hallMux);
      pulses = hallPulses;
      hallPulses = 0;
      lastPulseCopy = lastPulseTime;
      portEXIT_CRITICAL(&hallMux);

      float speedKmhLocal = 0.0f;

      if (pulses > 0) {
        float dt = (now - lastSpeedCalcMillis) / 1000.0f;
        if (dt <= 0) dt = 0.5f;
        float turns = (float)pulses; // 1 pulso = 1 vuelta
        float mps = (turns * WHEEL_CIRCUMFERENCE_M) / dt;
        speedKmhLocal = mps * 3.6f;
      }

      bool sendTimeout = false;
      if ((now - lastPulseCopy) > SPEED_TIMEOUT_MS) {
        speedKmhLocal = 0.0f;
        sendTimeout = true;
      }

      currentSpeedKmh = speedKmhLocal;

      // Enviar evento de actualización de velocidad
      Event evtSpeed;
      evtSpeed.type = EVT_SPEED_UPDATE;
      evtSpeed.value = speedKmhLocal;
      xQueueSend(eventQueue, &evtSpeed, 0);

      // Enviar evento de timeout si corresponde
      if (sendTimeout) {
        Event evtTimeout;
        evtTimeout.type = EVT_SPEED_TIMEOUT;
        evtTimeout.value = 0;
        xQueueSend(eventQueue, &evtTimeout, 0);
      }

      lastSpeedCalcMillis = now;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ======================= TAREA: SWITCH =====================================

void switchTask(void *pvParameters) {
  int lastReading = digitalRead(PIN_SWITCH);
  int stableState = lastReading;
  unsigned long lastDebounceTime = millis();
  const unsigned long debounceDelay = 50;

  for (;;) {
    int reading = digitalRead(PIN_SWITCH);
    unsigned long now = millis();

    if (reading != lastReading) {
      lastDebounceTime = now;
    }

    if ((now - lastDebounceTime) > debounceDelay) {
      if (reading != stableState) {
        stableState = reading;
        Event evt;
        if (stableState == LOW) {
          // Encendido
          evt.type = EVT_SWITCH_ON;
        } else {
          // Apagado
          evt.type = EVT_SWITCH_OFF;
        }
        evt.value = 0;
        xQueueSend(eventQueue, &evt, portMAX_DELAY);
      }
    }

    lastReading = reading;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ======================= TAREA: COMUNICACIONES =============================

void commsTask(void *pvParameters) {
  unsigned long lastMqttAttempt = 0;
  unsigned long lastPublishMillis = 0;

  for (;;) {
    // WiFi
    if (WiFi.status() != WL_CONNECTED) {
      connectWiFi();
    }

    // MQTT: reconexión periódica
    if (!mqttClient.connected()) {
      unsigned long now = millis();
      if (now - lastMqttAttempt > 5000) {
        lastMqttAttempt = now;
        Serial.print("Conectando a MQTT...");
        if (mqttClient.connect("biciInteligenteESP32", MQTT_USER, MQTT_PASS)) {
          Serial.println(" conectado");
          mqttClient.subscribe(TOPIC_CONTROL);
          mqttClient.subscribe(TOPIC_TRIPS);
        } else {
          Serial.print(" fallo, rc=");
          Serial.println(mqttClient.state());
        }
      }
    }

    mqttClient.loop();

    // Publicar datos a Ubidots si viaje activo
    if (mqttClient.connected() && tripActive) {
      unsigned long now = millis();
      if (now - lastPublishMillis >= PUBLISH_INTERVAL_MS) {
        unsigned long turnsCopy;
        portENTER_CRITICAL(&hallMux);
        turnsCopy = wheelTurnCount;
        portEXIT_CRITICAL(&hallMux);

        StaticJsonDocument<128> doc;
        doc["data"] = turnsCopy;

        char buffer[128];
        size_t n = serializeJson(doc, buffer);

        bool ok = mqttClient.publish(TOPIC_DATA, buffer, n);
        Serial.print("Publicando en Ubidots: ");
        Serial.println(buffer);
        if (!ok) {
          Serial.println("Error publicando en MQTT");
        }

        lastPublishMillis = now;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ======================= TAREA: FSM ========================================

void fsmTask(void *pvParameters) {
  Event evt;

  for (;;) {
    if (xQueueReceive(eventQueue, &evt, portMAX_DELAY) == pdTRUE) {
      processEvent(evt);
    }
  }
}

// ======================= WIFI / MQTT AUX ===================================

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.print("Conectando a WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi conectado. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("No se pudo conectar a WiFi");
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("MQTT mensaje en topic: ");
  Serial.print(topic);
  Serial.print(" payload: ");
  Serial.println(msg);

  Event evt;
  evt.value = 0;

  if (strcmp(topic, TOPIC_CONTROL) == 0) {
    if (msg.startsWith("1")) {
      evt.type = EVT_MQTT_POWER_ON;
    } else {
      evt.type = EVT_MQTT_POWER_OFF;
    }
    xQueueSend(eventQueue, &evt, portMAX_DELAY);
  } else if (strcmp(topic, TOPIC_TRIPS) == 0) {
    if (msg.startsWith("1")) {
      evt.type = EVT_TRIP_START;
    } else {
      evt.type = EVT_TRIP_STOP;
    }
    xQueueSend(eventQueue, &evt, portMAX_DELAY);
  }
}

// ======================= ULTRASONIDO / BUZZER / SERVO ======================

float readUltrasonicCM() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duration = pulseIn(PIN_ECHO, HIGH, 25000); // timeout ~25 ms (máx ~4m)
  if (duration == 0) {
    return -1.0f; // sin lectura
  }

  float distance = (duration / 2.0f) * 0.0343f; // velocidad sonido ~343 m/s
  return distance; // cm
}

void setBuzzerFrequency(int freq) {
  if (freq <= 0) {
    ledcWriteTone(BUZZER_CHANNEL, 0);
  } else {
    ledcWriteTone(BUZZER_CHANNEL, freq);
  }
  currentBuzzerFreq = freq;
}

void setBrakeAngle(float angle) {
  if (angle < 0) angle = 0;
  if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
  brakeServo.write((int)angle);
  currentBrakeAngle = angle;
}

void updateSpeedOnLCD(float speed) {
  if (currentState == STATE_OFF) return;

  lcd.setCursor(0, 1);
  lcd.print("                "); // borrar línea
  lcd.setCursor(0, 1);

  char line[17];
  snprintf(line, sizeof(line), "%5.1f km/h", speed);
  lcd.print(line);
}

// ======================= ESTADOS Y EVENTOS =================================

void enterOffState() {
  currentState = STATE_OFF;
  ultrasonicEnabled = false;

  setBuzzerFrequency(0);
  setBrakeAngle(SERVO_RELEASE_ANGLE);
  digitalWrite(PIN_LED, LOW);

  lcd.noDisplay();
  lcd.setRGB(0, 0, 0);

  Serial.println("[FSM] Estado -> OFF");
}

void enterStoppedState() {
  currentState = STATE_STOPPED;
  ultrasonicEnabled = false;

  setBuzzerFrequency(0);
  setBrakeAngle(SERVO_RELEASE_ANGLE);

  lcd.display();
  lcd.setRGB(0, 50, 50);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Velocidad:");
  updateSpeedOnLCD(0.0f);

  Serial.println("[FSM] Estado -> STOPPED");
}

void enterRidingState() {
  currentState = STATE_RIDING;
  ultrasonicEnabled = true;
  setBrakeAngle(SERVO_RELEASE_ANGLE);

  Serial.println("[FSM] Estado -> RIDING");
}

void enterBrakingState() {
  currentState = STATE_BRAKING;
  ultrasonicEnabled = true;

  Serial.println("[FSM] Estado -> BRAKING");
}

// Clasificación de distancia
enum DistanceZone {
  DIST_ZONE_FAR,
  DIST_ZONE_MEDIUM,
  DIST_ZONE_NEAR,
  DIST_ZONE_VERY_NEAR
};

DistanceZone classifyDistance(float cm) {
  if (cm < 0) {
    return DIST_ZONE_FAR;
  }
  if (cm < DIST_VERY_NEAR_CM) {
    return DIST_ZONE_VERY_NEAR;
  } else if (cm >= DIST_NEAR_MIN_CM && cm < DIST_NEAR_MAX_CM) {
    return DIST_ZONE_NEAR;
  } else if (cm >= DIST_MEDIUM_MIN_CM && cm < DIST_MEDIUM_MAX_CM) {
    return DIST_ZONE_MEDIUM;
  } else if (cm >= DIST_FAR_CM) {
    return DIST_ZONE_FAR;
  } else {
    // Entre 10 y 20 ya está contemplado, pero por seguridad:
    if (cm < DIST_NEAR_MIN_CM) return DIST_ZONE_VERY_NEAR;
    return DIST_ZONE_NEAR;
  }
}

int buzzerForZone(DistanceZone z) {
  switch (z) {
    case DIST_ZONE_FAR:        return BUZZER_FREQ_FAR;
    case DIST_ZONE_MEDIUM:     return BUZZER_FREQ_MEDIUM;
    case DIST_ZONE_NEAR:       return BUZZER_FREQ_NEAR;
    case DIST_ZONE_VERY_NEAR:  return BUZZER_FREQ_VERY_NEAR;
  }
  return 0;
}

float brakeAngleForDistance(float cm) {
  if (cm <= 0) return SERVO_RELEASE_ANGLE;
  if (cm >= 20.0f) return SERVO_RELEASE_ANGLE;
  if (cm <= 10.0f) return SERVO_MAX_ANGLE;

  float t = (20.0f - cm) / 10.0f; // 0..1
  float angle = t * SERVO_MAX_ANGLE;
  if (angle < 0) angle = 0;
  if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
  return angle;
}

void processEvent(const Event &evt) {
  // Primero, manejar eventos globales de encendido/apagado y viaje
  switch (evt.type) {
    case EVT_SWITCH_ON:
      physicalOn = true;
      break;
    case EVT_SWITCH_OFF:
      physicalOn = false;
      break;
    case EVT_MQTT_POWER_ON:
      mqttOn = true;
      break;
    case EVT_MQTT_POWER_OFF:
      mqttOn = false;
      break;
    case EVT_TRIP_START:
      tripActive = true;
      Serial.println("[FSM] Viaje -> ACTIVO");
      break;
    case EVT_TRIP_STOP:
      tripActive = false;
      Serial.println("[FSM] Viaje -> INACTIVO");
      break;
    default:
      break;
  }

  bool powerDemand = physicalOn || mqttOn;

  // Transiciones globales OFF/STOPPED según encendido
  if (!powerDemand) {
    if (currentState != STATE_OFF) {
      enterOffState();
    }
  } else {
    if (currentState == STATE_OFF) {
      enterStoppedState();
    }
  }

  // Si está OFF, ignorar otros eventos
  if (currentState == STATE_OFF) {
    return;
  }

  // Manejo por tipo de evento
  switch (evt.type) {
    case EVT_SPEED_UPDATE: {
      float speed = evt.value;
      updateSpeedOnLCD(speed);

      if (currentState == STATE_STOPPED) {
        if (speed > 0.1f) {
          // Detecta movimiento -> RIDING
          enterRidingState();
        }
      }
      // En otros estados solo se actualiza la pantalla
      break;
    }

    case EVT_SPEED_TIMEOUT: {
      // Sin movimiento por 5s -> STOPPED (desde RIDING o BRAKING)
      if (currentState == STATE_RIDING || currentState == STATE_BRAKING) {
        enterStoppedState();
      }
      break;
    }

    case EVT_DISTANCE_UPDATE: {
      float distance = evt.value;
      DistanceZone zone = classifyDistance(distance);

      // Buzzer siempre según distancia cuando hay ultrasonido activo
      int freq = buzzerForZone(zone);
      setBuzzerFrequency(freq);

      if (currentState == STATE_RIDING) {
        // Si está cerca o muy cerca -> BRAKING
        if (zone == DIST_ZONE_NEAR || zone == DIST_ZONE_VERY_NEAR) {
          float angle = brakeAngleForDistance(distance);
          setBrakeAngle(angle);
          enterBrakingState();
        } else {
          // Lejos o medio -> freno liberado
          setBrakeAngle(SERVO_RELEASE_ANGLE);
        }
      } else if (currentState == STATE_BRAKING) {
        // Ajustar freno según distancia
        float angle = brakeAngleForDistance(distance);
        setBrakeAngle(angle);

        // Si vuelve a medio o lejos -> liberar freno y pasar a RIDING
        if (zone == DIST_ZONE_MEDIUM || zone == DIST_ZONE_FAR) {
          setBrakeAngle(SERVO_RELEASE_ANGLE);
          enterRidingState();
        }
      } else {
        // STOPPED: por seguridad mantener todo liberado
        setBrakeAngle(SERVO_RELEASE_ANGLE);
        setBuzzerFrequency(0);
      }
      break;
    }

    default:
      // Otros eventos ya fueron gestionados en la parte global
      break;
  }
}
