#include "PubSubClient.h"
#include "rgb_lcd.h"
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <Wire.h>

#define LOG // Comment this line to turn off logs

static void task_sensors(void *pv);
static void task_switch(void *pv);
static void task_wifi(void *pv);
static void enable_sensors();
static void disable_sensors();

// ------------------------------------------------
// Constants
// ------------------------------------------------
#define SPEED_OF_SOUND 0.01723
#define NUM_SENSORS 3
#define WHEEL_DIAMETER 0.71 // meters
#define PI 3.14159265359
#define MICROS_IN_SEC 1000000.0
#define MS_TO_KMH 3.6
#define ULTRASONIC_TIMEOUT_US 30000
#define MIN_ANGLE 0
#define MAX_ANGLE 180
#define TMP_EVENTS_MS 50
#define BUZZER_RES_BITS 10
#define BUZZER_START_HZ 2000
#define BUZZER_CHANNEL 0
#define PWM_TIMER_FOR_BUZZER 0
#define PWM_CHANNELS_PER_TIMER 4
#define SERVO_HZ 50
#define HALL_EFFECT_DEBOUNCE_US 250000
#define SPEED_TIMEOUT_US 5000000
#define LCD_I2C_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2400
#define SERVO_MIN_ANGLE 0
#define QUEUE_SIZE_EVENTS 20
#define QUEUE_SIZE_TURNS 20
#define TASK_STACK_SIZE 4096
#define TASK_PRIORITY_SENSORS 4
#define TASK_PRIORITY_SWITCH 3
#define TASK_PRIORITY_WIFI 2
#define TASK_PRIORITY_FSM 1
#define SERIAL_BAUD_RATE 115200
#define LCD_CURSOR_COL_0 0
#define LCD_CURSOR_ROW_0 0
#define LCD_CURSOR_ROW_1 1
#define DELAY_MICROSECONDS_2 2
#define DELAY_MICROSECONDS_10 10
#define INIT_SPEED 0.0
#define QUEUE_NO_WAIT 0
#define SENSOR_INIT_INDEX 0
#define TRUE 1
#define RED 255
#define NO_RED 0
#define GREEN 255
#define NO_GREEN 0
#define BLUE 255
#define NO_BLUE 0
#define TMP_WIFI_CHECK 5000
#define WIFI_SSID "SSID"
#define WIFI_PASSWORD "PASSWORD"
#define MQTT_SERVER "industrial.api.ubidots.com"
#define PORT 1883
#define USER_NAME "BBUS-HdFdBXCCMsjGsKwFNnLh7Y7vzLTasv"
#define USER_PASS "BBUS-HdFdBXCCMsjGsKwFNnLh7Y7vzLTasv"
#define TOPIC_CONTROL "/v1.6/devices/bici/control/lv"
#define TOPIC_TRIPS "/v1.6/devices/bici/trips/lv"
#define TOPIC_DATA "/v1.6/devices/bici"
#define JSON_DOC_SIZE 512
#define TMP_WIFI_PUBLISH 3000
#define MSG_ON "1.0"
#define MSG_OFF "0.0"
#define MSG_START "1.0"
#define MSG_END "0.0"

// ------------------------------------------------
// Sensor pins (A = Analog | D = Digital)
// ------------------------------------------------
#define PIN_D_ULTRASONIC_TRIG 5
#define PIN_D_ULTRASONIC_ECHO 18
#define PIN_D_HALL_EFFECT 4
#define PIN_A_LDR 32
#define PIN_D_SWITCH 26

// ------------------------------------------------
// Actuator pins (P = PWM | D = Digital)
// ------------------------------------------------
#define PIN_P_SERVO 23
#define PIN_D_LED 12
#define PIN_P_BUZZER 19

// ------------------------------------------------
// Distance thresholds
// ------------------------------------------------
#define THRESHOLD_MEDIUM 30.0
#define THRESHOLD_CLOSE 20.0
#define THRESHOLD_TOO_CLOSE 10.0

// ------------------------------------------------
// Light thresholds
// ------------------------------------------------
#define THRESHOLD_LIGHT_LOW 1500
#define THRESHOLD_LIGHT_HIGH 2000

// ------------------------------------------------
// Buzzer frequencies
// ------------------------------------------------
#define FREQUENCY_TOO_CLOSE 2000
#define FREQUENCY_CLOSE 1000
#define FREQUENCY_MEDIUM 500
#define FREQUENCY_OFF 0

// ------------------------------------------------
// States
// ------------------------------------------------
enum state_e
{
  ST_OFF,
  ST_STOPPED,
  ST_RIDING,
  ST_BRAKING
};

// ------------------------------------------------
// Events
// ------------------------------------------------
enum event_e
{
  EV_LOW_LIGHTING,
  EV_HIGH_LIGHTING,
  EV_WHEEL_TURN,
  EV_FAR_DISTANCE,
  EV_MEDIUM_DISTANCE,
  EV_CLOSE_DISTANCE,
  EV_TOO_CLOSE_DISTANCE,
  EV_TIMEOUT,
  EV_TURN_ON,
  EV_TURN_OFF,
  EV_CONTINUE
};

// ------------------------------------------------
// States for sensors
// ------------------------------------------------
enum light_state_e
{
  LIGHT_HIGH,
  LIGHT_LOW
};

enum distance_state_e
{
  DIST_FAR,
  DIST_MEDIUM,
  DIST_CLOSE,
  DIST_TOO_CLOSE
};

enum switch_state_e
{
  SWITCH_OFF,
  SWITCH_ON
};

// ------------------------------------------------
// Event struct
// ------------------------------------------------
typedef struct event_s
{
  event_e type;
} event_t;

// ------------------------------------------------
// Sensor structs
// ------------------------------------------------
typedef struct distance_sensor_s
{
  int pin_echo;
  int pin_trigger;
  float cm;
  distance_state_e state;
} distance_sensor_t;

typedef struct light_sensor_s
{
  int pin;
  int level;
  light_state_e state;
} light_sensor_t;

typedef struct switch_sensor_s
{
  int pin;
  int value;
  switch_state_e state;
} switch_sensor_t;

// ------------------------------------------------
// Actuator structs
// ------------------------------------------------
typedef struct led_actuator_s
{
  int pin;
  bool state;
} led_actuator_t;

typedef struct buzzer_actuator_s
{
  int pin;
  int frequency;
} buzzer_actuator_t;

typedef Servo servo_actuator_t;

// ------------------------------------------------
// Global variables
// ------------------------------------------------
state_e current_state;
event_t event;
distance_sensor_t distance_sensor;
switch_sensor_t switch_sensor;
light_sensor_t light_sensor;
led_actuator_t led_actuator;
buzzer_actuator_t buzzer_actuator;
servo_actuator_t servo_actuator;
rgb_lcd lcd_actuator;
unsigned long last_interrupt_time;
unsigned long current_turn;
unsigned long last_turn;
float speed;
QueueHandle_t events_queue;
QueueHandle_t turns_queue;
static TaskHandle_t h_task_switch;
static TaskHandle_t h_task_sensors;
static TaskHandle_t h_task_wifi;
SemaphoreHandle_t mutex;
unsigned int wheel_turn_count;
WiFiClient esp_client;
PubSubClient client(esp_client);
unsigned long last_wifi_check;

// ------------------------------------------------
// Logs
// ------------------------------------------------
void console_log(const char *state, const char *event)
{
#ifdef LOG
  Serial.println("------------------------------------------------");
  Serial.println(state);
  Serial.println(event);
  Serial.println("------------------------------------------------");
#endif
}

void console_log(const char *msg)
{
#ifdef LOG
  Serial.println(msg);
#endif
}

void console_log(String msg)
{
#ifdef LOG
  Serial.println(msg);
#endif
}

void console_log(int val)
{
#ifdef LOG
  Serial.println(val);
#endif
}

void console_log(float val_float)
{
#ifdef LOG
  Serial.println(val_float);
#endif
}

// ------------------------------------------------
// Sensors logic
// ------------------------------------------------
float read_distance_sensor()
{
  long pulse_time;
  digitalWrite(PIN_D_ULTRASONIC_TRIG, LOW);
  delayMicroseconds(DELAY_MICROSECONDS_2);
  digitalWrite(PIN_D_ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(DELAY_MICROSECONDS_10);
  digitalWrite(PIN_D_ULTRASONIC_TRIG, LOW);
  pulse_time = pulseIn(PIN_D_ULTRASONIC_ECHO, HIGH, ULTRASONIC_TIMEOUT_US);

  // convert to cm
  return pulse_time * SPEED_OF_SOUND;
}

event_t
check_distance_sensor()
{
  event_t new_event;
  if (current_state == ST_STOPPED)
  {
    new_event.type = EV_CONTINUE;
    return new_event;
  }

  distance_sensor.cm = read_distance_sensor();
  distance_state_e new_state;

  if (distance_sensor.cm < THRESHOLD_TOO_CLOSE)
    new_state = DIST_TOO_CLOSE;
  else if (distance_sensor.cm < THRESHOLD_CLOSE)
    new_state = DIST_CLOSE;
  else if (distance_sensor.cm < THRESHOLD_MEDIUM)
    new_state = DIST_MEDIUM;
  else
    new_state = DIST_FAR;

  switch (new_state)
  {
  case DIST_TOO_CLOSE:
    if (distance_sensor.state != DIST_TOO_CLOSE)
      new_event.type = EV_TOO_CLOSE_DISTANCE;
    else
      new_event.type = EV_CONTINUE;
    break;

  case DIST_CLOSE:
    new_event.type = EV_CLOSE_DISTANCE;
    break;

  case DIST_MEDIUM:
    if (distance_sensor.state != DIST_MEDIUM)
      new_event.type = EV_MEDIUM_DISTANCE;
    else
      new_event.type = EV_CONTINUE;
    break;

  case DIST_FAR:
    if (distance_sensor.state != DIST_FAR)
      new_event.type = EV_FAR_DISTANCE;
    else
      new_event.type = EV_CONTINUE;
    break;
  }

  distance_sensor.state = new_state;
  return new_event;
}

int read_light_sensor(int pin)
{
  return analogRead(pin);
}

event_t
check_light_sensor()
{
  light_sensor.level = read_light_sensor(light_sensor.pin);
  light_state_e new_state;
  event_t new_event;

  if (light_sensor.level < THRESHOLD_LIGHT_LOW)
    new_state = LIGHT_LOW;
  else if (light_sensor.level > THRESHOLD_LIGHT_HIGH)
    new_state = LIGHT_HIGH;
  else
    new_state = light_sensor.state;

  switch (new_state)
  {
  case LIGHT_LOW:
    if (light_sensor.state != LIGHT_LOW)
      new_event.type = EV_LOW_LIGHTING;
    else
      new_event.type = EV_CONTINUE;
    break;

  case LIGHT_HIGH:
    if (light_sensor.state != LIGHT_HIGH)
      new_event.type = EV_HIGH_LIGHTING;
    else
      new_event.type = EV_CONTINUE;
    break;
  }

  light_sensor.state = new_state;
  return new_event;
}

void IRAM_ATTR
read_halleffect_sensor()
{
  unsigned long interrupt_time = micros();
  if (interrupt_time - last_interrupt_time > HALL_EFFECT_DEBOUNCE_US)
  {
    last_interrupt_time = interrupt_time;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(turns_queue, &interrupt_time, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    {
      portYIELD_FROM_ISR();
    }
  }
}

event_t
check_halleffect_sensor()
{
  unsigned long current_time;
  event_t new_event;
  if (xQueueReceive(turns_queue, &current_time, QUEUE_NO_WAIT) == pdTRUE)
  {
    last_turn = current_turn;
    current_turn = current_time;
    if (current_turn > last_turn)
    {
      float distanceTraveled = PI * WHEEL_DIAMETER;
      speed = (distanceTraveled / ((current_turn - last_turn) / MICROS_IN_SEC)) * MS_TO_KMH;
    }
    else
    {
      speed = INIT_SPEED;
    }
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
    {
      wheel_turn_count++;
      xSemaphoreGive(mutex);
    }
    new_event.type = EV_WHEEL_TURN;
    return new_event;
  }

  current_time = micros();
  if (current_time - current_turn > SPEED_TIMEOUT_US)
  {
    speed = INIT_SPEED;
    last_turn = current_turn;
    current_turn = current_time;
    new_event.type = EV_TIMEOUT;
    return new_event;
  }
  new_event.type = EV_CONTINUE;
  return new_event;
}

int read_switch_sensor(int pin)
{
  return digitalRead(pin);
}

event_t
check_switch_sensor()
{
  switch_sensor.value = read_switch_sensor(switch_sensor.pin);
  event_t new_event;
  switch_state_e new_state;
  if (switch_sensor.value == LOW)
    new_state = SWITCH_ON;
  else
    new_state = SWITCH_OFF;

  switch (new_state)
  {
  case SWITCH_OFF:
    if (switch_sensor.state != SWITCH_OFF)
      new_event.type = EV_TURN_OFF;
    else
      new_event.type = EV_CONTINUE;
    break;

  case SWITCH_ON:
    if (switch_sensor.state != SWITCH_ON)
      new_event.type = EV_TURN_ON;
    else
      new_event.type = EV_CONTINUE;
    break;
  }

  switch_sensor.state = new_state;
  return new_event;
}

// ------------------------------------------------
// Actuators logic
// ------------------------------------------------

void turn_on_screen()
{
  lcd_actuator.begin(LCD_COLUMNS, LCD_ROWS);
  lcd_actuator.setRGB(RED, GREEN, BLUE);
  lcd_actuator.clear();
  lcd_actuator.setCursor(LCD_CURSOR_COL_0, LCD_CURSOR_ROW_0);
  lcd_actuator.print("Velocidad:");
  lcd_actuator.setCursor(LCD_CURSOR_COL_0, LCD_CURSOR_ROW_1);
  lcd_actuator.print("  0.0 km/h");
}

void turn_off_screen()
{
  lcd_actuator.clear();
  lcd_actuator.setRGB(NO_RED, NO_GREEN, NO_BLUE);
}

void update_screen()
{
  lcd_actuator.setCursor(LCD_CURSOR_COL_0, LCD_CURSOR_ROW_1);
  char buffer[LCD_COLUMNS];
  snprintf(buffer, sizeof(buffer), "%5.1f km/h", speed);
  lcd_actuator.print(buffer);
}

void play_buzzer(int frequency)
{
  buzzer_actuator.frequency = frequency;
  ledcWriteTone(PIN_P_BUZZER, frequency);
}

void stop_buzzer()
{
  buzzer_actuator.frequency = FREQUENCY_OFF;
  ledcWriteTone(PIN_P_BUZZER, FREQUENCY_OFF);
}

void turn_on_led()
{
  led_actuator.state = HIGH;
  digitalWrite(PIN_D_LED, HIGH);
}

void turn_off_led()
{
  led_actuator.state = LOW;
  digitalWrite(PIN_D_LED, LOW);
}

void activate_brakes(float distance)
{
  int angle = get_angle((int)distance, THRESHOLD_CLOSE, THRESHOLD_TOO_CLOSE,
                        MIN_ANGLE, MAX_ANGLE);
  servo_actuator.write(angle);
}

void deactivate_brakes()
{
  servo_actuator.write(MIN_ANGLE);
}

int get_angle(float distance, float max_distance, float min_distance,
              int min_angle, int max_angle)
{
  if (distance <= min_distance)
    return max_angle;

  float ratio = (max_distance - distance) / (max_distance - min_distance);
  int angle = min_angle + (max_angle - min_angle) * ratio;
  return angle;
}

void sync_actuators_with_sensors()
{
  light_sensor.state = LIGHT_HIGH;
  distance_sensor.state = DIST_FAR;
}

// ------------------------------------------------
// Event detection
// ------------------------------------------------
int sensor_index = SENSOR_INIT_INDEX;
event_t (*check_sensor[NUM_SENSORS])() = {check_distance_sensor, check_light_sensor, check_halleffect_sensor};

static void
task_sensors(void *pv) // Disabled when ST_OFF
{
  while (TRUE)
  {
    event_t new_event = check_sensor[sensor_index]();

    if (new_event.type != EV_CONTINUE)
      xQueueSend(events_queue, &new_event.type, QUEUE_NO_WAIT);

    sensor_index = ++sensor_index % NUM_SENSORS;
    vTaskDelay(pdMS_TO_TICKS(TMP_EVENTS_MS));
  }
}

static void
task_switch(void *pv) // Always enabled
{
  while (TRUE)
  {
    event_t new_event = check_switch_sensor();
    if (new_event.type != EV_CONTINUE)
      xQueueSend(events_queue, &new_event.type, QUEUE_NO_WAIT);
    vTaskDelay(pdMS_TO_TICKS(TMP_EVENTS_MS));
  }
}

static inline void
enable_sensors()
{
  if (h_task_sensors)
  {
    vTaskResume(h_task_sensors);
    gpio_intr_enable((gpio_num_t)PIN_D_HALL_EFFECT);
  }
}

static inline void
disable_sensors()
{
  if (h_task_sensors)
  {
    gpio_intr_disable((gpio_num_t)PIN_D_HALL_EFFECT);
    vTaskSuspend(h_task_sensors);
  }
}

// ------------------------------------------------
// Wifi and MQTT
// ------------------------------------------------
void wifi_connect()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
}

void mqtt_reconnect()
{
  char client_id[50];
  console_log("Attempting MQTT connection...");
  long r = random(1000);
  sprintf(client_id, "clientId-%ld", r);

  if (client.connect(client_id, USER_NAME, USER_PASS))
  {
    console_log(client_id);
    console_log(" connected");
    if (client.subscribe(TOPIC_CONTROL))
    {
      console_log("Subscribed to CONTROL OK");
    }
    else
    {
      console_log("ERROR subscribing to CONTROL");
    }

    if (client.subscribe(TOPIC_TRIPS))
    {
      console_log("Subscribed to TRIPS OK");
    }
    else
    {
      console_log("ERROR subscribing to TRIPS");
    }
  }
  else
  {
    console_log("Failed, rc=");
    console_log(client.state());
  }
}

void check_wifi_connection()
{
  if (!client.connected())
  {
    mqtt_reconnect();
  }
}

String
generate_json(unsigned int value)
{
  StaticJsonDocument<JSON_DOC_SIZE> doc;
  String json;
  doc["data"] = value;
  serializeJson(doc, json);
  return json;
}

static void
task_wifi(void *pv)
{
  while (TRUE)
  {
    unsigned int local_wheel_turn_count;
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
    {
      local_wheel_turn_count = wheel_turn_count;
      xSemaphoreGive(mutex);
    }
    String json = generate_json(local_wheel_turn_count);
    char msg_to_send[json.length() + 1];
    json.toCharArray(msg_to_send, sizeof(msg_to_send));

    client.publish(TOPIC_DATA, msg_to_send);
    vTaskDelay(pdMS_TO_TICKS(TMP_WIFI_PUBLISH));
  }
}

void callback(char *topic, byte *message, unsigned int length)
{
  console_log("Message arrived on topic: ");
  console_log(topic);
  console_log("Message: ");
  String message_string;

  for (int i = 0; i < length; i++)
  {
    message_string += (char)message[i];
  }
  console_log(message_string);

  if (String(topic) == TOPIC_CONTROL)
  {
    event_t new_event;
    if (message_string == MSG_ON)
    {
      new_event.type = EV_TURN_ON;
      xQueueSend(events_queue, &new_event.type, QUEUE_NO_WAIT);
    }
    else if (message_string == MSG_OFF)
    {
      new_event.type = EV_TURN_OFF;
      xQueueSend(events_queue, &new_event.type, QUEUE_NO_WAIT);
    }
  }
  else if (String(topic) == TOPIC_TRIPS)
  {
    if (message_string == MSG_START)
    {
      console_log("Starting trip...");
      if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
      {
        wheel_turn_count = 0;
        xSemaphoreGive(mutex);
      }
      vTaskResume(h_task_wifi);
    }
    else if (message_string == MSG_END)
    {
      console_log("Ending trip...");
      vTaskSuspend(h_task_wifi);
    }
  }
}

// ------------------------------------------------
// Initialization
// ------------------------------------------------
void start()
{
  Serial.begin(SERIAL_BAUD_RATE);

  events_queue = xQueueCreate(QUEUE_SIZE_EVENTS, sizeof(event_e));
  turns_queue = xQueueCreate(QUEUE_SIZE_TURNS, sizeof(unsigned long));
  mutex = xSemaphoreCreateMutex();

  pinMode(PIN_D_SWITCH, INPUT_PULLUP);
  switch_sensor.pin = PIN_D_SWITCH;
  switch_sensor.state = SWITCH_OFF;

  pinMode(PIN_D_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_D_ULTRASONIC_ECHO, INPUT);
  distance_sensor.pin_echo = PIN_D_ULTRASONIC_ECHO;
  distance_sensor.pin_trigger = PIN_D_ULTRASONIC_TRIG;
  distance_sensor.state = DIST_FAR;

  pinMode(PIN_D_HALL_EFFECT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_D_HALL_EFFECT),
                  read_halleffect_sensor, RISING);

  pinMode(PIN_D_LED, OUTPUT);
  led_actuator.pin = PIN_D_LED;

  pinMode(PIN_A_LDR, INPUT);
  light_sensor.pin = PIN_A_LDR;
  light_sensor.state = LIGHT_HIGH;

  buzzer_actuator.pin = PIN_P_BUZZER;
  buzzer_actuator.frequency = FREQUENCY_OFF;
  ESP32PWM::timerCount[PWM_TIMER_FOR_BUZZER] = PWM_CHANNELS_PER_TIMER;
  ledcAttachChannel(PIN_P_BUZZER, BUZZER_START_HZ, BUZZER_RES_BITS, BUZZER_CHANNEL);

  servo_actuator.setPeriodHertz(SERVO_HZ);
  servo_actuator.attach(PIN_P_SERVO, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  servo_actuator.write(SERVO_MIN_ANGLE);

  lcd_actuator.begin(LCD_COLUMNS, LCD_ROWS);
  lcd_actuator.setRGB(RED, GREEN, BLUE);

  current_state = ST_OFF;

  current_turn = micros();
  last_turn = current_turn;
  last_interrupt_time = current_turn;
  speed = INIT_SPEED;

  console_log("Connecting to WiFi...");
  wifi_connect();
  console_log("Connected to WiFi");
  client.setServer(MQTT_SERVER, PORT);
  client.setCallback(callback);
  last_wifi_check = millis();
  mqtt_reconnect();

  h_task_sensors = nullptr;
  h_task_switch = nullptr;
  h_task_wifi = nullptr;

  xTaskCreate(task_sensors, "Task Sensors", TASK_STACK_SIZE, NULL, TASK_PRIORITY_SENSORS, &h_task_sensors);
  xTaskCreate(task_switch, "Task Switch", TASK_STACK_SIZE, NULL, TASK_PRIORITY_SWITCH, &h_task_switch);
  xTaskCreate(task_wifi, "Task Wifi", TASK_STACK_SIZE, NULL, TASK_PRIORITY_WIFI, &h_task_wifi);
  vTaskSuspend(h_task_wifi);

  if (h_task_sensors)
    disable_sensors();

  xTaskCreate(task_loop, "Task Loop", TASK_STACK_SIZE, NULL, TASK_PRIORITY_FSM, NULL);
}

// ------------------------------------------------
// Finite State Machine
// ------------------------------------------------
void fsm()
{
  switch (current_state)
  {
  case ST_OFF:
    switch (event.type)
    {
    case EV_TURN_ON:
      turn_on_screen();
      sync_actuators_with_sensors();
      console_log("ST_OFF", "EV_TURN_ON");
      current_state = ST_STOPPED;
      enable_sensors();
      break;

    default:
      break;
    }
    break;

  case ST_STOPPED:
    switch (event.type)
    {
    case EV_LOW_LIGHTING:
      turn_on_led();
      console_log("ST_STOPPED", "EV_LOW_LIGHTING");
      break;

    case EV_HIGH_LIGHTING:
      turn_off_led();
      console_log("ST_STOPPED", "EV_HIGH_LIGHTING");
      break;

    case EV_WHEEL_TURN:
      update_screen();
      console_log("ST_STOPPED", "EV_WHEEL_TURN");
      current_state = ST_RIDING;
      break;

    case EV_TURN_OFF:
      console_log("ST_STOPPED", "EV_TURN_OFF");
      disable_sensors();
      deactivate_brakes();
      stop_buzzer();
      turn_off_led();
      turn_off_screen();
      current_state = ST_OFF;
      break;

    default:
      break;
    }
    break;

  case ST_RIDING:
    switch (event.type)
    {
    case EV_LOW_LIGHTING:
      turn_on_led();
      console_log("ST_RIDING", "EV_LOW_LIGHTING");
      break;

    case EV_HIGH_LIGHTING:
      turn_off_led();
      console_log("ST_RIDING", "EV_HIGH_LIGHTING");
      break;

    case EV_WHEEL_TURN:
      update_screen();
      console_log("ST_RIDING", "EV_WHEEL_TURN");
      break;

    case EV_FAR_DISTANCE:
      deactivate_brakes();
      stop_buzzer();
      console_log("ST_RIDING", "EV_FAR_DISTANCE");
      break;

    case EV_MEDIUM_DISTANCE:
      deactivate_brakes();
      play_buzzer(FREQUENCY_MEDIUM);
      console_log("ST_RIDING", "EV_MEDIUM_DISTANCE");
      break;

    case EV_CLOSE_DISTANCE:
      activate_brakes(distance_sensor.cm);
      play_buzzer(FREQUENCY_CLOSE);
      console_log("ST_RIDING", "EV_CLOSE_DISTANCE");
      current_state = ST_BRAKING;
      break;

    case EV_TOO_CLOSE_DISTANCE:
      activate_brakes(distance_sensor.cm);
      play_buzzer(FREQUENCY_TOO_CLOSE);
      console_log("ST_RIDING", "EV_TOO_CLOSE_DISTANCE");
      current_state = ST_BRAKING;
      break;

    case EV_TIMEOUT:
      deactivate_brakes();
      stop_buzzer();
      update_screen();
      sync_actuators_with_sensors();
      console_log("ST_RIDING", "EV_TIMEOUT");
      current_state = ST_STOPPED;
      break;

    case EV_TURN_OFF:
      console_log("ST_RIDING", "EV_TURN_OFF");
      disable_sensors();
      deactivate_brakes();
      stop_buzzer();
      turn_off_led();
      turn_off_screen();
      current_state = ST_OFF;
      break;

    default:
      break;
    }
    break;

  case ST_BRAKING:
    switch (event.type)
    {
    case EV_LOW_LIGHTING:
      turn_on_led();
      console_log("ST_BRAKING", "EV_LOW_LIGHTING");
      break;

    case EV_HIGH_LIGHTING:
      turn_off_led();
      console_log("ST_BRAKING", "EV_HIGH_LIGHTING");
      break;

    case EV_WHEEL_TURN:
      update_screen();
      console_log("ST_BRAKING", "EV_WHEEL_TURN");
      break;

    case EV_FAR_DISTANCE:
      deactivate_brakes();
      stop_buzzer();
      console_log("ST_BRAKING", "EV_FAR_DISTANCE");
      current_state = ST_RIDING;
      break;

    case EV_MEDIUM_DISTANCE:
      deactivate_brakes();
      play_buzzer(FREQUENCY_MEDIUM);
      console_log("ST_BRAKING", "EV_MEDIUM_DISTANCE");
      current_state = ST_RIDING;
      break;

    case EV_CLOSE_DISTANCE:
      activate_brakes(distance_sensor.cm);
      play_buzzer(FREQUENCY_CLOSE);
      console_log("ST_BRAKING", "EV_CLOSE_DISTANCE");
      break;

    case EV_TOO_CLOSE_DISTANCE:
      activate_brakes(distance_sensor.cm);
      play_buzzer(FREQUENCY_TOO_CLOSE);
      console_log("ST_BRAKING", "EV_TOO_CLOSE_DISTANCE");
      break;

    case EV_TIMEOUT:
      deactivate_brakes();
      stop_buzzer();
      update_screen();
      sync_actuators_with_sensors();
      console_log("ST_BRAKING", "EV_TIMEOUT");
      current_state = ST_STOPPED;
      break;

    case EV_TURN_OFF:
      console_log("ST_BRAKING", "EV_TURN_OFF");
      disable_sensors();
      deactivate_brakes();
      stop_buzzer();
      turn_off_led();
      turn_off_screen();
      current_state = ST_OFF;
      break;

    default:
      break;
    }
    break;
  }
}

void task_loop(void *pv)
{
  while (TRUE)
  {
    if (xQueueReceive(events_queue, &event.type, pdMS_TO_TICKS(TMP_EVENTS_MS)) == pdPASS)
    {
      fsm();
    }
    unsigned long now = millis();
    if (now - last_wifi_check >= TMP_WIFI_CHECK)
    {
      check_wifi_connection();
      last_wifi_check = now;
    }
    client.loop();
    taskYIELD();
  }
}

void setup()
{
  start();
}

void loop()
{
}