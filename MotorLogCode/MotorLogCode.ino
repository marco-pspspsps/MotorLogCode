/// This code is for a Industrial Motor Datalogger made for a ESP32S3
/// Author: Marco Przybysz (marcoprzybysz at protonmail dot com)

#include <driver/pcnt.h>

#include <esp_wifi.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>

#include <LiquidCrystal.h>
#include <HX711.h>
#include <PubSubClient.h>

////////////////////// MCU PORTS ///////////////////////////////////////////////////////////////

#define HX711_DT 45
#define HX711_SCK 48

#define LCD_RS 13
#define LCD_EN 12
#define LCD_D4 7
#define LCD_D5 15
#define LCD_D6 16
#define LCD_D7 17

#define VEL_DIGITAL 40

#define SD_CS 1
#define SD_MOSI 2
#define SD_SCK 42
#define SD_MISO 4

//////////////////// CONFIG VARIABLES ////////////////////////////////////////////////////////

//WIFI
#define EAP_SSID "UTFPR-ALUNO"
#define EAP_IDENTITY "a2088240"
#define EAP_USERNAME "a2088240"
#define EAP_PASSWORD "mszgy28ebnXS58"
const uint8_t target_esp_mac[6] = { 0x24, 0x0a, 0x44, 0x9a, 0x38, 0x28 };  // MAC
#define TIMEOUT_RETRIES 10

//MQTT
#define PUBLISH_TOPIC_POWER "PI2_MotorLog/power"
#define PUBLISH_TOPIC_RPM "PI2_MotorLog/rpm"
#define PUBLISH_TOPIC_TORQUE "PI2_MotorLog/torque"
#define ID_MQTT "PI2_MotorLog_sensor"
const char* BROKER_MQTT = "test.mosquitto.org";
const int BROKER_PORT = 1883;

//LCD
const int LCD_COLLUMNS = 16;
const int LCD_ROWS = 2;
const int LCD_I2C_ADDRESS = 0x27;

//HX711
const float CALIBRATION_FACTOR = 23347.24468;
const float LEVER_LENGTH = 0.080343;

//SPEED
#define PCNT_SPEED_UNIT PCNT_UNIT_0
#define PCNT_H_LIM_VAL INT_MAX
#define PCNT_L_LIM_VAL -10     //some negative values for error margin (should never happen)
#define PCNT_THRESH0_VAL 1750  //nominal rpm of our application
#define PCNT_FILTER_VALUE 1000
#define NUMBER_OF_STRIPS 4

//TIME
const char* ntpServer1 = "0.pool.ntp.org";
const char* ntpServer2 = "1.pool.ntp.org";
const char* ntpServer3 = "time.nist.gov";
const long timezone = -3;
byte daysavetime = 0;

//SD
const char* dataLogFileName = "DadosPI2.txt";

////////////////// TASK DELAYS ////////////////////////////////////////////////////////////////////////////////////

const int MQTT_TASK_DELAY = 4000;
const int LCD_PRINTVARIABLES_TASK_DELAY = 2000;
const int HX711_TASK_DELAY = 2000;
const int RPM_TASK_DELAY = 2000;
const int SD_LOG_DELAY = 10000;

const int HX711_INIT_DELAY = 2000;
const int WIFI_TIMEOUT_DELAY = 500;

/////////////////// SYSTEM VARIABLES ///////////////////////////////////////////////////////////////////////////////
WiFiClient espClient;
PubSubClient MQTT(espClient);

LiquidCrystal LCD(LCD_RS, 
                  LCD_EN, 
                  LCD_D4, 
                  LCD_D5, 
                  LCD_D6, 
                  LCD_D7);

HX711 loadCell;
float loadCellreading;

int16_t speedSensorPulses = 0;

struct tm timeinfo;
File dataLog;

bool wifiConnected;

/// Variables we're interested
float rpm = 0;
float torque = 0;
float powerMultiplied = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(115200);
  SetupLCD();
  SetupLoadCell();
  SetupWifi();

  if (wifiConnected) {
    SetupMQTT();
    SetupTime();
    SetupSD();
  }
  Setup_PCNT();

  CreateTasks();
}


void loop() {
  vTaskDelete(NULL);
}

void CreateTasks() {

  //Tasks prios are: 1 for comms, 3 for calculates, 5 for polls
  xTaskCreate(
    ReadLoadCell,
    "Read Load Cell values",
    20000,
    NULL,
    5,
    NULL);

  xTaskCreate(
    CalculateRPM,
    "Calculate RPM",
    20000,
    NULL,
    3,
    NULL);

  xTaskCreate(
    SerialDebug,
    "Serial Debug",
    20000,
    NULL,
    1,
    NULL);

  xTaskCreate(
    UpdateLCD_Torque_RPMPower,
    "Print Variables in LCD",
    20000,
    NULL,
    1,
    NULL);

  if (wifiConnected) {
    xTaskCreate(
      SendDataMQTT,
      "Send variables via MQTT",
      20000,
      NULL,
      1,
      NULL);

    xTaskCreate(
      WriteVariablesToSD,
      "Save variables to SD",
      20000,
      NULL,
      1,
      NULL);
  }
}

///////////////// SERIAL /////////////////////////////////////////////////////////////////////////////////

void SerialDebug(void* parameter) {

  const int SERIALFREQUENCY = 1000;

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SERIALFREQUENCY));

    Serial.print("RPM: ");
    Serial.println(rpm);

    Serial.print("Torque: ");
    Serial.println(torque);

    Serial.print("Power: ");
    Serial.println(powerMultiplied);

    if (!MQTT.connected()) Serial.println("MQTT Disconnected!");
  }
}

///// WIFI /////////////////////////////

//If timeouts, return 0. Else, return 1.
int SetupWifi() {

  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, target_esp_mac);
  WiFi.begin(EAP_SSID, WPA2_AUTH_PEAP, EAP_IDENTITY, EAP_USERNAME, EAP_PASSWORD);

  Serial.println(EAP_SSID);
  Serial.print("Trying to connect");

  int timeoutCounter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    vTaskDelay(pdMS_TO_TICKS(WIFI_TIMEOUT_DELAY));
    timeoutCounter++;

    if (timeoutCounter >= TIMEOUT_RETRIES) {
      Serial.println("");
      Serial.print("Failed to wifi connect - Offline Mode");
      return 0;
    }
  }

  Serial.println("");
  Serial.print("Connected to local Wifi as IP ");
  Serial.println(WiFi.localIP());
  wifiConnected = true;
  return 1;
}

////////////////// MQTT ////////////////////////////////////

void SetupMQTT(void) {

  MQTT.setServer(BROKER_MQTT, BROKER_PORT);
  MQTT.connect(ID_MQTT);
}

void SendDataMQTT(void* parameter) {

  char messageRpm[50];
  char messageTorque[50];
  char messagePower[50];

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MQTT_TASK_DELAY));

    if (WiFi.status() != WL_CONNECTED) {
      WiFi.disconnect();
      WiFi.reconnect();
    } else {
      if (!MQTT.connected()) {
        MQTT.connect(ID_MQTT);
      } else {
        sprintf(messageRpm, "%f", rpm);
        sprintf(messageTorque, "%f", torque);
        sprintf(messagePower, "%f", powerMultiplied);

        MQTT.publish(PUBLISH_TOPIC_RPM, messageRpm);
        MQTT.publish(PUBLISH_TOPIC_TORQUE, messageTorque);
        MQTT.publish(PUBLISH_TOPIC_POWER, messagePower);

        MQTT.loop();
      }
    }
  }
}

///////////////// SPEED /////////////////////////////////////////////////////////////////////////////////

void Setup_PCNT() {

  pcnt_config_t speedUnitConfig = {

    .pulse_gpio_num = VEL_DIGITAL,
    .ctrl_gpio_num = PCNT_PIN_NOT_USED,
    .pos_mode = PCNT_COUNT_INC,
    .neg_mode = PCNT_COUNT_DIS,
    .counter_h_lim = PCNT_H_LIM_VAL,
    .counter_l_lim = PCNT_L_LIM_VAL,
    .unit = PCNT_SPEED_UNIT,
    .channel = PCNT_CHANNEL_0,
  };

  pcnt_unit_config(&speedUnitConfig);
  pcnt_set_filter_value(PCNT_SPEED_UNIT, PCNT_FILTER_VALUE);
  pcnt_filter_enable(PCNT_SPEED_UNIT);
  pcnt_counter_resume(PCNT_SPEED_UNIT);
}

void CalculateRPM(void* parameter) {

  float adjustToRpmValue = 60000 / (RPM_TASK_DELAY * NUMBER_OF_STRIPS);
  pcnt_counter_clear(PCNT_SPEED_UNIT);

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(RPM_TASK_DELAY));
    pcnt_get_counter_value(PCNT_SPEED_UNIT, &speedSensorPulses);
    rpm = speedSensorPulses * adjustToRpmValue;
    CalculatePowerMultiplied();
    pcnt_counter_clear(PCNT_SPEED_UNIT);
  }
}

////////// POWER /////////////////////////////////////////////////////
// Power has a trick to evade race conditions:
// We update it only when RPM is updated.

void CalculatePowerMultiplied() {

  powerMultiplied = (torque * rpm) / 9.5488;  //resultado em W
}
/////////////// LCD /////////////////////////////////////////////////

void SetupLCD() {

  LCD.begin(LCD_COLLUMNS, LCD_ROWS);
  LCD.clear();
}

void UpdateLCD_Torque_RPMPower(void* parameter) {

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(LCD_PRINTVARIABLES_TASK_DELAY));
    LCD.clear();

    LCD.setCursor(0, 0);
    LCD.print(torque, 4);
    LCD.print(" N.m");

    LCD.setCursor(0, 1);
    LCD.print(rpm, 0);
    LCD.print(" RPM");
    LCD.print(powerMultiplied, 0);
    LCD.print(" W");
  }
}

///////// LOAD CELL ///////////////////////////////////////////

void SetupLoadCell() {

  loadCell.begin(HX711_DT, HX711_SCK);
  vTaskDelay(pdMS_TO_TICKS(HX711_INIT_DELAY));
  loadCell.set_scale(CALIBRATION_FACTOR);
  loadCell.tare(10);
}

void ReadLoadCell(void* parameter) {

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(HX711_TASK_DELAY));
    loadCellreading = loadCell.get_units();
    torque = loadCellreading * LEVER_LENGTH;
  }
}
/////////////// TIME //////////////////////////////////////////////

void SetupTime() {

  configTime(3600 * timezone, 3600 * daysavetime, ntpServer1, ntpServer2, ntpServer3);
  getLocalTime(&timeinfo);
  Serial.print("Time connected: ");
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

/////////////// SD ///////////////////////////////////////////////

void SetupSD() {

  SPI.begin(SD_CS, SD_MOSI, SD_MISO, SD_SCK);

  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }

  dataLog = SD.open(dataLogFileName, FILE_WRITE);
  if (!SD.exists(dataLogFileName)) dataLog.println("Data Log de Projeto Integrador 2 - RPM, N.m, W");
  dataLog.close();
}

void WriteVariablesToSD(void* parameter) {

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SD_LOG_DELAY));

    dataLog = SD.open(dataLogFileName, FILE_WRITE);

    getLocalTime(&timeinfo);
    dataLog.print(asctime(&timeinfo));

    dataLog.print(" - ");
    dataLog.print(rpm);
    dataLog.print(",");
    dataLog.print(torque);
    dataLog.print(",");
    dataLog.print(powerMultiplied);

    dataLog.close();
  }
}