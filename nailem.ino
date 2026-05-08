/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : nailem.ino
  * @brief          : Source Code ESP32-S3 Nailem
  ******************************************************************************
  * @attention
  *
  * Projek AI Aquascape Nilem, source code ESP32-S3
  * Written By Abie
  * Anggota: Abie, Akbar, Nabil, Adzka, Kirana Shifa
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include "time.h" 
#include <Adafruit_NeoPixel.h> // https://learn.adafruit.com/adafruit-neopixel-uberguide/arduino-library-use
#include <LiquidCrystal_I2C.h> // https://randomnerdtutorials.com/esp32-esp8266-i2c-lcd-arduino-ide/
#include <DallasTemperature.h> // https://randomnerdtutorials.com/esp32-ds18b20-temperature-arduino-ide/
#include <Adafruit_TSL2561_U.h>// https://learn.adafruit.com/tsl2561/arduino-code


/* USER CODE END Includes */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Defines */
#define __NUM_ESP_LED 1
#define __NUM_STATUS_LED 58
#define __ADDR_PRI_LCD 0x27

/* USER CODE END Defines */
/* USER CODE BEGIN Pinouts */
// ===================== ESP / SYSTEM =====================
#define __PIN_ESP_LED        48

// ===================== I2C BUS ==========================
#define __PIN_I2C_SCL        9
#define __PIN_I2C_SDA        8

// ===================== ANALOG SENSORS ===================
#define __PIN_PHS_SENSOR      4
#define __PIN_TDS_SENSOR     5
#define __PIN_TRB_SENSOR     6

// ===================== DIGITAL SENSORS ==================
#define __PIN_DS18B20        7
#define __PIN_US_TRIG        15
#define __PIN_US_ECHO        16

// ===================== MOSFET OUTPUTS ===================
#define __PIN_MOSFET_1       42
#define __PIN_MOSFET_2       41
#define __PIN_MOSFET_3       40
#define __PIN_MOSFET_4       39

// ===================== RELAY OUTPUTS ====================
#define __PIN_RELAY_1        42
#define __PIN_RELAY_2        41
#define __PIN_RELAY_3        40
#define __PIN_RELAY_4        39

// ===================== STEPPER DRIVER ===================
#define __PIN_STEPPER_STEP   12
#define __PIN_STEPPER_DIR    13
#define __PIN_STEPPER_EN     14  

// ==================== STEPPER ENDSTOPS ==================

#define __PIN_RIGHT_STP      TBA
#define __PIN_LEFT_STP       TBA

// ===================== EXTRA OUTPUTS ====================
#define __PIN_LED_STRIP      17
#define __PIN_SERVO          18

// ===================== UART (JETSON BACKUP) =============
#define __PIN_UART_TX        TBA
#define __PIN_UART_RX        TBA

/* ISER CODE END Pinouts*/

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile uint8_t ESP_STATUS = 0;
uint8_t ESP_STATUS_BRIGHTNESS = 75; // In percentages
// 0 = STBY, 1 = BUSY, 2 = WARN, 3 = ERR

bool CDC_JETSON_ACK = false;
uint8_t PRI_LCD_MENU = 0;

char CDC_RX_BUFFER[128];
char JSON_TX_BUFFER[512];
uint8_t CDC_RX_INDEX;

char EXT_LCD_BUF[20];
uint16_t ESP_HEAP;
bool TM_SWITCH;

uint16_t SNS_CAP_DELAY = 100;
uint8_t TM_SW_INTERVAL = 5;

#define RELAY_OFF HIGH
#define RELAY_ON LOW

const char* ssid = "Techno_MTsLvl7_4G";
const char* pass = "sifatmuhammad";
// Sensor values
struct STRUCT_SENSOR {
  uint16_t SR_LUX; // Lux or brightness

  float    SR_PHS; // pH sense
  uint16_t SR_TDS; // TDS sense
  uint16_t SR_TRB; // Turbidity sense

  float SR_TM1; // Temperature 1
  float SR_TM2; // Temperature 2
  float SR_TM3; // Temperature 3
  float SR_TM4; // Temperature 4

  float SR_TMA; // Average Temp

  uint16_t SR_ULT; // Ultrasonic (water height)

  uint16_t SR_LST; // Left endstop
  uint16_t SR_RST; // Right endstop
};

STRUCT_SENSOR _SN;

struct STRUCT_STATES {
  bool ST_RL1;
  bool ST_RL2;
  bool ST_RL3;
  bool ST_RL4;

  uint16_t PWM_MS1;
  uint16_t PWM_MS2;
  uint16_t PWM_MS3;
  uint16_t PWM_MS4;
};

STRUCT_STATES _ST;

struct STRUCT_TIME {
  uint8_t sec  = 0;
  uint8_t min  = 0;
  uint8_t hour = 0;
  uint8_t day  = 0;
  uint8_t mon  = 0;
  uint8_t year = 0;
};

STRUCT_TIME _TM;

struct STRUCT_ERR_FLAGS {
  bool ERR_TM1 = false;
  bool ERR_TM2 = false;
  bool ERR_TM3 = false;
  bool ERR_TM4 = false;
  bool ERR_TMP = false;

  bool ERR_TSL = false;

  bool ERR_WIFI = true;

  bool WATER_CHANGE_ONGOING = false;
  bool TEMP_ABNORMAL = false;
};

STRUCT_ERR_FLAGS _ERR;

struct STRUCT_THRESHOLDS {
  uint16_t TMP_HIGH = 28;
  uint16_t TMP_LOW  = 26;
};

STRUCT_THRESHOLDS _TS;
/* USER CODE END Variables */

/* USER CODE BEGIN Constructors */
SemaphoreHandle_t xMutex;
Adafruit_NeoPixel OBJ_ESP_LED(__NUM_ESP_LED, __PIN_ESP_LED, NEO_GRB + NEO_KHZ800);
LiquidCrystal_I2C OBJ_PRI_LCD(__ADDR_PRI_LCD, 20, 4);
OneWire ow(__PIN_DS18B20);
DallasTemperature OBJ_SNS_TMP(&ow);
Adafruit_TSL2561_Unified OBJ_SNS_LUX(TSL2561_ADDR_FLOAT, 6767);
sensors_event_t EVENT_TSL_LUX;

/* USER CODE END Constructors */

/* Private FreeRTOS Tasks ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void RTOS_ESP_STATUS(void *pv) {
  while (1) {
    uint8_t status;
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      status = ESP_STATUS;
      xSemaphoreGive(xMutex);
    }

    switch (status) {
      case 0:
        OBJ_ESP_LED.setPixelColor(0, 0, map(ESP_STATUS_BRIGHTNESS, 0, 100, 0, 255), 0);
        OBJ_ESP_LED.show();
        break;

      case 1:
        OBJ_ESP_LED.setPixelColor(0, map(ESP_STATUS_BRIGHTNESS, 0, 100, 0, 255), map(ESP_STATUS_BRIGHTNESS, 0, 100, 0, 255), 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        OBJ_ESP_LED.show();

        OBJ_ESP_LED.setPixelColor(0, 0, map(ESP_STATUS_BRIGHTNESS, 0, 100, 0, 255), 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        OBJ_ESP_LED.show();
        break;
        
      case 2:
        OBJ_ESP_LED.setPixelColor(0, map(ESP_STATUS_BRIGHTNESS, 0, 100, 0, 255), map(ESP_STATUS_BRIGHTNESS, 0, 100, 0, 128), 0);
        vTaskDelay(pdMS_TO_TICKS(256));
        OBJ_ESP_LED.show();

        OBJ_ESP_LED.setPixelColor(0, 0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(256));
        OBJ_ESP_LED.show();
        break;

      case 3:
        OBJ_ESP_LED.setPixelColor(0, map(ESP_STATUS_BRIGHTNESS, 0, 100, 0, 255), 0, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
        OBJ_ESP_LED.show();
        
        OBJ_ESP_LED.setPixelColor(0, 0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
        OBJ_ESP_LED.show();
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void RTOS_EXT_LCD(void *pv) {
  
  OBJ_PRI_LCD.setCursor(0, 0);
  OBJ_PRI_LCD.print("><> Nailem Aquascape");

  OBJ_PRI_LCD.setCursor(0, 1);
  OBJ_PRI_LCD.print("--------------------");

  OBJ_PRI_LCD.setCursor(4, 2);
  OBJ_PRI_LCD.print("Initializing");

  FN_LCD_DOTS(8, 3, 5, 100);

  OBJ_PRI_LCD.setCursor(1, 2);
  OBJ_PRI_LCD.print("Waiting for Jetson");

  while (!CDC_JETSON_ACK) {
    FN_LCD_DOTS(8, 3, 3, 100);
  }
  OBJ_PRI_LCD.clear();
  OBJ_PRI_LCD.setCursor(3, 1);
  OBJ_PRI_LCD.print("Jetson Booted!");
  vTaskDelay(pdMS_TO_TICKS(500));

  OBJ_PRI_LCD.clear();
  while (1) {
    switch (PRI_LCD_MENU) { // placeholders
      case 0: {
        ESP_HEAP = 512 - (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1000);

        // Title
        snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), "<%hhu> Nailem", PRI_LCD_MENU);
        OBJ_PRI_LCD.setCursor(0, 0);
        OBJ_PRI_LCD.print(EXT_LCD_BUF);
        memset(EXT_LCD_BUF, 0, sizeof(EXT_LCD_BUF)); //reset buffer

        if (TM_SWITCH) {
          snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), "%02hhu:%02hhu:%02hhu", _TM.sec, _TM.min, _TM.hour);
        } else {
          snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), "%02hhu/%02hhu/%02hhu", _TM.day, _TM.mon, _TM.year);
        }
        OBJ_PRI_LCD.setCursor(12, 0);
        OBJ_PRI_LCD.print(EXT_LCD_BUF);
        memset(EXT_LCD_BUF, 0, sizeof(EXT_LCD_BUF)); //reset buffer

        // Separator
        OBJ_PRI_LCD.setCursor(0, 1);
        OBJ_PRI_LCD.print("=== At a glance ===");
        if (TM_SWITCH) {
          // Lux on 2,0
          OBJ_PRI_LCD.setCursor(4, 2);
          OBJ_PRI_LCD.print("               ");
          OBJ_PRI_LCD.setCursor(4, 3);
          OBJ_PRI_LCD.print("               ");

          snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), _ERR.ERR_TSL ? "ERR" : "LUX:%hu", _SN.SR_LUX);
          OBJ_PRI_LCD.setCursor(0, 2);
          OBJ_PRI_LCD.print(EXT_LCD_BUF);
          memset(EXT_LCD_BUF, 0, sizeof(EXT_LCD_BUF)); //reset buffer

          // Water Height on 3,0
          snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), " WH:%02hu cm", _SN.SR_PHS);
          OBJ_PRI_LCD.setCursor(0, 3);
          OBJ_PRI_LCD.print(EXT_LCD_BUF);
          memset(EXT_LCD_BUF, 0, sizeof(EXT_LCD_BUF)); //reset buffer

        } else {
          // TM1 on 2,0
          snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), _ERR.ERR_TMP ? "TMP:ERR" : "TMP:%.2fC", _SN.SR_TMA);
          OBJ_PRI_LCD.setCursor(0, 2);
          OBJ_PRI_LCD.print(EXT_LCD_BUF);
          memset(EXT_LCD_BUF, 0, sizeof(EXT_LCD_BUF)); //reset buffer

          // PH on 3,0
          snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), " pH:%04.2f", _SN.SR_PHS);
          OBJ_PRI_LCD.setCursor(0, 3);
          OBJ_PRI_LCD.print(EXT_LCD_BUF);
          memset(EXT_LCD_BUF, 0, sizeof(EXT_LCD_BUF)); //reset buffer

          // TDS on 2, 10
          snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), "TDS:%03hu", _SN.SR_TDS);
          OBJ_PRI_LCD.setCursor(13, 2);
          OBJ_PRI_LCD.print(EXT_LCD_BUF);
          memset(EXT_LCD_BUF, 0, sizeof(EXT_LCD_BUF)); //reset buffer

          // Turbidity on 3, 10
          snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), "TRB:%02hu%%", _SN.SR_TRB);
          OBJ_PRI_LCD.setCursor(13, 3);
          OBJ_PRI_LCD.print(EXT_LCD_BUF);
          memset(EXT_LCD_BUF, 0, sizeof(EXT_LCD_BUF)); //reset buffer
        }
        break;
      }

      case 1: {

        // Title
        snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), "<%hhu> Nailem", PRI_LCD_MENU);
        OBJ_PRI_LCD.setCursor(0, 0);
        OBJ_PRI_LCD.print(EXT_LCD_BUF);
        memset(EXT_LCD_BUF, 0, sizeof(EXT_LCD_BUF)); //reset buffer

        if (TM_SWITCH) {
          snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), "%02hhu:%02hhu:%02hhu", _TM.sec, _TM.min, _TM.hour);
        } else {
          snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), "%02hhu/%02hhu/%02hhu", _TM.day, _TM.mon, _TM.year);
        }
        OBJ_PRI_LCD.setCursor(12, 0);
        OBJ_PRI_LCD.print(EXT_LCD_BUF);
        memset(EXT_LCD_BUF, 0, sizeof(EXT_LCD_BUF)); //reset buffer

        // Separator
        OBJ_PRI_LCD.setCursor(0, 1);
        OBJ_PRI_LCD.print("===== Extended =====");

        // Lux on 2,0
        if (!_ERR.ERR_TSL) {
          snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), "LUX:%hu", _SN.SR_LUX);
        } else {
          snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), "ERR");
        }
        OBJ_PRI_LCD.setCursor(0, 2);
        OBJ_PRI_LCD.print(EXT_LCD_BUF);
        memset(EXT_LCD_BUF, 0, sizeof(EXT_LCD_BUF)); //reset buffer

        // Water Height on 3,0
        snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), " WH:%02hu cm", _SN.SR_PHS);
        OBJ_PRI_LCD.setCursor(0, 3);
        OBJ_PRI_LCD.print(EXT_LCD_BUF);
        memset(EXT_LCD_BUF, 0, sizeof(EXT_LCD_BUF)); //reset buffer

        break;
      }

      case 2: {
        snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), "<%hhu> Nailem", PRI_LCD_MENU);
        OBJ_PRI_LCD.setCursor(0, 0);
        OBJ_PRI_LCD.print(EXT_LCD_BUF);
        memset(EXT_LCD_BUF, 0, sizeof(EXT_LCD_BUF)); //reset buffer

        if (TM_SWITCH) {
          snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), "%02hhu:%02hhu:%02hhu", _TM.sec, _TM.min, _TM.hour);
        } else {
          snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), "%02hhu/%02hhu/%02hhu", _TM.day, _TM.mon, _TM.year);
        }
        OBJ_PRI_LCD.setCursor(12, 0);
        OBJ_PRI_LCD.print(EXT_LCD_BUF);
        memset(EXT_LCD_BUF, 0, sizeof(EXT_LCD_BUF)); //reset buffer

        OBJ_PRI_LCD.setCursor(0, 1);
        OBJ_PRI_LCD.print("=== Temperatures ===");
        
        
        snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), _ERR.ERR_TM1 == false ? "1: %.2fC" : "1: ERR", _SN.SR_TM1);
        OBJ_PRI_LCD.setCursor(0, 2);
        OBJ_PRI_LCD.print(EXT_LCD_BUF);
        memset(EXT_LCD_BUF, 0, sizeof(EXT_LCD_BUF)); //reset buffer
        
        snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), _ERR.ERR_TM2 == false ? "2: %.2fC" : "2: ERR", _SN.SR_TM2);
        OBJ_PRI_LCD.setCursor(0, 3);
        OBJ_PRI_LCD.print(EXT_LCD_BUF);
        memset(EXT_LCD_BUF, 0, sizeof(EXT_LCD_BUF)); //reset buffer

        snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), _ERR.ERR_TM3 == false ? "3: %.2fC" : "3: ERR", _SN.SR_TM3);
        OBJ_PRI_LCD.setCursor(12, 2);
        OBJ_PRI_LCD.print(EXT_LCD_BUF);
        memset(EXT_LCD_BUF, 0, sizeof(EXT_LCD_BUF)); //reset buffer

        snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), _ERR.ERR_TM4 == false ? "4: %.2fC" : "4: ERR", _SN.SR_TM4);
        OBJ_PRI_LCD.setCursor(12, 3);
        OBJ_PRI_LCD.print(EXT_LCD_BUF);
        memset(EXT_LCD_BUF, 0, sizeof(EXT_LCD_BUF)); //reset buffer

        break;
      }

      case 3: {
        snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), "<%hhu> Nailem", PRI_LCD_MENU);
        OBJ_PRI_LCD.setCursor(0, 0);
        OBJ_PRI_LCD.print(EXT_LCD_BUF);
        memset(EXT_LCD_BUF, 0, sizeof(EXT_LCD_BUF)); //reset buffer

        if (TM_SWITCH) {
          snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), "%02hhu:%02hhu:%02hhu", _TM.sec, _TM.min, _TM.hour);
        } else {
          snprintf(EXT_LCD_BUF, sizeof(EXT_LCD_BUF), "%02hhu/%02hhu/%02hhu", _TM.day, _TM.mon, _TM.year);
        }
        OBJ_PRI_LCD.setCursor(12, 0);
        OBJ_PRI_LCD.print(EXT_LCD_BUF);
        memset(EXT_LCD_BUF, 0, sizeof(EXT_LCD_BUF)); //reset buffer

        break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void RTOS_EXT_CDC(void *pv) {
  while (1) {
    while (Serial.available()) {
      char RX = Serial.read();
      if (RX == '\r') continue;
      if (RX == '\n') {
        CDC_RX_BUFFER[CDC_RX_INDEX] = '\0';

        if (strcmp(CDC_RX_BUFFER, "ACK") == 0) {
          if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
            CDC_JETSON_ACK = true;
            xSemaphoreGive(xMutex);
          }
          Serial.println("ACK");
        } else if (CDC_RX_BUFFER[0] == '{') {
          // JSON deserialize
        }
        CDC_RX_INDEX = 0;
      } else {
        if (CDC_RX_INDEX < sizeof(CDC_RX_BUFFER) - 1) {
          CDC_RX_BUFFER[CDC_RX_INDEX++] = RX;
        }
      }
    }

    // Json serialize / tx

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void RTOS_SEN_CAP(void *pv) {
  while (1) {
    // ADC
    uint16_t T_PHS = mapf(analogRead(__PIN_PHS_SENSOR), 0, 4095, 0, 14);
    uint16_t T_TDS = mapf(analogRead(__PIN_TDS_SENSOR), 0, 4095, 0, 999);
    uint16_t T_TRB = mapf(analogRead(__PIN_TRB_SENSOR), 0, 4095, 0, 99);

    // DS18B20
    static bool tempRequested = false;
    static TickType_t tempStartTime = 0;

    float T_TM1 = _SN.SR_TM1;
    float T_TM2 = _SN.SR_TM2;
    float T_TM3 = _SN.SR_TM3;
    float T_TM4 = _SN.SR_TM4;
    float T_TMA = _SN.SR_TMA;

    if (!tempRequested) {
      // Start conversion
      OBJ_SNS_TMP.requestTemperatures();
      tempStartTime = xTaskGetTickCount();
      tempRequested = true;
    }
    else {
      // Check if enough time has passed
      if (xTaskGetTickCount() - tempStartTime >= pdMS_TO_TICKS(400)) {
        // Read result (no waiting)
        T_TM1 = OBJ_SNS_TMP.getTempCByIndex(0);
        T_TM2 = OBJ_SNS_TMP.getTempCByIndex(1);
        T_TM3 = OBJ_SNS_TMP.getTempCByIndex(2);
        T_TM4 = OBJ_SNS_TMP.getTempCByIndex(3);

        float T_IN[4] = {T_TM1, T_TM2, T_TM3, T_TM4};
        uint8_t AVAIL_T = 0;
        float ACC_T = 0.0f;
        for (int i = 0; i < 4; i++) {
          if (T_IN[i] != -127) {
            AVAIL_T++;
            ACC_T += T_IN[i];
          }
        }

        T_TMA = AVAIL_T > 0 ? ACC_T / AVAIL_T : -127.0f;

        _ERR.ERR_TM1 = T_TM1 == -127 ? true : false;
        _ERR.ERR_TM2 = T_TM2 == -127 ? true : false;
        _ERR.ERR_TM3 = T_TM3 == -127 ? true : false;
        _ERR.ERR_TM4 = T_TM4 == -127 ? true : false; 

        _ERR.ERR_TMP = _ERR.ERR_TM1 == true && _ERR.ERR_TM2 == true && _ERR.ERR_TM3 == true && _ERR.ERR_TM4 == true ? true : false;

        tempRequested = false; // ready for next cycle
      }
    }

    OBJ_SNS_LUX.getEvent(&EVENT_TSL_LUX);
    uint16_t T_LUX;

    if (EVENT_TSL_LUX.light) {
      T_LUX = EVENT_TSL_LUX.light;
      _ERR.ERR_TSL = false;
    } else {
      _ERR.ERR_TSL = true;
    }
    
    // Write to struct

    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      _SN.SR_PHS = T_PHS;
      _SN.SR_TDS = T_TDS;
      _SN.SR_TRB = T_TRB;
      _SN.SR_TM1 = T_TM1;
      _SN.SR_TM2 = T_TM2;
      _SN.SR_TM3 = T_TM3;
      _SN.SR_TM4 = T_TM4;
      _SN.SR_TM4 = T_TM4;
      _SN.SR_TMA = T_TMA;
      _SN.SR_LUX = T_LUX;
      xSemaphoreGive(xMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(SNS_CAP_DELAY));
  }
}

void RTOS_JSON_ENC(void *pv) {
  StaticJsonDocument<512> doc;

  while (1) {

    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {

      // ===== Sensor Data =====
      doc["lux"] = _SN.SR_LUX;

      doc["ph"]  = _SN.SR_PHS;
      doc["tds"] = _SN.SR_TDS;
      doc["trb"] = _SN.SR_TRB;

      doc["tm1"] = _SN.SR_TM1;
      doc["tm2"] = _SN.SR_TM2;
      doc["tm3"] = _SN.SR_TM3;
      doc["tm4"] = _SN.SR_TM4;

      doc["tma"] = _SN.SR_TMA;

      doc["ult"] = _SN.SR_ULT;

      doc["lst"] = _SN.SR_LST;
      doc["rst"] = _SN.SR_RST;

      // ===== Relay States =====
      doc["rl1"] = _ST.ST_RL1;
      doc["rl2"] = _ST.ST_RL2;
      doc["rl3"] = _ST.ST_RL3;
      doc["rl4"] = _ST.ST_RL4;

      // ===== PWM Values =====
      doc["ms1"] = _ST.PWM_MS1;
      doc["ms2"] = _ST.PWM_MS2;
      doc["ms3"] = _ST.PWM_MS3;
      doc["ms4"] = _ST.PWM_MS4;

      // ===== System Flags =====
      doc["jetson_ack"] = CDC_JETSON_ACK;
      doc["esp_status"] = ESP_STATUS;
      doc["lcd_menu"]   = PRI_LCD_MENU;
      doc["tm_switch"]  = TM_SWITCH;

      // ===== Time =====
      JsonObject time = doc.createNestedObject("time");
      time["sec"]  = _TM.sec;
      time["min"]  = _TM.min;
      time["hour"] = _TM.hour;
      time["day"]  = _TM.day;
      time["mon"]  = _TM.mon;
      time["year"] = _TM.year;

      xSemaphoreGive(xMutex);
    }

    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      serializeJson(doc, JSON_TX_BUFFER);
    }
    doc.clear();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void RTOS_EXT_CHL(void *pv) {
  while (1) {
    if (!_ERR.ERR_TMP) {
      if (_ERR.WATER_CHANGE_ONGOING ) {
        _ST.ST_RL1 = false; // Chiller on relay 1
        digitalWrite(__PIN_RELAY_1, RELAY_OFF);

      } else {
        if (_SN.SR_TMA >= _TS.TMP_HIGH) {
          _ERR.TEMP_ABNORMAL = true;
          digitalWrite(__PIN_RELAY_1, RELAY_ON);
        } else if (_SN.SR_TMA <= _TS.TMP_LOW) {
          _ERR.TEMP_ABNORMAL = true;
          digitalWrite(__PIN_RELAY_1, RELAY_OFF);
        } else {
          _ERR.TEMP_ABNORMAL = false;
        }
      }
      
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void RTOS_WIFI_CONN(void *pv) {
  WiFi.begin(ssid, pass);
}

/* USER CODE END 0 */
void setup() {
  /* USER CODE BEGIN Setup_Init */
  Wire.begin(__PIN_I2C_SDA, __PIN_I2C_SCL);
  Serial.begin(115200);

  xMutex = xSemaphoreCreateMutex();

  OBJ_ESP_LED.begin();
  OBJ_ESP_LED.show();

  OBJ_PRI_LCD.init();
  OBJ_PRI_LCD.backlight();

  OBJ_SNS_TMP.begin();
  OBJ_SNS_TMP.setResolution(11);
  OBJ_SNS_TMP.setWaitForConversion(false);

  OBJ_SNS_LUX.enableAutoRange(true);
  OBJ_SNS_LUX.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);

  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);

  pinMode(0, INPUT_PULLUP);

  /* USER CODE END Setup_Init */

  /* USER CODE BEGIN FreeRTOS_Init */
  xTaskCreatePinnedToCore(RTOS_ESP_STATUS, "STATUS_RGB", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(RTOS_EXT_LCD, "PRIMARY LCD", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(RTOS_EXT_CDC, "SERIAL", 4096, NULL, 2, NULL, 1);
  
  while (!CDC_JETSON_ACK) {
    Serial.print(".");
    vTaskDelay(pdMS_TO_TICKS(250));
  }
  xTaskCreatePinnedToCore(RTOS_SEN_CAP, "SENSOR", 2048, NULL, 2, NULL, 0);
  xTaskCreate(t, "", 1024, 0, 1, 0);

  /* USER CODE END FreeRTOS_Init */

  /* USER CODE BEGIN Setup */

  /* USER CODE END Setup */
}
const int buttonPin = 0;

int state = 0;
bool lastButton = HIGH;

void loop() {
  /* USER CODE BEGIN Loop */
    bool currentButton = digitalRead(buttonPin);

    // Detect button press
    if (lastButton == HIGH && currentButton == LOW) {
        PRI_LCD_MENU++;

        if (PRI_LCD_MENU > 3) {
            PRI_LCD_MENU = 0;
        }
        OBJ_PRI_LCD.clear();
        delay(100); // simple debounce
    }

    lastButton = currentButton;

  /* USER CODE END Loop */

  /* USER CODE BEGIN Loop_End */
  
  /* USER CODE END Loop_End */
}

/* USER CODE BEGIN 1 */
void FN_LCD_DOTS(int x, int y, int cycles, int delayMs) {
  for (int i = 0; i < cycles; i++) {
    OBJ_PRI_LCD.setCursor(x, y);
    OBJ_PRI_LCD.print(".  ");
    vTaskDelay(pdMS_TO_TICKS(delayMs));

    OBJ_PRI_LCD.setCursor(x, y);
    OBJ_PRI_LCD.print(" . ");
    vTaskDelay(pdMS_TO_TICKS(delayMs));

    OBJ_PRI_LCD.setCursor(x, y);
    OBJ_PRI_LCD.print("  .");
    vTaskDelay(pdMS_TO_TICKS(delayMs));
  }
}

float mapf(float x,
           float in_min,  float in_max,
           float out_min, float out_max) {
    return (x - in_min) *
           (out_max - out_min) /
           (in_max - in_min) +
           out_min;
}

void t(void*) {
    TickType_t x = xTaskGetTickCount();
    
    for (;;) {
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
          TM_SWITCH = !TM_SWITCH;
          xSemaphoreGive(xMutex);
        }
        vTaskDelayUntil(&x, pdMS_TO_TICKS(TM_SW_INTERVAL * 1000));
    }
}

/* USER CODE END 1 */