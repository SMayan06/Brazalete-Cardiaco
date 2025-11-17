#pragma once
#include <Arduino.h>
// ---------- Implementación de sensores ----------
#define USE_MPU6050 // Habilita el uso del sensor MPU6050
//#define FC 
#define USE_BLE 
#define DEBUG

// ---------- Declaracion de librerias ----------
#ifdef USE_MPU6050
    #include "MPU6050.h"
    #include "DynamicThreshold.h"
    extern MPU6050 imu;
    extern IMUData rawData, data;
    extern DynamicThreshold dth;
#else 
    #include <Wire.h>
#endif

#ifdef USE_BLE
    #include "BLEHandler.h"
    extern BLEHandler ble;
#endif

#ifdef FC
    #include "MAX30105.h"
    #include "heartRate.h"
    extern MAX30105 max02;
#endif

// ---------- Configuración de pines ---------- 
#define SDA_PIN 8
#define SCL_PIN 9


#define MPU_ADDRESS 0x68 // Dirección I2C del MPU6050
#define MAX_ADDRESS 0x57 // Dirección I2C del MAX30102

// ---------- Declaracion de UUIDs para BLE ----------
// mpu
#define SERVICE_UUID_MPU "mpu6050x-0000-0000-0000-000000000000"
#define CHARACTERISTIC_UUID_STEPS "stepcoun-t000-0000-0000-000000000000"
#define CHARACTERISTIC_UUID_DURATION "duration-0000-0000-0000-030000000000"

//FC
#define SERVICE_UUID_FS "00000000-0000-0000-0000-000000000000"
#define CHARACTERISTIC_UUID_FC "00000000-0000-0000-0000-000000000001"

// BLE UUIDs generales
#define SERVICE_UUID_GENERAL "12345678-1234-5678-1234-56789abcdef0" // UUID del servicio BLE
#define CHARACTERISTIC_UUID_SESION "12345678-1234-5678-1234-56789abcdef1" // UUID de la característica BLE


// ---------- Declaración de tiempos ----------
#define WAIT_SERIAL 1000
#define WAIT 500
#define ERROR_TIMEOUT 5000
#define ACTIVITY_TIMEOUT 5000 
#define STEP_DELAY 300

// ---------- Declaración de valores de calibración ----------
// -------- mpu6050 --------
#define USE_OFFSETS
#define OFFSETS 0, 0, 0, 0, 0, 0 // Valores de offset para el sensor MPU6050

// -------- FC --------
#define LED_BRIGHTNESS 0xFF // Brillo del LED 
#define SAMPLE_AVERAGE 4 // Promedio de muestras 
#define LED_MODE 2 // Modo de LED  
#define SAMPLE_RATE 100 // Tasa de muestreo  
#define PULSE_WIDTH 411 // Ancho de pulso  
#define ADC_RANGE 2048 // Rango del ADC  


// ---------- Declaración de constantes para el flujo de estado ----------
enum State {
    REPOSE, 
    READING,
    MOVEMENT,
    ERROR
};
extern State state;

enum ConectionStatus {
    DF = -1,           // Estado por defecto
    ALL_OK = 0, // Conexión correcta
    NO_MPU = 1,             // No hay conexión con el MPU6050
    NO_FC = 2,              // No hay conexión con el FC
    NO_BLE = 3,             // No hay conexión con el BLE  
    
};

extern ConectionStatus connectionStatus;

enum ErrorStatus {
    NO_ERROR = 0, // Sin errores
    CONNECTION_ERROR = 1, // Error de conexión
};

extern ErrorStatus errorStatus;

// ---------- Declaración de variables globales ----------
extern int stepCount; // Contador de pasos
extern unsigned long duration; // Duración de la actividad
extern bool sessionActive; // Estado de la sesión de actividad