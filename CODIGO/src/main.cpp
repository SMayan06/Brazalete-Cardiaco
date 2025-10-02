#include <Arduino.h>
// ---------- archivos de configuracion y funciones ----------
#include "config.h"
#include "Functions.h"

bool timer() {
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= 200) { // Verifica si ha pasado 1 segundo
        lastTime = currentTime; // Actualiza el tiempo del último evento
        return true; // 
    }
    return false; //
}


void setup() {
    Serial.begin(115200);
    delay(WAIT_SERIAL); // Espera la inicialización del puerto serie 

    // ---------- Declaración de sensores ----------
    #ifdef USE_MPU6050
        imu.setPins(SDA_PIN, SCL_PIN); // Configura los pines SDA y SCL
        initMPU();
    #endif

    #ifdef FC
        max02.begin();
        max02.setup(LED_BRIGHTNESS, SAMPLE_AVERAGE, LED_MODE, SAMPLE_RATE, PULSE_WIDTH, ADC_RANGE);
    #endif
    
    #ifdef USE_BLE
        ble.begin(); // Inicializa el BLE
        SetBLEUUIDs(); 
    #endif
}

void loop() {
    // ---------- Lógica del sistema ----------
if (timer()) {
       // Serial.println(" . "); prueba de que el loop no se estanco 
    switch (state) {
        case REPOSE:
            #ifdef DEBUG
                Serial.println("Estado: Reposo");
            #endif
            if (checkConnections() == ALL_OK) state = READING; // Cambia al estado de lectura si las conexiones son correctas
            else {state = ERROR; Serial.println("hubo problemas en las conexiones");} // Cambia al estado de error si hay problemas de conexión
            break;
        case READING:
            #ifdef DEBUG
                Serial.println("Estado: Lectura");
            #endif


            if (checkActivity()) state = MOVEMENT; // Cambia al estado de movimiento si
            else if (checkConnections() != ALL_OK) state = ERROR; // Cambia al estado de error si hay problemas de conexión
            break;
        case MOVEMENT:
            #ifdef DEBUG
                Serial.println("Estado: Movimiento");
            #endif
            #ifdef USE_MPU6050
                StepCounter(); // Llama a la función de conteo de pasos
            #endif
            #ifdef FC
                heartRate();
            #endif

            
            if (checkConnections() != ALL_OK) state = ERROR; // Cambia al estado de error si hay problemas de conexión
            else if (!checkActivity()) state = READING; // Cambia al estado de lectura si no hay actividad
            break; 
        case ERROR:
            #ifdef DEBUG
                Serial.println("Estado: Error");
            #endif
            connectionStatus = checkConnections(true); // Fuerza la verificación de conexiones
            errorStatus = checkError(); // Verifica el estado de error
            if (errorStatus == NO_ERROR) state = READING; 
            else delay(WAIT); // Espera un tiempo antes de volver a intentar
                
            break;
    }
    #ifdef USE_BLE
                bleSendData(); 
            #endif
}
}



