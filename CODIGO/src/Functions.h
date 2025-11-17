#include <Arduino.h>
#include "Config.h"


// -------------------- Funciones de control de flujo ---------------------
bool timer() {
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= 200) { // Verifica si ha pasado 1 segundo
        lastTime = currentTime; // Actualiza el tiempo del último evento
        return true; // 
    }
    return false; //
}

// -------------------- Funcion de verificación de conexiones ---------------------
ConectionStatus checkConnections(bool forceCheck = false) {
    // Verifica las conexiones de los sensores y dispositivos
    static unsigned long lastCheck = 0; // Tiempo del último chequeo
    unsigned long now = millis();
    if (forceCheck || (now - lastCheck >= ERROR_TIMEOUT)) {
        lastCheck = now; 
        #ifdef USE_MPU6050  
            if (!imu.isConnected()) {
                Serial.println("Error: MPU6050 no conectado.");
                connectionStatus = NO_MPU;
                return connectionStatus; // Devuelve un estado de error si el MPU6050 no está conectado
            }
        #endif
        
        #ifdef FC
         if (!max02.begin(Wire, I2C_SPEED_STANDARD)) {
                Serial.println("Error: FC no conectado.");
                connectionStatus = NO_FC;
                return connectionStatus; // Devuelve un estado de error si el FC no está conectado
            }
        #endif

        #ifdef USE_BLE
            if (!ble.isClientConnected()) {
                Serial.println("Error: BLE no conectado.");
                connectionStatus = NO_BLE;
                return connectionStatus; // Devuelve un estado de error si el BLE no está conectado
            }
        #endif

     connectionStatus = ALL_OK; // Actualiza el estado de conexión a todo OK 
    }
    return connectionStatus; // Todas las conexiones están correctas
}

#ifdef USE_MPU6050
void initMPU(){
    if (imu.begin()){ // Inicializa el sensor MPU6050
    #ifdef OFFSETS
        imu.setOffsets(OFFSETS); // Ajusta los offsets del sensor
    #else
        imu.calibrate(); // Calibra el sensor si no se han definido offsets
        imu.printOffsets(); // Imprime los offsets calibrados
    #endif
    }
}
#endif

#ifdef USE_MPU6050
void hardResetMPU() {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x6B);    // PWR_MGMT_1
    Wire.write(0x80);    // reset
    Wire.endTransmission();
    delay(100);
}
#endif

#ifdef USE_MPU6050
void wakeUp(){
    hardResetMPU();
    delay(100);
    if (!imu.isConnected()) {
    Wire.end();
    delay(10);
    Wire.begin(SDA_PIN, SCL_PIN);
    initMPU();
    }
}
#endif

// -------------------- Función de verificación de actividad física ---------------------
bool checkActivity() {
    // Verifica la actividad física utilizando el sensor MPU6050
    static unsigned long lastActivityCheck = 0; // Tiempo del último chequeo de actividad
    static unsigned long activityStartTime = 0; // Tiempo de inicio de la actividad
    static unsigned long activityEndTime = 0; // Tiempo de finalización de la actividad
   
    unsigned long now = millis();

    static bool mpuOk = false;
    static bool fcOk = false; 

    #ifdef USE_MPU6050

        rawData = imu.readRaw();
        data = imu.convertToUnits(rawData);
        float accelMag = imu.getAccelerationMagnitude(); 
        dth.updateBuffer(accelMag); 
        dth.update(accelMag);
        bool isAbove = dth.isAbove(accelMag); 
        mpuOk = isAbove; 

        /*
        #ifdef DEBUG
            Serial.print("accelMag: ");
            Serial.print(accelMag, 3);
            Serial.print("  isAbove: ");
            Serial.println(isAbove ? 1 : 0);
        #endif  */
    #else
        mpuOk = true; // Si no hay MPU6050, asumimos que la actividad es correcta
    #endif

    #ifdef FC
    // Aquí se puede agregar la lógica para verificar la actividad física utilizando el FC
    #else
        fcOk = true; // Si no hay FC, asumimos que la actividad es correcta
    #endif

    if(mpuOk && fcOk) {
        if (!sessionActive) {
            sessionActive = true; // Inicia la sesión si no estaba activa
            duration = 0; // Reinicia la duración de la actividad
            activityStartTime = now; // Marca el inicio de la actividad
            #ifdef DEBUG
            Serial.println("Sesión iniciada por actividad detectada.");
            #endif
        }
        lastActivityCheck = now; // Actualiza el tiempo del último chequeo de actividad
   
    } 
         if (sessionActive && (now - lastActivityCheck >= ACTIVITY_TIMEOUT)) {
        activityEndTime = now; // Marca el final de la actividad
        duration = activityEndTime - activityStartTime; // Calcula la duración de la actividad
        #ifdef DEBUG
        Serial.println("Sesión inactiva por más de 5 segundos. Finalizando sesión.");
        Serial.print("Duración de la actividad: ");
        Serial.print(duration / 1000.0); // Imprime la duración en segundos
        #endif
        sessionActive = false; // Finaliza la sesión si no hay actividad durante el tiempo especificado
        stepCount = 0; // Reinicia el contador de pasos al finalizar la sesión
    }

  /*  
  #ifdef DEBUG
        Serial.print("mpuOk: "); Serial.print(mpuOk ? 1 : 0);
        Serial.print("  session: "); Serial.println(sessionActive ? 1 : 0);
    
    #endif */
    return sessionActive; // Devuelve el estado de la sesión de actividad
}
// -------------------- Función de conteo de pasos ---------------------
#ifdef USE_MPU6050
    void StepCounter() {
        // Función para contar pasos utilizando el sensor MPU6050
        static bool wasAboveThreshold = false; // Estado anterior del umbral
        static unsigned long lastStepTime = 0; // Tiempo del último paso
        unsigned long currentTime = millis(); 
        float accelMag = imu.getAccelerationMagnitude(); 

        dth.updateBuffer(accelMag); 
        dth.update(accelMag); 
        bool isAbove = dth.isAbove(accelMag); 

        if (isAbove && !wasAboveThreshold && (currentTime - lastStepTime >= STEP_DELAY)) {
            stepCount++; // Incrementa el contador de pasos
            lastStepTime = currentTime; // Actualiza el tiempo del último paso
            #ifdef DEBUG
            Serial.print("Paso detectado. Total: ");
            Serial.println(stepCount); // Imprime el número total de pasos
            #endif
        }
        wasAboveThreshold = isAbove; // Actualiza el estado anterior del umbral
    }
#endif

#ifdef USE_BLE
    // ---------- Configuración de UUIDs para BLE ----------
    void SetBLEUUIDs() {
     
        ble.addService(SERVICE_UUID_GENERAL); // Añade el servicio BLE general
        ble.addCharacteristic(CHARACTERISTIC_UUID_SESION, SERVICE_UUID_GENERAL); 

        #ifdef USE_MPU6050
            ble.addService(SERVICE_UUID_MPU); 
            ble.addCharacteristic(CHARACTERISTIC_UUID_STEPS, SERVICE_UUID_MPU);
            ble.addCharacteristic(CHARACTERISTIC_UUID_DURATION, SERVICE_UUID_MPU);
        #endif
        #ifdef FC
        #endif

        ble.startAdvertising(); // Comienza a anunciar el servicio BLE
    }
    // ---------- Envío de datos a través de BLE ----------
    void bleSendData() {

        checkActivity(); // Asegura que la actividad esté actualizada
        ble.notify(CHARACTERISTIC_UUID_SESION, sessionActive ? "activa" : "inactiva"); // Notifica el estado de la sesión    
        #ifdef USE_MPU6050
            ble.notify(CHARACTERISTIC_UUID_STEPS, stepCount); 
            ble.notify(CHARACTERISTIC_UUID_DURATION, duration / 1000); // Envía la duración en segundos
        #endif
        #ifdef FC
        #endif

    }
#endif

// -------------------- deteccion de errores ---------------------
ErrorStatus checkError() {
    if (connectionStatus != ALL_OK){
        switch (connectionStatus) {
            case NO_MPU:
            #ifdef USE_MPU6050
                wakeUp(); // Intenta reactivar el MPU6050
            #endif
                #ifdef DEBUG
                    Serial.println("Error: No se detecta el MPU6050.");
                    Serial.println("Intentando reactivar el MPU6050...");
                #endif
                #ifdef USE_BLE
                    ble.notify(CHARACTERISTIC_UUID_SESION, "error: no MPU6050");
                #endif
                return CONNECTION_ERROR; // Devuelve un estado de error de conexión
            case NO_FC:
                #ifdef DEBUG
                    Serial.println("Error: No se detecta el FC.");  
                #endif
                #ifdef USE_BLE
                    ble.notify(CHARACTERISTIC_UUID_SESION, "error: no FC");
                #endif
                return CONNECTION_ERROR; 
            case NO_BLE:
                Serial.println("Error: No se detecta el BLE.");
                return CONNECTION_ERROR; 
            default:
                #ifdef DEBUG
                    Serial.println("Error: error desconocido.");
                #endif
                #ifdef BLE
                    ble.notify(CHARACTERISTIC_UUID_SESION, "error: desconocido");
                #endif
                return CONNECTION_ERROR; 
        }
    }

    return NO_ERROR; // Devuelve un estado de no error si no hay problemas de conexión

}
#ifdef FC
void heartRate(){
    
    long lastBeat = 0; // Time at which the last beat occurred
    const byte RATE_SIZE = 6; //Increase this for more averaging. 4 is good.
    byte rates[RATE_SIZE]; //Array of heart rates
    byte rateSpot = 0;

    float beatsPerMinute;
    int beatAvg;
    
    long irValue = max02.getIR();
    

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  #ifdef DEBUG
    Serial.print("IR=");
    Serial.print(irValue);
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.print(beatAvg);

    if (irValue < 50000)
    Serial.print(" No finger?");
  
    Serial.println();
  #endif
}
#endif
