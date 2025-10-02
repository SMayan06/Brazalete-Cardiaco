#include "Config.h"

State state = REPOSE; // Estado inicial del sistema
ConectionStatus connectionStatus = ALL_OK; // Estado de conexión por defecto
ErrorStatus errorStatus = NO_ERROR; // Estado de error por defecto


int stepCount = 0; // Contador de pasos
unsigned long duration = 0; // Duración de la actividad
bool sessionActive = false; // Estado de la sesión de actividad

#ifdef USE_MPU6050
    MPU6050 imu;
    IMUData rawData, data;
    DynamicThreshold dth;
#endif
#ifdef USE_BLE
    BLEHandler ble("SensorDeActividad");
#endif
#ifdef FC
    MAX30105 max02;
#endif