#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <MPU6050.h>
#include <DynamicThreshold.h>
#include "MAX30105.h"
#include "heartRate.h"

// Instancia del sensor
MPU6050 imu;
DynamicThreshold dth;

// Variables para contar pasos
int stepCount = 0;
bool wasAboveThreshold = false;
unsigned long lastStepTime = 0;
const unsigned long stepDelay = 300;

MAX30105 particleSensor;

// AVG BPM y buffer de los últimos 60 valores distintos
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;

float beatsPerMinute;
int beatAvg;

const byte BPM_BUFFER_SIZE = 30;
float bpmBuffer[BPM_BUFFER_SIZE];
byte bpmIndex = 0;
byte bpmCount = 0;

float BPM;

// Control de impresión cada 500 ms
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 40;

// Variables BLE
BLEServer* pServer = NULL;
BLECharacteristic* txCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_UUID             "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define TX_UUID             "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("✓ Conexión BLE establecida");
    }
    
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("✗ Conexión BLE perdida");
    }
};

void setup() {
    Serial.begin(115200);

    // MPU6050
    int sda = 8, scl = 9;
    imu.setPins(sda, scl);
    if (!imu.begin()) { Serial.println("Error al iniciar MPU6050"); while (1) delay(1000); }

    imu.setOffsets(0, 0, 0, 0, 0, 0);
    imu.printOffsets();
    float delta = imu.calibrateMagnitudeDelta();
    dth.setDelta(delta);

    // MAX30105
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) { Serial.println("MAX30102 no encontrado."); while(1); }
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeGreen(0);

    // Configurar BLE
    BLEDevice::init("Brazalete");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    txCharacteristic = pService->createCharacteristic(
        TX_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    txCharacteristic->addDescriptor(new BLE2902());
    
    BLECharacteristic* rxCharacteristic = pService->createCharacteristic(
        RX_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    
    pService->start();
    
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLEUUID(SERVICE_UUID));
    BLEDevice::startAdvertising();
    
    Serial.println("=== Sensor BLE Activo ===");
    Serial.println("Busca: ESP32_Sensor_BLE");
    Serial.println();
}

void loop() {
    // MPU6050
    IMUData rawData = imu.readRaw();
    IMUData data = imu.convertToUnits(rawData);
    float magnitude = imu.getAccelerationMagnitude();

    // MAX30105
    long irValue = particleSensor.getIR();

    // Umbral dinámico
    dth.updateBuffer(magnitude);
    dth.update(magnitude);
    bool isAbove = dth.isAbove(magnitude);

    // Detección de pasos
    if (isAbove && !wasAboveThreshold && millis() - lastStepTime > stepDelay) {
        stepCount++;
        lastStepTime = millis();
    }
    wasAboveThreshold = isAbove;

    // Detección de latido
    if (checkForBeat(irValue) == true) {
        long delta = millis() - lastBeat;
        lastBeat = millis();

        beatsPerMinute = 60 / (delta / 1000.0);

        if (beatsPerMinute < 255 && beatsPerMinute > 20) {
            rates[rateSpot++] = (byte)beatsPerMinute;
            rateSpot %= RATE_SIZE;

            beatAvg = 0;
            for (byte x = 0; x < RATE_SIZE; x++)
                beatAvg += rates[x];
            beatAvg /= RATE_SIZE;

            if (bpmCount == 0 || beatAvg != bpmBuffer[(bpmIndex + BPM_BUFFER_SIZE - 1) % BPM_BUFFER_SIZE]) {
                bpmBuffer[bpmIndex] = beatAvg;
                bpmIndex = (bpmIndex + 1) % BPM_BUFFER_SIZE;
                if (bpmCount < BPM_BUFFER_SIZE) bpmCount++;
            }
        }
    }

    float bpmSum = 0;
    for (byte i = 0; i < bpmCount; i++) bpmSum += bpmBuffer[i];
    if (bpmCount > 0) BPM = bpmSum / 30.0;
    else BPM = 0;

    // Impresión cada 500 ms
    if (millis() - lastPrintTime >= printInterval) {
        lastPrintTime = millis();
        Serial.print("   | Pasos: ");
        Serial.print(stepCount);
        Serial.print(" | LxM: ");
        Serial.println(BPM, 4);

        // ← LÍNEAS AGREGADAS
        Serial.print(" Avg BPM=");
        Serial.print(beatAvg);
        if (irValue < 50000) Serial.print(" No finger?");
        Serial.println();
        // ← FIN DE LÍNEAS AGREGADAS

        // BLE
        if (deviceConnected) {
            String bleData = "Pasos: " + String(stepCount) + " | LxM: " + String(BPM, 4) + " | LxS: " + String(beatAvg);
            txCharacteristic->setValue(bleData);
            txCharacteristic->notify();
        }
    }
    
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);
        BLEDevice::startAdvertising();
    }
    
    oldDeviceConnected = deviceConnected;
}
