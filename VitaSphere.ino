// Librerías
#include <Wire.h>
#include <Adafruit_MLX90614.h> // Librería para el GY-906 temperatura
#include "MAX30105.h"
#include "heartRate.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "time.h"
#include <Arduino.h>
#include <WebServer.h>

// Interacción con dispositivos Bluetooth BLE
#include <BLEDevice.h> // Configuración e inicialización
#include <BLEServer.h> // Creación y manejo de servicios - Procesamiento solicitudes de lectura y escritura
#include <BLEUtils.h>  // Trabajo con UUIDs (Servicios y características de datos, y descriptores de características)
#include <BLE2902.h>   // Descriptor BLE para notificaciones

// Nombre del dispositivo
#define DEVICE_NAME "VitaSphereV1"

// UUIDs para servicios y características
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b" // Servicio que permite controlar el dispositivo
#define LED_CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BUZZER_CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define VIBRADOR_CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26aa"

// Configuración del servidor BLE
BLEServer* pServer = NULL; // pServer se encarga de gestionar conexiones entre el ESP32 y el dispositivo móvil
BLECharacteristic* pLedChar = NULL; // pLedChar permite leer y escribir valores asociados a características
BLECharacteristic* pBuzzerChar = NULL;
BLECharacteristic* pVibradorChar = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Sensores que se utilizaran
const int MPU_addr = 0x68; // Dirección I2C del MPU-6050  giroscopio
TwoWire myWire = TwoWire(1);
Adafruit_MLX90614 mlx = Adafruit_MLX90614(); 
MAX30105 particleSensor;

// Variables para la medición de los sensores
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; // Datos crudos de aceleración y giroscopio
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0; // Normalización de datos de ace y gir
boolean fall = false, trigger1 = false, trigger2 = false, trigger3 = false;
byte trigger1count = 0, trigger2count = 0, trigger3count = 0;
int angleChange = 0;

// Variables para la temperatura
float tempAmb, tempObj;

// Medicion de pulso, oxigenacion, etc.
const byte RATE_SIZE = 8;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;
int umbralDedo = 5000;  
int contadorLecturas = 0;

// Intervalo de tiempo entre medición de los sensores
const int sampleInterval = 4;
const int ALERT_TIME = 1000;
bool alertActive = false;

// Pines que se utilizaran
const int buzzerPin = 5;
const int rojo = 4;
const int vib = 18;
const int buttonPin = 19;

//Constantes de wifi
const char* ssid="Totalplay-7D9E"; //nombre de al red
const char* password="93DucK18!3312P"; //contraseña
// const char* ssid="WIFI EDIFICIO I";
// const char* password="";

//Constantes de tiempo de envio
unsigned long lastSend = 0;
const unsigned long sendInterval = 15000; 
const char* metricaUrl = "http://192.168.100.92:8000/metrica/"; //ip de la direccion de tu internet
//const char* metricaUrl = "http://10.1.125.77:8000/metrica/";

const int maxSize = 5;
int Datpul[maxSize] = {0,0,0,0,0};
float Datoxi[maxSize] = {0,0,0,0,0};

// Variables para el filtrado de señal
#define FILTER_SIZE 5
long irFilterBuffer[FILTER_SIZE];
byte filterIndex = 0;

// Función de filtrado
long applyFilter(long value) {
    irFilterBuffer[filterIndex] = value;
    filterIndex = (filterIndex + 1) % FILTER_SIZE;
    
    long sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) sum += irFilterBuffer[i];
    return sum / FILTER_SIZE;
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Device disconnected");
    }
};

// Callback handlers modificados para leer valores numéricos
class LedCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      uint8_t* data = pCharacteristic->getData();
      size_t length = pCharacteristic->getLength();
      
      if (length > 0) {
        Serial.print("LED: ");
        Serial.println(data[0]);
        digitalWrite(rojo, data[0] ? HIGH : LOW);
      }
    }
};

class BuzzerCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      uint8_t* data = pCharacteristic->getData();
      size_t length = pCharacteristic->getLength();
      
      if (length > 0) {
        Serial.print("BUZZER: ");
        Serial.println(data[0]);
        digitalWrite(buzzerPin, data[0] ? HIGH : LOW);
      }
    }
};

class VibradorCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      uint8_t* data = pCharacteristic->getData();
      size_t length = pCharacteristic->getLength();
      
      if (length > 0) {
        Serial.print("VIBRATOR: ");
        Serial.println(data[0]);
        digitalWrite(vib, data[0] ? HIGH : LOW);
      }
    }
};

// Inicialización y configuración del Bluetooth BLE
void setupBLE () {
  BLEDevice::init(DEVICE_NAME); // Inicialización del nombre

  pServer = BLEDevice::createServer(); // Servidor BLE (dispositivo) que ofrece servicios y responde solicitudes de dispositivos clientes
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID); // Se accede al UUID del servicio BLE previamente establecido
  
  uint8_t initialValue = 0; // Establecer valores iniciales como matrices de bytes
  
  pLedChar = pService->createCharacteristic( // Contenedor de datos que pueden ser leídos o notificados a dispositivos clientes
    LED_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | // Permite que un cliente lea el valor de la característica
    BLECharacteristic::PROPERTY_WRITE  // Permite que un cliente escriba el valor de la característica
  );
  pLedChar->setCallbacks(new LedCallbacks());
  pLedChar->setValue(&initialValue, 1);

  pBuzzerChar = pService->createCharacteristic(
    BUZZER_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE
  );
  pBuzzerChar->setCallbacks(new BuzzerCallbacks());
  pBuzzerChar->setValue(&initialValue, 1);

  pVibradorChar = pService->createCharacteristic(
    VIBRADOR_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE
  );
  pVibradorChar->setCallbacks(new VibradorCallbacks());
  pVibradorChar->setValue(&initialValue, 1);

  pService->start(); // Inicia el servicio BLE, haciendo disponible el ESP32 a otros dispositivos

  // Configuración y publidicad BLE
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising(); // Obtención del objeto de publicidad para configurar cómo se presenta el dispositivo a clientes
  pAdvertising->addServiceUUID(SERVICE_UUID); // Se añade el UUID del servicio 
  pAdvertising->setScanResponse(true); // Configuración de la respuesta de escaneo, enviando información adicional durante el proceso de escaneo en otros dispositivos
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising(); // Inicia la publicidad del ESP32, haciéndolo visible a los dispositivos que estén escaneando
  Serial.println("BLE device advertising!");
}

void setup() {
    Serial.begin(115200);

    pinMode(buzzerPin, OUTPUT);
    pinMode(rojo, OUTPUT);
    pinMode(vib, OUTPUT);
    pinMode(buttonPin, INPUT);

    digitalWrite(rojo, LOW);
    digitalWrite(buzzerPin, LOW);
    digitalWrite(vib, LOW);

    setupWifi();
    setupSensors();
    setupBLE();
}

void loop() {
    static unsigned long lastTime = 0;

    if (digitalRead(buttonPin) == HIGH && alertActive) {
        cancelAlert();
    }

    if (fall) {
        Serial.println("FALL DETECTED");
        activateAlarms();
    } else {
        unsigned long currentMillis = millis();
        if (currentMillis - lastSend >= sendInterval) {
            lastSend = currentMillis;
            if (Datoxi[maxSize - 1] != 0 && Datpul[maxSize - 1] != 0) {
                prepararJsonMetrica();
            }
        }

        if (millis() - lastTime >= sampleInterval) {
            lastTime = millis();
            readSensors();
            processSensorData();
        }
    }
  
    if (!deviceConnected && oldDeviceConnected) {
      pServer->startAdvertising();
      Serial.println("Restart advertising");
      oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
    }
}

void setupWifi() {
    WiFi.begin(ssid, password);
    Serial.println("Conectando al WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi conectado");
    Serial.println("IP: " + WiFi.localIP().toString());
}

void setupSensors() {
  // Inicialización del MAX30102 pulso oxigenacion
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("Error al inicializar MAX30102");
        while (1);
    }
    Serial.println("MAX30102 inicializado pulso");

    // Configuración avanzada para SpO2 y HR
    byte ledBrightness = 0x3F;  // Intensidad LED (63 en decimal)
    byte sampleAverage = 10;     // Mayor promediado para reducir ruido
    byte ledMode = 2;           // MODE_MULTILED (Rojo + IR)
    int sampleRate = 400;       // Mayor tasa de muestreo (400 Hz)
    int pulseWidth = 118;       // 69μs (óptimo para pulsos)
    int adcRange = 4096;        // Rango del ADC
    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    particleSensor.setPulseAmplitudeRed(0x1F);
    particleSensor.setPulseAmplitudeGreen(0);
    particleSensor.setPulseAmplitudeIR(0x1F);


  // Inicialización del MPU-6050 giro acelerometro
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    Serial.println("MPU-6050 inicializado giro");

  // Inicialización del GY-906 temperatura
    myWire.begin(25, 26);
    if (!mlx.begin(0x5A, &myWire)) {
        Serial.println("Error al inicializar GY-906");
    }
    Serial.println("GY-906 inicializado temp");
}

void readSensors() {
    mpu_read();
    mlx_temp();
    max_datos();
}

void processSensorData() {
    ax = (AcX - 2050) / 16384.00;
    ay = (AcY - 77) / 16384.00;
    az = (AcZ - 1947) / 16384.00;
    gx = (GyX + 270) / 131.07;
    gy = (GyY - 351) / 131.07;
    gz = (GyZ + 136) / 131.07;

    float Raw_Amp = sqrt(ax * ax + ay * ay + az * az);
    int Amp = Raw_Amp * 10;
      Serial.print("Aceleración: ");
	    Serial.println(Amp);

    if (Amp <= 2 && !trigger2) {
        trigger1 = true;
        Serial.println("TRIGGER 1 ACTIVADO");
    }

    if (trigger1) {
        trigger1count++;
        if (Amp >= 12) {
            trigger2 = true;
              Serial.println("TRIGGER 2 ACTIVADO");
            trigger1 = false;
            trigger1count = 0;
        }
    }

    if (trigger2) {
        trigger2count++;
        angleChange = sqrt(gx * gx + gy * gy + gz * gz);
              Serial.println(angleChange);
        if (angleChange >= 30 && angleChange <= 400) {
            trigger3 = true;
            trigger2 = false;
            trigger2count = 0;
              Serial.println("TRIGGER 3 ACTIVADO");
        }
    }

    if (trigger3) {
        trigger3count++;
        if (trigger3count >= 10) {
            angleChange = sqrt(gx * gx + gy * gy + gz * gz);
              Serial.println(angleChange);
            if (angleChange >= 0 && angleChange <= 10) {
                fall = true;
                trigger3 = false;
                trigger3count = 0;
                Serial.println("FALL DETECTED");
                activateAlarms();
                  
                  enviarAlerta(0,0,0,0);
            } else {
                trigger3 = false;
                trigger3count = 0;
                  Serial.println("TRIGGER 3 DEACTIVADO");
            }
        }
    }

    if (trigger2count >= 6) {
        trigger2 = false;
        trigger2count = 0;
          Serial.println("TRIGGER 2 DEACTIVADO");
    }

    if (trigger1count >= 6) {
        trigger1 = false;
        trigger1count = 0;
          Serial.println("TRIGGER 1 DEACTIVADO");
    }

    // Leer temperatura del GY-906
            
            Serial.print("Temperatura Ambiente: ");
	          Serial.print(tempAmb);
	          Serial.print(" °C | Temperatura del Objeto: ");
	          Serial.print(tempObj);
	          Serial.println(" °C");
            delay(10);  // Reducción del delay
}

void mpu_read() {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true);
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    Tmp = Wire.read() << 8 | Wire.read();
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();
}

void mlx_temp() {
    tempObj = mlx.readObjectTempC();
    tempAmb = mlx.readAmbientTempC();
    if(tempObj>41 && tempObj<100){
      enviarAlerta(3,0,0,tempObj);
    }
}

void max_datos() {
    if (particleSensor.check()== true) {
        long irValue = applyFilter(particleSensor.getFIFOIR());
        long redValue = particleSensor.getFIFORed();

        if (irValue > umbralDedo) {

          if (contadorLecturas < 30) {
            contadorLecturas++;
          if (contadorLecturas == 30) {
              umbralDedo = 10000;  // Ajustar a umbral normal después de "calentamiento"
            }
          }

            if (checkForBeat(irValue)== true) {
                long delta = millis() - lastBeat;
                lastBeat = millis();
                beatsPerMinute = 60 / (delta / 1000.0);

                if (beatsPerMinute < 255 && beatsPerMinute > 20) {
                   
                    rates[rateSpot++] = (byte)beatsPerMinute;
                    rateSpot %= RATE_SIZE;

                    beatAvg = 0;
                    for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
                    beatAvg /= RATE_SIZE;

                        Serial.print(" | BPM: ");
                        Serial.print(beatAvg);

                        // Calcula y muestra SpO2
                        float spo2 = calcularSpO2(irValue, redValue);
                        Serial.print(" | SpO2: ");
                        Serial.print(spo2);
                        Serial.println("%");


                    if (Datoxi[maxSize - 1] == 0 && Datpul[maxSize - 1] == 0) {
                        for (int j = 0; j < maxSize; j++) {
                            if (Datoxi[j] == 0) {
                                Datoxi[j] = spo2;
                                Datpul[j] = beatAvg;
                                break;
                            }
                        }
                    } else {
                        for (int j = 0; j < maxSize - 1; j++) {
                            Datoxi[j] = Datoxi[j + 1];
                            Datpul[j] = Datpul[j + 1];
                        }
                        Datoxi[maxSize - 1] = spo2;
                        Datpul[maxSize - 1] = beatAvg;
                    }
                }
            }
        } else {
            Serial.println("No hay dedo detectado");
           for (int j = 0; j < maxSize - 1; j++) {
                            Datoxi[j] = 0;
                            Datpul[j] = 0;
                        } 
        }
        particleSensor.nextSample();
    }
}

void activateAlarms() {
  alertActive = true;
    for (int i = 0; i < 5; i++) {
        digitalWrite(rojo, HIGH);
        digitalWrite(vib, HIGH);
        delay(100);
        digitalWrite(rojo, LOW);
        digitalWrite(vib, LOW);
        delay(100);
        digitalWrite(rojo, HIGH);
        digitalWrite(vib, HIGH);
    }

    for (int i = 0; i < 3; i++) {
        tone(buzzerPin, 2000);
        delay(100);
        noTone(buzzerPin);
        delay(100);
        tone(buzzerPin, 1000);
    }
    noTone(buzzerPin);
    delay(100);
}

void cancelAlert() {
    Serial.println("Alerta cancelada por el usuario");
    alertActive = false;
    fall = false;
    digitalWrite(rojo, LOW);
    digitalWrite(vib, LOW);
    noTone(buzzerPin);
    Serial.print("Se apago");
}

float calcularSpO2(long ir, long red) {
    if (red == 0) return 0;
    float ratio = (float)red / (float)ir;
    return 104 - (15 * ratio);
}

void prepararJsonMetrica() {
    String jsonD;
  StaticJsonDocument<200> doc;
       int promPulsa = 0;
       float promOxi = 0;
    for (int i = 0; i < maxSize; i++) {
        promOxi += Datoxi[i]; 
        Datoxi[i]=0;
        promPulsa += Datpul[i];// Sumar todos los elementos
        Datpul[i]=0;
    }
    promOxi /= maxSize;
    promPulsa /= maxSize;
    if(promPulsa<45 || promPulsa>130){

      Serial.println("Alerta en BPM");
      enviarAlerta(1,promPulsa,0,0);
      delay(1000);

    }else if (promOxi<60 || promOxi>110) {
      enviarAlerta(2,0,promOxi,0);
    }

    doc["tipo"] = "Metricas";
    doc["pulsaciones"] = promPulsa;
    doc["oxigenacion"] = promOxi;
    doc["temperatura"] = tempObj;
    

    serializeJsonPretty(doc, jsonD);
    Serial.println("Enviando datos a Django API:");
    Serial.println(jsonD);
    delay(1000);
    
    enviarjson(jsonD);
    
}

void enviarAlerta(int ta,int puls, float oxi, float temp){
  String jsond;
    StaticJsonDocument<200> docs; 
  switch (ta) 
  {
    
  case 0: 
    docs["tipo"] = "Alerta";
    docs["categoria"] = "caida";
    docs["valor"]="se_cayo";
    break;
  case 1:
    docs["tipo"] = "Alerta";
    docs["categoria"] = "BPM";
    docs["valor"]=puls;
    break;
  case 2:
    docs["tipo"] = "Alerta";
    docs["categoria"] = "Oxigenacion";
    docs["valor"]=oxi;
    break;
  case 3:
    docs["tipo"] = "Alerta";
    docs["categoria"] = "Temperatura";
    docs["valor"]=temp;
    break;
  }
    serializeJsonPretty(docs, jsond);
    Serial.println("Enviando alerta a Django API:");
    Serial.println(jsond);
    delay(1000);
    enviarjson(jsond);
}

void enviarjson(String datos) {
    if (WiFi.status() == WL_CONNECTED) {
      WiFiClient cliente;
       HTTPClient http;
       http.begin(cliente,metricaUrl);
        http.addHeader("Content-Type", "application/json");
      
      int httpResponseCode = http.POST(datos);
       Serial.println(httpResponseCode);
      if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println("Código HTTP: " + String(httpResponseCode));
        Serial.println("Respuesta: " + response);
      } else {
        Serial.println("Error en solicitud HTTP: " + http.errorToString(httpResponseCode));
      }
      delay(5000);
      http.end();
    } else {
      Serial.println("WiFi desconectado. Reconectando...");
      WiFi.reconnect();
      delay(5000);  // Retardo para evitar reconexiones demasiado rápidas
    }
}
