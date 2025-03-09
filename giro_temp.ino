#include <Wire.h>
#include <Adafruit_MLX90614.h> // Librería para el GY-906
#include "MAX30105.h"
#include "heartRate.h"


//Sensores que se utilizaran 
const int MPU_addr = 0x68; // Dirección I2C del MPU-6050 ace gri
Adafruit_MLX90614 mlx = Adafruit_MLX90614(); // Instancia del sensor GY-906 temp
MAX30105 particleSensor;

//Variables para la medicion de los sensores 
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;// Datos crudos de acerelarcion y giroscopio 
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0; //Normalizacion de datos de ace y gir
boolean fall = false, trigger1 = false, trigger2 = false, trigger3 = false;
byte trigger1count = 0, trigger2count = 0, trigger3count = 0;
int angleChange = 0;

//Variables para la teperatura
float tempAmb, tempObj;

//Medicion de pulso, oxigenacion  etc.
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;
float spo2;
unsigned long startTimeBPM = 0;
unsigned long startTimeSpO2 = 0;
bool bpmAlertReady = false;
bool spo2AlertReady = false;


//intervalo de timepo entre medicion de los sensores
const int sampleInterval = 10; 
const int ALERT_TIME=1000;
bool alertActive = false;

//Pines que se utilizaran 
const int buzzerPin=5; 
const int ledPin=4; 
const int motorvib=12;
const int buttonPin=15;



void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Inicialización del MPU-6050
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // Despertar el MPU-6050
    Wire.endTransmission(true);
    Serial.println("MPU-6050 inicializado");

    // Inicialización del GY-906 
    if (!mlx.begin()) {
        Serial.println("Error al inicializar el GY-906");
        while (1);
    }
    Serial.println("GY-906 inicializado");

    //
   





}

void loop() {
      if (digitalRead(buttonPin) == LOW && alertActive) {
    	cancelAlert();
  }
    if (fall) { 
        Serial.println("FALL DETECTED");
        activateAlarms();
    } 
    else{

    


    static unsigned long lastTime = 0;
     if (millis() - lastTime >= sampleInterval) {
	  lastTime = millis();
    //Logica de pulso y de oximetro 

    




    //Logica de giroscopio
    mpu_read();
    ax = (AcX - 2050) / 16384.00;
    ay = (AcY - 77) / 16384.00;
    az = (AcZ - 1947) / 16384.00;
    gx = (GyX + 270) / 131.07;
    gy = (GyY - 351) / 131.07;
    gz = (GyZ + 136) / 131.07;

    // Calcular vector de amplitud
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
            trigger1 = false; trigger1count = 0;
        }
    }

    if (trigger2) {
        trigger2count++;
        angleChange = sqrt(gx * gx + gy * gy + gz * gz);
        Serial.println(angleChange);
        if (angleChange >= 30 && angleChange <= 400) { 
            trigger3 = true;
            trigger2 = false; trigger2count = 0;
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
                trigger3 = false; trigger3count = 0;
                Serial.println("FALL DETECTED");
            } else { 
                trigger3 = false; trigger3count = 0;
                Serial.println("TRIGGER 3 DEACTIVADO");
            }
        }
    }

    

    if (trigger2count >= 6) { 
        trigger2 = false; trigger2count = 0;
        Serial.println("TRIGGER 2 DEACTIVADO");
    }

    if (trigger1count >= 6) { 
        trigger1 = false; trigger1count = 0;
        Serial.println("TRIGGER 1 DEACTIVADO");
    }

    // Leer temperatura del GY-906
    
    mlx_temp();
    Serial.print("Temperatura Ambiente: ");
    Serial.print(tempAmb);
    Serial.print(" °C | Temperatura del Objeto: ");
    Serial.print(tempObj);
    Serial.println(" °C");
    

    delay(100);
}}}

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
void mlx_temp(){
 tempObj = mlx.readObjectTempC();
     tempAmb = mlx.readAmbientTempC();
}

// Función para activar las alarmas
void activateAlarms() {
  // Patrón de parpadeo del LED
  for (int i = 0; i < 5; i++) {
	digitalWrite(ledPin, HIGH);
	delay(100);
	digitalWrite(ledPin, LOW);
	delay(100);
  }
 
  // Mantener LED encendido mientras la alerta esté activa
  digitalWrite(ledPin, HIGH);
 
  // Activar buzzer con patrón de alarma
  for (int i = 0; i < 3; i++) {
	// Tono agudo
	tone(buzzerPin, 2000);
	delay(300);
	// Tono grave
	tone(buzzerPin, 1000);
	delay(300);
  }
  noTone(buzzerPin);
  delay(10000);
}

// Función para cancelar la alerta
void cancelAlert() {
  Serial.println("Alerta cancelada por el usuario.");
  alertActive = false;
  fall = false;
 
  // Apagar indicaciones
  digitalWrite(ledPin, LOW);
  noTone(buzzerPin);
}

