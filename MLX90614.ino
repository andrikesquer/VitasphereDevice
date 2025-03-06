#include <Wire.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando MLX90614");  

  while (!Serial);

  if (!mlx.begin()) {
    Serial.println("Error conectando el sensor MLX. Revisar conexión.");
    while (1);
  };
}

void loop() {
  Serial.print("Temperatura Ambiente= "); 
  Serial.print(mlx.readAmbientTempC()); Serial.println(" °C");
  Serial.print("Temperatura del Objeto= "); 
  Serial.print(mlx.readObjectTempC()); Serial.println(" °C"); 
  //Serial.print("Temperatura Ambiente = "); 
  //Serial.print(mlx.readAmbientTempF()); Serial.println(" °F"); 
  //Serial.print("Temperatura del Objeto = ");  
  //Serial.print(mlx.readObjectTempF()); Serial.println(" °F"); 
  Serial.println("-----------------------------------------------------------------");
  delay(1000);
}
