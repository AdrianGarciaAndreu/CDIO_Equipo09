//#include "Funciones.h"
#include "Sensors.h"

SalinitySensor ss(5,0); //Sensor de salinidad, pin 5 = salida, pin 0 = entrada
HumitySensor hr(1); //Sensor de humeddad relativa, pin 1 = entrda analogica
TemperatureSensor tr(2); //Sensor de temperatura, pin 2 = entrada anlogica

PressureSensor pr; //Sensor de presión, SDA = entrada digital, SCL = señal de reloj.

GPSensor gps;

void setup() {

  //Carga los parametros iniciales
  Initialization();
  delay(1500);
  
}

void loop() {
  //getReadings();
  //geoLocating();

  /*ss.readSalinity();
  hr.readHR();
  pr.readPressure();
  tr.readAmbientTemp();
*/
  //Prueba de datos
  Serial.print("HR: "); Serial.print(hr.readHR()); Serial.println("%");
  Serial.print("Salinity: "); Serial.print(ss.readSalinity()); Serial.println("%");
  Serial.print("Pressure: "); Serial.print(pr.readPressure()); Serial.println("mb");
  Serial.print("Temp.: ");  Serial.print(tr.readAmbientTemp()); Serial.println("ºC");
  
  Serial.println("");
  Serial.print("Location: ");  Serial.print(gps.getLocation()); Serial.println("");
  Serial.println();
  delay(1500);


//getReadings();
  
  //NO SE MUESTRAN DATOS POR PANTALLA
  
}
