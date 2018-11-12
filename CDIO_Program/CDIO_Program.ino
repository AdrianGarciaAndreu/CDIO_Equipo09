int power_pin = 5;

#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <SFE_BMP180.h>

#include "Funciones.h"


void setup() {

  //Carga los parametros iniciales
  Initialization();  
}

void loop() {


//Muestra datos por el monitor serie
/*Serial.print("HR (%): ");
Serial.print(getHR(WaterValue, AirValue)); Serial.println("%");

Serial.print("Salinidad: ");
Serial.print(getSalinity(DryValue, SaltWaterValue)); Serial.println("%");
Serial.println("");
Serial.println("");
*/

getPR();
delay(3000);

}
