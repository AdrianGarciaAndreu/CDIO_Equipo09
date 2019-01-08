#include "Sensors.h"

#define DATOS_A_ENVIAR 5 //Numero de medidas a enviar al servidor REST (Entre 1 y 8)


SalinitySensor ss(5,1); //Sensor de salinidad, pin 5 = salida, pin 0 = entrada
HumitySensor hr(0); //Sensor de humeddad relativa, pin 1 = entrda analogica
TemperatureSensor tr(2); //Sensor de temperatura, pin 2 = entrada anlogica

PressureSensor pr; //Sensor de presión, SDA = entrada digital, SCL = señal de reloj.
//GPSensor gps;
LuminitySensor Ls(3);



void setup() {

  //Carga los parametros iniciales
  Initialization();
  delay(1500);

}



void loop() {





  //    Envio de datos a un servidor a través de HTTP
/*
    String datos[ DATOS_A_ENVIAR + 1]; //Cadena de datos a enviar al servidor    
    datos[1] = String(hr.readHR()); //Se envia el dato de humedad
    datos[2] = String(ss.readSalinity()); //Se envía el dato de salinidad
    datos[3] = String(pr.readPressure()); //Se envía el dato de presión atmosferica
    datos[4] = String(tr.readAmbientTemp()); //Se envía el dato de temperatura ambiente
    datos[5] = String(Ls.readLuminity()); //Se envía el dato de iluminación

    //HTTPPost( datos, DATOS_A_ENVIAR );
    HTTPGet( datos, DATOS_A_ENVIAR ); //Se envían los datos vía GET.

*/





 

  //Se comprueba si el dispositivo está en movimiento



   Serial.print("HR: "); Serial.print(hr.readHR()); Serial.println("%");
   Serial.print("Salinity: "); Serial.print(ss.readSalinity()); Serial.println("%");
   Serial.print("Temp.: ");  Serial.print(tr.readAmbientTemp()); Serial.println("ºC");
   Serial.print("Pressure: "); Serial.print(pr.readPressure()); Serial.println("mb");
   Serial.print("Iluminacion: "+Ls.readLuminity());  Serial.println("");


    //Detección de movimiento mediante un contador de interrupciones
    int res = readPosition();
    if(res>lastCounter){
      Serial.println("Dispositivo en movimiento");
      lastCounter=res;
    }
   
   Serial.println("");

   // Serial.print(gps.getLocation()); Serial.println("");
   // Serial.println("");


  // Serial.println("ESP8266 in sleep mode");
   ESP.deepSleep(10 * 1000000);

  //delay(1000);
  

}
