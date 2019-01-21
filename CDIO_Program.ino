#include "Sensors.h"

#define DATOS_A_ENVIAR 7 //Numero de medidas a enviar al servidor REST (Entre 1 y 8)


SalinitySensor ss(5,1); //Sensor de salinidad, pin 5 = salida, pin 0 = entrada
HumitySensor hr(0); //Sensor de humeddad relativa, pin 1 = entrda analogica
TemperatureSensor tr(2); //Sensor de temperatura, pin 2 = entrada anlogica
PressureSensor pr; //Sensor de presión, SDA = entrada digital, SCL = señal de reloj.
//LuminitySensor Ls(3);

//Sensor GPS, y posición por defecto 0,0 hasta que no conecte con NEMA
GPSensor gps; 
String gpsLat = "0.00000" ,gpsLong = "0.00000";

//Variables de resultados que visualizar o enviar al servidor
String finalHR, finalSalinity, finalAmbientTemp, finalPressure, finalMovement, finalLat, finalLong;



void processData(){

  
  finalHR = hr.readHR();
  finalSalinity = ss.readSalinity();
  finalAmbientTemp = tr.readAmbientTemp();
  finalPressure = pr.readPressure();
  int res = readPosition();
  finalMovement = "0";
   if(res>lastCounter){
      finalMovement = "1";
      lastCounter=res;
    }
   String gpsTmp = gps.getLocation();
    if(gpsTmp.length()>1){ //Si la lectura del GPS devuelve coordenadas...
       gpsLat = gpsTmp.substring(0,gpsTmp.indexOf(";"));
       gpsLong = gpsTmp.substring((gpsTmp.indexOf(";")+1),gpsTmp.length());
      }
    finalLat=gpsLat;
    finalLong=gpsLong;
  
}



void setup() {

  //Carga los parametros iniciales
  Initialization();
  delay(1500);

}



void loop() {

  processData(); //procesar los datos de los sensores
  bool PrintDebug=false;


  if (!PrintDebug){
  //    Envio de datos a un servidor a través de HTTP

    String datos[ DATOS_A_ENVIAR + 1]; //Cadena de datos a enviar al servidor    
    datos[1] = String(finalHR); //Se envia el dato de humedad
    datos[2] = String(finalSalinity); //Se envía el dato de salinidad
    datos[3] = String(pr.readPressure()); //Se envía el dato de presión atmosferica
    datos[4] = String(finalAmbientTemp); //Se envía el dato de temperatura ambiente
    datos[5] = String(finalMovement); //Se envía el dato de detección de movimiento
    datos[6] = String(finalLat); //Se envía el dato de latitud en función del GPS
    datos[7] = String(finalLong); //Se envía el dato de longitud en función del GPS
    
    //HTTPPost( datos, DATOS_A_ENVIAR );
    HTTPGet( datos, DATOS_A_ENVIAR ); //Se envían los datos vía GET.

   delay(5000);
   
   Serial.println("ESP8266 dormido...");
   ESP.deepSleep(1200 * 1000000); 

  } else {
  //Se comprueba si el dispositivo está en movimiento

   Serial.print("HR: "); Serial.print(finalHR); Serial.println("%");
   Serial.print("Salinity: "); Serial.print(finalSalinity); Serial.println("%");
   Serial.print("Temp: ");  Serial.print(finalAmbientTemp); Serial.println("ºC");
   Serial.print("Pressure: "); Serial.print(finalPressure); Serial.println("mb");
   //Serial.print("Iluminacion: "+Ls.readLuminity());  
   Serial.println("");
   Serial.print("Localización --> lat:"+finalLat+", lang:"+finalLong); Serial.println("");
   Serial.print("Movimiento: "+finalMovement); Serial.println("");
   Serial.println("");

   
   Serial.println("");Serial.println("");

     delay(2000);
  }
 

  

}
