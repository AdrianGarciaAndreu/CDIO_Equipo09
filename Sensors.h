#include <Wire.h>



#include <Adafruit_ADS1015.h>

#include <SFE_BMP180.h>

#include <TinyGPS++.h>

#include <SoftwareSerial.h>

#include <ESP8266WiFi.h>





//Parametros del sensorGPS, se entiende que solo habrá un sensor GPS

#define RX_PIN 12 //Pin de lectura (UART virtual)

#define TX_PIN 13 //Pin de escritura (UART virtual)

#define INIT_PIN 15 //Pin para enviar señal de inicio/fin

#define GPS_BAUD 4800  //  velocidad de comunicación serie 





//Definición de direcciones para el sensor de aceleración y giroscópio.

#define    MPU9250_ADDRESS            0x68

#define    MAG_ADDRESS                0x0C

 

#define    GYRO_FULL_SCALE_250_DPS    0x00  

#define    GYRO_FULL_SCALE_500_DPS    0x08

#define    GYRO_FULL_SCALE_1000_DPS   0x10

#define    GYRO_FULL_SCALE_2000_DPS   0x18

 

#define    ACC_FULL_SCALE_2_G        0x00  

#define    ACC_FULL_SCALE_4_G        0x08

#define    ACC_FULL_SCALE_8_G        0x10

#define    ACC_FULL_SCALE_16_G       0x18



#define interruptPin  4 //pin para manejar las interrupciones

volatile int intCounter = 0; //contador de interrupciones
int lastCounter = 0; //última cuenta de interrupción mostrada por pantalla.



////////////////////////////////

//////Parametros conexión WiFi

////////////////////////////////

// Comentar/Descomentar para ver mensajes de depuracion en monitor serie y/o respuesta del HTTP server
#define PRINT_DEBUG_MESSAGES
#define PRINT_HTTP_RESPONSE


//#define WiFi_CONNECTION_UPV //comentar en caso de no usar la red de la UPV


// Selecciona que servidor REST quieres utilizar entre ThingSpeak y Dweet
//#define REST_SERVER_THINGSPEAK //Selecciona tu canal para ver los datos en la web (https://thingspeak.com/channels/360979)
//#define REST_SERVER_DWEET //Selecciona tu canal para ver los datos en la web (http://dweet.io/follow/PruebaGTI)

#define REST_DEV_TEAM08 //Conectamos con nuestro ordenador


//Conexion

#ifdef WiFi_CONNECTION_UPV //Conexion UPV

  const char WiFiSSID[] = "GTI1";

  const char WiFiPSK[] = "1PV.arduino.Toledo";

#else //Conexion fuera de la UPV

  const char WiFiSSID[] = "Redmi-Adrian"; //nombre de la red de conexión
 
  const char WiFiPSK[] = "carnefresca"; //contraseña de la red de conexion

#endif



//Servidor de envío

#if defined(WiFi_CONNECTION_UPV) //Conexion UPV

  const char Server_Host[] = "proxy.upv.es";

  const int Server_HttpPort = 8080;

#elif defined(REST_SERVER_THINGSPEAK) //Conexion fuera de la UPV

  const char Server_Host[] = "api.thingspeak.com";

  const int Server_HttpPort = 80;

#elif defined(REST_DEV_TEAM08) //Conexion de desarrollo para la asignatura de Proyecto Web

  //const IPAddress Server_Host(192,168,0,155);
    const char Server_Host[] = "adgaran1.upv.edu.es";

  const int Server_HttpPort = 80;

#else

  const char Server_Host[] = "dweet.io";

  const int Server_HttpPort = 80;

#endif





Adafruit_ADS1115 ads1115(0x48); // instanciación de ADS utilizado.

SFE_BMP180 pr_bmp180; // Instancia del sensor de presion.

SoftwareSerial sfSerial(RX_PIN, TX_PIN); //Instancia de UART virtual

WiFiClient client; //Instancia de conexión WiFi





#ifdef REST_SERVER_THINGSPEAK 

  const char Rest_Host[] = "api.thingspeak.com";

  String MyWriteAPIKey="5ZKIZ5IY6YOY7M0T"; // Escribe la clave de tu canal ThingSpeak

#elif defined(REST_DEV_TEAM08)
  const char Rest_Host[] = "adgaran1.upv.edu.es/api/v4.0/medidas";
  String MyWriteAPIKey="";
  
#else 

  const char Rest_Host[] = "dweet.io";

  String MyWriteAPIKey="PruebaGTI"; // Escribe la clave de tu canal Dweet

#endif




/*

 * Función alternativa de lectura

 */

void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)

{

   Wire.beginTransmission(Address);

   Wire.write(Register);

   Wire.endTransmission();

 

   Wire.requestFrom(Address, Nbytes);

   uint8_t index = 0;

   while (Wire.available())

   {

      Data[index++] = Wire.read();

   }

}

 



/* 

 * Funcion alternativa de escritura 

 */

void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)

{

   Wire.beginTransmission(Address);

   Wire.write(Register);

   Wire.write(Data);

   Wire.endTransmission();

}







/*

 * Función para manejar interrupción producida por el sensor de aceleración. 

 * Está lleva una cuenta de interrupciones y envía/muestra una señal para indicar

 * el movimiento del dispositivo.

 */

 void handleInterrupt()

{

  uint8_t intStatus[1];

  I2Cread(MPU9250_ADDRESS,0x3A, 1, intStatus); //leemos de intStatus para desactivar la interrupcion
  intCounter++;


} 











/*

 * Función para conectar WiFi

 */

void connectWiFi()

{



  #ifdef PRINT_DEBUG_MESSAGES

    Serial.print("MAC: ");

    Serial.println(WiFi.macAddress());

  #endif

  

  WiFi.begin(WiFiSSID, WiFiPSK);



  while (WiFi.status() != WL_CONNECTED)

  {

    #ifdef PRINT_DEBUG_MESSAGES
       Serial.println(".");

    #endif

    delay(500);

  }

  #ifdef PRINT_DEBUG_MESSAGES

     Serial.println( "WiFi Connected" );

     Serial.println(WiFi.localIP()); // Print the IP address

  #endif

}









/*

 * Inicializacion de conexiones y parametros

 */

 

void Initialization(){



  //Inicia comunicacionn serie

  Serial.begin(9600);



  //Inicia la comunicacion con el ADS 1115

  ads1115.begin(); //Initialize ads1115

  ads1115.setGain(GAIN_ONE); // Rango del ADC: +/- 4.096V (1 bit=2mV)



  //Se inicia la comunicación con el sensor de presión BMP 180

   if(pr_bmp180.begin()){}
   else{Serial.println("Error al iniciar el BMP180");}



  //GPS

  sfSerial.begin(GPS_BAUD); // Inicializar la comunicación con el GPS

  pinMode(INIT_PIN,OUTPUT); 



  //Iniciar comunicación con el acelerometro

  Wire.begin();

  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_2_G);

  pinMode(interruptPin,INPUT_PULLUP); //Configuración del pin que este usara para manejar interrupciones
  attachInterrupt(digitalPinToInterrupt(interruptPin),handleInterrupt, FALLING); //Condicionamiento de función que manejará las interrupciones

  I2CwriteByte(MPU9250_ADDRESS, 0x6B, 0);

  I2CwriteByte(MPU9250_ADDRESS, 0x6C, 7);

  I2CwriteByte(MPU9250_ADDRESS, 0x1D, 9);

  

  I2CwriteByte(MPU9250_ADDRESS, 0x38, 64);

  I2CwriteByte(MPU9250_ADDRESS, 0x69, 192);

  I2CwriteByte(MPU9250_ADDRESS, 0x1F, 15);

  I2CwriteByte(MPU9250_ADDRESS, 0x1E, 2);

  I2CwriteByte(MPU9250_ADDRESS, 0x6B, 32);

  I2CwriteByte(MPU9250_ADDRESS, 0x37, 224);
  Serial.println("Acelerometro OK");



 connectWiFi(); //Inicia conexión WiFi.



  

}



// Función de mapeado de lecturas, a diferencia de la nativa de arduino, esta contempla numeros decimales.

double mapDecimal(double x, double in_min, double in_max, double out_min, double out_max)

{

 double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

 return temp;

}





/*

 * Función para enviar datos vía HTTP POST al servidor. 

 */

void HTTPPost(String fieldData[], int numFields){

        Serial.println("HOST: "+String(Server_Host));
        Serial.println("PORT: "+String(Server_HttpPort));
    
    int tmpConn = client.connect( Server_Host , Server_HttpPort );
    if (tmpConn){


        //Se construye la cadena de caracteres a enviar como parámetros.   

        

       // String PostData= "api_key=" + MyWriteAPIKey ;
        String PostData= "" ;

        for ( int field = 1; field < (numFields + 1); field++ ){

            PostData += String( field ) + "=" + fieldData[ field ] + "&";

        }     

            PostData.remove(PostData.length()-1);
        

        // POST data via HTTP

        client.println( "POST http://" + String(Rest_Host) + " HTTP/1.1" );

        client.println( "Host: " + String(Rest_Host) );

        client.println( "Connection: close" );

        client.println( "Content-Type: application/x-www-form-urlencoded" );

        client.println( "Content-Length: " + String( PostData.length() ) );

        client.println();

        client.println( PostData );

      

            Serial.println( PostData ); //Muestra por pantalla los datos enviados al servidor //SOLO PARA DEPURAR

           

            //En caso de que el servidor responda, dicha respuesta se envia por pantalla

            #ifdef PRINT_HTTP_RESPONSE

              delay(500);

              Serial.println();

              while(client.available()){String line = client.readStringUntil('\r');Serial.print(line); }

              Serial.println();

              Serial.println();

            #endif

    } else{
      
        Serial.println("Problema en la conexion con el HOST");
        Serial.println(tmpConn);
        
      }

}





/*

 * Función para enviar datos vía HTTP GET al servidor. 

 */

void HTTPGet(String fieldData[], int numFields){

    

    if (client.connect( Server_Host , Server_HttpPort )){

           #ifdef REST_SERVER_THINGSPEAK 
              String PostData= "GET https://api.thingspeak.com/update?api_key=";
              PostData= PostData + MyWriteAPIKey ;
           #else 
              String PostData= "GET http://dweet.io/dweet/for/";
              PostData= PostData + MyWriteAPIKey +"?" ;
           #endif





           //Construye la cadena de envío

           

           for ( int field = 1; field < (numFields + 1); field++ ){

              PostData += "&field" + String( field ) + "=" + fieldData[ field ];

           }

          

           //Envía datos al servidor

           client.print(PostData);         

           client.println(" HTTP/1.1");

           client.println("Host: " + String(Rest_Host)); 

           client.println("Connection: close");

           client.println();

           

              Serial.println( PostData ); //Escribe cadena enviada al servidor vía POST //SOLO PARA DEPURAR

              Serial.println();

           

              //Sí hay respuesta del servidor, se escribe por el monitor serie

              #ifdef PRINT_HTTP_RESPONSE

                delay(500);

                Serial.println();

                while(client.available()){String line = client.readStringUntil('\r');Serial.print(line); }

                Serial.println();

                Serial.println();

              #endif

           

    }

}











////////////////////////////////////

////// Sensores 

///////////////////////////////////



/*

 * Clase para definir sensores de salinidad

 * Se construyen mediante un pin de estimulación y uno de lectura (analogica), se establecen unos minimos y máximos de rango

 * y se captan lecturas.

 */

class SalinitySensor{

  private:

   int power_pin, input_pin; // pin de estimulación (salida), y pin de recepción de datos analógicos (entrada)

   int sensorMin, sensorMax; //margenes minimo/maximo del sensor.

   //int addr; //dirección del ADS a utilizar

  public:

    //Constructor, los parametros a introducir son el pin de salida y el pin de entrada analogico, dirección del ADS utilizado

    SalinitySensor(int,int);

    void setMinMax(int,int); //establece minimo/maximo a contemplar

    int readSalinity(); //Obtiene una medida de salinidad

}; 





SalinitySensor::SalinitySensor(int pin_out, int pin_in){

  power_pin=pin_out;

  input_pin=pin_in;



  //configuramos el modo de los pines 

  pinMode(power_pin, OUTPUT); // Pin en modo salida, para el sensor de salinidad

  

  sensorMin = 15000; //valor por defecto de AGUA DULCE

  sensorMax = 30000; //valor por defecto de lo considerado como AGUA SALADA

  

  //ads1115.m_i2cAddress = ads_addr;

}



/*

 * Función para re-calibrar el rango de valores del sensor de salinidad

 */

void SalinitySensor::setMinMax(int minimum, int maximum){

  sensorMin=minimum;

  sensorMax=maximum;

}



/*

 * Función para tomar medida con el sensor, se devuelve como un número entero.

 */

int SalinitySensor::readSalinity(){

  int readingS, resultS;

  

  int const SALT_MIN = 0;

  int const SALT_MAX = 100;

  

  digitalWrite(power_pin, HIGH);

  delay(100);

  readingS = ads1115.readADC_SingleEnded(input_pin);

  digitalWrite(power_pin, LOW);

  delay(100);



  resultS = map(readingS,sensorMin, sensorMax, 0, 100);



  if (resultS<SALT_MIN){

      resultS=SALT_MIN;

      }

  if (resultS>SALT_MAX){

      resultS=SALT_MAX;

      }



  return resultS;

}









/*

 * Clase para definir sensores de humedad

 */

class HumitySensor{

  private:

   int input_pin; // pin de recepción de datos analógicos (entrada)

   int sensorMin, sensorMax; //margenes minimo/maximo del sensor.

   //int addr; //dirección del ADS a utilizar

  public:

    //Constructor, los parametros a introducir son el pin de entrada analogico.

    HumitySensor(int);

    void setMinMax(int,int); //establece minimo/maximo a contemplar

    int readHR(); //Obtiene una medida de la humedad relativa

}; 





/*

 * constructor del sensor, establece el pin de entrada, y unos valores de rango por defecto

 * en función de la calibración al sensor utilizado (%NOMBRE/MODELO_SENSOR%)

 */

HumitySensor::HumitySensor(int pin_in){

  input_pin=pin_in;

  sensorMin = 12341; //Valor por defecto del sensor en agua (HR=100%)

  sensorMax = 20200; //Valor por defecto del sensor en seco

}



/*

 * Función para re-calibrar el rango de valores del sensor de humedad;

 */

void HumitySensor::setMinMax(int minimum, int maximum){

  sensorMin=minimum;

  sensorMax=maximum;

}



/*

 * Función para tomar medida con el sensor, se devuelve como un número entero.

 */

int HumitySensor::readHR(){

  int readingHr, resultHr;

  

  int const HR_MIN = 0;

  int const HR_MAX = 100;

  

  readingHr = ads1115.readADC_SingleEnded(input_pin);

  resultHr = map(readingHr,sensorMin, sensorMax, 100, 0);



  if (resultHr<HR_MIN){

      resultHr=HR_MIN;

      }

  if (resultHr>HR_MAX){

      resultHr=HR_MAX;

      }


  //Serial.print("HR tmp: "); Serial.println(readingHr);
  return resultHr;

}









/*

 * Clase para definir sensores de salinidad

 * Se construyen mediante un pin de estimulación y uno de lectura (analogica), se establecen unos minimos y máximos de rango

 * y se captan lecturas.

 */



class PressureSensor{

  public:

    double readPressure();

};





/*

 * Función para obtener una medida de presión (en milibares)

 * lectura digital

 */

double PressureSensor::readPressure(){

 char status;

 double T,P;
 T=0.00;
 P=0.00;


  // Empezamos a medir Temperatura

  status = pr_bmp180.startTemperature();



  if (status != 0)

  {

    // Wait for the measurement to complete:

    delay(status);

    status = pr_bmp180.getTemperature(T);

    // Mostramos por pantalla el valor leido

    if (status !=0)

    {

      status = pr_bmp180.startPressure(3);

      if(status!=0){

        

        delay(status);

        status = pr_bmp180.getPressure(P,T);

        if(status!=0){

          

        } // Fin sobre la lectura de temperatura

        

      } // Fin sobre la inicializacion de presion

      

     } // Fin sobre la lectura de temperatura

  } //Fin sobre la inicializacion de temperatura

  else Serial.println("error starting temperature measurement\n");



 return P;



}





/*

 * Clase para definir sensores de de temperatura (LDR).

 * sensor analogico, con alimentación de 3.3V

 */

class TemperatureSensor{

  private:

   int input_pin; // pin de recepción de datos analógicos (entrada)

   int sensorMin, sensorMax; //margenes minimo/maximo del sensor.

   //int addr; //dirección del ADS a utilizar

  public:

    //Constructor, los parametros a introducir son el pin de entrada analogico.

    TemperatureSensor(int);

    void setMinMax(int,int); //establece minimo/maximo a contemplar

    float readAmbientTemp(); //Obtiene una medida de la humedad relativa

}; 





/*

 * Constructor del sensor de temperatura, establece un pin de entrada para captar los datos analogicos

 * y unos valores por defecto de calibración (en función a las medidas tomadas)

 */

TemperatureSensor::TemperatureSensor(int pin_in){

  this->sensorMin=6000; //Valor por defecto de temperaturas consideradas como 0º tomadas con la LDR

  this->sensorMax=17256; //Valor por defecto de temperaturas consideradas como 40º tomadas con la LDR

  

  this->input_pin = pin_in;

}





/*

 * Función para re-calibrar el rango de valores del sensor de humedad;

 */

void TemperatureSensor::setMinMax(int minimum, int maximum){

  this->sensorMin=minimum; 

  this->sensorMax=maximum;

}



/*

 * Función para tomar medida con el sensor, se devuelve como un número decimal (coma flotante).

 */

float TemperatureSensor::readAmbientTemp(){

  int reading, result;

    

    reading = ads1115.readADC_SingleEnded(this->input_pin);

    result = mapDecimal(reading,this->sensorMin, this->sensorMax, 0, 40);

  //Serial.print("temp tmp: "); Serial.println(reading);


  return result;

}







class GPSensor{

  private:

   TinyGPSPlus gps;

  public:

    void gpsDelay(unsigned long);

    void switchOnOff();

    String getLocation();

    GPSensor();

  };





// Función espera 1s para leer del GPS

void GPSensor::gpsDelay(unsigned long ms)

{

  unsigned long start = millis();

  do

  {

    while(sfSerial.available())

    {

      gps.encode(sfSerial.read());  // leemos del gps

    }

  } while(millis() - start < ms);

}





//Función para encender/apagar mediante un pulso

void GPSensor::switchOnOff()

{

   // Power on pulse

  digitalWrite(INIT_PIN,LOW);

  delay(200);

  digitalWrite(INIT_PIN,HIGH);

  delay(200); 

  digitalWrite(INIT_PIN,LOW);

 }





GPSensor::GPSensor(){

    this->switchOnOff(); // Pulso para encender el GPS

}



String GPSensor::getLocation(){

  

  String resultGPS;

  

  char gpsDate[10]; 

  char gpsTime[10];



  if(gps.location.isValid()){ // Si el GPS está recibiendo los mensajes NMEA

    sprintf(gpsDate,"%d/%d/%d", gps.date.month(),gps.date.day(),gps.date.year()); // Construimos string de datos fecha

    sprintf(gpsTime,"%d/%d/0%d", gps.time.hour(),gps.time.minute(),gps.time.second());  // Construimos string de datos hora



    Serial.print(gps.location.lat(),6);

    Serial.print('\t');

    Serial.print(gps.location.lng(),6);

    Serial.print('\t');





    //resultGPS = "Location: ";
    //resultGPS = resultGPS + gpsDate;
    //resultGPS = resultGPS + ", ";
    //resultGPS = resultGPS + gpsTime + ", ";
    //resultGPS = resultGPS + gps.location.lat() + " lat, " +gps.location.lng()+ " lang";
    
    resultGPS = "";
    resultGPS = resultGPS + gps.location.lat()+";"+gps.location.lng();

  }

  else  // Si no recibe los mensajes

  {

    //resultGPS = ("Satelites GPS captados: ");
    resultGPS = resultGPS + (gps.satellites.value());

  }

  gpsDelay(1500);



  return resultGPS;

}





/*

 * Clase para definir sensores de iluminación

 */

class LuminitySensor{

  private:

   int input_pin; // pin de recepción de datos analógicos (entrada)

   int sensorMin, sensorMax; //margenes minimo/maximo del sensor.

   //int addr; //dirección del ADS a utilizar

  public:

    //Constructor, los parametros a introducir son el pin de entrada analogico.

    LuminitySensor(int);

    void setMinMax(int,int); //establece minimo/maximo a contemplar

    int readLuminity(); //Obtiene una medida de la iluminación relativa

}; 





/*

 * constructor del sensor, establece el pin de entrada, y unos valores de rango por defecto

 * en función de la calibración al sensor utilizado (%NOMBRE/MODELO_SENSOR%)

 */

LuminitySensor::LuminitySensor(int pin_in){

  this->input_pin=pin_in;

  //this->sensorMin = 100; //Valor por defecto del sensor en oscuridad (tapado)
  //this->sensorMax = 30000; //Valor por defecto del sensor a plena luz de un día soleado

  this->sensorMin = 0; //Valor por defecto del sensor en oscuridad (tapado)
  this->sensorMax = 25000; //Valor por defecto del sensor a plena luz de un día soleado

}



/*

 * Función para re-calibrar el rango de valores del sensor de humedad;

 */

void LuminitySensor::setMinMax(int minimum, int maximum){

  this->sensorMin=minimum;

  this->sensorMax=maximum;

}



/*

 * Función para tomar medida con el sensor, se devuelve como un número entero.

 */

int LuminitySensor::readLuminity(){

  int readingLu, resultLu;

  String resultLuToSend;

  

  int const LU_MIN = 0;

  int const LU_MAX = 100;

  

  readingLu = ads1115.readADC_SingleEnded(this->input_pin);
  //Serial.print("IL Anag: "); Serial.println(readingLu);

  resultLu = map(readingLu,this->sensorMin, this->sensorMax, 0, 100);



  if (resultLu<LU_MIN ){

      resultLu=LU_MIN ;

      }

  if (resultLu>LU_MAX){

      resultLu=LU_MAX;

      }



  if (resultLu<20){

    resultLuToSend = "Poca luz";

  }

  else if(resultLu<80){

    resultLuToSend = "Luz indirecta o escasa";

  }

  else {

    resultLuToSend = "Luz directa";

  }



  return resultLu;

}









/*

 * Funció para leer y mostrar datos de posicionamiento locales, acelerometro/giroscopio

 */

int readPosition(){

    int result;

    result = intCounter;
    delay(500);
    
    //Serial.println("Leemos del acelerometro");
   // Serial.print("Número de interrupciones");
    //Serial.println(intCounter, DEC);

    

   // ---  Lectura acelerometro y giroscopio --- 

   uint8_t Buf[14];

   int FS_ACC=2;

   //int FS_GYRO=2000;

   I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

 

   // Convertir registros acelerometro

   float ax = (Buf[0] << 8 | Buf[1]);

   float ay = (Buf[2] << 8 | Buf[3]);

   float az = Buf[4] << 8 | Buf[5];

 

   ax=ax*FS_ACC/32768;

   ay=ay*FS_ACC/32768;

   az=az*FS_ACC/32768;
 

   // --- Mostrar valores ---

 

   // Acelerometro

 /*  Serial.println("Lectura Acelerometro");

   Serial.print("AX="); Serial.print(ax, 2); Serial.print("g"); Serial.print("\t");

   Serial.print("AY="); Serial.print(ay, 2); Serial.print("g"); Serial.print("\t");

   Serial.print("AZ="); Serial.print(az, 2); Serial.println("g");

   Serial.println("");

  */


  //movimientos considerados intencionados (desplazamiento forzoso del dispositivo)

  /*
   if(ax>3 || ay>3 || az>1.5){
    result = "DISPOSITIVO EN MOVIMIENTO ";
    myInterrupts++;
    //delay(500);

    }
  */

  return result;

  }
