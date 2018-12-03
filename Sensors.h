#include <Wire.h>

#include <Adafruit_ADS1015.h>
#include <SFE_BMP180.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

//Parametros del sensorGPS, se entiende que solo habrá un sensor GPS
#define RX_PIN 12
#define TX_PIN 13
#define INIT_PIN 15
#define GPS_BAUD 4800  //  velocidad de comunicación serie 


Adafruit_ADS1115 ads1115(0x48); // instanciación de ADS utilizado.
SFE_BMP180 pr_bmp180; // Instancia del sensor de presion.
SoftwareSerial sfSerial(RX_PIN, TX_PIN);


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
   if(pr_bmp180.begin())
    Serial.println("BMP180 iniciado correctamente");
   else{
    Serial.println("Error al iniciar el BMP180");
  }


  //GPS
  sfSerial.begin(GPS_BAUD); // Inicializar la comunicación con el GPS
  pinMode(INIT_PIN,OUTPUT); 

  
}

// Función de mapeado de lecturas, a diferencia de la nativa de arduino, esta contempla numeros decimales.
double mapDecimal(double x, double in_min, double in_max, double out_min, double out_max)
{
 double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
 return temp;
}


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

    resultGPS = gpsDate;
    resultGPS = resultGPS + ", ";
    resultGPS = resultGPS + gpsTime;
  }
  else  // Si no recibe los mensajes
  {
    resultGPS = ("Satellites in view: ");
    resultGPS = (gps.satellites.value());
  }
  gpsDelay(1500);

  return resultGPS;
}
