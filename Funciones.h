#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <SFE_BMP180.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define vRX_PIN 12 //PIN UART virtual de lectura del sensor GPS
#define vTX_PIN 13 //PIN UART virtual de transmisión del sensor GPS
#define initGPS_PIN 15 //PIN para iniciar/interrumpir la comunicación con el sensor GPS

int power_pin = 5; //Pin usado para el sensor de salinidad //PIN con capacidad de generar interrupciones



Adafruit_ADS1115 ads1115(0x48); // construct an ads1115 at address 0x48
SFE_BMP180 pr_bmp180; // Instancia del sensor de presion.
TinyGPSPlus gps; //Instancia del sensor GPS
SoftwareSerial sfSerial(vRX_PIN, vTX_PIN); //Instancia de la UART virtual.



const int AirValue = 20200;  // Medimos valor en seco
const int WaterValue = 12341;  // Medimos valor en agua

const int DryValue = 15000;  // Medimos valor en seco
const int SaltWaterValue = 30000;  // Medimos valor en agua con un dedo de sal.

const int WarmValue = 17256; //17020; //Medimos valor en agua considerada caliente (40º)
const int ColdValue = 6000; // Medimos valor en agua considerada fría (0º)


int read_count = 1; //contador de tomas realizadas





// Función de mapeado de lecturas, a diferencia de la nativa de arduino, esta contempla numeros decimales.
double mapDecimal(double x, double in_min, double in_max, double out_min, double out_max)
{
 double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
 return temp;
}



// Espera para leer del GPS cuando esta está disponible
static void smartDelay(unsigned long ms)
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





/*
 * Inicializacion de conexiones y parametros
 */
 
void Initialization(){

  //Inicia comunicacionn serie
  Serial.begin(9600);

  //Inicia la comunicacion con el ADS 1115
  ads1115.begin(); //Initialize ads1115
  ads1115.setGain(GAIN_ONE); // Rango del ADC: +/- 4.096V (1 bit=2mV)

  //Configuramos el modo de los pines que se utilizan 
  pinMode(power_pin, OUTPUT); // Pin en modo salida, para el sensor de salinidad
  
  //Se inicia la comunicación con el sensor de presión BMP 180
   if(pr_bmp180.begin())
    Serial.println("BMP180 iniciado correctamente");
   else{
    Serial.println("Error al iniciar el BMP180");
  }


  //Se inicia la comunicación serie (vUART)
  

  delay(3000);
  sfSerial.begin(4800);
}



/*
 * Función para usar el GPS
 */
void geoLocating(){

  Serial.println("Conectando con el sensor GPS....");
  
  digitalWrite(initGPS_PIN,LOW);
  delay(200);
  digitalWrite(initGPS_PIN,HIGH);
  delay(200);
  digitalWrite(initGPS_PIN,LOW);

  char gpsFecha[10];
  char gpsHora[10];

  if(gps.location.isValid()){
    sprintf(gpsFecha,"%d/%d/%d", gps.date.month(),gps.date.day(),gps.date.year()); // Construimos string de datos fecha
    sprintf(gpsHora,"%d/%d/0%d", gps.time.hour(),gps.time.minute(),gps.time.second());  // Construimos string de datos hora
  
    Serial.print("Fecha: ");Serial.println(gpsFecha);
    Serial.print("Ubicacion(x,y):"); 
    Serial.print(gps.location.lat(),6); 
    Serial.print(","); 
    Serial.println(gps.location.lng(),6);
    Serial.println("");
  } else{
      Serial.print("Satelites disponibles: "); Serial.println(gps.satellites.value()); 
  }

  smartDelay(1000);
  

    
  
}



/*
 * Funcion para obtener la humedad relativa a traves del sensor de humedad capacitativo.
 * ADC pin 1
 */


int getHR(int sensorMin, int sensorMax){

//Valores maximo y minimo de medida de humedad
int const HR_Max=100;
int const HR_Min=0;

int adc0=0;
int humedad=0;

adc0 = ads1115.readADC_SingleEnded(1);
humedad = map(adc0, sensorMin, sensorMax, 100, 0);

  if (humedad<0){
      humedad=HR_Min;
      }
  if (humedad>100){
      humedad=HR_Max;
      }

 return humedad;
  
}



/*
 * Funcion para obtener la salinidad en el agua (en base a agua apta para el consumo)
 * ADC pin 0
 */
 int getSalinity(int sensorMin, int sensorMax){

  int input_pin = 2;
  int reading;

  int Salt_Min = 0;
  int Salt_Max = 100;

  int result = 0;

  digitalWrite(power_pin, HIGH);
  delay(100);
  reading = ads1115.readADC_SingleEnded(0);
  digitalWrite(power_pin, LOW);
  delay(100);

  result = map(reading,sensorMin, sensorMax, 0, 100);

  if (result<0){
      result=Salt_Min;
      }
  if (result>100){
      result=Salt_Max;
      }

  return (result);
  
}



/*
 * Funcion para obtener la preson atmosferica (dependiente de la altitud y el clima
 * Sensor digital, comunicacion a traves de SDA y SCL
 * temp: parametro para devolver la temperatura ofrecida por el sensor en vez de la presión
 */
double getPR(bool temp){
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

          //Serial.print("Temperatura BMP180: "+String(T,2)+"C\n");
          //Serial.print("Presion atmosferica: "+String(P,2)+"mb\n");
          
        } // Fin sobre la lectura de temperatura
        
      } // Fin sobre la inicializacion de presion
      
     } // Fin sobre la lectura de temperatura
  } //Fin sobre la inicializacion de temperatura
  else Serial.println("error starting temperature measurement\n");

 if(!temp){return P;}
 else{
  return T;
 }
  
}

/*
 * Función para obtener la temperatura ambiente (obtenida en función del voltage generado con una resistencia NTC)
 * ADC pin 3
 */
double getAmbientTemp(float sensorMin, float sensorMax){

double  reading=0;
double  result=0;

//Establece la señal del pin 5 en Bajo, ya que este interfiere en la lectura del sensor de temperatura
//digitalWrite(power_pin, LOW);
digitalWrite(power_pin, LOW);

reading = ads1115.readADC_SingleEnded(2);
result = mapDecimal(reading,sensorMin, sensorMax, 0, 40);
digitalWrite(power_pin, LOW);
//Serial.println("Lectura: "+String(reading,2));
return result;

}





//Función que controla las tomas de medidas de los sensores.
void getReadings(){

//Muestra datos por el monitor serie
Serial.print("Toma: "); Serial.print(read_count);


Serial.print("\n");
Serial.print("Temperatura: ");
Serial.print(getAmbientTemp(ColdValue,WarmValue),2); Serial.print("ºC");
Serial.print("\n");

Serial.print("Presion atmosferica: ");
Serial.print(getPR(false), 2);
Serial.print("\n");

Serial.print("HR (%): ");
Serial.print(getHR(WaterValue, AirValue)); Serial.print("%");
Serial.print("\n");

Serial.print("Salinidad (%): ");
Serial.print(getSalinity(DryValue, SaltWaterValue)); Serial.print("%");
Serial.print("\n");

//geoLocating(); //localización

Serial.print("\n");

read_count=read_count+1;
delay(3000);

}
