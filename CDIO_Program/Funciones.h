
Adafruit_ADS1115 ads1115(0x48); // construct an ads1115 at address 0x48
SFE_BMP180 pr_bmp180; // Instancia del sensor de presion.

const int AirValue = 20200;  // Medimos valor en seco
const int WaterValue = 12341;  // Medimos valor en agua

const int DryValue = 15000;  // Medimos valor en seco
const int SaltWaterValue = 30000;  // Medimos valor en agua con un dedo de sal.


/*
 * Inicializacion de conexiones y parametros
 */
 
int Initialization(){

  int errors=0; 
  
  //Inicia comunicacionn serie
  Serial.begin(9600);

  //Inicia la comunicacion con el ADS 1115
  ads1115.begin(); //Initialize ads1115
  ads1115.setGain(GAIN_ONE); // Rango del ADC: +/- 4.096V (1 bit=2mV)

  //Configuramos el modo de los pines que se utilizan 
  pinMode(power_pin, OUTPUT); // Pin en modo salida, para el sensor de salinidad


   if(pr_bmp180.begin())
    Serial.println("BMP180 iniciado correctamente");
   else{
    Serial.println("Error al iniciar el BMP180");
    errors=1;
  }

  return errors;
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
 */
int getPR(){
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
      // Print out the measurement:
      /*Serial.print("temperatura: ");
      Serial.print(T,2);
      Serial.print(" deg C, ");
      Serial.print((9.0/5.0)*T+32.0,2);
      Serial.println(" deg F"); */
      status = pr_bmp180.startPressure(3);
      if(status!=0){
        
        delay(status);
        status = pr_bmp180.getPressure(P,T);
        if(status!=0){

          Serial.print("Temperatura BMP180: "+String(T,2)+"C\n");
          Serial.print("Presion atmosferica: "+String(P,2)+"mb\n");
          Serial.println("");
          
        } // Fin sobre la lectura de temperatura
        
      } // Fin sobre la inicializacion de presion
      
     } // Fin sobre la lectura de temperatura
  } //Fin sobre la inicializacion de temperatura
  else Serial.println("error starting temperature measurement\n");
 
  
}
