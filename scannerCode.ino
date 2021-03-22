/********************************************************************************************************************************************/
/* ▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄  M≋E≋R≋A≋K≋I ⊶ 【L】【I】【D】【A】【R】  ▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄▀▄                                                  */
/*                                                              - PROYECTO MODULAR -
 *  
 *  CRÉDITOS © y REFERENCIAS:
 * Basado en <<Arduino Pro Micro Scanner Code (DIY 3D Scanner - Super Make Something Episode 8)>> - https://youtu.be/-qeD2__yK4c
 * por: Alex - Super Make Something
 * licencia: Creative Commons - Attribution - Non-Commercial.  More information available at: http://creativecommons.org/licenses/by-nc/3.0/
 * Incluye  información de código para aplicar "ReadWrite" por David A. Mellis y Tom Igoe disponible en: https://www.arduino.cc/en/Tutorial/ReadWrite
 * Incluye  información de código para aplicar los módulos <<EasyDriver board>> por Joel Bartlett disponible en: https://www.sparkfun.com/tutorials/400
 */
 
 
/* Este código contiene las siguientes funciones:
 * - void setup(): Para inicializar el puerto serial y el adaptador SD card
 * - void loop(): main loop
 * - double lecturaSensor(): Calcula las distancias capturadas en cm. Valores detectados calculados como un promedio de numMuestras (número de muestras) llamadas analogRead() consecutivas para eliminar el ruido
 * - void writeToSD(): Escribe la distancia detectada en cm en el archivo de la tarjeta SD especificado por la variable de nombre de archivo.
 * - void readFromSD(): Imprime el contenido del archivo de la tarjeta SD especificado por la variable de nombre de archivo en el monitor serial, sólo se usa como debug
 */
 
 /* PINOUT:
 * Adaptador SD card conectado al bus SPI de la siguiente manera:
 * - MOSI - (pin 23 en ESP32)
 * - MISO - (pin 19 en ESP32)
 * - CLK  - (pin 18 en ESP32)
 * - CS   - (pin 5 en ESP32)
 * 
 * IR Sensor (SHARP GP2Y0A51SK0F: 2-15cm, 5V) attached conectado al microcontrolador como:
 * - SensePin - 25 (pin ADC)
 * 
 * Driver del motor a pasos para la tornamesa (Theta):
 * - STEP - pin 2
 * - DIR - pin 3 
 * - MS1 - pin 4
 * - MS2 - pin 5
 * - ENABLE - pin 6
 * 
 * Driver del motor a pasos para el eje Z:
 * - STEP - pin 7
 * - DIR - pin 8
 * - MS1 - pin 9
 * - MS2 - pin 18 
 * - ENABLE - pin 19 
 */
/*************************************************************************************************************************************/
#include <SPI.h>
#include <SD.h>
#include "FS.h"


File scannerValores;
const char* filename="/data.txt";
int csPin=5;
int sensePin=25;

int tStep=12;
int tDir=13;
int tMS1=14;
int tMS2=27;
int tEnable=26;

int zStep=4;
int zDir=2;
int zMS1=15;
int zMS2=21;
int zEnable=22;


void setup() 
{ 

  //Definir pines PARA LOS DRIVERS DE LOS MOTORES PASO A PASO como pines de salida digital
  pinMode(tStep,OUTPUT);
  pinMode(tDir,OUTPUT);
  pinMode(tMS1,OUTPUT);
  pinMode(tMS2,OUTPUT);
  pinMode(tEnable,OUTPUT);
  pinMode(zStep,OUTPUT);
  pinMode(zDir,OUTPUT);
  pinMode(zMS1,OUTPUT);
  pinMode(zMS2,OUTPUT);
  pinMode(zEnable,OUTPUT);

  
  
  //Motor Theta: sin micro pasos (MS1 Low, MS2 Low) = 1.8 grados / paso (200 pasos / rev) 
  digitalWrite(tMS1,LOW);
  digitalWrite(tMS2,LOW);
  
  
  //Motor Z : sin micro pasos (MS1 Low, MS2 Low) = 1.8 grados / paso (200 pasos / rev) -> (200 pasos / 1 cm, es decir, 200 pasos / 10 mm). Por lo tanto, movimiento lineal / paso de 0,05 mm.
  digitalWrite(zMS1,LOW);
  digitalWrite(zMS2,LOW);

  //Habilitación de los controladores de los motores
  digitalWrite(tEnable,LOW);
 digitalWrite(zEnable,LOW);
    
  // Apertura de comunicación serial
  Serial.begin(9600);
  analogReadResolution(10); //Adecuar la lectura de datos para la ESP32 que tiene una resolución de 12 bits


  //Debug para monitor serial
  Serial.print("Inicializando SD card... ");
  if (!SD.begin(csPin))
  {
    Serial.println("ERROR de Inicialización!");
    return;
  }
  Serial.println("Inicialización EXITOSA!");

}

void loop() 
{
  int zDist=4; //Distancia total de movimiento del eje z en cm
  int noZSteps=20; //No. de pasos del Motor Z por rotación. Distancia = No.de pasos* 0,05 mm / paso, es decir, para 20 pasos, el motor se movería milímetro a milímetro
  int zCounts=(200/1*zDist)/noZSteps; //Número total de zCounts hasta que el eje z regrese al origen
  int thetaCounts=200;

  // Etapa de Escaneo
  digitalWrite(zDir,LOW); 
  for (int j=0; j<zCounts; j++) //Ciclo para la rotación del eje z
  {
    for (int i=0; i<thetaCounts; i++)   //Gire el motor Theta por una revolución, lee el sensor y almacena posteriormente el valor capturado
    {
      rotarMotor(tStep, 1); //Rotar el motor Theta un paso
      delay(200);
      double senseDistance=0; //Reset a la variable <<senseDistance>>;
      senseDistance= lecturaSensor(); //Leer sensor IR y calcular distancia.
      writeToSD(senseDistance); //Escribir valor de distancia en la tarjeta SD
    }
  
    rotarMotor(zStep, noZSteps); //Mover el sensor en el eje z un paso hacia arriba
    delay(1000);
     writeToSD(9999); //Escribir un valor ficticio en SD para analizarlo posteriormente en el programa de MATLAB
  }

  // Escaneo completo. Girar el eje z de regreso al origen y hecer una pausa.
  digitalWrite(zDir,HIGH);
  delay(10);  
  for (int j=0; j<zCounts; j++)
  {
    rotarMotor(zStep, noZSteps);
    delay(10);
  }

  for (int k=0; k<3600; k++) //Realizar una pausa de una hora (3600 segundos), es decir, congelar hasta que se apague porque el escaneo está completo.
  {
    delay(1000); 
  }

  
}

void rotarMotor(int pinNo, int steps)
{
  
  for (int i=0; i<steps; i++)
  {
    digitalWrite(pinNo, LOW); 
    delay(1);
    digitalWrite(pinNo, HIGH); //"Rising Edge" (Flanco ascendente) para que el driver del motor sepa cuándo comenzar.
    delay(1);
  }
}


double lecturaSensor()
{
  int numMuestras=20;
  long sumatoriaMuetras=0;
  
  for (int i=0; i<numMuestras; i++)
  {
    analogReadResolution(10);
    sumatoriaMuetras=sumatoriaMuetras+analogRead(sensePin); //Sumatoria de las muetras de las distancias capturadas
  }
  
  float senseVal=sumatoriaMuetras/numMuestras; //Calcular promedio
  double senseDistance = 17569.7 * pow(senseVal, -1.2062); //Convertir voltaje en distancia en cm via ajuste cúbico del sensor Sharp utilizado
  //senseDistance=mapDouble(senseDistance,0.0,1023.0,0.0,5.0); //Convertir pin análogo capturado en voltaje
  //Serial.print("Voltaje: ");     //Debug
  //Serial.println(senseDistance);   //Debug
 // Serial.print(" | "); //Debug
  //senseDistance=-5.40274*pow(senseDistance,3)+28.4823*pow(senseDistance,2)-49.7115*senseDistance+31.3444; 
  Serial.print("Distance: ");    //Debug
  Serial.println(senseDistance); //Debug
  return senseDistance;
}


void writeToSD(double senseDistance)
{
  // Abrir archivo
  scannerValores = SD.open(filename, FILE_APPEND);
  
  if(!scannerValores) {
    Serial.println("El archivo no existe");
    Serial.println("Creando archivo...");
    writeFile(SD, "/data.txt", "Distancia \r\n");
  }
  else {
    Serial.println("El archivo ya existe");  
     Serial.print("Guardando en ");
     Serial.print(filename);
     Serial.println("..."); 
    

    //Escribir en el archivo
    scannerValores.print(senseDistance);
    scannerValores.println();
      // Cerrar el archivo
    scannerValores.close();
    }
}

void readFromSD()//Sólo como Debug
{  
  // Abrir el archivo para lectura:
  scannerValores = SD.open(filename);
  if (scannerValores)
  {
    Serial.print("Contenido de ");
    Serial.print(filename);
    Serial.println(":");

    // Leer del archivo hasta que no haya nada más en él:
    while (scannerValores.available()) 
    {
      Serial.write(scannerValores.read());
    }
     // Cerrar el archivo
    scannerValores.close();
  } 
  else
  {
    // Si el archivo no se abrió, imprima un error:
    Serial.print("error al abrir archivo ");
    Serial.println(filename);
  }
}

// Escriba en la tarjeta SD (NO MODIFIQCAR ESTA FUNCIÓN)
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Escribiendo en archivo: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Error al abrir archivo para escritura");
    return;
  }
  if(file.print(message)) {
    Serial.println("Archivo guardado con éxito");
  } else {
    Serial.println("Error: Escritura en archivo falló");
  }
  file.close();
}


double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
