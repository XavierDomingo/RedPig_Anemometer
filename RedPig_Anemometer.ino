#include "Wire.h"
#include <EEPROM.h>
#include <Stepper.h>
#define I2C_PANT 0x3C //Dirección pantalla i2c. ¡OJO solo se usa en este fichero, la librería ug8lib tiene su propia declaración

uint8_t dataBuf[3];   // I2C buffer para lectura de sensor
uint16_t pressure;    // Lectura 14-bit de presión en formato del ADC del sensor
uint16_t temperature; // Lectura 11-bit de temperatur en formato ADC sensor
double parcPres;      // Para promediar, lectura 14-bit de presión en formato del ADC del sensor
double parcTemp;      // Para promediar, lectura 11-bit de temperatur en formato ADC sensor
int numLectMedia = 0; // número de lecturas para hacer la media
double temperatureC;  // temp. centígrados
double presPascal;    // Presión en pascales
double velocidad;
uint8_t status;       // flag 2-bit para status de sensor
int pos_larga;        // posición larga (nombre heredado del altímetro, aquí solo hay una aguja)
int offset_larga;     // offset para aguja larga sobre los 180 grados teóricos
double offset_Pres;   // PROVISIONAL offset para presión en Pascales
int error_ini;
long msegs;
long ms_habAajPres;   // Se almacena lectura de millis() para entrar en modo ajuste de presión si se mantiene pulsado más de 20 segundos
boolean modoTest = false;
int interm;           // Para desplazar intermitentemente punto por pantalla mostrando que está vivo
boolean antPuls = false;  //Estado pulsado / no pulsado encoder en lectura anterior


#define V_PERDIDA 67.0  // Velocidad de pérdida en las unidades indicadas por FACTOR_M
                        // Se activará a 1.1*V_PERDIDA siempre que previamente se haya superado 1.1*V_PERDIDA.
                        // Se desactivará a 0.7*V_PERDIDA. Revisar si suena después de aterrizar, durante la rodadura
#define VNO 200.0       // Comienzo de arco amarillo = fin arco verde, también suena el zumbador 
                        // Las velocided de pérdida (55 km/h), Vno (200 km/h) y Vne (250 km/h) se han tomado de una
                        // Jodel D112 como ejemplo para la carátula del prototipo 
//#define PMIN 0.0        //Para margen de medida 10% a 90%
//#define PMAX 2490.889

//Para margen 0 a 100%:
const double presMin = -311.36;  //Correspndiene a -1,25 "H2O 
const double presMax = 2802.25;  //Correspndiene a 11,25 "H2O 

#define TMP_AJ_PRES 10000     //Tiempo que es necesario mantener pulsado para que entre en modo ajuste del offset de presión (10 seg)

volatile long tmp_reb = 250;  //temporización para evitar rebotes de encoder
volatile char ajustando_presion;
volatile char habilitar_perdida;
volatile char en_perdida;
volatile char en_amarillo;
char UNID[8] = " km/h";
const double FACTOR_M = 3.6;  //La velocidad se calcula en metros/segundo, este valor es el factor por el que se multiplica para Km/h
                              //Para nudos (KNOTS) el valor debe ser 1,94384
const double FACTOR_S = 1;    //Factor de escala de la carátula, 1 para fondo de escala = 250 Km/h
const long F_ESCALA_L = 250.0f;    //Valor correspondiente al extremo de la aguja larga. Inicialmente a 250 ya
                                   //que se ha previsto para una carátula con fondo de escala 250 Km/h. Es la velocidad máxima que
                                   //se puede medir con el sensor NPA-700B-10WG suponiendo densidad del aire = 1.
                                   //En realidad se debe tomar dens. aire = 1.225 con lo que el máximo medible son 230 km/h dentro del
                                   //margen de error del sensor y hasta 240 km/h con el 10% adicional fuera del margen de error
                                   //NOTA: Siempre habrá un offset de 20 km/h por lo que el recorrido de la aguja será de 230 km/h                               

#define ENC_CLK 2         //Señal CLK de encoder
#define ENC_DT 3          //Señal DT de encoder
#define ENC_SW 12         //Pulsador de eje encoder, no tiene resistencia de pull up, habrá que usar la del micro
#define ZUMBA 13          //Zumbador. Avisador pérdida
#define S_HALL_L 16       //Pin 16 (A2) sensor e. Hall aguja larga


/********************************************************************************************************
drawPpal
      Muestra la pantalla que aparecerá normalmente.
      Aparecen en fuente grande la velocidad en km/h y en nudos en fuente pequeña.
      Si está por encima de la Vno o por debajo de la pérdida alterna la visualización
      entre modo normal e inverso
*********************************************************************************************************/
void drawPpal(void) {
  char buf1[12];
  char *pbuf;
  double adoub;
  int i;
  
  // graphic commands to redraw the complete screen should be placed here 
  interm++;
  if (interm > 10)
    interm = 0;
  do
  {  
    if ((double(velocidad / FACTOR_M)) > 5.56f)
      adoub = velocidad;
    else
      adoub = 0;  //No mostrar velocidades menores de 20;
    
    dtostrf(adoub, 3, 0, buf1);
    
    if (en_perdida || en_amarillo)
    {
      //Comandos para SSD1306: 0xA6 (normal), 0xA7 (inverso).
      //Alternar texto normal e inverso durante la pérdida 012 345 678 90
      Wire.beginTransmission(I2C_PANT);
      Wire.write(0x00);
      if ((interm == 2) || (interm == 8))
        Wire.write(0xA6);
      if ((interm == 5) || (interm == 10))
        Wire.write(0xA7);
      Wire.endTransmission();
    }
    else
    { //Modo normal, sobre fondo negro
      Wire.beginTransmission(I2C_PANT);
      Wire.write(0x00);
      Wire.write(0xA6);
      Wire.endTransmission();  
    }
    
  } while( u8g.nextPage() );
}

/********************************************************************************************************
display
      Función para mostrar pantallas completas.
      Pensada para mostrar códigos de error, aunque también se usa para mostrar las líneas de
      información de búsqueda de cero que puede acabar en error, ver llamadas desde iniMotor.
*********************************************************************************************************/
void display(unsigned int cod_err)
{ 
  
}  

/********************************************************************************************************
ajusteOffset
      Permite ajustar el cero de la aguja, se espera durante 2 segundos a que el usuario pulse el
      botón del encoder. Se ha ampliado a 2 segundos ya que este tiempo sirve para comprobar que la
      aguja no han sufrido desviación.
 *********************************************************************************************************/
void ajusteOffset(void)
{  
  long lact;
  
  //mostrar pantalla ofreciendo ajuste durante 2 seg
   display(0x0400);
   msegs = millis();
   lact = msegs;
   while ((lact < (msegs + 2000)) && digitalRead(ENC_SW))
   {  //esperar 2 sg. o a que se pulse
    lact = millis();
   }
   delay(150);  //evitar rebotes
   if (!digitalRead(ENC_SW))
   {  //se ha pulsado
    display(0x0800);
    delay(150);
    while (!digitalRead(ENC_SW));  //esperar a que se deje de pulsar
    attachInterrupt(0, ajAgujaEncoder, RISING);  //interrupción 0 por flanco - pin 2
    while (digitalRead(ENC_SW))  //esperar a que se pulse para grabar y salir
    {
      display(0x0800);
    }
    detachInterrupt(0);
    while (offset_larga > 720)
      offset_larga = offset_larga - 720;
    while (offset_larga < -720)
      offset_larga = offset_larga + 720;
    EEPROM.put(DIR_AJ_LARGA, offset_larga);
   }
    
    //Comandos para SSD1306: 0xA6 (normal), 0xA7 (inverso).
    //Alternar texto normal e inverso durante la pérdida
    Wire.beginTransmission(I2C_PANT);
    Wire.write(0x00);
    if (ciclo_son)
      Wire.write(0xA6);
    else
      Wire.write(0xA7);
    Wire.endTransmission();
}

/********************************************************************************************************
/***************************************************************************************
****************************************************************************************
Interrupciones encoder
****************************************************************************************
***************************************************************************************/

/**************************************************************************************
 *  Interrupción para variación offset de la presión mediante el encoder
 *************************************************************************************/
 void PresEncoder() 
{  
  int i, lecnva, lecant, lecDT;
  
  // Si CLK y DT son iguales está girando en sentido horario,
  // si son distintos gira a izquierda.
  //OJO con encoder de BOURNS cambia el sentido respecto al prototipo
  //100 ms entre lecturas para evitar rebotes y comprobar que la señal que ha provocado
  //la interrupción (ENC-CLK) sigue a nivel bajo, la interrupción es por flanco de bajada
  if ((millis() > tmp_reb) && (digitalRead(ENC_CLK) == LOW))                             
  {   
    tmp_reb = millis();
    //esperar a a que se estabilice la señal, pequeño retardo con for, durante
    //interrupción no se actualiza millis()
    for (i = 0; i < 1000; i++)
      __asm__("nop\n\t");
    if (digitalRead(ENC_CLK) == digitalRead(ENC_DT))
    { 
      if (ajustando_presion)
      {  //Se está ajustando la presión, limitar a +-50 Pa
        offset_Pres = offset_Pres - 0.1f;
        if (offset_Pres < -50.0f)
          offset_Pres = -50.0f;
      }
    }
    else 
    {
      if(ajustando_presion)
      {  //Se está ajustando la presión, limitar a +-50 Pa
        offset_Pres = offset_Pres + 0.1f;
        if (offset_Pres > 50.0f)
          offset_Pres = 50.0f;
      }
    }
  }
}

/********************************************************************************************************
readBytes
      Leer un array de bytes de bus I2C
      addr:   dirección I2C del sensor
      values: array donde se almacenaran las lecturas.
      length: número de bytes a leer
*********************************************************************************************************/
char readBytes(uint8_t addr, uint8_t length, uint8_t *values)
{
  char x;

    Wire.requestFrom(addr,length);
    while(Wire.available() != length) ; // wait until bytes are ready
    for(x=0;x<length;x++)
    {
      values[x] = Wire.read();
    }
    return(1);
  return(0);
}

/********************************************************************************************************
setup
      Función que se ejecuta solo al iniciar
*********************************************************************************************************/
void setup() {
  // put your setup code here, to run once:
  double adoub;
  error_ini = 0;
  offset_larga = 0;
  Wire.begin();
  //---Serial.begin(9600);

  pinMode(S_HALL_L, INPUT_PULLUP); //Pull up en sensores para evitar resistencia externa
  pinMode(ENC_CLK, INPUT); //En el circuito hay resistencia de pull up de 10k
  pinMode(ENC_DT, INPUT);  //En el circuito hay resistencia de pull up de 10k
  pinMode(ENC_SW, INPUT_PULLUP);  //No hay resistencia de pull up en el circuito
  pinMode(ZUMBA, OUTPUT);
  digitalWrite(ZUMBA, LOW);  //Zumbador apagado
  
  //---Serial.println("\nNPA700");
    
  habilitar_perdida = 0;
  en_perdida = 0;
  en_amarillo = 0;

  EEPROM.get(DIR_AJ_PRES, adoub);   
  if (isnan(adoub) || (adoub < -50.0) || (adoub > 50.0))
    offset_Pres = 0.0;
  else
    offset_Pres = (double)adoub;
  
  ajustando_presion = 0;
  numLectMedia = 0;
  parcTemp = 0;
  parcPres = 0;
  ms_habAajPres = millis();
  msegs = millis();
  if ((!error_ini))
  {
    drawPpal();
    //Admitir interrupciones (las provocadas por el encoder) cuando ya se vea 
    //en pantalla el por si se modifica el offset de la presión
    attachInterrupt(0, PresEncoder, RISING);  //interrupción 0 por flanco - pin 2
  }
  antPuls = digitalRead(ENC_SW);
}

/********************************************************************************************************
readSensor
      Lectura del sensor
*********************************************************************************************************/
void readSensor()
{
    if (readBytes(0x28,4,dataBuf))
    {
      msegs = millis();
      //Serial.println("Leyendo-----");
      status = (dataBuf[0] & 0xC0) >> 6; // status is two most MSb of first byte
      if (status != 0)
        error_ini = 0x0080;
      pressure = (dataBuf[0] & 0x3F);
      pressure = pressure << 8; // pressure MSB less the status bits
      pressure += dataBuf[1]; // pressure LSB
      temperature = dataBuf[2] << 3; // most significant 8 bits in the 11-bit temperature read
      temperature += dataBuf[3] >> 5; // least significant 3 bits in the 11-bit temperature read
    }  //if (readBytes(0x28,4,dataBuf))
    else
    {
      //---Serial.println("\nError!!!!!"); 
      error_ini = 0x0080;
    }
}

/********************************************************************************************************
modoAjustandoTest
      Está pulsado el botón del encoder, o se está ajustando el offset de la presión
*********************************************************************************************************/
void modoAjustandoTest()
{
  if (ajustando_presion || ((millis() > (ms_habAajPres + TMP_AJ_PRES))))
  //La primera parte del if solo necesaria para permitir alternancia entre test y normal, no solo test si manteniendo pulsado
  {  //En modo ajuste del offset de presión
    if (!ajustando_presion)
    {
      ajustando_presion = 1;
      display(0x1000);  //Mostrar, "Activado el ajuste", no usar error_ini ya que realmente no es un error
    }
    switch (ajustando_presion)
    {
      case 1:
        if (digitalRead(ENC_SW))  //esperar a que se deje de pulsar
        {
          ajustando_presion = 2;  //indicador de que se ha soltado, ahora se espera a que ajuste y pulse para garabar
          display(0x2000);  //Mostrar, "ajustar y pulsar para grabar", no usar error_ini ya que realmente no es un error
        }    
        break;
        
      case 2:
        display(0x2000);  //Mostrar, "ajustar y pulsar para grabar", actualizar offset por si se está girando el encoder
        if (!digitalRead(ENC_SW)) //esperar 2 lecturas iguales seguidas continuando en case 3, para evitar rebotes
          ajustando_presion = 3;
        break;
        
      case 3:  
        display(0x2000);  //Mostrar, "ajustar y pulsar para grabar",  actualizar offset por si se está girando el encoder
        if (!digitalRead(ENC_SW))
        {
          EEPROM.put(DIR_AJ_PRES, offset_Pres);
          ajustando_presion = 0; //Fin del proceso de ajuste de presión
          ms_habAajPres = millis();
        }
        break;
    } //switch
  }
  else
    drawTest();
}

/********************************************************************************************************
ctrlVelocidades
      Control de velocidad de pérdida y arco amarillo
*********************************************************************************************************/
void ctrlVelocidades()
{
  if (velocidad > (1.1*V_PERDIDA))
    habilitar_perdida = 1;
  if (velocidad < (0.70*V_PERDIDA))
    habilitar_perdida = 0;
  if (velocidad > VNO)
    en_amarillo = 1;
  else
    en_amarillo = 0;
  if (en_amarillo || (habilitar_perdida && (velocidad < (1.1*V_PERDIDA))))
  {
    en_perdida = 1;
    if (sonando == 0)
      ms_interm = millis();  //es la primera vez, tomar referencia de tiempo
    if (millis() < (ms_interm + 750)) //sonido intermitente
    {
      digitalWrite(ZUMBA, HIGH);
    }
    else
    {
      if (ciclo_son == 1)  //pasado el tiempo y todavía en ciclo 1. Es porque no ha pasado en los 750 ms
      {
        digitalWrite(ZUMBA, LOW);

        ciclo_son = 0; //Es necesario "ciclo_son" ya que puede no pasar en 750 y 250 mS debido a que
                     //drawppal y otras funciones son lentas
      }
      
      if (millis() < (ms_interm + 1000))
      {
        digitalWrite(ZUMBA, LOW);
      }
      else
      {
        ms_interm = millis();  //iniciar ciclo sonido 0,75 + 0,25 seg
        if (ciclo_son == 0)  //pasado el tiempo y todavía en ciclo 0. Es porque no ha pasado en los 250 ms
        {
          digitalWrite(ZUMBA, HIGH);
          ciclo_son = 1;
        }
        else
          ciclo_son = 0;
      }
    }
    sonando = 1;
  }
  else
  {
    digitalWrite(ZUMBA, LOW);
    sonando = 0;
    en_perdida = 0;
    ciclo_son = 1;
  }
}

/********************************************************************************************************
loop
      Bucle que se ejecuta indefinidamente
*********************************************************************************************************/
void loop() {
  // put your main code here, to run repeatedly:
  long lecact;

  //Se hacen 5 lecturas, una cada 100 mS y se promedian. Se obtiene una lectura promediada cada medio segundo
  //He probado con 10 lecturas, una cada 50mS pero no hay diferencia aparente de estabilidad
  lecact =  millis();
  if (lecact > (msegs + 100))
  { //una lectura cada 0.1 segundos
    readSensor();  
    parcTemp = parcTemp + temperature;
    parcPres = parcPres + pressure;
    numLectMedia++;
    if (numLectMedia >= 5)
    {  //Se hacen 5 lecturas y se promedian
      numLectMedia = 0;
      temperature = parcTemp / 5;
      pressure = parcPres / 5;
      parcTemp = 0;
      parcPres = 0;
      temperatureC= ((temperature*200.0)/2048.0)-50.0;
      /*
      El sensor del prototipo con presión diferencial 0 y alimentado a través del programador oscila
      entre 1693 y 1723, (entre 1652 y 1674 en Lugo 01/01/2018, 27,3º medidos por sensor después de
      unos minutos conectado) curiosamentesiempre en incrementos múltiplos de 6:
      1693, 1699, 1705, 1711, 1717, 1723, nunca valores distintos a estos.
      La tensión es de 4,37 V, por debajo de la mínima (4,75)
      Si se alimenta por la entrada RAW con tensión de 5V o mayor los valores son:
      1723, 1730, 1736, 1742, 1748, (entre 1748 y 1766 en Lugo 01/01/2018, 28,9,º medidos por sensor).
      La tensión es de 4,85 V
      Según datasheet la presión debe calcularse según la siguiente fórmula:
      P = Pmin + ((OUT - OUTmin)/OUTmax - OUTmin))*(Pmax - Pmin)
      OUT = measured sensor output
      
      En el ejemplo de la nota de aplicación usa la siguiente instrucción donde presure_min = -1.25
      y presure_max = 11.25:
      Pressure_float = ((float)Pressure_value * (pressure_max - pressure_min) / 16383 + pressure_min; // conversion to real pressure units
      
      Al utilizar -1.25 y 11.25 para los valores de presión el resultado se obtendrá en pulgadas de agua.
      Mide 0 a 10"H2O en el margen de 10% a 90% con error máximo de 1.5% del margen total (10"H2O).
      Si 10"H2O corresponde al 80% (90%-10%) del rango de medida el 100% es 12.5, 2,5" más que se distribuirá en
      1,25 entre 0 y 10% y otros 1,25 entre el 90% y 100%, es decir de -1.25 a 11.25
      */
//      //Usando la ecuación de la datasheet y para el rango de 10% a 90%, 0 a 10"H2O
//      presPascal= PMIN + double((pressure-1638.0)/(14745.0-1638.0))*(PMAX - PMIN); //PMIN = 0, PMAX = 2490.889
//      //Los valores 1638 y 14745 corresponden al 10% y 90% del rango total 0 a 16383. En el margen 10% a 90%
//      //mide 0 a 10"H2O con precisión de 1,5% del fondo de escala, entre 0 y 10% y entre 90% y 100% el error es indeterminado

      //Usando la ecuación del ejemplo de la nota de aplicación:
      presPascal = double(pressure*(presMax - presMin)/16383) + presMin;
      
      presPascal= presPascal + offset_Pres;
      //Velocidad (m/s) = raíz cuadrada (2 x presión diferencial / densidad)  densidad = 1.225kg/m3
      //Velocidad (km/h) = 3.6 * raíz cuadrada (2/1225 x presión diferencial)
      if (presPascal > 0)
        velocidad = FACTOR_M * sqrt(double(presPascal/0.6125));   //0.6125 = 1.225/2
      else
        velocidad = 0;
    }
    //---Serial.print("Status: ");
    //---Serial.print(status, HEX);
    //---Serial.print("\t  -.- Presion: ");
    //---Serial.print(pressure);
    //---Serial.print("\t  -.- Velocidad: ");
    //---Serial.print(velocidad);
    //---Serial.print("\t  -.- Temperatura: ");
    //---Serial.print(temperature);

    //---Serial.print("\t  -*-  ");
    //---Serial.print("Pascales: ");
    //---Serial.print(presPascal);
    //---Serial.print("\t -.- Grados C: ");
    //---Serial.println(temperatureC);
 
    //Serial.print("-----");
  }  //if (lecact > (msegs + 100))
  
  //modoTest = !digitalRead(ENC_SW);  //Modo test si pulsando encoder
  //sustituyo la anterior instrucción por lo siguiente, y nueva variable antPuls
  //Con cada pulsación cambiar alternativamente a modo normal o test para poder mantener lectura en modo test y facilitar calibracón
  if ((!ajustando_presion) && antPuls && (!digitalRead(ENC_SW))) //Si está pulsado y en la lectura anterior no lo estaba cambiar de normal a test o viceversa
    modoTest = !modoTest;
  antPuls = digitalRead(ENC_SW);
    
  if ((modoTest) || ajustando_presion)
  {
    modoAjustandoTest();
    if ((!ajustando_presion) && (digitalRead(ENC_SW))) //para permitir alternancia entre test y normal, no solo test si manteniendo pulsado
        ms_habAajPres = millis();
  }
  else
  {
    ms_habAajPres = millis();
    if (!error_ini)
      drawPpal();
    else
      display(error_ini);
  //Se muestra en pantalla permanentemente la indicación de error si se ha producido, pero se mantiene la lectura
  //del sensor e indicación. No se muestra error en modo test para poder comprobar lecturas
      
  }  //if ((modoTest) || ajustando_presion)
    


  ctrlVelocidades(); 
}
