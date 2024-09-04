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

