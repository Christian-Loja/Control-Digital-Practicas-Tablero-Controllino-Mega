// Integrantes: Christian Loja, Nathaly Ramón, Freddy Lopez.
// Práctica 1.
#include <SPI.h>
#include <Controllino.h>

int vector[9] = {  //Leds en orden de encendido
  CONTROLLINO_D0,
  CONTROLLINO_D6,
  CONTROLLINO_D12,
  CONTROLLINO_D13,
  CONTROLLINO_D14,
  CONTROLLINO_D8,
  CONTROLLINO_D2,
  CONTROLLINO_D1,
  CONTROLLINO_D7
};

int sizev = 9;                 //Tamaño del vector de leds

unsigned long t_actual = 0;    //Variables para tiempo
unsigned long t_previo = 0;
unsigned long intervalo = 500; //Tiempo de espera

// Puntero 
int *ptrLed = vector;   //ptrLed no guarda el valor, sino la 
                        //dirección del primer elemento de vector

void setup() {
                                      //Configurar todos los leds como salida
  for (int i = 0; i < sizev; i++) { 
    pinMode(vector[i], OUTPUT);
    digitalWrite(vector[i], LOW);     //Iniciar todos apagados
  }
}

void loop() {
  t_actual = millis();                      //Tiempo transcurrido actual
  if (t_actual - t_previo >= intervalo) {   //Si han pasado 50ms, entonces:
    t_previo = t_actual;                    //El tiempo anterior es el actual
    digitalWrite(*ptrLed, LOW);             // Se apaga el led actual
    ptrLed++;                               //Se recorre la dirección 1 entero completo
    if (ptrLed >= vector + sizev) {         //Si se llega a la última dirección del vector
      ptrLed = vector;                      //Se apunta al inicio nuevamente
    }                               
  }
  digitalWrite(*ptrLed, HIGH);               //Encender led en la dirección *ptrLed
}
