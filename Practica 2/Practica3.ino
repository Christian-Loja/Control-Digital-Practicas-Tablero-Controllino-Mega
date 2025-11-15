// Integrantes: Christian Loja, Nathaly Ramón, Freddy Lopez.
// Práctica 3.
#include <Controllino.h>
#include "Stone_HMI_Define.h"
#include "Procesar_HMI.h"

int       pwmValue_d0         = 0;              // valor convertido (0-255)
float     dutyCyclePercent_d0 = 0;              // valor en porcentaje (0-100)
int       pwmValue_d6         = 0;              // valor convertido (0-255)
float     dutyCyclePercent_d6 = 0;              // valor en porcentaje (0-100)
bool      led_d0 = false;                       // estado del led D0
bool      led_d6 = false;                       // estado del led D6
bool      prevboton_d0 = HIGH;                  // estado del botón I16
bool      prevboton_d6 = HIGH;                  // estado del botón I17

void setup() {
  Serial.begin(115200);   // Comunicación serial con el PC
  Serial2.begin(115200);  // Comunicación serial con el HMI
  pinMode(CONTROLLINO_D0, OUTPUT);   // led como salida
  pinMode(CONTROLLINO_D6, OUTPUT);   // led como salida
  pinMode(CONTROLLINO_I16, INPUT_PULLUP);
  pinMode(CONTROLLINO_I17, INPUT_PULLUP);
  HMI_init();             // Inicializa el sistema de colas para las respuestas el HMI
  Stone_HMI_Set_Value("spin_box", "spin_box_d0", NULL, 0);  // Pone en 0 el valor del spin box en el HMI. 
  Stone_HMI_Set_Value("spin_box", "spin_box_d6", NULL, 0);  // Pone en 0 el valor del spin box en el HMI. 
}

void loop() {
  dutyCyclePercent_d0=HMI_get_value("spin_box", "spin_box_d0"); // Obtiene el valor del spin_box_d0
  dutyCyclePercent_d6=HMI_get_value("spin_box", "spin_box_d6"); // Obtiene el valor del spin_box_d1

  bool boton_d0 = digitalRead(CONTROLLINO_I16);   // lectura del botón I16
  bool boton_d6 = digitalRead(CONTROLLINO_I17);   // lectura del botón I17

  // Detección de flanco descendente del botón I16
  if (prevboton_d0 == HIGH && boton_d0 == LOW){
    led_d0 = !led_d0;  
  }

  // Detección de flanco descendente del botón I17
  if (prevboton_d6 == HIGH && boton_d6 == LOW){
    led_d6 = !led_d6;  
  }

  // Actualización de variables
  prevboton_d0 = boton_d0;
  prevboton_d6 = boton_d6;
  
  if (dutyCyclePercent_d0 >= 0 && dutyCyclePercent_d0 <=100){
    pwmValue_d0 = map(dutyCyclePercent_d0, 0, 100, 0, 255);      // Mapea el valor de duty cycle en porcentaje a valores de 0 a 255

    if (led_d0 == true){
      analogWrite(CONTROLLINO_D0, pwmValue_d0);
      Serial.print("Duty cycle D0 (%): ");
      Serial.print(dutyCyclePercent_d0);
      Serial.print(" -> PWM value: ");
      Serial.println(pwmValue_d0);
    }else{
      analogWrite(CONTROLLINO_D0, 0);
    }
  }
  
  if (dutyCyclePercent_d6 >= 0 && dutyCyclePercent_d6 <=100){

    if (led_d6 == true){
      pwmValue_d6 = map(dutyCyclePercent_d6, 0, 100, 0, 255);      // Mapea el valor de duty cycle en porcentaje a valores de 0 a 255
      analogWrite(CONTROLLINO_D6, pwmValue_d6);
      Serial.print("Duty cycle D6 (%): ");
      Serial.print(dutyCyclePercent_d6);
      Serial.print(" -> PWM value: ");
      Serial.println(pwmValue_d6);
    }else{
      analogWrite(CONTROLLINO_D6, 0);
    }
  }
}
