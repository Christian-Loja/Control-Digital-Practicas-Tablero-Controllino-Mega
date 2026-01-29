// CONTROLADOR LQR PARA PENDULO DE FURUTA
#include <Arduino.h>
#include <MsTimer2.h>

#define PWM 9
#define IN1 11
#define IN2 10
#define ENCODER_A 2
#define ENCODER_B 4
#define POT A5

// Parámetros del sistema
const float Ts = 0.002;  // 2 ms = 500 Hz
const float GAIN_SCALE = 0.75;  // Escalar valores de K

// Ganancia LQR discretizada (ZOH con Ts=2ms)
//const float K[4] = {-7.1404, -1.3194, -0.4115, -0.2977};
const float K[4] = {-7.0931, -1.3107, -0.4088, -0.2958};

// Variables de estado estimadas
float theta = 0.0;        // x1: Ángulo del péndulo (rad)
float theta_dot = 0.0;    // x2: Velocidad del péndulo (rad/s)
float phi = 0.0;          // x3: Ángulo del brazo (rad)
float phi_dot = 0.0;      // x4: Velocidad del brazo (rad/s)

// Variables anteriores para filtrado
float theta_prev = 0.0;
float phi_prev = 0.0;

// Variables de medición
int pendulum_raw = 0;     // Valor crudo del potenciómetro
int arm_counts = 10000;   // Conteo del encoder (posición inicial)

// Factores de conversión
const float ADC_TO_RAD = 0.3352;      // Valor de calibración calculado con el péndulo en posición vertical
const float ENCODER_TO_RAD = 2.0 * PI / 2000.0;  // 2000 conteo/rev

// Saturación
const float MAX_VOLTAGE = 5.0;      // Voltaje máximo al motor
const float DEAD_ZONE = 0.2;        // Zona muerta para evitar oscilaciones

// Filtros
const float ALPHA = 0.3;           // Para filtrado de velocidades

void timerCallback();
void READ_ENCODER_A();
void READ_ENCODER_B();
float estimatePendulumAngle(int raw_adc);
void estimateStates();
float computeControlAction();
void applyMotorControl(float u);

void setup() {
  Serial.begin(115200);
  
  // Configuración de pines
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  // Inicialización
  analogWrite(PWM, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  // Timer para control a 500 Hz (2 ms)
  delay(200);
  MsTimer2::set(2, timerCallback);  // 2 ms
  MsTimer2::start();

  // Interrupciones del encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), READ_ENCODER_A, CHANGE);           
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), READ_ENCODER_B, CHANGE);
}

void loop() {
  // El control se ejecuta en timerCallback
  delay(100);
}

// Convertir lectura ADC a ángulo del péndulo (radianes)
float estimatePendulumAngle(int raw_adc) {
  // ADC_VERTICAL es el valor cuando el péndulo está vertical
  const int ADC_VERTICAL = 758;  // Set point (Valor capturado por el ADC cuando el péndulo está en posición vertical)
  
  // Convertir a radianes (usa el factor de calibración)
  float angle = (raw_adc - ADC_VERTICAL) * ADC_TO_RAD;
  
  // Limitar para linealidad (aproximación de pequeño ángulo)
  return constrain(angle, -0.7, 0.7);  // +-28.6 grados
}

// Estimar estados (ángulos y velocidades)
void estimateStates() {
  // 1. Leer sensores
  pendulum_raw = analogRead(POT);
  
  // 2. Estimar ángulos
  theta = estimatePendulumAngle(pendulum_raw);
  phi = (arm_counts - 10000) * ENCODER_TO_RAD;  // Posición relativa
  
  // 3. Estimar velocidades por diferenciación con filtro
  static unsigned long last_time = 0;
  unsigned long current_time = micros();
  float dt = (current_time - last_time) / 1e6;
  
  if (dt > 0 && dt < 0.1) {  // Evitar divisiones por cero o dt grandes
    // Diferenciación
    float theta_dot_raw = (theta - theta_prev) / dt;
    float phi_dot_raw = (phi - phi_prev) / dt;
    
    // Filtrado pasa-bajos
    theta_dot = ALPHA * theta_dot + (1 - ALPHA) * theta_dot_raw;
    phi_dot = ALPHA * phi_dot + (1 - ALPHA) * phi_dot_raw;
  }
  
  // 4. Guardar valores anteriores
  theta_prev = theta;
  phi_prev = phi;
  last_time = current_time;
}

// Calcular acción de control LQR
float computeControlAction() {
  float u = -GAIN_SCALE * (K[0] * theta + 
                           K[1] * theta_dot + 
                           K[2] * phi + 
                           K[3] * phi_dot);
  
  // Aplicar zona muerta para evitar oscilaciones pequeñas
  if (fabs(u) < DEAD_ZONE) {
    u = 0;
  }
  
  // Saturación
  return constrain(u, -MAX_VOLTAGE, MAX_VOLTAGE);
}

// Aplicar control al motor
void applyMotorControl(float u) {
  // Determinar dirección
  if (u > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (u < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    // Frenar cuando u=0
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  
  // Convertir voltaje a PWM (0-255 para 0-5V)
  int pwm_value = (int)(fabs(u) * 255.0 / MAX_VOLTAGE);
  pwm_value = constrain(pwm_value, 0, 255);
  
  analogWrite(PWM, pwm_value);
}

// Frenado de emergencia
bool emergencyBrake() {
  // Si el péndulo se inclina demasiado (> 30 grados)
  if (fabs(theta) > 0.5236) {  // 30 grados en radianes
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(PWM, 0);
    return true;
  }
  return false;
}

// Función principal del timer
void timerCallback() {
  // 1. Estimación de estados
  estimateStates();
  
  // 2. Verificar freno de emergencia
  if (emergencyBrake()) {
    Serial.println("Frenado de Emergencia!");
    return;
  }
  
  // 3. Calcular acción de control LQR
  float u = computeControlAction();
  
  // 4. Aplicar al motor
  applyMotorControl(u);
  
  // 5. Monitoreo (Serial)
  static int counter = 0;
  if (counter++ % 10 == 0) {  // Enviar cada 20ms
    Serial.print("Theta: ");
    Serial.print(theta, 4);
    Serial.print(" Phi: ");
    Serial.print(phi, 4);
    Serial.print(" U: ");
    Serial.println(u, 4);
  }
}

// Interrupciones del encoder
void READ_ENCODER_A() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    arm_counts++;
  } else {
    arm_counts--;
  }
}

void READ_ENCODER_B() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    arm_counts--;
  } else {
    arm_counts++;
  }
}