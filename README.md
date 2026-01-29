# LQR-Controller-for-Furuta-Pendulum

Este repositorio presenta el diseño, la simulación y la implementación de un **controlador Regulador Cuadrático Lineal (LQR)** para estabilizar un **péndulo de Furuta** en su posición vertical invertida. Desarrollado como proyecto final del curso de Control Digital, este proyecto demuestra un control óptimo en tiempo real implementado en una plataforma Arduino.

### 📋 Archivos Incluidos
- **Código**: [Practica1.ino](/Practica%201/Practica1.ino)
- **Simulación MATLAB**: [Practica1.ino](/Practica%201/Practica1.ino)
- **Informe**: [Practica1.ino](/Practica%201/Practica1.ino)
- **Video demostrativo**: [P1.mp4](https://github.com/user-attachments/assets/b3eb3eb0-7d70-45d3-8084-0a8f7e737347) **-->** Muestra el péndulo de Furuta en equilibrio vertical.

## 🎯 Objetivos
| Objetivo | Descripción |
|-----------|-------------|
| **Modelado matemático** | Modelar el sistema de péndulo de Furuta no lineal mediante la formulación de Lagrange |
| **Linealización** | Linealizar el modelo alrededor del punto de equilibrio (posición vertical) |
| **Diseño LQR** | Diseñar un controlador LQR óptimo resolviendo la ecuación de Riccati |
| **Implementación en tiempo real** | Implementar el controlador en tiempo real en una plataforma Arduino |
| **Validación del rendimiento** | Validar el rendimiento mediante simulaciones y pruebas experimentales |

## 🔬 Metodología

### Modelado matemático
El sistema se modeló mediante la formulación de Lagrange con los siguientes parámetros:

| Parámetro | Símbolo | Valor | Unidad |
|-----------|--------|-------|------|
| Longitud del brazo | `r` | 0,235 | m |
| Longitud del péndulo | `l` | 0,413 | m |
| Masa del péndulo | `M` | 0,01 | kg |
| Masa del brazo | `m` | 0,02 | kg |
| Inercia del péndulo | `J_P` | 9×10⁻⁴ | kg·m² |
| Inercia del brazo | `J` | 0,05 | kg·m² |

### Linealización
Linealización alrededor del punto de equilibrio (θ = 0°) mediante aproximaciones de ángulo pequeño:
- **sin(θ) ≈ θ**
- **cos(θ) ≈ 1**

### Diseño del controlador LQR
La ecuación algebraica de Riccati se resolvió con matrices de ponderación:

```matlab
Q = diag([10, 1, 1, 0.1]) % Penaliza las desviaciones de estado
R = 1 % Penaliza el esfuerzo de control
```

**Vector de ganancia óptima:**
```
K = [-28.6407, -5.197, -1, -0.8264]
```

### Discretización
El controlador continuo se discretizó con `T_s = 2 ms` mediante:
- **Retención de Orden Cero (ZOH):** Ganancia discretizada (K_d) = [-7.1404, -1.3194, -0.4115, -0.2977]
- **Método Tustin:** Ganancia discretizada (K_d) = [-7.0931, -1.3107, -0.4088, -0.2958]

## 💻 Implementación de Arduino

### Arquitectura del Sistema
- **Frecuencia de Muestreo**: 500 Hz (periodo de 2 ms)
- **Estimación de Estado**:
- Ángulo del péndulo (θ): Potenciómetro con conversión ADC
- Ángulo del brazo (φ): Encoder incremental (2000 pulsos/revolución)
- Velocidades: Diferenciación numérica con filtro paso bajo
- **Actuación**: Puente H con control PWM
- **Seguridad**: Frenado de emergencia para |θ| > 30°

### Ley de Control
El controlador implementa:
```
u = -K·x = -(k₁θ + k₂θ̇ + k₃φ + k₄φ̇)
```

### Pasos de calibración
1. Subir el código a Arduino
2. Colocar el péndulo en posición vertical
3. Registrar la lectura del ADC (se convierte en `ADC_VERTICAL`)
4. Ajustar las constantes de calibración si es necesario

### Ajuste del sistema
```arduino
// Parámetros clave de ajuste en el código:
const float GAIN_SCALE = 0.75; // Factor de escala de ganancia inicial
const float DEAD_ZONE = 0.2; // Zona muerta del actuador (voltios)
const float MAX_VOLTAGE = 5.0; // Límite de saturación del motor
const int ADC_VERTICAL = 758; // Valor del ADC en posición vertical
```

### Guía de cableado
- Pines de Arduino → Componentes:
- Pin 9 → Entrada PWM (Controlador del motor)
- Pin 10 → IN2 (Controlador del motor)
- Pin 11 → IN1 (Controlador del motor)
- Pin A5 → Salida del potenciómetro
- Pin 2 → Codificador A (interrupción)
- Pin 4 → Codificador B

### Resultados de la implementación
✅ **Estabilidad local** mantenida alrededor del punto de equilibrio
⚠️ **Degradación del rendimiento** para ángulos grandes (> 30°)
🛡️ **Mecanismo de frenado de emergencia** que protege el hardware

## 🛠️ Requisitos del Proyecto
- Arduino UNO
- Motor Driver TB6612FNG
- Péndulo de Furuta con Motor y Encoder

## 📄 License
This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.
