# Desarrollo y calibración de un robot móvil diferencial con ROS 2 y micro-ROS

## Resumen

Este informe describe el desarrollo de una plataforma móvil diferencial basada
en ESP32, con control de dos motores de corriente continua mediante
controladores DRV8871, realimentación de encoders, una unidad inercial MPU6500,
tres sensores de distancia VL53L0X, una pantalla OLED y comunicación
inalámbrica con ROS 2 mediante micro-ROS. En el computador se implementaron
nodos de telemetría y una aplicación gráfica para navegación y ensayos de
calibración.

La integración combina control de velocidad de rueda, odometría diferencial,
un filtro de Kalman extendido (EKF) planar que fusiona información de ruedas y
giroscopio, y una rutina local de reacción a obstáculos. La calibración se
realizó mediante recorridos rectos con referencias manuales y ToF, y giros
manuales de 90° y 360°. Los resultados indicaron una escala lineal cercana a la
nominal, pero una subestimación significativa del giro cuando se usaba una
separación de ruedas incorrecta en el modelo. Se confirmó después que la
separación física es 0.21 m y se ajustó la separación efectiva de odometría a
0.20 m; el radio efectivo se estableció en aproximadamente 0.02399 m.

Según la evaluación cualitativa del usuario, después de ajustar la odometría la
orientación general del robot funciona mucho mejor. Esta mejora aún debe
cuantificarse con nuevas pruebas realizadas bajo la configuración actual. El
control transitorio de los motores y la evasión de obstáculos siguen siendo
áreas de trabajo.

**Palabras clave:** robot diferencial, odometría, micro-ROS, ROS 2, ESP32, EKF,
encoders, MPU6500, VL53L0X.

## 1. Introducción

Un robot diferencial se desplaza mediante dos ruedas motrices independientes.
La diferencia entre sus velocidades produce la rotación del chasis, mientras
que el promedio produce su velocidad longitudinal. Aunque este modelo es
geométricamente sencillo, su precisión práctica depende de medir correctamente
las ruedas, calibrar el radio efectivo, conocer la separación entre ellas y
controlar los transitorios de los motores.

El proyecto evolucionó desde un control de motores y lectura de sensores en
ESP32 hasta una arquitectura distribuida: el microcontrolador atiende la
adquisición y el control local, mientras que ROS 2 ejecuta la estimación de
estado, las interfaces de ensayo y la navegación de alto nivel. Esta separación
permite mantener el lazo de velocidad en el robot y modificar la lógica de
prueba o navegación sin trasladar cada cálculo al firmware.

## 2. Objetivos

### 2.1 Objetivo general

Integrar el hardware de un robot diferencial con ROS 2 y desarrollar métodos
reproducibles para observar, calibrar y mejorar su odometría y orientación.

### 2.2 Objetivos específicos

- Controlar de forma independiente la velocidad angular de las ruedas.
- Publicar velocidades de rueda, datos planares del IMU y distancias ToF en ROS
  2 a través de micro-ROS sobre Wi-Fi.
- Estimar posición y orientación relativas a partir de encoders y giroscopio.
- Proporcionar una aplicación para ensayos manuales, visualización de
  telemetría y comparación de mediciones con referencias físicas.
- Implementar navegación básica a destinos y una respuesta reactiva a
  obstáculos frontales.
- Separar los errores de escala y geometría de odometría de los problemas del
  controlador de motores.

## 3. Plataforma de hardware

El sistema utiliza los siguientes componentes:

| Componente | Función |
| --- | --- |
| ESP32 | Adquisición, PID de velocidad, comunicaciones y gestión de sensores |
| Dos motores DC y controladores DRV8871 | Accionamiento independiente de las ruedas |
| Encoders incrementales | Medición de velocidad y desplazamiento angular de rueda |
| MPU6500 | Velocidad angular Z y aceleraciones X/Y |
| Tres VL53L0X | Detección de distancia frontal izquierda, central y derecha |
| OLED de 128 × 32 píxeles | Estado local y telemetría básica |
| Computador con ROS 2 Humble | Agente micro-ROS, odometría, pruebas y navegación |

Las ruedas tienen diámetro nominal de 47.5 mm. La distancia física confirmada
entre centros de ruedas es 210 mm. Los sensores ToF están situados
aproximadamente a 120 mm del centro del robot, con orientaciones de +30°, 0° y
−30°. Las lecturas ToF se consideran útiles hasta 700 mm; las lecturas inválidas
o fuera del alcance se representan como cero.

El firmware estima la velocidad de cada rueda cada 100 ms a partir de los
encoders y actualiza el PID con ese mismo período. El cálculo usa 11 pulsos por
revolución del motor y una relación de reducción de 30:1. Las velocidades de
rueda se publican a 10 Hz cuando hay una muestra nueva; el IMU y los ToF se
publican a 20 Hz. Estas frecuencias y la cuantización de cuentas a baja
velocidad condicionan la resolución temporal de la telemetría.

## 4. Arquitectura de software

### 4.1 Firmware ESP32

El firmware se desarrolla como proyecto PlatformIO con Arduino y micro-ROS.
Sus responsabilidades principales son:

1. Contar pulsos de encoder mediante interrupciones.
2. Estimar las velocidades angulares de ambas ruedas.
3. Ejecutar dos controladores PID locales, uno por rueda.
4. Leer el MPU6500 y los tres sensores ToF.
5. Publicar datos de sensores y recibir consignas de velocidad por Wi-Fi.
6. Mostrar en la OLED información del robot y mensajes de estado.

Las ganancias del PID son independientes por rueda. En la versión consultada
para este informe están configuradas como `Kp=1.7`, `Ki=1.2`, `Kd=0.02` para
la rueda izquierda y `Kp=1.8`, `Ki=1.2`, `Kd=0.02` para la derecha. Estos
parámetros son de firmware y no se modifican desde la aplicación de
calibración.

El firmware deja la salida PWM en cero cuando el setpoint está por debajo del
mínimo configurado de 0.6 rad/s. Además, aplica un timeout de un segundo:
cuando dejan de recibirse consignas, pone los dos setpoints en cero. Esto
proporciona una condición de parada ante pérdida del flujo de comandos, aunque
no constituye un freno activo del motor.

### 4.2 ROS 2 y micro-ROS

El microcontrolador se conecta al agente micro-ROS sobre Wi-Fi. La interfaz
publicada incluye:

| Tópico | Contenido |
| --- | --- |
| `/robot/wheel_angular_velocities` | Velocidades angulares `[izquierda, derecha]` en rad/s |
| `/robot/imu/planar` | `[gyro_z, accel_x, accel_y]` en rad/s y m/s² |
| `/robot/tof/distances_mm` | Distancias `[izquierda, centro, derecha]` en mm |
| `/robot/wheel_angular_velocity_setpoints` | Setpoints de rueda `[izquierda, derecha]` en rad/s |

El paquete `robot_differential_test` proporciona un monitor, una herramienta
de envío de setpoints, una aplicación de calibración manual y una aplicación
de navegación. Se utiliza un único controlador publicando consignas a la vez
para evitar conflictos entre nodos.

### 4.3 Aplicaciones de prueba y navegación

La aplicación de calibración permite ejecutar ensayos rectos, giros manuales y
aproximaciones a una pared detenidas por el ToF central. Durante los ensayos
muestra pose estimada, velocidades, setpoints, gyro, lecturas ToF y una gráfica
de los setpoints y velocidades medidas. La gráfica retiene una ventana
temporal de 10 segundos en la implementación actual.

La aplicación también estima la demora entre una acción de interfaz y la
respuesta observada por los encoders. El usuario reportó valores de 0.4 a
1.0 s, con lecturas típicas cercanas a 0.6 s. Esta medida incluye períodos de
publicación, comunicación, muestreo y respuesta mecánica; no es una medida
aislada de latencia de red.

## 5. Fundamentos teóricos

### 5.1 Cinemática diferencial

Sean \(r\) el radio efectivo de rueda, \(b\) la separación efectiva usada por
el modelo, y \(\omega_L,\omega_R\) las velocidades angulares medidas. La
velocidad lineal longitudinal \(v\) y la velocidad angular del robot
\(\dot{\theta}\) se aproximan mediante:

\[
v = \frac{r}{2}(\omega_R+\omega_L), \qquad
\dot{\theta} = \frac{r}{b}(\omega_R-\omega_L).
\]

La pose se integra en un marco planar:

\[
\dot{x}=v\cos\theta,\qquad
\dot{y}=v\sin\theta.
\]

La segunda ecuación depende del cociente \(r/b\). Por eso, un radio incorrecto
o una separación mal configurada producen un error sistemático en el giro
estimado, incluso si las ruedas siguen adecuadamente sus consignas.

### 5.2 Control de velocidad

Cada rueda dispone de un controlador PID basado en el error entre velocidad
objetivo y velocidad estimada. El término derivativo se calcula a partir del
cambio de velocidad medida y se utiliza anti-windup para limitar la
acumulación integral cuando la salida PWM se satura. La estimación discreta
cada 100 ms implica que a velocidades bajas se cuentan pocas transiciones por
muestra; por ello, reducir el período sin revisar la cuantización puede
incrementar la variabilidad aparente de la velocidad.

### 5.3 Fusión de sensores mediante EKF

El EKF mantiene una estimación planar de posición, orientación, velocidades y
sesgo del giroscopio. Propaga la orientación con `gyro_z` y actualiza la
estimación usando velocidades de rueda. Las aceleraciones X/Y se muestran en
telemetría, pero no se integran en posición porque el sesgo y el ruido de
acelerómetro generarían deriva. La orientación del EKF no es una medición
independiente: combina el gyro con la velocidad angular calculada a partir de
las ruedas, de modo que un error persistente de odometría puede influir en el
resultado fusionado.

### 5.4 Detección reactiva de obstáculos

La navegación utiliza los tres ToF como una percepción local frontal. Reduce
la velocidad al acercarse a obstáculos, detiene el avance ante una detección
frontal cercana e intenta girar hacia el lado con más espacio. Esta lógica es
reactiva: no construye un mapa ni busca una ruta global. El campo de visión
está limitado al frente y la detección de un atasco no es concluyente si las
ruedas continúan girando mientras el chasis empuja un obstáculo.

## 6. Metodología de calibración

Se siguió un proceso empírico e incremental:

1. Verificación de signos de encoders, ejes y giroscopio con ruedas levantadas.
2. Ensayos de recta con distancias físicas marcadas y repetición en avance y
   retroceso.
3. Ensayos rectos con el ToF central como referencia de distancia frente a una
   pared plana.
4. Giros manuales de ±90° y ±360°, registrando al cruzar la orientación física
   de referencia.
5. Comparación de giro EKF, gyro y encoders, y cálculo de factores de escala.
6. Ajuste de parámetros geométricos de odometría, sin modificar
   simultáneamente ganancias PID ni escala del gyro.

Las pruebas manuales registran el estado al pulsar el botón de llegada. La
latencia hasta la detención física es posterior al registro y no se incorpora
al resultado almacenado. La referencia visual y el tiempo de reacción del
operador siguen siendo fuentes de incertidumbre, sobre todo en giros de 90°.

## 7. Resultados experimentales

Los siguientes valores se resumen del reporte proporcionado por el usuario.
Las estadísticas son descriptivas de esos ensayos; no representan una
caracterización metrológica formal ni una validación de la configuración nueva.

### 7.1 Traslación

Se registraron nueve rectas manuales de +2 m, seis rectas de −1.5 m y tres
recorridos detenidos por ToF. La mediana del factor lineal de los ensayos
rectos, considerando estas pruebas, fue aproximadamente **1.010**. La mayoría
de los ensayos manuales de 2 m estimó entre 1.97 y 1.99 m, con un resultado
atípico de 1.905 m. Las pruebas ToF estimaron desplazamientos cercanos a las
referencias obtenidas de la diferencia entre lecturas inicial y final.

En las rectas de retroceso, el error lateral del reporte fue positivo en los
seis ensayos. Este dato sugiere un posible sesgo direccional, pero no equivale
a una medición externa de desplazamiento lateral: proviene de la odometría y
debe interpretarse con cautela.

### 7.2 Rotación

Antes del ajuste final de la geometría se registraron seis giros de +360°, siete
de −360°, diez de +90° y cinco de −90°. En las vueltas completas:

| Referencia | Giro medio encoder | Giro medio gyro | Giro medio EKF |
| --- | ---: | ---: | ---: |
| +360° | +242° | +353° | +254° |
| −360° | −239° | −342° | −244° |

En los ensayos de 90°, los encoders y el EKF acumularon aproximadamente ±59° y
el gyro aproximadamente ±83–85°. La escala de giro por encoders era, por
tanto, consistentemente inferior a la referencia en ambos sentidos. El EKF
seguía de cerca el giro de encoder, coherente con la fusión descrita.

La diferencia entre los factores sugeridos por ensayos de 90° y 360° no
permitió justificar una reescala independiente del gyro. Por ello, su escala
se mantuvo sin cambios.

## 8. Ajustes y correcciones realizados

### 8.1 Geometría efectiva de odometría

La separación física se había introducido inicialmente como 0.296 m, pero se
verificó después que la distancia real entre centros de rueda es **0.210 m**.
Las pruebas de giro se habían realizado con la configuración antigua. Para
trasladar su factor de giro a la geometría actual se combinó el factor mediano
de encoder (aproximadamente 0.6685), el factor lineal (1.010) y la separación
antigua:

\[
b_{\mathrm{efectiva}} \approx 0.296 \times 0.6685 \times 1.010
\approx 0.200\ \mathrm{m}.
\]

La odometría ROS quedó configurada con radio efectivo
\(r=0.0475/2\times1.01\approx0.02399\) m y separación efectiva
\(b=0.200\) m. La separación efectiva es un parámetro de calibración del
modelo, distinto de la distancia física medida (0.210 m). El firmware no se
modificó para este ajuste.

Al aplicar retrospectivamente el cambio a las mediciones de giro antiguas,
el giro por encoder predicho queda cerca de la referencia. Esto es una
comprobación aritmética sobre los datos usados para calibrar, no una prueba
independiente. Los nuevos ensayos deben hacerse con el valor actual para
comprobar la mejora sin reutilizar el conjunto que produjo el ajuste.

### 8.2 Correcciones de control y diagnóstico

Durante el desarrollo se corrigieron aspectos de la lectura y fusión de
velocidad, incluyendo evitar fusionar repetidamente una misma muestra de
encoder. Se alineó la ejecución del PID con la frecuencia de medición y se
añadió anti-windup. También se implementaron ganancias PID independientes
para cada rueda, conservando al principio valores iguales y permitiendo
después ajustes individuales. El usuario ha ajustado posteriormente dichas
ganancias; los valores del firmware pueden cambiar respecto a los documentados
en versiones previas del README.

La aplicación de calibración incorpora una gráfica de consignas y velocidades
de ambas ruedas, ensayos manuales/ToF y una medición aproximada de latencia
botón-respuesta. Estas herramientas ayudaron a distinguir el error geométrico
de odometría de los transitorios de velocidad.

## 9. Dificultades y limitaciones

- **Diferencia entre ruedas al arranque:** una rueda puede adquirir velocidad
  antes que la otra y producir una desviación inicial; al estabilizarse,
  ambas pueden quedar sincronizadas. Esto apunta a una respuesta transitoria
  y no necesariamente a un error permanente de escala.
- **Latencia observable:** se han observado aproximadamente 0.4–1.0 s entre
  la acción en la interfaz y la respuesta detectada. La medición combina
  publicación, red, muestreo de encoder y respuesta mecánica.
- **Resolución de encoder:** el conteo cada 100 ms reduce la resolución a
  velocidades bajas. Una tasa mayor puede aumentar la cuantización si no se
  cambia también el método de estimación.
- **Referencias manuales:** la decisión visual del instante exacto de llegada
  y el tiempo de reacción introducen incertidumbre en las pruebas angulares.
- **Evasión limitada:** los ToF cubren solamente el frente, no proporcionan
  percepción posterior ni mapa; la evasión local puede detenerse si el
  obstáculo persiste y requiere supervisión.
- **Validación pendiente:** la orientación ha mejorado según la observación
  del usuario, pero aún no se ha aportado un nuevo conjunto de pruebas
  cuantitativas con la separación efectiva de 0.20 m.

## 10. Conclusiones

1. La integración de ESP32, micro-ROS y ROS 2 separa adecuadamente el control
   de bajo nivel de la estimación y las interfaces de operación. El PID
   permanece en el microcontrolador y la aplicación ROS ofrece observabilidad
   y herramientas de ensayo.
2. Las pruebas lineales sitúan la corrección de escala de distancia alrededor
   del 1 %, mientras que los ensayos de giro mostraron un error geométrico
   importante cuando se utilizaba una separación configurada incorrectamente.
3. La confirmación de la separación física de 0.210 m y el ajuste de la
   separación efectiva de odometría a 0.200 m constituyen una corrección
   coherente con el conjunto de datos disponible. La percepción del usuario
   de que la orientación funciona mucho mejor es consistente con esa
   corrección, aunque requiere corroboración con nuevas pruebas.
4. No se modificó la escala del gyro porque las estimaciones obtenidas en
   giros cortos y completos no fueron suficientemente consistentes. Tampoco
   debe atribuirse el error geométrico al PID.
5. La plataforma es funcional para experimentación y navegación relativa
   supervisada, pero todavía no debe considerarse un sistema de navegación
   autónoma robusta: falta afinar la respuesta dinámica de los motores y
   validar la rutina de evasión.

## 11. Trabajo futuro

- Repetir varias veces giros de ±90° y ±360° con la separación efectiva actual,
  registrando también la referencia física y las condiciones de velocidad.
- Repetir rectas en avance y retroceso para cuantificar el error lateral con
  una referencia externa y comprobar si existe un sesgo por dirección.
- Evaluar el seguimiento de velocidad por rueda a diferentes consignas y
  medir sobreimpulso, tiempo de subida y error estacionario antes de volver a
  ajustar las ganancias.
- Explorar una rampa de aceleración común o compensación de zona muerta si las
  pruebas confirman que el desbalance inicial limita los recorridos.
- Mejorar la evasión con pruebas controladas de diferentes posiciones de
  obstáculos, y documentar explícitamente sus límites de campo de visión.
- Mantener supervisión física y una parada de emergencia accesible durante
  toda prueba de velocidad o navegación.

## 12. Fuentes del proyecto

- Firmware ESP32 y comunicaciones: `bot_diferencial/esp-32/CodigosRobotDiferencial/src/main.cpp`.
- Configuración PlatformIO: `bot_diferencial/esp-32/CodigosRobotDiferencial/platformio.ini`.
- Estimación, control de navegación y ensayos: `microros_agent_ws/src/robot_differential_test/robot_differential_test/navigation.py`.
- Aplicación de calibración y gráfica: `microros_agent_ws/src/robot_differential_test/robot_differential_test/manual_control.py`.
- Instrucciones de instalación, operación y calibración: `bot_diferencial/esp-32/CodigosRobotDiferencial/README.md`.
- Resultados experimentales: reporte de calibración suministrado por el usuario en la conversación, correspondiente a las pruebas realizadas antes de actualizar la separación efectiva de odometría.
