# Firmware PlatformIO del robot diferencial

Este proyecto integra el control PID de los motores DRV8871, los encoders, el
MPU6500, tres sensores VL53L0X y la pantalla OLED con micro-ROS por Wi-Fi.
Los sketches originales para Arduino IDE permanecen en sus carpetas.

## Carpetas del proyecto

- **Firmware ESP32 (abrir en PlatformIO):**
  `~/RoboticaMovil/bot_diferencial/esp-32/CodigosRobotDiferencial`
- **Workspace ROS 2 Humble (agente y paquete de prueba):**
  `~/RoboticaMovil/microros_agent_ws`
- **Paquete de agente micro-ROS:**
  `~/RoboticaMovil/microros_agent_ws/src/micro-ROS-Agent/micro_ros_agent`
- **Paquete de prueba del robot:**
  `~/RoboticaMovil/microros_agent_ws/src/robot_differential_test`

El agente y el paquete de prueba ya están dentro de `~/RoboticaMovil`; no hace
falta copiarlos desde otra ubicación. El paquete `robot_differential_test`
incluye el monitor de sensores, el publicador de setpoint y la aplicación
gráfica de navegación y la aplicación de control manual para calibrar la
odometría.

## 1. Configuración de red

El firmware usa las mismas credenciales y dirección del agente que
`microros_wifi_test`, definidas en:

`~/RoboticaMovil/bot_diferencial/esp-32/microros_wifi_test/include/secrets.h`

Ese archivo local contiene las credenciales y no debe compartirse. Verifica que
la dirección configurada para el agente corresponda a la IP de este computador
en la misma red Wi-Fi (`hostname -I`) y que el puerto sea `8888`. El computador
y el robot deben poder comunicarse entre sí por la red local.

## 2. Compilar y cargar el firmware

En VS Code, selecciona **File > Open Folder...** y abre exactamente:

`~/RoboticaMovil/bot_diferencial/esp-32/CodigosRobotDiferencial`

No abras `src` ni `microros_wifi_test` como proyecto PlatformIO. También puedes
compilar y cargar desde una terminal:

```sh
cd ~/RoboticaMovil/bot_diferencial/esp-32/CodigosRobotDiferencial
pio run
pio run --target upload
pio device monitor
```

Deja el robot quieto durante la calibración del MPU6500 al encenderlo. Si la
terminal de carga no encuentra el puerto serial, conecta la placa y revisa el
puerto detectado por PlatformIO.

## 3. Preparar el workspace ROS 2

Los comandos ROS no se ejecutan automáticamente en una terminal nueva. Ejecuta
los siguientes comandos **una vez por cada terminal** que vayas a usar:

```sh
source /opt/ros/humble/setup.bash
source ~/RoboticaMovil/microros_agent_ws/install/setup.bash
```

El workspace y sus paquetes están en `~/RoboticaMovil/microros_agent_ws`. Si
modificas o necesitas reconstruir los paquetes, compílalos así:

```sh
cd ~/RoboticaMovil/microros_agent_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 4. Iniciar el agente micro-ROS

Abre una terminal nueva (Terminal 1), carga el entorno ROS con los dos comandos
de la sección anterior y ejecuta:

```sh
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Mantén esta terminal abierta. El agente debe iniciarse antes de que el ESP32
intente conectarse. Al arrancar, la placa se conecta a Wi-Fi y vuelve a intentar
encontrar el agente si aún no está disponible.

## 5. Verificar los sensores

Abre otra terminal (Terminal 2), carga nuevamente el entorno ROS y ejecuta:

```sh
ros2 run robot_differential_test monitor
```

El monitor muestra las velocidades angulares de ambas ruedas, la velocidad
angular Z y aceleraciones X/Y del MPU6500, y las distancias de los ToF. También
puedes consultar un tópico directamente con `ros2 topic echo`; cada comando
permanece activo hasta presionar `Ctrl+C`:

```sh
ros2 topic list
ros2 topic echo /robot/wheel_angular_velocities
ros2 topic echo /robot/imu/planar
ros2 topic echo /robot/tof/distances_mm
```

## 6. Enviar un setpoint de prueba

Coloca el robot de forma segura, con las ruedas motrices levantadas del suelo.
Abre una tercera terminal (Terminal 3), carga el entorno ROS y ejecuta:

```sh
ros2 run robot_differential_test setpoint 6.0
```

Con un argumento, se usa el mismo setpoint para ambas ruedas. También puedes
dar dos valores iniciales independientes, primero izquierda y luego derecha:

```sh
ros2 run robot_differential_test setpoint 6.0 8.0
```

Los valores se expresan en rad/s y se publican continuamente a 10 Hz. Para
cambiar una sola rueda al vuelo sin detener el nodo, abre otra terminal, carga
el entorno ROS y establece el parámetro correspondiente:

```sh
ros2 param set /robot_differential_setpoint_test velocidad_izquierda_rad_s 6.0
```

Para cambiar ambas ruedas en una sola operación atómica, llama al servicio
estándar `set_parameters_atomically` del nodo:

```sh
ros2 service call /robot_differential_setpoint_test/set_parameters_atomically \
  rcl_interfaces/srv/SetParametersAtomically \
  "{parameters: [{name: velocidad_izquierda_rad_s, value: {type: 3, double_value: 6.0}}, {name: velocidad_derecha_rad_s, value: {type: 3, double_value: 8.0}}]}"
```

Los valores `6.0` y `8.0` son ejemplos en rad/s. Para detener ambas ruedas,
envía cero a los dos parámetros en una llamada atómica:

```sh
ros2 service call /robot_differential_setpoint_test/set_parameters_atomically \
  rcl_interfaces/srv/SetParametersAtomically \
  "{parameters: [{name: velocidad_izquierda_rad_s, value: {type: 3, double_value: 0.0}}, {name: velocidad_derecha_rad_s, value: {type: 3, double_value: 0.0}}]}"
```

El nodo sigue publicando el tópico
`/robot/wheel_angular_velocity_setpoints` (`std_msgs/msg/Float32MultiArray`) a
10 Hz, en el orden `[izquierda, derecha]`. Si detienes el nodo con `Ctrl+C`, el
firmware pone ambos motores en cero al vencer su watchdog de 1 segundo.

## 7. Aplicación gráfica de navegación

Primero inicia el agente micro-ROS como se indica en la sección 4 y espera a
que el ESP32 publique sus sensores. En una terminal nueva, carga el entorno ROS
2 de la sección 3 y ejecuta:

```sh
ros2 run robot_differential_test navigate
```

La ventana permite indicar un destino `(x, y)` en metros respecto al origen
inicial y a la orientación al arrancar (X hacia delante, Y hacia la izquierda),
ajustar límites conservadores de velocidad y ver la pose estimada, la
trayectoria, las lecturas ToF, los setpoints enviados y las velocidades
medidas. Pulsa **Ir al destino** para
empezar y **DETENER** para mandar velocidad cero. La pose solo se puede
reiniciar cuando la navegación está detenida y las ruedas casi quietas. Levanta
las ruedas del suelo para la primera prueba.

La geometría física es diámetro de rueda 47.5 mm, distancia entre centros de
rueda 210 mm y sensores a 12 cm del centro con orientación izquierda/centro/
derecha de +30°, 0° y -30°. La odometría emplea una separación efectiva
calibrada de 200 mm. Si la orientación de giro o el eje X/Y no coincide con la
del robot, detén la prueba y verifica el montaje y las convenciones.

La estimación es odometría relativa con un EKF que usa las velocidades de las
ruedas y el gyro Z; ax/ay se muestran, pero no se integran para evitar que el
sesgo del acelerómetro derive la posición. Sin cámara, GPS, lidar ni balizas,
la posición todavía deriva y no representa una ubicación absoluta. Los ToF
solo cubren el abanico frontal y reportan `0` cuando no hay una distancia válida
dentro de 70 cm; no se puede asumir que el espacio esté libre detrás del robot.
La evasión es reactiva y local, no crea un mapa ni planifica rutas globales.

Si los datos ROS dejan de llegar, la aplicación envía cero y cancela el
objetivo. Si las ruedas no responden al comando durante 2 segundos, informa de
un posible atasco y se detiene. Si una evasión no consigue despejar un obstáculo
cercano durante 3 segundos, cancela el objetivo y se detiene. No puede detectar
de forma fiable que el robot está empujando una pared mientras las ruedas
siguen girando. El operador debe supervisar el robot y usar **DETENER** si hay
riesgo.

### Comprobaciones antes de calibrar la navegación

Las velocidades del tópico `/robot/wheel_angular_velocities` son medidas por
los encoders; verificar el diámetro y la separación de ruedas no verifica que
el conteo de pulsos por vuelta sea correcto. Con el robot elevado, mueve ambas
ruedas a mano hacia delante: el monitor debe mostrar dos velocidades positivas
y parecidas. Muévelas hacia atrás: ambas deben ser negativas. Haz girar el
robot sobre su centro hacia la izquierda: la rueda izquierda debe ser negativa,
la derecha positiva, y `gyro_z` positivo. Si no se cumplen esos signos, no uses
la navegación hasta corregir encoder, eje o convención del IMU.

Después, con los sensores frescos, prueba una distancia recta corta y compara
el desplazamiento físico con el estimado. Un factor de error consistente indica
que hay que calibrar el conteo efectivo de cada encoder (pulsos por vuelta,
decodificación y relación de reducción), no cambiar las dimensiones ya medidas.
Los valores de aceleración en reposo del ejemplo, cercanos a ±0.1 m/s², y
`gyro_z` cercano a cero no muestran por sí solos un sesgo grande; ToF en `0`
significa que no hubo una detección válida dentro del alcance configurado, no
que haya un obstáculo a distancia cero.

La aplicación limita cada comando de rueda a 8 rad/s. El mínimo medido estable
se configuró en 0.6 rad/s tanto en firmware como en navegación; una rueda por
debajo de ese límite se detiene, y ambas se elevan al mínimo solo para avance
recto lento. Si ese valor no resulta estable con el robot en el suelo, mídelo
por rueda bajo carga y ajusta ambos límites.

En las pruebas manuales y ToF, el selector de velocidad lineal permite hasta
0.18 m/s. Con el radio efectivo actual (0.02399 m), esto equivale a unos
7.5 rad/s por rueda, por debajo del límite de 8 rad/s de la aplicación ROS.
Empieza las pruebas de mayor velocidad en un área despejada, con el robot
supervisado y el botón **DETENER** disponible.

Las ganancias PID se configuran independientemente en `kLeftPidGains` y
`kRightPidGains` (`kp`, `ki`, `kd`) en `src/main.cpp`. Los valores actuales son
`(1.7, 1.2, 0.02)` para la rueda izquierda y `(1.8, 1.2, 0.02)` para la derecha.
Para calibrar, cambia una sola ganancia de una rueda por vez y vuelve a
compilar y cargar el firmware; verifica primero la respuesta con las ruedas
levantadas y después con el robot supervisado en el suelo.

La velocidad de encoder se estima y el PID se actualiza cada 100 ms. Las
velocidades de las ruedas se publican a 10 Hz (solo cuando hay una medición
nueva); IMU y ToF se publican a 20 Hz. A 0.6 rad/s y 330 cuentas por vuelta se
reciben aproximadamente 3 cuentas por muestra; bajar el intervalo del encoder
a 50 ms daría menos de dos cuentas por muestra y aumentaría la cuantización.
La navegación solo fusiona cada muestra de encoder una vez.

La aplicación muestra `ωz` del gyro, `ωz` calculado por diferencia de ruedas y
el sesgo estimado del gyro. En un giro sobre el centro, ambos `ωz` deberían
tener el mismo signo y magnitudes razonablemente parecidas. Una diferencia
persistente apunta a revisar el signo del gyro, los signos de encoder o la
distancia entre ruedas antes de ajustar el control de orientación.

## 8. Ensayos de calibración de odometría

Reconstruye el paquete si aún no has compilado esta aplicación y carga el
workspace en una terminal nueva:

```sh
cd ~/RoboticaMovil/microros_agent_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_differential_test
source install/setup.bash
```

Con el agente micro-ROS y el robot encendidos, ejecuta en otra terminal:

```sh
source /opt/ros/humble/setup.bash
source ~/RoboticaMovil/microros_agent_ws/install/setup.bash
ros2 run robot_differential_test manual
```

Durante los ensayos, la aplicación muestra una gráfica de los setpoints y las
velocidades medidas de las ruedas izquierda y derecha. Las líneas punteadas son
los setpoints y las continuas son las lecturas de encoder; la ventana conserva
los últimos 60 segundos. Así puedes comparar el seguimiento de cada rueda y
observar sus fluctuaciones sin cambiar el firmware.

La telemetría también muestra la latencia desde **Iniciar prueba** hasta que
ambos encoders detectan movimiento, y desde **Marcar llegada y registrar** o
**DETENER** hasta que detectan que ambas ruedas se detuvieron. El arranque se
confirma con dos muestras consecutivas sobre el umbral; la parada, con tres
muestras bajo 0.15 rad/s. Como los encoders se publican a 10 Hz, la resolución
es de aproximadamente 100 ms. La medición incluye el siguiente ciclo de
publicación del comando, la comunicación ROS/Wi-Fi y la respuesta mecánica; no
es una medición aislada de la red. Si una rueda no responde dentro de 5 s, se
indica en la telemetría.

La aplicación ofrece ensayos predefinidos: recta de ±2 m con final manual,
recta hasta un umbral del ToF central, giros de ±360° con final manual y giros
de ±90°. Cada ensayo manda velocidades constantes y **no se detiene por lo que
indique la odometría**. En los ensayos manuales, pulsa **Marcar llegada y
registrar** cuando el robot alcance la marca física. El ensayo ToF se detiene
después de tres lecturas válidas consecutivas del sensor central por debajo
del umbral. **DETENER sin registrar** cancela la prueba sin añadirla al
reporte. Los resultados se acumulan en la ventana y se copian con **Copiar
reporte**.
**Reiniciar pose aquí** pone a cero la pose y las integraciones al estar las
ruedas quietas, sin borrar resultados ya registrados.

La recta con ToF requiere una pared plana, robot inicialmente paralelo a ella
y lectura central válida menor de 700 mm. Coloca el robot a unos 60 cm, usa
como primer objetivo 150–200 mm y empieza despacio. La referencia de avance
es lectura central inicial menos lectura final; el sensor está desplazado
respecto al centro del robot, así que no acerques el chasis a una pared sin
considerar su tamaño. Si la lectura central se invalida, el ensayo ToF se
cancela sin registrar. Todos los ensayos tienen un límite temporal de seguridad
y también se detienen por datos de odometría caducados o atasco. El ToF **no**
protege contra otros obstáculos.

### Cómo usar y leer los ensayos

1. Confirma los signos con las ruedas levantadas: avance positivo en ambas
   ruedas y giro antihorario con izquierda negativa, derecha positiva y `ωz`
   positivo. Después baja el robot a una superficie plana, despeja el área y
   coloca cinta para marcar distancia y rumbo.
2. Para la recta manual de 2 m, alinea el eje del robot con la cinta y pulsa
   **Iniciar prueba**. Cuando el eje llegue a la marca de 2 m, pulsa
   **Marcar llegada y registrar**. Repite varias veces y en reversa. Para la
   alternativa de pared, selecciona el ensayo ToF: registra automáticamente
   la posición cuando el sensor central alcance el umbral.
3. Para el giro, alinea el robot con una marca o una pared y selecciona
   `+360°` o `-360°`. Pulsa **Marcar llegada y registrar** cuando vuelva a
   alinearse; repite varias vueltas en ambos sentidos. Usa ±90° como ensayo
   adicional, no como única calibración.
4. Copia el reporte. En pruebas lineales, compara `referencia` con
   `avance_odom_m`; el reporte incluye `factor_radio`, calculado como
   `distancia_referencia / avance_odom`; como primera aproximación, multiplica
   el radio efectivo actual por ese factor.
   `error_lateral_m` indica cuánto se desvió lateralmente. En giros compara
   `referencia` con `giro_encoder_deg`, `giro_gyro_deg` y `giro_ekf_deg`.
   Para ajustar la separación entre ruedas usa `factor_separacion_ruedas`,
   calculado como `giro_encoder / giro_físico`; `factor_escala_gyro` es
   `giro_físico / giro_gyro`. Si aplicas en conjunto factores obtenidos con la
   misma geometría base, la separación nueva es la anterior multiplicada por
   `factor_radio * factor_separacion_ruedas`; si ya corregiste el radio y
   repetiste los giros, usa solo el nuevo `factor_separacion_ruedas`.
   Si el gyro concuerda con la referencia física y los encoders no, no cambies
   el factor del gyro: el error apunta a geometría/deslizamiento/escala de
   ruedas. Si el gyro es el que discrepa, investiga su signo, sesgo y escala.
   Un factor negativo apunta primero a una convención de signo incorrecta, no
   a cambiar una dimensión física.
5. Cambia un solo parámetro por vez y repite. Comparte el reporte completo,
   incluyendo las repeticiones de cada sentido; la aplicación no modifica
   automáticamente las constantes.

Los giros acumulados de gyro y encoders son integraciones independientes; el
EKF combina ambas fuentes y puede quedar sesgado hacia una de ellas. En la
captura compartida, por ejemplo, EKF y encoders marcaban cerca de `-356°`
mientras el gyro marcaba `-486°`. El dato por sí solo no dice cuál es correcto:
la marca física al terminar el giro es la referencia que permite decidirlo.
Ningún ensayo con pared sustituye la parada de emergencia ni la supervisión.

Con las pruebas de calibración realizadas, la odometría ROS usa radio efectivo
de rueda `0.0475 / 2 * 1.01` (factor lineal mediano aproximado 1.010) y
separación efectiva de odometría `0.20 m`. La distancia física entre los
centros de las ruedas es `0.21 m`; la separación efectiva se ajustó con los
giros registrados cuando el programa aún usaba `0.296 m`, combinando el factor
de giro mediano (0.6685) y el factor lineal (1.010). Estos parámetros están en
`navigation.py`; el firmware y la escala del gyro no se modifican. Los giros
anteriores se midieron con la configuración antigua, por lo que las nuevas
pruebas deben confirmar el resultado con la separación efectiva actual.

## Interfaz ROS 2

El nodo del ESP32 es `/robot/differential_robot_esp32`.
La OLED se inicializa al principio del arranque y muestra las últimas cuatro
líneas de los mensajes enviados al monitor serial. Cuando pasan 3 segundos sin
nuevos mensajes, vuelve a mostrar el estado de conexión y los sensores.

| Tópico | Tipo | Datos |
| --- | --- | --- |
| `/robot/wheel_angular_velocities` | `std_msgs/msg/Float32MultiArray` | `[izquierda, derecha]` en rad/s |
| `/robot/imu/planar` | `std_msgs/msg/Float32MultiArray` | `[gyro_z, accel_x, accel_y]`; rad/s y m/s² |
| `/robot/tof/distances_mm` | `std_msgs/msg/Int32MultiArray` | `[izquierda, centro, derecha]` en mm |
| `/robot/wheel_angular_velocity_setpoints` | `std_msgs/msg/Float32MultiArray` | Setpoints `[izquierda, derecha]` en rad/s |

Las lecturas ToF inválidas o mayores a 700 mm se publican como `0`. Cada
setpoint de rueda se convierte a RPM y se aplica a su PID; el umbral mínimo de
control es 0.6 rad/s (aproximadamente 5.73 RPM).
