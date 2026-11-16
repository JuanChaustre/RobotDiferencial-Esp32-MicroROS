# Pruebas ESP32 con PlatformIO y micro-ROS por Wi-Fi

Proyecto de prueba independiente de los sketches originales. Usa PlatformIO con
el framework Arduino para mantener compatibilidad con el ESP32 clásico
(`esp32dev`) y con las bibliotecas Arduino que se integrarán más adelante.
La plataforma Espressif está fijada en 6.12.0 porque el plugin micro-ROS no es
compatible con los flags de compilación de la plataforma 8.x.

## Preparación

1. En este equipo, PlatformIO Core 6.2.0 está instalado en un entorno virtual
   de usuario y `pio` está disponible desde `~/.local/bin`. Si VS Code ya estaba
   abierto durante la instalación, reinicia VS Code para actualizar su PATH.
2. Copia `include/secrets.h.example` a `include/secrets.h`.
3. En `include/secrets.h`, configura el SSID, la contraseña Wi-Fi y la dirección
   IP LAN de este equipo. `AGENT_PORT` debe coincidir con el puerto UDP del
   agente (8888 por defecto).
4. En PlatformIO, selecciona el puerto serie correspondiente al ESP32. Si tu
   placa no es un ESP32 Dev Module clásico, cambia `board = esp32dev` en
   `platformio.ini` por el identificador de placa correcto.

`include/secrets.h` está excluido de Git para evitar publicar credenciales.
No compartas ese archivo.

## Prueba de parpadeo

Compila y carga el firmware de parpadeo:

```sh
pio run -e blink
pio run -e blink -t upload
pio device monitor -b 115200
```

El LED integrado (GPIO 2 en el ESP32 Dev Module clásico) debe encenderse y
apagarse cada medio segundo.

## Prueba micro-ROS por Wi-Fi

Este entorno compila micro-ROS para ROS 2 Humble y transporte UDP/Wi-Fi. La
primera compilación descarga la plataforma y las dependencias de PlatformIO, y
genera las bibliotecas micro-ROS:

```sh
pio run -e microros_wifi
pio run -e microros_wifi -t upload
pio device monitor -b 115200
```

En este equipo, el agente ya está compilado en
`~/RoboticaMovil/microros_agent_ws/install`. En otra terminal, inicia el agente con el mismo
ROS 2 Humble:

```sh
source /opt/ros/humble/setup.bash
source ~/RoboticaMovil/microros_agent_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

El workspace fuente está en `~/microros_agent_ws`. Para reconstruirlo después
de actualizar sus fuentes:

```sh
cd ~/RoboticaMovil/microros_agent_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to micro_ros_agent
```

La dirección IP que se configura en `secrets.h` debe ser la dirección LAN del
equipo, no `127.0.0.1`. El equipo y el ESP32 deben poder comunicarse entre sí
por UDP; una red de invitados o el aislamiento de clientes Wi-Fi puede
impedirlo.

Al conectar, el ESP32 publica un contador una vez por segundo y recibe órdenes
para encender o apagar su LED:

```sh
ros2 topic echo /esp32/status_counter
ros2 topic pub -r 1 /esp32/led_cmd std_msgs/msg/Bool "{data: true}"
ros2 topic pub -r 1 /esp32/led_cmd std_msgs/msg/Bool "{data: false}"
```

Si el monitor serie indica que el agente no está disponible, verifica que el
agente esté ejecutándose, la IP configurada, el puerto UDP 8888 y que ambos
dispositivos estén en la misma red.
