# Robot diferencial ESP32 + ROS 2

Proyecto de robótica móvil diferencial con firmware ESP32/PlatformIO,
micro-ROS sobre Wi-Fi y herramientas ROS 2 para telemetría, calibración y
navegación.

## Contenido

- Firmware PlatformIO, PID de los motores, encoders, MPU6500, tres VL53L0X y
  OLED: [`bot_diferencial/esp-32/CodigosRobotDiferencial/`](bot_diferencial/esp-32/CodigosRobotDiferencial/).
- Paquete ROS 2 para monitor, setpoints, pruebas de calibración y navegación:
  [`microros_agent_ws/src/robot_differential_test/`](microros_agent_ws/src/robot_differential_test/).
- Informe técnico y resultados de calibración:
  [`INFORME_TECNICO_ROBOT_DIFERENCIAL.md`](INFORME_TECNICO_ROBOT_DIFERENCIAL.md).
- Instrucciones completas de compilación y operación:
  [`README del firmware`](bot_diferencial/esp-32/CodigosRobotDiferencial/README.md).

## Configuración privada

Las credenciales Wi-Fi y la dirección IP del agente micro-ROS no se incluyen
en Git. Antes de compilar el firmware, crea el archivo local de configuración
a partir de la plantilla:

```sh
cd ~/RoboticaMovil
cp bot_diferencial/esp-32/microros_wifi_test/include/secrets.h.example \
  bot_diferencial/esp-32/microros_wifi_test/include/secrets.h
```

Edita `secrets.h` con el SSID, la contraseña Wi-Fi y la IP local del computador
que ejecuta el agente. El archivo está excluido por `.gitignore`; no lo
compartas ni lo añadas manualmente al repositorio.

## Dependencias ROS 2

Se requiere ROS 2 Humble, `colcon` y `vcstool`. Desde la raíz del workspace,
descarga las dependencias de micro-ROS y compila:

```sh
cd ~/RoboticaMovil/microros_agent_ws
vcs import src < robot_differential.repos
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

La aplicación de calibración se inicia con:

```sh
ros2 run robot_differential_test manual
```

El agente micro-ROS debe estar ejecutándose en el puerto y dirección
configurados en `secrets.h`. Las instrucciones de operación y seguridad están
en el README del firmware.

## Firmware

Abre `bot_diferencial/esp-32/CodigosRobotDiferencial` como proyecto PlatformIO.
Para compilar desde terminal:

```sh
cd ~/RoboticaMovil/bot_diferencial/esp-32/CodigosRobotDiferencial
pio run
```

Consulta el README del firmware para cargarlo a la ESP32, ejecutar el agente y
operar las aplicaciones ROS 2.
