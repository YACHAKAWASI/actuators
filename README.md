# actuator_motor

Paquete C/ROS 2 para controlar un driver L298N en Raspberry Pi. Expone una librería compartida con utilidades de GPIO y un ejecutable de prueba para mover el motor.

## Requisitos
- ROS 2 Jazzy (Ubuntu 24.04) con `ament_cmake`.
- Raspberry Pi 5 (o compatible) con acceso a GPIO.
- Backend de GPIO: libgpiod (por defecto en este paquete). No se usa WiringPi en Pi 5/Ubuntu 24.04.
- Compilador compatible con C99 y C++14.

## Estructura
- `include/actuator_motor/` headers públicos (`DEV_Config.h`, `motor_driver.h`, `pins.h`).
- `src/DEV_Config.c` utilidades de GPIO/tiempos basadas en libgpiod.
- `src/motor_driver.c` funciones para L298N.
- `src/test_motor.c` ejecutable de prueba.
- `src/motor_node.cpp` nodo ROS 2.
- `CMakeLists.txt` configuración con `ament_cmake`.
- `package.xml` metadatos del paquete.

## Selección de backend
Las definiciones en CMake activan libgpiod y desactivan WiringPi por defecto:
- `USE_WIRINGPI_LIB=0`
- `USE_GPIOD_LIB=1`
- `USE_I2CDEV_LIB=1`
- `USE_SPIDEV_LIB=0`

Si necesitas PWM por software, descomenta `USE_SW_PWM=1` en `CMakeLists.txt` y asegúrate de enlazar con `pthread` (ya configurado).

## Instalación de dependencias del sistema
```bash
# Ubuntu Server 24.04 (Noble) en Raspberry Pi
sudo apt update
# ROS 2 Jazzy (desktop o base, según tu preferencia)
sudo apt install -y ros-jazzy-ros-base \
  build-essential cmake git \
  libgpiod-dev libgpiod2
```

## Compilación
```bash
# Activa tu entorno ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# En la raíz del workspace (donde está src/ o scr/ según tu estructura)
colcon build

# Opcional: cambiar flags (ejemplo: desactivar I2C)
colcon build --cmake-args -DUSE_I2CDEV_LIB=0
```

## Ejecución del ejemplo
```bash
source install/setup.bash
ros2 run actuator_motor test_motor
```
Secuencia: inicializa módulo → configura pines → gira adelante → frena → gira atrás → detiene.

## Nodo ROS 2
```bash
source install/setup.bash
ros2 run actuator_motor motor_node
```

## Configuración de pines
Ajusta `include/actuator_motor/pins.h` para tu cableado:
- `MOTOR_A_IN1`, `MOTOR_A_IN2`, `MOTOR_A_EN`

## Permisos y grupos (libgpiod)
En Ubuntu 24.04 los GPIO se exponen vía `gpiochip*` y grupo `gpio`:
```bash
# Añade tu usuario al grupo gpio y reinicia sesión
sudo usermod -aG gpio $USER
# O ejecuta con sudo si es solo para probar
sudo -E ros2 run actuator_motor test_motor
```
Si ves errores de permisos al abrir el chip GPIO, verifica `/dev/gpiochip*` y pertenencia al grupo.

## Notas
- Comprueba distro activa: `echo $ROS_DISTRO` (debería ser `jazzy`).
- WiringPi está descontinuado y no se soporta en Raspberry Pi 5/Ubuntu 24.04.
- Si usas un HAT diferente, revisa `pins.h` y temporizaciones en `DEV_Config.c`.

## Licencia
Define la licencia en `package.xml` y aquí.
