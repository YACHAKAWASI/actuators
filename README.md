# device_pack

Paquete C/ROS 2 para controlar un driver L298N en Raspberry Pi. Expone una librería compartida con utilidades de GPIO y un ejecutable de prueba para mover el motor.

## Requisitos
- ROS 2 (recomendado: Humble) con `ament_cmake`.
- Raspberry Pi con acceso a GPIO.
- Uno de estos backends de GPIO:
  - WiringPi (PWM por software vía `softPwm`)
  - BCM2835
  - Acceso directo `/dev` (sysfs)

## Estructura
- `include/device_pack/` headers públicos (`DEV_Config.h`, `motor_driver.h`, `pins.h`).
- `src/DEV_Config.c` utilidades de GPIO/tiempos por backend.
- `src/motor_driver.c` funciones para L298N.
- `src/test_motor.c` ejecutable de prueba.
- `CMakeLists.txt` configuración con `ament_cmake`.
- `package.xml` metadatos del paquete.

## Selección de backend
Opciones CMake (elige solo una en ON):
- `USE_WIRINGPI_LIB` (ON por defecto)
- `USE_BCM2835_LIB`
- `USE_DEV_LIB`

## Compilación
```bash
# Activa tu ROS 2 (ejemplo Humble)
source /opt/ros/humble/setup.bash

# WiringPi (por defecto)
colcon build

# Forzar BCM2835
colcon build --cmake-args -DUSE_WIRINGPI_LIB=OFF -DUSE_BCM2835_LIB=ON -DUSE_DEV_LIB=OFF

# Forzar /dev (sysfs)
colcon build --cmake-args -DUSE_WIRINGPI_LIB=OFF -DUSE_BCM2835_LIB=OFF -DUSE_DEV_LIB=ON
```

## Ejecución del ejemplo
```bash
source install/setup.bash
ros2 run device_pack test_motor
```
Secuencia: inicializa módulo → configura pines → gira adelante → frena → gira atrás → detiene.

## Configuración de pines
Ajusta `include/device_pack/pins.h` para tu cableado:
- `MOTOR_A_IN1`, `MOTOR_A_IN2`, `MOTOR_A_EN`

## Notas
- Comprueba distro activa: `echo $ROS_DISTRO` (debería ser `humble` si usas Humble).
- Si eliges WiringPi, asegúrate de tenerla instalada.

## Licencia
Define la licencia en `package.xml` y aquí.
