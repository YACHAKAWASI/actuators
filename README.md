# actuator_motor

Paquete C/ROS 2 para controlar **dos motores independientes** con driver L298N en Raspberry Pi. Expone una librería compartida con utilidades de GPIO y un nodo ROS 2 para control dual de motores.

## Requisitos
- ROS 2 Jazzy (Ubuntu 24.04) con `ament_cmake`.
- Raspberry Pi 5 (o compatible) con acceso a GPIO.
- Backend de GPIO: libgpiod (por defecto en este paquete). No se usa WiringPi en Pi 5/Ubuntu 24.04.
- Compilador compatible con C99 y C++14.

## Estructura
- `include/actuator_motor/` headers públicos (`DEV_Config.h`, `motor_driver.h`, `pins.h`).
- `src/DEV_Config.c` utilidades de GPIO/tiempos basadas en libgpiod.
- `src/motor_driver.c` funciones para L298N.
- `src/test_motor.c` ejecutable de prueba (Motor A únicamente).
- `src/motor_node.cpp` nodo ROS 2 para **control dual de motores**.
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

## Nodo ROS 2 - Control Dual

```bash
source install/setup.bash
ros2 run actuator_motor motor_node
```

### Tópicos de Suscripción

El nodo `motor_node_dual` se suscribe a **4 tópicos** para control independiente de cada motor:

| Tópico | Tipo | Función | Rango |
|--------|------|---------|-------|
| `cmd_velA` | `geometry_msgs/Twist` | Control de velocidad Motor A | -vmax a +vmax |
| `cmd_velB` | `geometry_msgs/Twist` | Control de velocidad Motor B | -vmax a +vmax |
| `motor_pwmA` | `std_msgs/Int16` | PWM directo Motor A | -100 a +100 |
| `motor_pwmB` | `std_msgs/Int16` | PWM directo Motor B | -100 a +100 |

### Control Manual con teleop_twist_keyboard

Para controlar cada motor independientemente usando el teclado:

#### Motor A
```bash
# Terminal 1: Ejecutar el nodo de motores
source install/setup.bash
ros2 run actuator_motor motor_node

# Terminal 2: Control manual del Motor A
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=cmd_velA
```

#### Motor B
```bash
# Terminal 1: Ejecutar el nodo de motores (si no está ejecutándose)
source install/setup.bash
ros2 run actuator_motor motor_node

# Terminal 3: Control manual del Motor B
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=cmd_velB
```

### Control Directo por Tópicos

#### Control de Velocidad (geometry_msgs/Twist)
```bash
# Motor A hacia adelante
ros2 topic pub /cmd_velA geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}}"

# Motor A hacia atrás
ros2 topic pub /cmd_velA geometry_msgs/msg/Twist "{linear: {x: -0.3, y: 0.0, z: 0.0}}"

# Motor B hacia adelante
ros2 topic pub /cmd_velB geometry_msgs/msg/Twist "{linear: {x: 0.8, y: 0.0, z: 0.0}}"

# Parar ambos motores
ros2 topic pub /cmd_velA geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 topic pub /cmd_velB geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}}"
```

#### Control PWM Directo (std_msgs/Int16)
```bash
# Motor A al 60% hacia adelante
ros2 topic pub /motor_pwmA std_msgs/msg/Int16 "{data: 60}"

# Motor A al 40% hacia atrás
ros2 topic pub /motor_pwmA std_msgs/msg/Int16 "{data: -40}"

# Motor B al 80% hacia adelante
ros2 topic pub /motor_pwmB std_msgs/msg/Int16 "{data: 80}"

# Parar ambos motores
ros2 topic pub /motor_pwmA std_msgs/msg/Int16 "{data: 0}"
ros2 topic pub /motor_pwmB std_msgs/msg/Int16 "{data: 0}"
```

### Parámetros Configurables

```bash
# Configurar velocidad máxima
ros2 param set /motor_node_dual vmax 2.0

# Configurar PWM máximo
ros2 param set /motor_node_dual max_pwm 80

# Habilitar/deshabilitar PWM software
ros2 param set /motor_node_dual use_sw_pwm true

# Ver parámetros actuales
ros2 param list /motor_node_dual
ros2 param get /motor_node_dual vmax
```

### Ejemplo: Robot Diferencial

Para crear un robot diferencial básico:

```bash
# Terminal 1: Nodo de motores
ros2 run actuator_motor motor_node

# Terminal 2: Control Motor A (izquierdo)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=cmd_velA

# Terminal 3: Control Motor B (derecho) 
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=cmd_velB
```

**Comandos útiles:**
- **Avanzar**: `w` en ambos terminales
- **Retroceder**: `s` en ambos terminales  
- **Girar izquierda**: `a` en Motor A, `w` en Motor B
- **Girar derecha**: `w` en Motor A, `a` en Motor B
- **Parar**: `espacio` en ambos terminales

### Monitoreo y Debugging

```bash
# Ver todos los tópicos disponibles
ros2 topic list

# Monitorear tópicos en tiempo real
ros2 topic echo /cmd_velA
ros2 topic echo /cmd_velB
ros2 topic echo /motor_pwmA
ros2 topic echo /motor_pwmB

# Ver información del nodo
ros2 node info /motor_node_dual

# Ver información de un tópico específico
ros2 topic info /cmd_velA
ros2 topic info /motor_pwmA

# Ver la frecuencia de publicación
ros2 topic hz /cmd_velA
```

## Configuración de pines
Ajusta `include/actuator_motor/pins.h` para tu cableado:

### Motor A (Principal)
- `MOTOR_A_IN1` = GPIO26 (Pin 37)
- `MOTOR_A_IN2` = GPIO19 (Pin 35) 
- `MOTOR_A_EN`  = GPIO27 (Pin 13)

### Motor B (Secundario)
- `MOTOR_B_IN1` = GPIO23 (Pin 16)
- `MOTOR_B_IN2` = GPIO24 (Pin 18)
- `MOTOR_B_EN`  = GPIO25 (Pin 22)

## Permisos y grupos (libgpiod)
En Ubuntu 24.04 los GPIO se exponen vía `gpiochip*` y grupo `gpio`:
```bash
# Añade tu usuario al grupo gpio y reinicia sesión
sudo usermod -aG gpio $USER
# O ejecuta con sudo si es solo para probar
sudo -E ros2 run actuator_motor test_motor
```
Si ves errores de permisos al abrir el chip GPIO, verifica `/dev/gpiochip*` y pertenencia al grupo.

## Notas Importantes

### Hardware
- Comprueba distro activa: `echo $ROS_DISTRO` (debería ser `jazzy`).
- WiringPi está descontinuado y no se soporta en Raspberry Pi 5/Ubuntu 24.04.
- Si usas un HAT diferente, revisa `pins.h` y temporizaciones en `DEV_Config.c`.
- **Motor A** y **Motor B** son completamente independientes.

### Control
- Usa `teleop_twist_keyboard` con `--remap` para control individual de cada motor.
- Los valores de PWM van de -100 a +100 (negativo = reversa).
- Los valores de velocidad van de -vmax a +vmax (configurable).
- El deadband mínimo es 30% por defecto (configurable en código).

### Troubleshooting
```bash
# Si no tienes permisos de GPIO
sudo usermod -aG gpio $USER
# Reinicia sesión o ejecuta con sudo

# Si no encuentra el nodo
source install/setup.bash
ros2 pkg list | grep actuator

# Si los motores no responden
ros2 topic list  # Verifica que los tópicos existen
ros2 topic echo /cmd_velA  # Verifica que recibe comandos
```

## Licencia
Define la licencia en `package.xml` y aquí.
