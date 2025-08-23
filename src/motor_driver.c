#include "device_pack/motor_driver.h" // interfaz pública del driver L298N (tipos y prototipos)

#ifdef USE_WIRINGPI_LIB // si se compila con backend WiringPi
#include <softPwm.h> // funciones de PWM por software de WiringPi
#endif // USE_WIRINGPI_LIB

static UBYTE pwm_enabled_pins[64] = {0}; // mapa para saber si un pin usa PWM por software

void L298N_MotorInit(const L298NMotorPins *motor, UBYTE use_pwm) // inicializa GPIOs y PWM del motor
{
    if (motor == NULL) { // validar puntero
        return; // nada que hacer si motor es nulo
    }

    DEV_GPIO_Mode(motor->input_pin_1, 1); // configurar IN1 como salida
    DEV_GPIO_Mode(motor->input_pin_2, 1); // configurar IN2 como salida
    DEV_GPIO_Mode(motor->enable_pin, 1); // configurar ENA/ENB como salida

    DEV_Digital_Write(motor->input_pin_1, 0); // estado inicial bajo
    DEV_Digital_Write(motor->input_pin_2, 0); // estado inicial bajo

#ifdef USE_WIRINGPI_LIB // rama con PWM por software disponible
    if (use_pwm) { // si se solicita PWM
        if (softPwmCreate((int)motor->enable_pin, 0, 100) == 0) { // crear PWM [0..100]
            pwm_enabled_pins[motor->enable_pin] = 1; // marcar pin con PWM activo
        } else { // fallo al crear PWM
            pwm_enabled_pins[motor->enable_pin] = 0; // sin PWM
            DEV_Digital_Write(motor->enable_pin, 0); // asegurar salida en bajo
        }
    } else { // no se usará PWM
        pwm_enabled_pins[motor->enable_pin] = 0; // sin PWM
        DEV_Digital_Write(motor->enable_pin, 0); // salida inicial en bajo
    }
#else // sin WiringPi: no hay PWM por software
    (void)use_pwm; // suprimir warning de parámetro no usado
    pwm_enabled_pins[motor->enable_pin] = 0; // marcar sin PWM
    DEV_Digital_Write(motor->enable_pin, 0); // salida inicial en bajo
#endif // USE_WIRINGPI_LIB
}

void L298N_MotorSetDirection(const L298NMotorPins *motor, UBYTE forward) // fija sentido: 1 adelante, 0 atrás
{
    if (motor == NULL) { // validar puntero
        return; // sin acción
    }
    if (forward) { // sentido adelante
        DEV_Digital_Write(motor->input_pin_1, 1); // IN1 alto
        DEV_Digital_Write(motor->input_pin_2, 0); // IN2 bajo
    } else { // sentido atrás
        DEV_Digital_Write(motor->input_pin_1, 0); // IN1 bajo
        DEV_Digital_Write(motor->input_pin_2, 1); // IN2 alto
    }
}

void L298N_MotorBrake(const L298NMotorPins *motor) // frena cortocircuitando bobinas (IN1=IN2=1)
{
    if (motor == NULL) { // validar puntero
        return; // sin acción
    }
    DEV_Digital_Write(motor->input_pin_1, 1); // IN1 alto
    DEV_Digital_Write(motor->input_pin_2, 1); // IN2 alto

#ifdef USE_WIRINGPI_LIB // si PWM por software está activo para ENA/ENB
    if (motor->enable_pin < 64 && pwm_enabled_pins[motor->enable_pin]) { // rango válido y PWM activo
        softPwmWrite((int)motor->enable_pin, 100); // duty 100% para freno máximo
    } else { // sin PWM en ese pin
        DEV_Digital_Write(motor->enable_pin, 1); // habilitar salida alta
    }
#else // sin WiringPi
    DEV_Digital_Write(motor->enable_pin, 1); // habilitar salida alta
#endif // USE_WIRINGPI_LIB
}

void L298N_MotorStop(const L298NMotorPins *motor) // libera el motor (ambos IN en bajo, EN en 0/0%)
{
    if (motor == NULL) { // validar puntero
        return; // sin acción
    }
#ifdef USE_WIRINGPI_LIB // si PWM por software está activo
    if (motor->enable_pin < 64 && pwm_enabled_pins[motor->enable_pin]) { // válido y con PWM
        softPwmWrite((int)motor->enable_pin, 0); // duty 0%
    } else { // sin PWM
        DEV_Digital_Write(motor->enable_pin, 0); // salida baja
    }
#else // sin WiringPi
    DEV_Digital_Write(motor->enable_pin, 0); // salida baja
#endif // USE_WIRINGPI_LIB
    DEV_Digital_Write(motor->input_pin_1, 0); // IN1 bajo
    DEV_Digital_Write(motor->input_pin_2, 0); // IN2 bajo
}

void L298N_MotorSetSpeed(const L298NMotorPins *motor, UBYTE duty_cycle) // ajusta velocidad vía EN (0..100)
{
    if (motor == NULL) { // validar puntero
        return; // sin acción
    }
    if (duty_cycle >= 100) { // saturar al máximo permitido
        duty_cycle = 100; // limitar a 100
    }
#ifdef USE_WIRINGPI_LIB // si hay PWM por software disponible y activo
    if (motor->enable_pin < 64 && pwm_enabled_pins[motor->enable_pin]) { // válido y con PWM
        softPwmWrite((int)motor->enable_pin, (int)duty_cycle); // aplicar duty
        return; // no continuar con salida digital
    }
#endif // USE_WIRINGPI_LIB
    if (duty_cycle == 0) { // sin PWM: aproximación ON/OFF
        DEV_Digital_Write(motor->enable_pin, 0); // apagado
    } else { // cualquier valor > 0 se considera encendido
        DEV_Digital_Write(motor->enable_pin, 1); // encendido
    }
}


