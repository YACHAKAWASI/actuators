#ifndef _MOTOR_DRIVER_H_
#define _MOTOR_DRIVER_H_

#include "actuator_motor/DEV_Config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Estructura de pines para un motor controlado por L298N.
 * input_pin_1 y input_pin_2 definen el sentido de giro.
 * enable_pin se usa para habilitar el puente y/o PWM.
 */
typedef struct {
    UWORD input_pin_1;
    UWORD input_pin_2;
    UWORD enable_pin;   /* Pin ENA/ENB (PWM opcional) */
} L298NMotorPins;

/**
 * Inicializa los pines de un motor (dirección y enable).
 * Si use_pwm != 0, se espera que el enable_pin pueda usarse con PWM.
 */
void L298N_MotorInit(const L298NMotorPins *motor, UBYTE use_pwm);

/**
 * Establece el sentido del motor.
 * forward = 1 (adelante), 0 (atrás). Para frenar o parar, usar funciones dedicadas.
 */
void L298N_MotorSetDirection(const L298NMotorPins *motor, UBYTE forward);

/**
 * Freno activo: pone IN1 y IN2 al mismo nivel con enable activo.
 */
void L298N_MotorBrake(const L298NMotorPins *motor);

/**
 * Parada: deshabilita el puente (enable en 0) y pone entradas en bajo.
 */
void L298N_MotorStop(const L298NMotorPins *motor);

/**
 * Fija la velocidad (duty-cycle 0-100). Requiere soporte PWM en enable_pin.
 * Si no hay PWM disponible, valores distintos de 0/100 pueden no tener efecto.
 */
void L298N_MotorSetSpeed(const L298NMotorPins *motor, UBYTE duty_cycle);

#ifdef __cplusplus
}
#endif

#endif /* _MOTOR_DRIVER_H_ */


