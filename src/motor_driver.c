#include "actuator_motor/motor_driver.h"  // interfaz pública del driver L298N
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

// ================================================================
//  Opcional: PWM por software con hilos (desactivado por defecto)
//  Pon USE_SW_PWM 1 si quieres PWM por software sencillo.
// ================================================================
#ifndef USE_SW_PWM
#  define USE_SW_PWM 0
#endif

#if USE_SW_PWM
  #include <pthread.h>
  #include <stdatomic.h>

  typedef struct {
      UWORD pin;
      atomic_int duty;      // 0..100
      atomic_int running;   // 0/1
      pthread_t thread;
      unsigned pwm_hz;      // frecuencia (p.ej. 200 Hz)
  } sw_pwm_t;

  // Mapa simple: un PWM por pin BCM (hasta 128 por si acaso)
  static sw_pwm_t* sw_pwm_map[128] = {0};

  static void* sw_pwm_loop(void* arg) {
      sw_pwm_t* ctx = (sw_pwm_t*)arg;
      if (!ctx) return NULL;

      // Periodo en microsegundos
      unsigned period_us = (ctx->pwm_hz > 0) ? (1000000u / ctx->pwm_hz) : 5000u; // 200 Hz por defecto

      while (atomic_load(&ctx->running)) {
          int duty = atomic_load(&ctx->duty);    // 0..100
          if (duty <= 0) {
              DEV_Digital_Write(ctx->pin, 0);
              usleep(period_us);
              continue;
          }
          if (duty >= 100) {
              DEV_Digital_Write(ctx->pin, 1);
              usleep(period_us);
              continue;
          }
          // Tiempo ON/OFF según duty
          unsigned on_us  = (period_us * (unsigned)duty) / 100u;
          unsigned off_us = period_us - on_us;
          DEV_Digital_Write(ctx->pin, 1);
          if (on_us)  usleep(on_us);
          DEV_Digital_Write(ctx->pin, 0);
          if (off_us) usleep(off_us);
      }

      // Asegura el pin en bajo al salir
      DEV_Digital_Write(ctx->pin, 0);
      return NULL;
  }

  static void sw_pwm_start(UWORD pin, unsigned hz, int initial_duty) {
      if (pin >= (UWORD)(sizeof(sw_pwm_map)/sizeof(sw_pwm_map[0]))) return;
      if (sw_pwm_map[pin]) return; // ya iniciado

      sw_pwm_t* ctx = (sw_pwm_t*)calloc(1, sizeof(sw_pwm_t));
      if (!ctx) return;
      ctx->pin = pin;
      ctx->pwm_hz = (hz > 0) ? hz : 200; // 200 Hz por defecto
      atomic_store(&ctx->duty, initial_duty);
      atomic_store(&ctx->running, 1);

      // Asegura que el pin está como salida
      DEV_GPIO_Mode(pin, 1);
      // Lanza hilo
      if (pthread_create(&ctx->thread, NULL, sw_pwm_loop, ctx) != 0) {
          free(ctx);
          return;
      }
      sw_pwm_map[pin] = ctx;
  }

  static void sw_pwm_set_duty(UWORD pin, int duty) {
      if (pin >= (UWORD)(sizeof(sw_pwm_map)/sizeof(sw_pwm_map[0]))) return;
      sw_pwm_t* ctx = sw_pwm_map[pin];
      if (!ctx) return;
      if (duty < 0) duty = 0;
      if (duty > 100) duty = 100;
      atomic_store(&ctx->duty, duty);
  }

  static void sw_pwm_stop(UWORD pin) {
      if (pin >= (UWORD)(sizeof(sw_pwm_map)/sizeof(sw_pwm_map[0]))) return;
      sw_pwm_t* ctx = sw_pwm_map[pin];
      if (!ctx) return;
      atomic_store(&ctx->running, 0);
      pthread_join(ctx->thread, NULL);
      DEV_Digital_Write(pin, 0);
      free(ctx);
      sw_pwm_map[pin] = NULL;
  }
#endif // USE_SW_PWM

// Mantener el mismo mapa que ya tenías (pero ahora lo usamos para PWM opcional)
static UBYTE pwm_enabled_pins[128] = {0}; // aumentado a 128 por seguridad

void L298N_MotorInit(const L298NMotorPins *motor, UBYTE use_pwm)
{
    if (motor == NULL) return;

    // Configura pines como salida
    DEV_GPIO_Mode(motor->input_pin_1, 1);
    DEV_GPIO_Mode(motor->input_pin_2, 1);
    DEV_GPIO_Mode(motor->enable_pin, 1);

    // Estado inicial
    DEV_Digital_Write(motor->input_pin_1, 0);
    DEV_Digital_Write(motor->input_pin_2, 0);
    DEV_Digital_Write(motor->enable_pin, 0);

#if USE_SW_PWM
    if (use_pwm) {
        // Inicia PWM software en enable_pin a 200 Hz, duty 0%
        sw_pwm_start(motor->enable_pin, 200, 0);
        pwm_enabled_pins[motor->enable_pin] = 1;
    } else {
        pwm_enabled_pins[motor->enable_pin] = 0;
    }
#else
    (void)use_pwm; // sin PWM por defecto
    pwm_enabled_pins[motor->enable_pin] = 0;
#endif
}

void L298N_MotorSetDirection(const L298NMotorPins *motor, UBYTE forward)
{
    if (motor == NULL) return;

    if (forward) {
        DEV_Digital_Write(motor->input_pin_1, 1);
        DEV_Digital_Write(motor->input_pin_2, 0);
    } else {
        DEV_Digital_Write(motor->input_pin_1, 0);
        DEV_Digital_Write(motor->input_pin_2, 1);
    }
}

void L298N_MotorBrake(const L298NMotorPins *motor)
{
    if (motor == NULL) return;

    // Freno activo: IN1=IN2=1
    DEV_Digital_Write(motor->input_pin_1, 1);
    DEV_Digital_Write(motor->input_pin_2, 1);

#if USE_SW_PWM
    if (motor->enable_pin < 128 && pwm_enabled_pins[motor->enable_pin]) {
        sw_pwm_set_duty(motor->enable_pin, 100); // duty 100%
    } else {
        DEV_Digital_Write(motor->enable_pin, 1);
    }
#else
    DEV_Digital_Write(motor->enable_pin, 1);
#endif
}

void L298N_MotorStop(const L298NMotorPins *motor)
{
    if (motor == NULL) return;

#if USE_SW_PWM
    if (motor->enable_pin < 128 && pwm_enabled_pins[motor->enable_pin]) {
        sw_pwm_set_duty(motor->enable_pin, 0); // duty 0%
    } else {
        DEV_Digital_Write(motor->enable_pin, 0);
    }
#else
    DEV_Digital_Write(motor->enable_pin, 0);
#endif

    // Entradas en bajo
    DEV_Digital_Write(motor->input_pin_1, 0);
    DEV_Digital_Write(motor->input_pin_2, 0);
}

void L298N_MotorSetSpeed(const L298NMotorPins *motor, UBYTE duty_cycle)
{
    if (motor == NULL) return;
    if (duty_cycle > 100) duty_cycle = 100;

#if USE_SW_PWM
    if (motor->enable_pin < 128 && pwm_enabled_pins[motor->enable_pin]) {
        sw_pwm_set_duty(motor->enable_pin, (int)duty_cycle);
        return;
    }
#endif
    // Sin PWM: aproximación ON/OFF
    if (duty_cycle == 0) {
        DEV_Digital_Write(motor->enable_pin, 0);
    } else {
        DEV_Digital_Write(motor->enable_pin, 1);
    }
}
