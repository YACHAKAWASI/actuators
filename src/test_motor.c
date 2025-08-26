#include "actuator_motor/DEV_Config.h"
#include "actuator_motor/motor_driver.h"
#include "actuator_motor/pins.h"

int main() {
  if (DEV_ModuleInit() != 0) {
    return 1;
  }

  L298NMotorPins motorA = { MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_EN };
  L298N_MotorInit(&motorA, 1);

  L298N_MotorSetDirection(&motorA, 1);
  L298N_MotorSetSpeed(&motorA, 60);

  DEV_Delay_ms(2000);

  L298N_MotorBrake(&motorA);
  DEV_Delay_ms(500);

  L298N_MotorSetDirection(&motorA, 0);
  L298N_MotorSetSpeed(&motorA, 50);
  DEV_Delay_ms(2000);

  L298N_MotorStop(&motorA);
  DEV_ModuleExit();
  return 0;
}


