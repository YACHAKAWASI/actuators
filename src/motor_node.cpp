#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int16.hpp>

extern "C" {
  #include "device_pack/DEV_Config.h"
  #include "device_pack/motor_driver.h"
  #include "device_pack/pins.h"
}

class MotorNode : public rclcpp::Node {
public:
  MotorNode()
  : Node("motor_node_one"),
    motorA_{MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_EN}
  {
    // Parámetros básicos
    declare_parameter("max_pwm", 255);
    declare_parameter("vmax", 1.0);     // m/s equivalente al PWM máximo
    get_parameter("max_pwm", max_pwm_);
    get_parameter("vmax", vmax_);

    if (DEV_ModuleInit() != 0) {
      RCLCPP_FATAL(get_logger(), "DEV_ModuleInit() falló");
      throw std::runtime_error("HW init failed");
    }
    L298N_MotorInit(&motorA_, 1);

    // Control por /cmd_vel (usa solo linear.x)
    sub_twist_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) { onTwist(*msg); });

    // (Opcional) Control directo por PWM con signo [-max_pwm, max_pwm]
    sub_pwm_ = create_subscription<std_msgs::msg::Int16>(
      "motor_pwm", rclcpp::QoS(10),
      [this](std_msgs::msg::Int16::SharedPtr m) { setPwmSigned(m->data); });
  }

  ~MotorNode() override {
    L298N_MotorStop(&motorA_);
    DEV_ModuleExit();
  }

private:
void onTwist(const geometry_msgs::msg::Twist & t) {
    const double v = t.linear.x;
  
    // 1) Caso cero o casi-cero: asegurar freno duro y PWM=0
    const double EPS = 1e-4;
    if (std::abs(v) <= EPS) {
      // Aplica un orden seguro: primero quita PWM, luego frena
      L298N_MotorSetSpeed(&motorA_, 0);
      L298N_MotorBrake(&motorA_);        // evita "coast" y detiene sí o sí
      return;
    }
  
    // 2) Para no-cero: ajusta dirección y PWM
    const int dir = (v > 0.0) ? 1 : 0;   // (nota: 0 exacto ya fue manejado arriba)
    int pwm = mapToPwm(std::abs(v));
  
    // (opcional) deadband para vencer fricción estática
    const int MIN_START = 40;            // calibra 20–60 según tu motor
    if (pwm > 0 && pwm < MIN_START) pwm = MIN_START;
  
    // Orden seguro: primero dirección, después PWM
    L298N_MotorSetDirection(&motorA_, dir);
    L298N_MotorSetSpeed(&motorA_, pwm);
  }
  

  void setPwmSigned(int pwm_signed) {
    // pwm_signed ∈ [-max_pwm_, max_pwm_]
    const int dir = (pwm_signed >= 0) ? 1 : 0;
    int pwm = std::abs(pwm_signed);
    if (pwm > max_pwm_) pwm = max_pwm_;
    L298N_MotorSetDirection(&motorA_, dir);
    L298N_MotorSetSpeed(&motorA_, pwm);
  }

  int mapToPwm(double v_abs) const {
    // v_abs en m/s → PWM (lineal; calibra vmax y max_pwm a tu robot)
    if (v_abs > vmax_) v_abs = vmax_;
    return static_cast<int>((v_abs / vmax_) * max_pwm_);
  }

  L298NMotorPins motorA_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_pwm_;
  int max_pwm_{255};
  double vmax_{1.0};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorNode>());
  rclcpp::shutdown();
  return 0;
}
