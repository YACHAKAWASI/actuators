#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int16.hpp>
#include <cmath>

extern "C" {
  #include "actuator_motor/DEV_Config.h"
  #include "actuator_motor/motor_driver.h"
  #include "actuator_motor/pins.h"
}

class MotorNode : public rclcpp::Node {
public:
  MotorNode()
  : Node("motor_node_one"),
    motorA_{MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_EN}
  {
    // Parámetros
    declare_parameter<int>("max_pwm", 100);          // 0..100 (driver)
    declare_parameter<double>("vmax", 1.0);          // m/s al 100%
    declare_parameter<bool>("use_sw_pwm", true);     // true si compilaste con USE_SW_PWM=1

    get_parameter("max_pwm", max_pwm_);
    get_parameter("vmax", vmax_);
    get_parameter("use_sw_pwm", use_sw_pwm_);

    if (max_pwm_ < 1) max_pwm_ = 1;
    if (max_pwm_ > 100) max_pwm_ = 100;

    if (DEV_ModuleInit() != 0) {
      RCLCPP_FATAL(get_logger(), "DEV_ModuleInit() falló");
      throw std::runtime_error("HW init failed");
    }

    // Nota: si no compilaste con USE_SW_PWM=1, pasar 1 no activará PWM (el driver lo ignora),
    // pero por coherencia puedes pasar 0 cuando no lo uses.
    L298N_MotorInit(&motorA_, use_sw_pwm_ ? 1 : 0);

    // Suscripciones
    sub_twist_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) { onTwist(*msg); });

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
    const double EPS = 1e-4;

    if (std::abs(v) <= EPS) {
      L298N_MotorSetSpeed(&motorA_, 0);
      L298N_MotorBrake(&motorA_);   // freno activo
      return;
    }

    const int dir = (v > 0.0) ? 1 : 0;
    int pwm = mapToPwm(std::abs(v));

    // Deadband (0..100)
    const int MIN_START = 30;       // calibra 20–40
    if (pwm > 0 && pwm < MIN_START) pwm = MIN_START;

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
    if (v_abs < 0) v_abs = 0;
    if (v_abs > vmax_) v_abs = vmax_;
    // escala lineal a [0..max_pwm_] y el driver lo interpreta como 0..100
    return static_cast<int>((v_abs / vmax_) * max_pwm_);
  }

  L298NMotorPins motorA_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_pwm_;
  int max_pwm_{100};         // ahora 0..100
  double vmax_{1.0};
  bool use_sw_pwm_{true};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorNode>());
  rclcpp::shutdown();
  return 0;
}
