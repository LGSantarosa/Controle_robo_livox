// This file is based on work by Franz Pucher, Diffbot, (2020), GitHub repository, https://github.com/fjp/diffbot
#pragma once

#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>   // logger macros + rclcpp::Clock
#include <rclcpp/clock.hpp>
#include <control_toolbox/pid.hpp>

class PID : public control_toolbox::Pid
{
public:
  /**
   * @brief Construct a new PID object
   * @param p The proportional gain.
   * @param i The integral gain.
   * @param d The derivative gain.
   * @param i_max The max integral windup.
   * @param i_min The min integral windup.
   * @param antiwindup Enable or disable antiwindup.
   * @param out_max The max computed output.
   * @param out_min The min computed output.
   */
  PID(double p = 0.0, double i = 0.0, double d = 0.0,
      double i_max = 0.0, double i_min = 0.0, bool antiwindup = false,
      double out_max = 0.0, double out_min = 0.0);

  /**
   * @brief Initialize the controller
   * @param f feed-forward gain
   * @param p kP
   * @param i kI
   * @param d kD
   * @param i_max integral clamp max
   * @param i_min integral clamp min
   * @param antiwindup enable antiwindup
   * @param out_max output upper limit
   * @param out_min output lower limit
   */
  void init(double f, double p, double i, double d,
            double i_max, double i_min, bool antiwindup,
            double out_max, double out_min);

  /**
   * @brief Compute PID output using measured value, setpoint and dt
   */
  double operator()(const double & measured_value,
                    const double & setpoint,
                    const rclcpp::Duration & dt);

  /**
   * @brief Get parameters
   */
  void getParameters(double & f, double & p, double & i, double & d,
                     double & i_max, double & i_min);
  void getParameters(double & f, double & p, double & i, double & d,
                     double & i_max, double & i_min, bool & antiwindup);

  /**
   * @brief Set parameters
   */
  void setParameters(double f, double p, double i, double d,
                     double i_max, double i_min, bool antiwindup = false);

  /**
   * @brief Set output limits
   */
  void setOutputLimits(double out_min, double out_max);

  /**
   * @brief Clamp value to [lower, upper]
   */
  double clamp(const double & value, const double & lower_limit, const double & upper_limit);

  inline double getError() const { return error_; }

private:
  double f_{0.0};
  double error_{0.0};
  double out_min_{0.0};
  double out_max_{0.0};
};
