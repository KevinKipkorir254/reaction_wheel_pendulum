#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <stdio.h>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#define PI 3.14159

// ANSI escape codes for colors
#define GREEN_TEXT "\033[0;32m"
#define BLUE_TEXT "\033[0;34m"
#define RED_TEXT "\033[0;31m"
#define RESET_COLOR "\033[0m"

// Additional colors
#define BLACK_TEXT "\033[0;30m"
#define YELLOW_TEXT "\033[0;33m"
#define MAGENTA_TEXT "\033[0;35m"
#define CYAN_TEXT "\033[0;36m"
#define WHITE_TEXT "\033[0;37m"

// Bright colors
#define BRIGHT_BLACK_TEXT "\033[0;90m"
#define BRIGHT_RED_TEXT "\033[0;91m"
#define BRIGHT_GREEN_TEXT "\033[0;92m"
#define BRIGHT_YELLOW_TEXT "\033[0;93m"
#define BRIGHT_BLUE_TEXT "\033[0;94m"
#define BRIGHT_MAGENTA_TEXT "\033[0;95m"
#define BRIGHT_CYAN_TEXT "\033[0;96m"
#define BRIGHT_WHITE_TEXT "\033[0;97m"

// Background colors
#define BLACK_BACKGROUND "\033[40m"
#define RED_BACKGROUND "\033[41m"
#define GREEN_BACKGROUND "\033[42m"
#define YELLOW_BACKGROUND "\033[43m"
#define BLUE_BACKGROUND "\033[44m"
#define MAGENTA_BACKGROUND "\033[45m"
#define CYAN_BACKGROUND "\033[46m"
#define WHITE_BACKGROUND "\033[47m"

// Bright background colors
#define BRIGHT_BLACK_BACKGROUND "\033[100m"
#define BRIGHT_RED_BACKGROUND "\033[101m"
#define BRIGHT_GREEN_BACKGROUND "\033[102m"
#define BRIGHT_YELLOW_BACKGROUND "\033[103m"
#define BRIGHT_BLUE_BACKGROUND "\033[104m"
#define BRIGHT_MAGENTA_BACKGROUND "\033[105m"
#define BRIGHT_CYAN_BACKGROUND "\033[106m"
#define BRIGHT_WHITE_BACKGROUND "\033[107m"

// Text styles
#define BOLD_TEXT "\033[1m"
#define UNDERLINE_TEXT "\033[4m"
#define REVERSED_TEXT "\033[7m"

using std::placeholders::_1;
using namespace std::chrono_literals;

class JointStateSubscriber : public rclcpp::Node
{
public:
    JointStateSubscriber()
        : Node("state_space_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&JointStateSubscriber::topic_callback, this, _1));
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("effort_controller/commands", 10);

        this->declare_parameter("K", rclcpp::PARAMETER_DOUBLE_ARRAY);
        this->declare_parameter("kd", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("ke", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("kv", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("lqr_transition_angle", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter("rviz_test", rclcpp::PARAMETER_BOOL);

        try
        {
            K_ = Eigen::Vector4d(this->get_parameter("K").as_double_array().data());
            kd_ = this->get_parameter("kd").as_double();
            ke_ = this->get_parameter("ke").as_double();
            kv_ = this->get_parameter("kv").as_double();
            lqr_transition_angle_ = this->get_parameter("lqr_transition_angle").as_double();
            rviz_test = this->get_parameter("rviz_test").as_bool();

            RCLCPP_WARN(this->get_logger(), "K: %.4f, %.4f, %.4f, %.4f", K_(0), K_(1), K_(2), K_(3));
            RCLCPP_WARN(this->get_logger(), "kd: %.4f", kd_);
            RCLCPP_WARN(this->get_logger(), "ke: %.4f", ke_);
            RCLCPP_WARN(this->get_logger(), "kp: %.4f", kv_);
            RCLCPP_WARN(this->get_logger(), "lqr_transition_angle_: %.4f", lqr_transition_angle_);
            if (rviz_test)
            {
                RCLCPP_WARN(this->get_logger(), "RVIZ_TEST: TRUE");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "RVIZ_TEST: FALSE");
            }
        }
        catch (const rclcpp::exceptions::ParameterUninitializedException &e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Required parameter not defined: " << e.what());
            throw e;
        }
    }

private:
    void topic_callback(const sensor_msgs::msg::JointState &msg)
    {
        // Find the indices of the "slider" and "swinger" joints
        auto swinger1_it = std::find(msg.name.begin(), msg.name.end(), "continuous_revolute_1");
        auto swinger2_it = std::find(msg.name.begin(), msg.name.end(), "continuous_revolute_2");

        double swinger1_position;
        double swinger1_velocity;

        double swinger2_position;
        double swinger2_velocity;

        double switching_range = lqr_transition_angle_;

        if (swinger1_it != msg.name.end())
        {
            auto index = std::distance(msg.name.begin(), swinger1_it);
            swinger1_position = msg.position[index];
            if (!rviz_test)
            {
                swinger1_velocity = msg.velocity[index];
            }
            else
            {
                swinger1_velocity = swinger1_position - swinger1_previous_position;
                swinger1_previous_position = swinger1_position;
            }

            // RCLCPP_INFO(this->get_logger(), "Slider  -> Position: %.2f, Velocity: %.2f", slider_position, slider_velocity);
        }

        else
        {
            RCLCPP_WARN(this->get_logger(), "Swinger1 joint not found in the message.");
        }

        if (swinger2_it != msg.name.end())
        {
            auto index = std::distance(msg.name.begin(), swinger2_it);
            swinger2_position = msg.position[index];
            if (!rviz_test)
            {
                swinger2_velocity = msg.velocity[index];
            }
            else
            {
                swinger2_velocity = swinger2_position - swinger2_previous_position;
                swinger2_previous_position = swinger2_position;
            }

            // RCLCPP_INFO(this->get_logger(), "Swinger -> Position: %.2f, Velocity: %.2f", swinger_position, swinger_velocity);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Swinger2 joint not found in the message.");
        }

        double f;

        if ((swinger1_it != msg.name.end()) || (swinger2_it != msg.name.end()))
        {

            // Normalize swinger_position to [ -PI, PI]
            double normalized_position = normalize_to_pi(swinger1_position);
            double torque;

            // USE VELOCITY TO CHECK THE DIRECTION WE ARE ENTERING FROM
            if (swinger1_velocity < 0.0) // anti-clockwise direction
            {
                // RCLCPP_INFO(this->get_logger(), "anti_clockwise");
                RCLCPP_INFO(this->get_logger(), BLUE_TEXT "CLOCKWISE -> %.4f, %.4f", convert_to_degrees(swinger1_position), convert_to_degrees(swinger1_velocity));

                double lower_limit = -PI - switching_range; //-210
                double upper_limit = -PI + switching_range; //-150

                if ((normalized_position > convert_to_rads(-150)) && (normalized_position < convert_to_rads(0.0))) // IT IS IN TH RANGE (0 TO -150) RUN THE CONTROLLER TO BRING IT TO THE HOMICLINIC ORBIT.
                {
                    torque = calculate_controller_force_output(swinger1_position, swinger2_position, swinger1_velocity, swinger2_velocity);
                    RCLCPP_INFO(this->get_logger(), YELLOW_TEXT "(0 TO -150)");
                }
                else if ((normalized_position < upper_limit) && (normalized_position > lower_limit)) // IT IS IN THE RANGE (-150 TO -210) THEN RUN THE LQR TO MAINTAIN IT TO THE HOMICLINIC ORBIT
                {
                    double error = -PI - swinger1_position;
                    torque = calculate_LQR_torque_output(swinger1_position, swinger2_position, swinger1_velocity, swinger2_velocity, error);
                    torque = -1 * torque;
                    RCLCPP_INFO(this->get_logger(), MAGENTA_TEXT "ENTER (-150 TO -210)E:-> %.4f, P: -> %.4f, V: -> %.4f, T: -> %.4f", error, convert_to_degrees(swinger1_position), convert_to_degrees(swinger1_velocity), torque);
                }
                else if ((normalized_position > convert_to_rads(150))) // IF IT TURNS VELOCITY MID LQR
                {
                    double error = PI - swinger1_position;
                    torque = calculate_LQR_torque_output(swinger1_position, swinger2_position, swinger1_velocity, swinger2_velocity, error);
                    torque = -1 * torque;
                    RCLCPP_INFO(this->get_logger(), BRIGHT_MAGENTA_TEXT "LEAVE (150 TO 210) E:-> %.4f, P: -> %.4f, V: -> %.4f, T: -> %.4f", error, convert_to_degrees(swinger1_position), convert_to_degrees(swinger1_velocity), torque);
                }
                else // HEADING BACK
                {
                    torque = calculate_controller_force_output(swinger1_position, swinger2_position, swinger1_velocity, swinger2_velocity);
                    RCLCPP_INFO(this->get_logger(), GREEN_TEXT "HEADING BACK");
                }
            }
            else if (swinger1_velocity > 0.0) // clockwise direction
            {
                // RCLCPP_INFO(this->get_logger(), "clockwise");
                RCLCPP_INFO(this->get_logger(), RED_TEXT "ANTI-CLOCKWISE -> %.4f, %.4f", convert_to_degrees(swinger1_position), convert_to_degrees(swinger1_velocity));

                double lower_limit = PI - switching_range; // 150
                double upper_limit = PI + switching_range; // 210

                if ((normalized_position < convert_to_rads(150)) && (normalized_position > convert_to_rads(0.0))) // IT IS IN TH RANGE (0 TO 150) RUN THE CONTROLLER TO BRING IT TO THE HOMICLINIC ORBIT.
                {
                    torque = calculate_controller_force_output(swinger1_position, swinger2_position, swinger1_velocity, swinger2_velocity);
                    RCLCPP_INFO(this->get_logger(), BRIGHT_YELLOW_TEXT "(0 TO 150)");
                }
                else if ((normalized_position < upper_limit) && (normalized_position > lower_limit)) // IT IS IN THE RANGE (150 TO 210) THEN RUN THE LQR TO MAINTAIN IT TO THE HOMICLINIC ORBIT
                {
                    double error = PI - swinger1_position;
                    torque = calculate_LQR_torque_output(swinger1_position, swinger2_position, swinger1_velocity, swinger2_velocity, error);
                    torque = -1 * torque;
                    RCLCPP_INFO(this->get_logger(), BRIGHT_MAGENTA_TEXT "ENTER (150 TO 210) E:-> %.4f, P: -> %.4f, V: -> %.4f, T: -> %.4f", error, convert_to_degrees(swinger1_position), convert_to_degrees(swinger1_velocity), torque);
                }
                else if ((normalized_position < convert_to_rads(-150))) // IF IT TURNS VELOCITY MID LQR
                {
                    double error = (PI - swinger1_position) - (PI * 2);
                    torque = calculate_LQR_torque_output(swinger1_position, swinger2_position, swinger1_velocity, swinger2_velocity, error);
                    torque = -1 * torque;
                    RCLCPP_INFO(this->get_logger(), BRIGHT_MAGENTA_TEXT "LEAVE (150 TO 210) E:-> %.4f, P: -> %.4f, V: -> %.4f, T: -> %.4f", error, convert_to_degrees(swinger1_position), convert_to_degrees(swinger1_velocity), torque);
                }
                else // HEADING BACK
                {
                    torque = calculate_controller_force_output(swinger1_position, swinger2_position, swinger1_velocity, swinger2_velocity);
                    RCLCPP_INFO(this->get_logger(), BRIGHT_GREEN_TEXT "HEADING BACK");
                }
            }

           //torque = -1 * torque;
            auto message = std_msgs::msg::Float64MultiArray();
            message.data.push_back( torque);
            publisher_->publish(message);
        }
    }

    /**
     * @brief Normalizes an angle to the range [-π, π].
     *
     * This function takes an angle in radians and normalizes it to the range [-π, π].
     * The normalization is performed by wrapping the angle to the range [-2π, 2π],
     * and then adjusting it to the range [-π, π].
     *
     * @param angle The angle in radians to be normalized.
     * @return The normalized angle in the range [-π, π].
     */
    double normalize_to_pi(double angle)
    {
        double normalized_position = fmod(angle, 2 * M_PI); // Wrap to [-2π, 2π]
        if (normalized_position > M_PI)
        {
            normalized_position -= 2 * M_PI; // Wrap to [-π, π]
        }
        else if (normalized_position < -M_PI)
        {
            normalized_position += 2 * M_PI; // Wrap to [-π, π]
        }
        return normalized_position;
    }

    /**
     * @brief Converts an angle from radians to degrees.
     *
     * This function takes an angle in radians and converts it to degrees.
     * The conversion is performed using the formula: degrees = radians * (180 / PI).
     *
     * @param rads The angle in radians to be converted.
     * @return The angle in degrees.
     */
    double convert_to_degrees(double rads)
    {
        return (rads * 180 / M_PI);
    }

    /**
     * @brief Converts an angle from degrees to radians.
     *
     * This function takes an angle in degrees and converts it to radians.
     * The conversion is performed using the formula: radians = degrees * (PI / 180).
     *
     * @param degrees The angle in degrees to be converted.
     * @return The angle in radians.
     */
    double convert_to_rads(double degrees)
    {
        return (degrees * M_PI / 180);
    }

    /**
     * @brief Calculates the controller force output for the reaction wheel pendulum system.
     *
     * This function calculates the force output required to maintain the pendulum in a stable
     * homoclinic orbit. The force output is determined using a combination of a proportional,
     * derivative, and energy-based controller. The controller parameters are obtained from
     * the provided parameters.
     *
     * @param swinger1_position The current position of the first swinger joint.
     * @param swinger2_position The current position of the second swinger joint.
     * @param swinger1_velocity The current velocity of the first swinger joint.
     * @param swinger2_velocity The current velocity of the second swinger joint.
     *
     * @return The calculated force output required to maintain the pendulum in a stable homoclinic orbit.
     */
    double calculate_controller_force_output(double swinger1_position, double swinger2_position, double swinger1_velocity, double swinger2_velocity)
    {
        /*
            m1 Mass of the pendulum
            m2 Mass of the wheel
            II Length of the pendulum
            lcl Distance to the center of mass of the pendulum
            h Moment of inertia of the pendulum
            h Moment of inertia of the wheel
            q1 Angle that the pendulum makes with the vertical
            oz Angle of the wheel
            T Motor torque input applied on the disk

            m_dash = m1lcl +m2l1,
        */

        double m1 = 0.78268947954770595743;
        double m2 = 2.0890783967314723313;
        double pendulum_length = 0.4;
        double pendulum_length_c1 = pendulum_length / 2;
        double m_dash = m1 * pendulum_length_c1 + m2 * pendulum_length;
        double g = 9.81;

        double q1 = M_PI - swinger1_position;
        double q2 = M_PI_2 - swinger2_position;

        double inertia1 = 0; //(m1 * pendulum_length_c1 * pendulum_length_c1)/12; //SOLVE THIS ONE SINCE IT IS A VERY IMPORTANT PART OF THIS PROJECT
        double inertia2 = 0; // 0.5*(m2 * 0.1); //SOLVE THIS ONE SINCE IT IS A VERY IMPORTANT PART OF THIS PROJECT

        double dq1 = swinger1_velocity; // CHECK FOR ACCURACY AS IN ROATATION DIRECTIOn
        double dq2 = swinger2_velocity; // CHECK FOR ACCURACY

        double d11 = m1 * pendulum_length_c1 * pendulum_length_c1 + m2 * pendulum_length * pendulum_length + inertia1 + inertia2;
        double d12 = inertia2;
        double d21 = inertia2;
        double d22 = inertia2;

        Eigen::Matrix2d Dq;
        Dq << d11, d12, d21, d22;

        Eigen::Vector2d Q_;
        Q_ << dq1, dq2;

        double energy = 0.5 * Q_.transpose() * Dq * Q_ + m_dash * g * (cos(q1) - 1);

        // Control parameters
        /*--------------------------------------------------------------------------------*/
        // THIS SHOULD BE ADDED TO A YAML FILE
        double kd = kd_;
        double ke = ke_;
        double kv = kv_;
        /*-------------------------------------------------------------------------------*/

        double torque = -kd * (ke * energy * dq2 + kv * (d21 * dq1 + d22 * dq2));
        double f = torque;
        return f;
    }

    /**
     * @brief Calculates the LQR torque output for the reaction wheel pendulum system.
     *
     * This function calculates the torque output required to maintain the pendulum in a stable
     * homoclinic orbit using a Linear Quadratic Regulator (LQR) controller. The LQR controller
     * parameters are obtained from the provided parameters.
     *
     * @param swinger1_position The current position of the first swinger joint.
     * @param swinger2_position The current position of the second swinger joint.
     * @param swinger1_velocity The current velocity of the first swinger joint.
     * @param swinger2_velocity The current velocity of the second swinger joint.
     * @param error The current error value used in the LQR controller.
     *
     * @return The calculated torque output required to maintain the pendulum in a stable homoclinic orbit.
     */
    double calculate_LQR_torque_output(double swinger1_position, double swinger2_position, double swinger1_velocity, double swinger2_velocity, double error)
    {
        double gains_[4] = {K_(0), K_(1), K_(2), K_(3)};

        double from_output = (gains_[0] * error);
        double first = (gains_[2] * swinger2_position) + (gains_[1] * swinger1_velocity) + (gains_[3] * swinger2_velocity);

        double final_gain = (from_output - first);
        double f = final_gain;
        return f;
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_; // effort controller publisher
    Eigen::Vector4d K_;
    double kd_;
    double ke_;
    double kv_;
    double lqr_transition_angle_;
    double error;
    bool rviz_test = true;
    double swinger1_previous_position = 0.0;
    double swinger2_previous_position = 0.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateSubscriber>());
    rclcpp::shutdown();
    return 0;
}