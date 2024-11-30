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
#define RESET_COLOR "\033[0m"

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

                        
            try {
                K_ = Eigen::Vector4d(this->get_parameter("K").as_double_array().data());
                kd_ = this->get_parameter("kd").as_double();
                ke_ = this->get_parameter("ke").as_double();
                kv_ = this->get_parameter("kv").as_double();
                lqr_transition_angle_ = this->get_parameter("lqr_transition_angle").as_double();

                
            RCLCPP_WARN(this->get_logger(), "K: %.4f, %.4f, %.4f, %.4f", K_(0), K_(1), K_(2), K_(3));
            RCLCPP_WARN(this->get_logger(), "kd: %.4f", kd_);
            RCLCPP_WARN(this->get_logger(), "ke: %.4f", ke_);
            RCLCPP_WARN(this->get_logger(), "kp: %.4f", kv_);
            RCLCPP_WARN(this->get_logger(), "lqr_transition_angle_: %.4f", lqr_transition_angle_);

            } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Required parameter not defined: " << e.what());
                throw e;
            }

    }

  private:
    void topic_callback(const sensor_msgs::msg::JointState & msg)
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
            swinger1_velocity = msg.velocity[index];

            //RCLCPP_INFO(this->get_logger(), "Slider  -> Position: %.2f, Velocity: %.2f", slider_position, slider_velocity);
        }

        else
        {
            RCLCPP_WARN(this->get_logger(), "Swinger1 joint not found in the message.");
        }

        if (swinger2_it != msg.name.end())
        {
            auto index = std::distance(msg.name.begin(), swinger2_it);
            swinger2_position = msg.position[index];
            swinger2_velocity = msg.velocity[index];

            //RCLCPP_INFO(this->get_logger(), "Swinger -> Position: %.2f, Velocity: %.2f", swinger_position, swinger_velocity);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Swinger2 joint not found in the message.");
        }

        double f;

        if((swinger1_it != msg.name.end()) || (swinger2_it != msg.name.end()))
        {
              // Define the range boundaries
              double lower_limit = PI - switching_range;
              double upper_limit = PI + switching_range;

                  // Normalize swinger_position to [0, 2*PI]
              double normalized_position = fmod(swinger2_position, 2 * PI);

                  // If the normalized position is negative, adjust it by adding 2*PI
              if (normalized_position < 0) {
                  normalized_position += 2 * PI;
              }
                  
         //RCLCPP_INFO(this->get_logger(), "normalised_position: %.4f", normalized_position);

         //Before the pendulum is in the homoclinic orbit
    if( normalized_position < lower_limit || normalized_position > upper_limit)
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
       double pendulum_length_c1 = pendulum_length/2;
       double m_dash = m1 * pendulum_length_c1 + m2 * pendulum_length;
       double g = 9.81;

       double q1 = M_PI - swinger1_position;
       double q2 = M_PI_2 - swinger2_position;

       double inertia1 = 0.0; //SOLVE THIS ONE SINCE IT IS A VERY IMPORTANT PART OF THIS PROJECT
       double inertia2 = 0.0; //SOLVE THIS ONE SINCE IT IS A VERY IMPORTANT PART OF THIS PROJECT


       double dq1 = swinger1_velocity;//CHECK FOR ACCURACY AS IN ROATATION DIRECTIOn
       double dq2 = swinger2_velocity;//CHECK FOR ACCURACY

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
    //THIS SHOULD BE ADDED TO A YAML FILE
    double kd = kd_;
    double ke = ke_;
    double kv = kv_;
     /*-------------------------------------------------------------------------------*/

    double torque = -kd * (ke * energy *dq2 + kv * (d21 * dq1 + d22 * dq2));
    f = torque;

    RCLCPP_INFO(this->get_logger(), BLUE_TEXT"F: %.4f", f);
    //RCLCPP_INFO(this->get_logger(), "Force: %.4f Numerator: %.4f Denominator: %.4f Energy: %.4f", f, numerator, denominator, E);
    }

    else 
    {
       double gains_[4] = { K_(0), K_(1), K_(2), K_(3)};

       double from_output = (gains_[2]*swinger2_position);
       double first = (gains_[0]*swinger1_position) + (gains_[1]*swinger1_velocity) +(gains_[3]*swinger2_velocity);

       double final_gain = (from_output - first);
       f = final_gain;
       RCLCPP_INFO(this->get_logger(), GREEN_TEXT"F: %.4f", f);

    }

    auto message = std_msgs::msg::Float64MultiArray();
    message.data.push_back(f);
    publisher_->publish(message);
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_; // effort controller publisher
    Eigen::Vector4d K_;
    double kd_;
    double ke_;
    double kv_;
    double lqr_transition_angle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateSubscriber>());
  rclcpp::shutdown();
  return 0;
}