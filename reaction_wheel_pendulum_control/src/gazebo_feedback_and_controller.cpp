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



        if (swinger1_it != msg.name.end())
        {
            auto index = std::distance(msg.name.begin(), swinger1_it);
            swinger1_position = msg.position[index];
            swinger1_velocity = msg.velocity[index];

            //RCLCPP_INFO(this->get_logger(), "Slider  -> Position: %.2f, Velocity: %.2f", slider_position, slider_velocity);
        }

        else
        {
            RCLCPP_WARN(this->get_logger(), "Slider joint not found in the message.");
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
            RCLCPP_WARN(this->get_logger(), "Swinger joint not found in the message.");
        }

        double f;

        if((swinger1_it != msg.name.end()) || (swinger2_it != msg.name.end()))
        {
              // Define the range boundaries
              double lower_limit = PI - 0.523599;
              double upper_limit = PI + 0.523599;

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
    double kd = 1.0;
    double ke = 1.0;
    double kv = 1.0;
     /*-------------------------------------------------------------------------------*/

    double torque = -kd * (ke * energy *dq2 + kv * (d21 * dq1 + d22 * dq2));
    f = torque;

    RCLCPP_INFO(this->get_logger(), BLUE_TEXT"F: %.4f", f);
    //RCLCPP_INFO(this->get_logger(), "Force: %.4f Numerator: %.4f Denominator: %.4f Energy: %.4f", f, numerator, denominator, E);
    }

    else 
    {
       double gains_[4] = { -141.4214, -77.6558, -238.7684, -36.5906};

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
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateSubscriber>());
  rclcpp::shutdown();
  return 0;
}