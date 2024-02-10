// Author: DeeKay Goswami

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"

#include "dbw_ford_msgs/msg/gear_cmd.hpp"
#include "dbw_ford_msgs/msg/brake_cmd.hpp"
#include "dbw_ford_msgs/msg/throttle_cmd.hpp"
#include "dbw_ford_msgs/msg/steering_cmd.hpp"
#include "dbw_ford_msgs/msg/gear_report.hpp"
#include "dbw_ford_msgs/msg/steering_report.hpp"
#include "sensor_msgs/msg/joy.hpp"
using namespace std;

class Lincoln_MKZ_Bridge : public rclcpp::Node
{
public:
    Lincoln_MKZ_Bridge() : Node("Lincoln_MKZ_Bridge")
    {   
        //Output to DBW Node
        gear_publisher_ = this->create_publisher<dbw_ford_msgs::msg::GearCmd>("/vehicle/gear_cmd", 10);
        throttle_publisher_ = this->create_publisher<dbw_ford_msgs::msg::ThrottleCmd>("/vehicle/throttle_cmd", 10);
        brake_publisher_ = this->create_publisher<dbw_ford_msgs::msg::BrakeCmd>("/vehicle/brake_cmd", 10);
        steering_publisher_ = this->create_publisher<dbw_ford_msgs::msg::SteeringCmd>("/vehicle/steering_cmd", 10);

        //Output to Autoware
        gear_report_publisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", 10);
        steering_report_publisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 10);

        //Input from DBW Node
        gear_report_subscription_ = this->create_subscription<dbw_ford_msgs::msg::GearReport>(
            "/vehicle/gear_report", 10,
            std::bind(&Lincoln_MKZ_Bridge::gearReportCallback, this, std::placeholders::_1));
        steering_report_subscription_ = this->create_subscription<dbw_ford_msgs::msg::SteeringReport>(
            "/vehicle/steering_report", 10,
            std::bind(&Lincoln_MKZ_Bridge::steeringReportCallback, this, std::placeholders::_1));

        //Input from Autoware    
        gear_subscription_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
            "/control/command/gear_cmd", 10,
            std::bind(&Lincoln_MKZ_Bridge::gearCallback, this, std::placeholders::_1));
        control_subscription_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
            "/control/command/control_cmd", 10,
            std::bind(&Lincoln_MKZ_Bridge::controlCallback, this, std::placeholders::_1));

        // Joystick subscription
        sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&Lincoln_MKZ_Bridge::recvJoy, this, std::placeholders::_1));
    }

private:

   // Joystick variables
   sensor_msgs::msg::Joy joy_;
   const size_t AXIS_THROTTLE = 1;
   const size_t AXIS_BRAKE = 2;
   const size_t AXIS_STEER = 3;
   bool joy_throttle_valid = false;
   bool joy_brake_valid = false;
   float dummy_steering = 1.0;

   void gearCallback(const autoware_auto_vehicle_msgs::msg::GearCommand::SharedPtr msg)
   {
       dbw_ford_msgs::msg::GearCmd ford_msg;
       ford_msg.cmd.gear = translate_gear(msg->command);
       gear_publisher_->publish(ford_msg);
   }

   void controlCallback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
    {
        // Throttle
        float throttle_min = 0.15;
        float throttle_max = 0.80;

        // vehicle's maximum speed
        float max_speed = 34; /* 122 km/h */
        float desired_speed = msg->longitudinal.speed;
        float normalized_speed = desired_speed / max_speed;

        // Normalized speed to the range [throttle_min, throttle_max] of DBW System
        float throttle_cmd = throttle_min + normalized_speed * (throttle_max - throttle_min);

        // Adjust throttle command based on joystick input (if available)
        if (joy_throttle_valid) {
            float joy_modifier = 0.5 - 0.5 * joy_.axes[AXIS_THROTTLE];
            throttle_cmd += joy_modifier;
        }

        // This ensures the throttle command is within the valid range [throttle_min, throttle_max]
        throttle_cmd = std::clamp(throttle_cmd, throttle_min, throttle_max);

        dbw_ford_msgs::msg::ThrottleCmd throttle_msg;
        throttle_msg.pedal_cmd = throttle_cmd;
        throttle_msg.pedal_cmd_type = dbw_ford_msgs::msg::ThrottleCmd::CMD_PEDAL;
        throttle_msg.enable = true;
        throttle_publisher_->publish(throttle_msg);

        // Brake
        float brake_gain_;
        brake_gain_ = 1.0;
        float brake_value = msg->longitudinal.acceleration < 0 ? -msg->longitudinal.acceleration : 0;
        if(joy_brake_valid) {
            brake_value = 0.5 - 0.5 * joy_.axes[AXIS_BRAKE];
        }
        if(brake_value > 0) {
            dbw_ford_msgs::msg::BrakeCmd brake_msg;
            brake_msg.pedal_cmd = brake_value;
            brake_msg.pedal_cmd_type = dbw_ford_msgs::msg::BrakeCmd::CMD_PERCENT;
            brake_msg.enable = true;
            // brake_msg.pedal_cmd = brake_value * brake_gain_;
            brake_publisher_->publish(brake_msg);
        }

        // Steering
        dbw_ford_msgs::msg::SteeringCmd steering_msg;
        if (dummy_steering == 1.0){
            steering_msg.steering_wheel_angle_cmd = 0.0;
            dummy_steering = 2.0;
        }          
        // Mapping autoware steering rate (-1.0 to 1.0) into DBW steering range (-9.6 to 9.6 radians)
        else{
            cout<<"Steering Value:"<<5.5*msg->lateral.steering_tire_angle<<endl;
            steering_msg.steering_wheel_angle_cmd = 5.5*msg->lateral.steering_tire_angle;
        }

        if(joy_.axes.size() > AXIS_STEER) {
            // Adjust the steering angle based on joystick input
            steering_msg.steering_wheel_angle_cmd += joy_.axes[AXIS_STEER];
        }
        
        steering_msg.steering_wheel_angle_velocity = 0.0;
        steering_msg.cmd_type = dbw_ford_msgs::msg::SteeringCmd::CMD_ANGLE;
        steering_msg.enable = true;
        
        steering_publisher_->publish(steering_msg);
    }
   
    // float mapSteering(float autoware_steering_value)
    // {
        // Autoware steering range: [-1, 1]
        // DBW steering range: [-9.6, 9.6] radians
       
        //float autoware_steering_min = -1.0;
        //float autoware_steering_max = 1.0;
        // float dbw_steering_min = -9.6;
        // float dbw_steering_max = 9.6;

        // Map the autoware_steering_value to the DBW steering range
        // return ((autoware_steering_value + 1) / 2) * (dbw_steering_max - dbw_steering_min) + dbw_steering_min;
    // }

   uint8_t translate_gear(uint8_t autoware_gear)
   {
       using FordGear = dbw_ford_msgs::msg::Gear;

       switch(autoware_gear)
       {
            case autoware_auto_vehicle_msgs::msg::GearCommand::NEUTRAL: return FordGear::NEUTRAL;
            case autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE:
            case autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE_2:
               return FordGear::DRIVE;
            case autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE:
            case autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE_2:
               return FordGear::REVERSE;
            case autoware_auto_vehicle_msgs::msg::GearCommand::PARK: return FordGear::PARK;
            case autoware_auto_vehicle_msgs::msg::GearCommand::LOW:
            case autoware_auto_vehicle_msgs::msg::GearCommand::LOW_2:
               return FordGear::LOW;
            default: return FordGear::NONE; 
       }
   }

    void gearReportCallback(const dbw_ford_msgs::msg::GearReport::SharedPtr msg)
    {
        autoware_auto_vehicle_msgs::msg::GearReport aw_report;
        aw_report.stamp = msg->header.stamp;
        aw_report.report = translateFordGearToAutowareGear(msg->state.gear);
        gear_report_publisher_->publish(aw_report);
    }

    uint8_t translateFordGearToAutowareGear(uint8_t ford_gear)
    {
        using AutowareGear = autoware_auto_vehicle_msgs::msg::GearReport;

        switch(ford_gear)
        {
            case dbw_ford_msgs::msg::Gear::PARK: return AutowareGear::PARK;
            case dbw_ford_msgs::msg::Gear::REVERSE: return AutowareGear::REVERSE;
            case dbw_ford_msgs::msg::Gear::NEUTRAL: return AutowareGear::NEUTRAL;
            case dbw_ford_msgs::msg::Gear::DRIVE: return AutowareGear::DRIVE;
            case dbw_ford_msgs::msg::Gear::LOW: return AutowareGear::LOW;
            default: return AutowareGear::NONE; 
        }
    }

    void steeringReportCallback(const dbw_ford_msgs::msg::SteeringReport::SharedPtr msg)
    {
        autoware_auto_vehicle_msgs::msg::SteeringReport aw_steering_report;
        aw_steering_report.stamp = msg->header.stamp;
        aw_steering_report.steering_tire_angle = (msg->steering_wheel_angle)/4.8;
        // steering_report_publisher_->publish(aw_steering_report);
    }

    // Joystick callback
    void recvJoy(const sensor_msgs::msg::Joy::ConstSharedPtr msg) 
    {
        // Check for valid axes
        if (msg->axes.size() <= AXIS_STEER) return;

        // Handle joystick values
        joy_throttle_valid = msg->axes[AXIS_THROTTLE] != 0.0;
        joy_brake_valid = msg->axes[AXIS_BRAKE] != 0.0;

        // Save the received message
        joy_ = *msg;
    }

    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_subscription_;
    rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr control_subscription_;
    rclcpp::Subscription<dbw_ford_msgs::msg::GearReport>::SharedPtr gear_report_subscription_;
    rclcpp::Subscription<dbw_ford_msgs::msg::SteeringReport>::SharedPtr steering_report_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;

    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_report_publisher_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_report_publisher_;
    rclcpp::Publisher<dbw_ford_msgs::msg::GearCmd>::SharedPtr gear_publisher_;
    rclcpp::Publisher<dbw_ford_msgs::msg::ThrottleCmd>::SharedPtr throttle_publisher_;
    rclcpp::Publisher<dbw_ford_msgs::msg::BrakeCmd>::SharedPtr brake_publisher_;
    rclcpp::Publisher<dbw_ford_msgs::msg::SteeringCmd>::SharedPtr steering_publisher_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Lincoln_MKZ_Bridge>();
    rclcpp::spin(node);
    // rclcpp::shutdown();
    return 0;
}
