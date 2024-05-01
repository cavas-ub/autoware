#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"

#include "dbw_ford_msgs/msg/gear_cmd.hpp"
#include "dbw_ford_msgs/msg/gear_report.hpp"
#include "dbw_ford_msgs/msg/steering_cmd.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "dbw_ford_msgs/msg/steering_report.hpp"
#include <dataspeed_ulc_msgs/msg/ulc_cmd.hpp>

using namespace std;

class Lincoln_MKZ_Bridge : public rclcpp::Node
{
public:
    Lincoln_MKZ_Bridge() : Node("Lincoln_MKZ_Bridge")
    {   
        rclcpp::Parameter wheel_base_param, steering_gain_param;

        this->declare_parameter<float>("Lincoln_Wheel_Base");
        this->declare_parameter<float>("Lincoln_Steering");

        wheel_base_param = this->get_parameter("Lincoln_Wheel_Base");
        if (wheel_base_param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            wheel_base = wheel_base_param.as_double();
        } else {
            cout<<"Invalid type given! Check vehicle_params.yaml";
        }

        steering_gain_param = this->get_parameter("Lincoln_Steering");
        if (steering_gain_param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            steering_gain = steering_gain_param.as_double();
        } else {
            cout<<"Invalid type given! Check vehicle_params.yaml";
        }

        //To DBW Node
        gear_publisher_ = this->create_publisher<dbw_ford_msgs::msg::GearCmd>("/vehicle/gear_cmd", 10);
        steering_publisher_ = this->create_publisher<dbw_ford_msgs::msg::SteeringCmd>("/vehicle/steering_cmd", 10);
        ulc_publisher = this->create_publisher<dataspeed_ulc_msgs::msg::UlcCmd>("/vehicle/ulc_cmd", 10);

        //To Autoware
        gear_report_publisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", 10);
        steering_report_publisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 10);
        vehicle_speed_publisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 10);

        //From DBW Node
        gear_report_subscription_ = this->create_subscription<dbw_ford_msgs::msg::GearReport>(
            "/vehicle/gear_report", 10,
            std::bind(&Lincoln_MKZ_Bridge::gearReportCallback, this, std::placeholders::_1));
        steering_report_subscription_ = this->create_subscription<dbw_ford_msgs::msg::SteeringReport>(
            "/vehicle/steering_report", 10,
            std::bind(&Lincoln_MKZ_Bridge::steeringReportCallback, this, std::placeholders::_1));
        vehicle_speed_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/vehicle/twist", 10,
            std::bind(&Lincoln_MKZ_Bridge::vehicleTwistCallback, this, std::placeholders::_1));

        //From Autoware    
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

    // Lincoln variables
    float steering_gain;
    float curr_steer;
    float wheel_base;

   void gearCallback(const autoware_auto_vehicle_msgs::msg::GearCommand::SharedPtr msg)
   {
       dbw_ford_msgs::msg::GearCmd ford_msg;
       ford_msg.cmd.gear = translate_gear(msg->command);
       gear_publisher_->publish(ford_msg);
   }

   void controlCallback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
    {
        // Throttle & Brake
        dataspeed_ulc_msgs::msg::UlcCmd ulc_publisher_;
        ctrl_time = this->now();
        ulc_publisher_.header.stamp = get_clock()->now();
        ulc_publisher_.pedals_mode = dataspeed_ulc_msgs::msg::UlcCmd::SPEED_MODE;
        ulc_publisher_.linear_velocity = msg->longitudinal.speed;

        ulc_publisher_.clear = false;
        ulc_publisher_.enable_pedals = true;
        ulc_publisher_.linear_accel = 0;
        ulc_publisher_.linear_decel = 0;
        ulc_publisher_.angular_accel = 0;
        ulc_publisher_.lateral_accel = 0;
        ulc_publisher_.jerk_limit_throttle=0;
        ulc_publisher_.jerk_limit_brake=0;

        ulc_publisher->publish(ulc_publisher_);

        // Steering
        dbw_ford_msgs::msg::SteeringCmd steering_msg;
        if (dummy_steering == 1.0){
            steering_msg.steering_wheel_angle_cmd = 0.0;
            dummy_steering = 2.0;
        }          
        // Mapping autoware steering rate (-1.0 to 1.0) into DBW steering range (-9.6 to 9.6 radians)
        else{
            // cout<<"Steering Value:"<<steering_gain*msg->lateral.steering_tire_angle<<endl;
            steering_msg.steering_wheel_angle_cmd = steering_gain*msg->lateral.steering_tire_angle;
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
        curr_steer = msg->steering_wheel_angle;
        aw_steering_report.steering_tire_angle = curr_steer / steering_gain;
        steering_report_publisher_->publish(aw_steering_report);
    }

    void vehicleTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        float longitudinal_velocity = msg->twist.linear.x;
        float lateral_velocity = msg->twist.linear.y;
        cout << "Vehicle Speed: " << longitudinal_velocity * 3.6f << " km/h" << endl;

        autoware_auto_vehicle_msgs::msg::VelocityReport velocity_msg;
        velocity_msg.header.stamp = this->now();
        velocity_msg.longitudinal_velocity = longitudinal_velocity;
        velocity_msg.lateral_velocity = lateral_velocity;
        velocity_msg.heading_rate = longitudinal_velocity * std::tan(curr_steer) / wheel_base;


        vehicle_speed_publisher_->publish(velocity_msg);
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
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vehicle_speed_subscription_;


    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_report_publisher_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_report_publisher_;
    rclcpp::Publisher<dbw_ford_msgs::msg::GearCmd>::SharedPtr gear_publisher_;
    rclcpp::Publisher<dbw_ford_msgs::msg::SteeringCmd>::SharedPtr steering_publisher_;
    rclcpp::Publisher<dataspeed_ulc_msgs::msg::UlcCmd>::SharedPtr ulc_publisher;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_speed_publisher_;
    rclcpp::Time ctrl_time;


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Lincoln_MKZ_Bridge>();
    rclcpp::spin(node);
    // rclcpp::shutdown();
    return 0;
}