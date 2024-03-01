#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

using std::placeholders::_1;

DynamixelWorkbench dxl_wb;

//Parametri robota
double a_ = 0.225/2;
double b_ = 0.225/2;
double r_ = 0.0254;
double x_w_r_[4] = {a_, -a_, -a_, a_};
double y_w_r_[4] = {b_, b_, -b_, -b_};

uint8_t driving_motors_ids_[4] = {1, 2, 3, 4};
uint8_t steering_motors_ids_[4] = {5, 6, 7, 8};
constexpr uint8_t kGoalPositionIndex = 0;
constexpr uint8_t kGoalVelocityIndex = 1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("fwsfwd_controller")
    {
        const char * log = nullptr;
    
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    
        RCLCPP_INFO(this->get_logger(), "fwsfwd_controller Running");

        //RCLCPP_INFO(this->get_logger(), "usb_port: %s", usb_port.c_str());
        //RCLCPP_INFO(this->get_logger(), "baud_rate: %d", baud_rate);

        if (!dxl_wb.init("/dev/ttyUSB0", 3000000, &log)) {
            RCLCPP_INFO(this->get_logger(), "Dynamixels failed to init");
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Dynamixels init succesful");
        }

        //Pinging motors
        for (uint i = 1; i < 9; ++i) {
            uint16_t model_number = 0;
            if (dxl_wb.ping(i, &model_number, &log)) {
                RCLCPP_INFO(this->get_logger(), "Dynamixel [%i] pinged", i);
            }
        }

        //Changing control modes
        for (uint i = 1; i < 5; ++i) {
            if(dxl_wb.setVelocityControlMode(i, &log)){
                RCLCPP_INFO(this->get_logger(), "Dynamixel [%i] velocity control mode", i);
            }
            else{
                RCLCPP_INFO(this->get_logger(), "Dynamixel [%i] didn't change velocity control mode", i);
            }
        }

        for (uint i = 5; i < 9; ++i) {
            if(dxl_wb.setPositionControlMode(i, &log)){
                RCLCPP_INFO(this->get_logger(), "Dynamixel [%i] position control mode", i);
            }
            else{
                RCLCPP_INFO(this->get_logger(), "Dynamixel [%i] didn't change position control mode", i);
            }
        }

        //Enabling torque
        for (uint i = 1; i < 9; ++i) {
            if (dxl_wb.torqueOn(i, &log)) {
                RCLCPP_INFO(this->get_logger(), "Dynamixel [%i] torque on", i);
            }
        }

        //Setting zero position
        
        int32_t steering_angles[4] = {2048, 2048, 2048, 2048};

        
        dxl_wb.addSyncWriteHandler(steering_motors_ids_[0], "Goal_Position", &log);
        dxl_wb.syncWrite(kGoalPositionIndex, steering_motors_ids_, 4, steering_angles, 1, &log);

        dxl_wb.addSyncWriteHandler(driving_motors_ids_[0], "Goal_Velocity", &log);

    }


    private:
        void topic_callback(const geometry_msgs::msg::Twist & msg) const
        {
            double motor_speeds[4] = {0.0, 0.0, 0.0, 0.0};
            double motor_angles[4] = {0.0, 0.0, 0.0, 0.0};
            for(int i = 0; i < 4; i++){
                double v_x = msg.linear.x - y_w_r_[i] * msg.angular.z;
                double v_y = msg.linear.y + x_w_r_[i] * msg.angular.z;
                motor_speeds[i] = sqrt(pow(v_x, 2) + pow(v_y, 2));
                motor_angles[i] = atan2(v_y, v_x);
            } 
            MinimalSubscriber::set_motor_angles(motor_angles);
            MinimalSubscriber::set_motor_speeds(motor_speeds);
        }

        void set_motor_angles(double angles[4]) const{
            const char * log = nullptr;
            int32_t angle_values[4] = {0, 0, 0, 0};
            for(int i = 0; i < 4; i++){
                angle_values[i] = dxl_wb.convertRadian2Value(steering_motors_ids_[i], angles[i]);
            }
            dxl_wb.syncWrite(kGoalPositionIndex, steering_motors_ids_, 4, angle_values, 1, &log);
        }

        void set_motor_speeds(double speeds[4]) const{
            const char * log = nullptr;
            int32_t speed_values[4] = {0, 0, 0, 0};
            for(int i = 0; i < 4; i++){
                speed_values[i] = dxl_wb.convertVelocity2Value(driving_motors_ids_[i], speeds[i] / r_);
                if(speed_values[i] > 210)
                    speed_values[i] = 210;
                else if(speed_values[i] < -210)
                    speed_values[i] = -210;
                
            }
            dxl_wb.syncWrite(kGoalVelocityIndex, driving_motors_ids_, 4, speed_values, 1, &log);
        }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}