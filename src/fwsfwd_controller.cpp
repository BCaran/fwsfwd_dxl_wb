#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#define FL_WHEEL 1
#define BL_WHEEL 2
#define BR_WHEEL 3
#define FR_WHEEL 4

#define FL_STEERING 5
#define BL_STEERING 6
#define BR_STEERING 7
#define FR_STEERING 8

#define PULS_PER_RAD 651.8986469



using std::placeholders::_1;
using namespace std::chrono_literals;

DynamixelWorkbench dxl_wb;

//Parametri robota
double a_ = 0.225/2;
double b_ = 0.225/2;
double r_ = 0.0254;
double D_ = sqrt((a_ * a_) + (b_ * b_));
double Rx_ = a_ / D_;
double Ry_ = b_ / D_;

//double x_w_r_[4] = {a_, -a_, -a_, a_};
//double y_w_r_[4] = {b_, b_, -b_, -b_};

//Parametri motora
const double rad_to_value = 651.8986469;
const int32_t angle_offsets[4] = {2048, 2048, 2048, 2048};

uint8_t driving_motors_ids_[4] = {1, 2, 3, 4};
uint8_t steering_motors_ids_[4] = {5, 6, 7, 8};
uint8_t motors_ids_[8] = {1, 2, 3, 4, 5, 6, 7, 8};
constexpr uint8_t kGoalPositionIndex = 0;
constexpr uint8_t kGoalVelocityIndex = 1;
constexpr uint8_t kPresentPositionVelocityCurrentIndex = 0;

class FWSFWDController : public rclcpp::Node
{
  public:
    FWSFWDController()
    : Node("fwsfwd_controller")
    {
        const char * log = nullptr;
    
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&FWSFWDController::cmd_vel_callback, this, _1));

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        timer_ = this->create_wall_timer(2ms, std::bind(&FWSFWDController::joint_state_callback, this));
    
        RCLCPP_INFO(this->get_logger(), "fwsfwd_controller Running");

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
        dxl_wb.addSyncWriteHandler(driving_motors_ids_[0], "Goal_Velocity", &log);

        dxl_wb.syncWrite(kGoalPositionIndex, steering_motors_ids_, 4, steering_angles, 1, &log);
        dxl_wb.addSyncReadHandler(126, 12, &log);
    }


    private:
        void cmd_vel_callback(const geometry_msgs::msg::Twist & msg) const
        {
            bool result = false;
            const char* log = NULL;

            uint8_t velocity_id_array[4] = {FL_WHEEL, FR_WHEEL, BL_WHEEL, BR_WHEEL}; //ID's for each motor that produces speed of robot
            uint8_t position_id_array[4] = {FL_STEERING, FR_STEERING, BL_STEERING, BR_STEERING}; //ID's for each motor that rotates speed motor
            int32_t dynamixel_velocity[4]; //array for motor speed values
            int32_t dynamixel_position[4]; //array for motor angle values
            double velocity_array_m_s[4] = {0.0, 0.0, 0.0, 0.0}; //translational speed each motor need's to produce
            double steering_array[4] = {0.0, 0.0, 0.0, 0.0}; //angle that each motor need's to make

            double robotDimMatrix[8][3] = {{1, 0, (-1 * Ry_)}, {0, 1, Rx_}, {1, 0, Ry_}, {0, 1, Rx_}, {1, 0, (-1 * Ry_)}, {0, 1, (-1 * Rx_)}, {1, 0, Ry_}, {0, 1, (-1 * Rx_)}};
            double inputRobotSpeeds[3] = {msg.linear.x, msg.linear.y, msg.angular.z};
            double outputSpeedsVxVy[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //needed speed for each motor

            double velocity_constant_value = 1 / (0.229 * 0.10472 * r_);

            //Množenje matrica dimenzija robota i ulaznih brzina kako bi se dobile izlazne brzine
            for(int i = 0; i < 8; i++)
                for(int j = 0; j < 3; j++)
                outputSpeedsVxVy[i] += robotDimMatrix[i][j]*inputRobotSpeeds[j];
            
            velocity_array_m_s[0] = sqrt((outputSpeedsVxVy[0] * outputSpeedsVxVy[0]) + (outputSpeedsVxVy[1] * outputSpeedsVxVy[1]));
            velocity_array_m_s[1] = sqrt((outputSpeedsVxVy[2] * outputSpeedsVxVy[2]) + (outputSpeedsVxVy[3] * outputSpeedsVxVy[3]));
            velocity_array_m_s[2] = sqrt((outputSpeedsVxVy[4] * outputSpeedsVxVy[4]) + (outputSpeedsVxVy[5] * outputSpeedsVxVy[5]));
            velocity_array_m_s[3] = sqrt((outputSpeedsVxVy[6] * outputSpeedsVxVy[6]) + (outputSpeedsVxVy[7] * outputSpeedsVxVy[7]));

            steering_array[0] = atan2(outputSpeedsVxVy[1], outputSpeedsVxVy[0]);
            steering_array[1] = atan2(outputSpeedsVxVy[3], outputSpeedsVxVy[2]);
            steering_array[2] = atan2(outputSpeedsVxVy[5], outputSpeedsVxVy[4]);
            steering_array[3] = atan2(outputSpeedsVxVy[7], outputSpeedsVxVy[6]);

            for(int i = 0; i < 4; i++){
                if(steering_array[i] > M_PI_2){
                    velocity_array_m_s[i] = -1 * velocity_array_m_s[i];
                    steering_array[i] = steering_array[i] - M_PI;
                }
                else if(steering_array[i] <=  -M_PI_2){
                    velocity_array_m_s[i] = -1 * velocity_array_m_s[i];
                    steering_array[i] = steering_array[i] + M_PI;
                }
                else{
                    velocity_array_m_s[i] = velocity_array_m_s[i];
                    steering_array[i] = steering_array[i];
                }
            }

            for(int i = 0;i < 4;i++)
                dynamixel_position[i] = 2048 + steering_array[i] * PULS_PER_RAD;

            for(int i = 0;i < 4;i++)
                dynamixel_velocity[i] = velocity_array_m_s[i] * velocity_constant_value;

            result = dxl_wb.syncWrite(kGoalPositionIndex, position_id_array, 4, dynamixel_position  , 1, &log);
            if (result == false){
                RCLCPP_INFO(this->get_logger(), "%s", log);
            }

            result = dxl_wb.syncWrite(kGoalVelocityIndex, velocity_id_array, 4, dynamixel_velocity  , 1, &log);
            if (result == false){
               RCLCPP_INFO(this->get_logger(), "%s", log);
            }
        }

        void joint_state_callback()
        {
            int32_t positions[8];
            int32_t velocities[8];
            int32_t currents[8];
            const char* log = NULL;
            dxl_wb.syncRead(kPresentPositionVelocityCurrentIndex, motors_ids_, 8, &log);
            dxl_wb.getSyncReadData(kPresentPositionVelocityCurrentIndex, motors_ids_, 8, 126, 2, currents, &log);
            dxl_wb.getSyncReadData(kPresentPositionVelocityCurrentIndex, motors_ids_, 8, 128, 4, velocities, &log);
            dxl_wb.getSyncReadData(kPresentPositionVelocityCurrentIndex, motors_ids_, 8, 132, 4, positions, &log);
            auto joint_states = sensor_msgs::msg::JointState();
            joint_states.name = {"FL_wheel", "BL_wheel", "BR_wheel", "FR_wheel", "FL_steering", "BL_steering", "BR_steering", "FR_steering"};
            joint_states.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            joint_states.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            joint_states.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            for(int i = 0; i < 8;i++){
                joint_states.position[i] = dxl_wb.convertValue2Radian(motors_ids_[i], positions[i]);
                joint_states.velocity[i] = dxl_wb.convertValue2Velocity(motors_ids_[i], velocities[i]);
                joint_states.effort[i] = dxl_wb.convertValue2Current(motors_ids_[i], currents[i]);
            }
            publisher_->publish(joint_states);
        }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    size_t count_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FWSFWDController>());
  rclcpp::shutdown();
  return 0;
}