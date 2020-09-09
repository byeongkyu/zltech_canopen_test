#ifndef MOTOR_DRIVER_ZLAC8015_CANOPEN_H_
#define MOTOR_DRIVER_ZLAC8015_CANOPEN_H_

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

#include <boost/thread.hpp>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>

#include "k3_mobile_hw/can_ixxat_usb_compact_if.h"

class MotorDriverZLAC8015CANOpenInterface: public hardware_interface::RobotHW
{
    public:
        MotorDriverZLAC8015CANOpenInterface();
        ~MotorDriverZLAC8015CANOpenInterface();

    public:
        virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
        virtual void read(const ros::Time& time, const ros::Duration& period);
        virtual void write(const ros::Time& time, const ros::Duration& period);
        virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);
        virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list);

    private:
        void callback_activate_e_stop(const std_msgs::BoolConstPtr& e_stop_active);
        bool handle_clear_alarm(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        // Driver
        bool reset_driver(int id);
        bool start_driver(int id);
        bool stop_driver(int id);

        // CANOpen
        bool NMTcommand(int command, int id);
        bool SDODownload(int id, int obj_index, int sub_index, int32_t data, int len);
        bool SDOUpload(int id, int obj_index, int sub_index, int32_t *data);

    private:
        CANIxxatUSBCompactInterface canopen_;

        hardware_interface::JointStateInterface jnt_state_interface_;
        hardware_interface::PositionJointInterface jnt_pos_interface_;
        hardware_interface::VelocityJointInterface jnt_vel_interface_;
        hardware_interface::EffortJointInterface jnt_eff_interface_;

        std::vector<double> joint_cmd_;
        std::vector<double> last_joint_cmd_;
        std::vector<double> joint_pos_;
        std::vector<double> joint_vel_;
        std::vector<double> joint_eff_;

        std::vector<int32_t> last_encoder_value_;

        bool e_stop_active_;
        bool last_e_stop_active_;
        bool is_has_error_;
        ros::Subscriber sub_e_stop_;

        boost::mutex lock;

        ros::ServiceServer srv_clear_alarm_;
};

#endif //MOTOR_DRIVER_ZLAC8015_CANOPEN_H_