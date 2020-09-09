#include "k3_mobile_hw/motor_driver_zlac8015_canopen.h"

MotorDriverZLAC8015CANOpenInterface::MotorDriverZLAC8015CANOpenInterface()
{
    last_encoder_value_.resize(2, 0);
}

MotorDriverZLAC8015CANOpenInterface::~MotorDriverZLAC8015CANOpenInterface()
{
    stop_driver(0);
}

bool MotorDriverZLAC8015CANOpenInterface::init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    assert(canopen_.init(1000));

    for(int i = 1; i <= 2; i++)
    {
        reset_driver(i);
        start_driver(i);
    }

    // Control Word
    SDODownload(1, 0x6040, 0x0, 0x0, 2);
    SDODownload(2, 0x6040, 0x0, 0x0, 2);
    ros::Duration(0.1).sleep();

    // Acceleration Time 200ms
    SDODownload(1, 0x6083, 0x0, 50, 4);
    SDODownload(2, 0x6083, 0x0, 50, 4);
    ros::Duration(0.1).sleep();

    // Decceleration Time 200ms
    SDODownload(1, 0x6084, 0x0, 50, 4);
    SDODownload(2, 0x6084, 0x0, 50, 4);
    ros::Duration(0.1).sleep();

    //
    SDODownload(1, 0x60FF, 0x0, 0, 4);
    SDODownload(2, 0x60FF, 0x0, 0, 4);
    ros::Duration(0.1).sleep();

    // Profile Velocity Mode
    SDODownload(1, 0x6060, 0x0, 0x3, 1);
    SDODownload(2, 0x6060, 0x0, 0x3, 1);
    ros::Duration(0.1).sleep();

    //
    SDODownload(1, 0x6040, 0x0, 6, 2);
    SDODownload(2, 0x6040, 0x0, 6, 2);
    ros::Duration(0.1).sleep();

    //
    SDODownload(1, 0x6040, 0x0, 7, 2);
    SDODownload(2, 0x6040, 0x0, 7, 2);
    ros::Duration(0.1).sleep();

    //
    SDODownload(1, 0x6040, 0x0, 0xF, 2);
    SDODownload(2, 0x6040, 0x0, 0xF, 2);
    ros::Duration(0.1).sleep();

    //
    SDODownload(1, 0x6040, 0x0, 0xF, 2);
    SDODownload(2, 0x6040, 0x0, 0xF, 2);
    ros::Duration(0.1).sleep();

    // Save Current Encoder Data
    if(!SDOUpload(1, 0x6064, 0x0, &last_encoder_value_[0]))
        return false;
    if(!SDOUpload(2, 0x6064, 0x0, &last_encoder_value_[1]))
        return false;


    // hardware_interface
    joint_cmd_.resize(2);
    for(int i = 0; i < joint_cmd_.size(); i++) {
        joint_cmd_[i] = 0.0;
    }
    joint_pos_.resize(2, 0);
    joint_vel_.resize(2, 0);
    joint_eff_.resize(2, 0);

    hardware_interface::JointStateHandle state_handle_l_wheel("l_wheel_joint", &joint_pos_[0], &joint_vel_[0], &joint_eff_[0]);
    jnt_state_interface_.registerHandle(state_handle_l_wheel);
    hardware_interface::JointStateHandle state_handle_r_wheel("r_wheel_joint", &joint_pos_[1], &joint_vel_[1], &joint_eff_[1]);
    jnt_state_interface_.registerHandle(state_handle_r_wheel);

    registerInterface(&jnt_state_interface_);

    // joint_velocity_interface
    hardware_interface::JointHandle vel_handle_l_wheel(jnt_state_interface_.getHandle("l_wheel_joint"), &joint_cmd_[0]);
    jnt_vel_interface_.registerHandle(vel_handle_l_wheel);
    hardware_interface::JointHandle vel_handle_r_wheel(jnt_state_interface_.getHandle("r_wheel_joint"), &joint_cmd_[1]);
    jnt_vel_interface_.registerHandle(vel_handle_r_wheel);

    registerInterface(&jnt_vel_interface_);

    e_stop_active_ = false;
    last_e_stop_active_ = false;
    sub_e_stop_ = nh.subscribe("activate_e_stop", 1, &MotorDriverZLAC8015CANOpenInterface::callback_activate_e_stop, this);

    // ROS
    srv_clear_alarm_ = nh.advertiseService("clear_motor_alarm", &MotorDriverZLAC8015CANOpenInterface::handle_clear_alarm, this);

    ROS_INFO("[%s] initialized successfully.", ros::this_node::getName().c_str());
    return true;
}

void MotorDriverZLAC8015CANOpenInterface::callback_activate_e_stop(const std_msgs::BoolConstPtr& e_stop_active)
{
    e_stop_active_ = e_stop_active->data;
}

void MotorDriverZLAC8015CANOpenInterface::read(const ros::Time& time, const ros::Duration& period)
{
    boost::mutex::scoped_lock scoped_lock(lock);

    int32_t current_enc[2] = {0, 0};
    if(!SDOUpload(1, 0x6064, 0x0, &current_enc[0]))
        return;
    if(!SDOUpload(2, 0x6064, 0x0, &current_enc[1]))
        return;


    joint_eff_[0] = 0.0; //u_current[0] * -1.0;
    joint_eff_[1] = 0.0; //u_current[1] * -1.0;

    joint_vel_[0] = (current_enc[0] - last_encoder_value_[0]) / 4096.0 * (1.0 / period.toSec()) * (2.0 * M_PI);
    joint_vel_[1] = (current_enc[1] - last_encoder_value_[1]) / 4096.0 * (1.0 / period.toSec()) * (2.0 * M_PI) * 1.0;

    joint_pos_[0] += (current_enc[0] - last_encoder_value_[0]) / 4096.0 * (2.0 * M_PI);
    joint_pos_[1] += (current_enc[1] - last_encoder_value_[1]) / 4096.0 * (2.0 * M_PI) * -1.0;

    last_encoder_value_[0] = current_enc[0];
    last_encoder_value_[1] = current_enc[1];
}

void MotorDriverZLAC8015CANOpenInterface::write(const ros::Time& time, const ros::Duration& period)
{
    if(is_has_error_)
        return;

    // When e_stop_active_ is true (E-STOP Button is pressed, all joint command for velocity is set 0)
    joint_cmd_[0] = e_stop_active_ ? 0 : joint_cmd_[0];
    joint_cmd_[1] = e_stop_active_ ? 0 : joint_cmd_[1];

    int32_t l_vel = (int32_t)(joint_cmd_[0] * 60.0 / (2.0 * M_PI));
    int32_t r_vel = (int32_t)(joint_cmd_[1] * 60.0 / (2.0 * M_PI)) * -1.0;

    boost::mutex::scoped_lock scoped_lock(lock);

    SDODownload(1, 0x60FF, 0x0, l_vel, 4);
    SDODownload(2, 0x60FF, 0x0, r_vel, 4);
}

bool MotorDriverZLAC8015CANOpenInterface::handle_clear_alarm(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    return true;
}


///
bool MotorDriverZLAC8015CANOpenInterface::reset_driver(int id)
{
    return NMTcommand(0x81, id);
}

bool MotorDriverZLAC8015CANOpenInterface::start_driver(int id)
{
    return NMTcommand(0x01, id);;
}

bool MotorDriverZLAC8015CANOpenInterface::stop_driver(int id)
{
    return  NMTcommand(0x02, id);;
}

bool MotorDriverZLAC8015CANOpenInterface::NMTcommand(int command, int id)
{
    CANMsg msg;
    msg.set_length(2);
    msg.set_ID(0);
    msg.set(
        (uint8_t) command,
        (uint8_t) id
    );

    if(!canopen_.send_message(msg))
    {
        ROS_WARN("[CANopen] Send error...");
        return false;
    }

    if(command == 0x02)
        return true;

    // Response
    CANMsg recv_msg;
    bool ret = false;
    do
    {
        ret = canopen_.recv_message(&recv_msg);
    }
    while(recv_msg.get_ID() == 0 || ret && ros::ok());

    if(command == 0x82)
    {
        if(recv_msg.get_ID() != (0x700 + id))
        {
            ROS_WARN("[CANopen] ID mismatch...");
            return false;
        }
    }

    return true;
}

bool MotorDriverZLAC8015CANOpenInterface::SDODownload(int id, int obj_index, int sub_index, int32_t data, int len)
{
    uint8_t command_code = 0x22;
    switch(len)
    {
        case 1:
            command_code = 0x2F;
            break;
        case 2:
            command_code = 0x2B;
            break;
        case 3:
            command_code = 0x27;
            break;
        case 4:
            command_code = 0x23;
            break;
        default:
            command_code = 0x22;
            break;
    }

    CANMsg msg;
    msg.set_length(8);
    msg.set_ID(0x600 + id);
    msg.set(
        (uint8_t) command_code,
        (uint8_t) obj_index,
        (uint8_t) (obj_index >> 8),
        (uint8_t) sub_index,
        //data
        (uint8_t) data,
        (uint8_t) (data >> 8),
        (uint8_t) (data >> 16),
        (uint8_t) (data >> 24)
    );

    if(!canopen_.send_message(msg))
    {
        ROS_WARN("[CANopen] Send error...");
        return false;
    }

    CANMsg recv_msg;
    if(!canopen_.recv_message(&recv_msg))
    {
        ROS_WARN("[CANopen] Recv error...");
        return false;
    }

    if(recv_msg.get_ID() != (0x580 + id))
    {
        ROS_WARN("[CANopen] ID mismatch...");
        return false;
    }

    if(recv_msg.get_at(0) == 0x80)
    {
        ROS_WARN("[CANopen] Return error at %x", obj_index);
        return false;
    }

    // recv_msg.printMessage();
    return true;
}

bool MotorDriverZLAC8015CANOpenInterface::SDOUpload(int id, int obj_index, int sub_index, int32_t *data)
{
    const int ciInitUploadReq = 0x40;

    CANMsg msg;
    msg.set_length(8);
    msg.set_ID(0x600 + id);
    msg.set(
        ciInitUploadReq,
        (uint8_t) obj_index,
        (uint8_t) (obj_index >> 8),
        (uint8_t) sub_index,
        0,
        0,
        0,
        0
    );

    if(!canopen_.send_message(msg))
    {
        ROS_WARN("[CANopen] Send error...");
        return false;
    }

    CANMsg recv_msg;
    if(!canopen_.recv_message(&recv_msg))
    {
        ROS_WARN("[CANopen] Recv error...");
        return false;
    }

    if(recv_msg.get_ID() != (0x580 + id))
    {
        ROS_WARN("[CANopen] ID mismatch...");
        return false;
    }

    if(recv_msg.get_at(0) == 0x80)
    {
        ROS_WARN("[CANopen] Return error at %x", obj_index);
        return false;
    }

    *data = (int32_t)(recv_msg.get_at(7) << 24) | (int32_t)(recv_msg.get_at(6) << 16) | (int32_t)(recv_msg.get_at(5) << 8) | (int32_t)(recv_msg.get_at(4));
    return true;
}


///

bool MotorDriverZLAC8015CANOpenInterface::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    for (std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it)
    {
        if (it->claimed_resources.empty())
        {
            continue;
        }
        for (std::vector<hardware_interface::InterfaceResources>::const_iterator res_it = it->claimed_resources.begin(); res_it != it->claimed_resources.end(); ++res_it)
        {
            std::vector<std::string> r_hw_ifaces = this->getNames();

            std::vector<std::string>::iterator if_name = std::find(r_hw_ifaces.begin(), r_hw_ifaces.end(), res_it->hardware_interface);
            if (if_name == r_hw_ifaces.end()) // this hardware_interface is not registered on this RobotHW
            {
                ROS_ERROR_STREAM("Bad interface: " << res_it->hardware_interface);
                std::cout << res_it->hardware_interface;
                return false;
            }

            std::vector<std::string> r_hw_iface_resources = this->getInterfaceResources(res_it->hardware_interface);
            for (std::set<std::string>::const_iterator ctrl_res = res_it->resources.begin(); ctrl_res != res_it->resources.end(); ++ctrl_res)
            {
                std::vector<std::string>::iterator res_name = std::find(r_hw_iface_resources.begin(), r_hw_iface_resources.end(), *ctrl_res);
                if (res_name == r_hw_iface_resources.end()) // this resource is not registered on this RobotHW
                {
                    ROS_ERROR_STREAM("Bad resource: " << (*ctrl_res));
                    std::cout << (*ctrl_res);
                    return false;
                }
            }
        }
    }
    return true;
}

void MotorDriverZLAC8015CANOpenInterface::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list)
{
    for (std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it)
    {
        if (it->claimed_resources.empty())
        {
            continue;
        }
        for (std::vector<hardware_interface::InterfaceResources>::const_iterator res_it = it->claimed_resources.begin(); res_it != it->claimed_resources.end(); ++res_it)
        {
            std::vector<std::string> r_hw_ifaces = this->getNames();

            std::vector<std::string>::iterator if_name = std::find(r_hw_ifaces.begin(), r_hw_ifaces.end(), res_it->hardware_interface);
            if (if_name == r_hw_ifaces.end()) // this hardware_interface is not registered on this RobotHW
            {
                throw hardware_interface::HardwareInterfaceException("Hardware_interface " + res_it->hardware_interface + " is not registered");
            }

            std::vector<std::string> r_hw_iface_resources = this->getInterfaceResources(res_it->hardware_interface);
            for (std::set<std::string>::const_iterator ctrl_res = res_it->resources.begin(); ctrl_res != res_it->resources.end(); ++ctrl_res)
            {
                std::vector<std::string>::iterator res_name = std::find(r_hw_iface_resources.begin(), r_hw_iface_resources.end(), *ctrl_res);
                if (res_name == r_hw_iface_resources.end()) // this resource is not registered on this RobotHW
                {
                    throw hardware_interface::HardwareInterfaceException("Resource " + *ctrl_res + " is not registered");
                }
            }
        }
    }
}
