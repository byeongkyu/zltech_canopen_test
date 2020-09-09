#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <k3_mobile_hw/motor_driver_zlac8015_canopen.h>

class ControlNode
{
    public:
        ControlNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        {
            double control_frequency = 0.0;
            pnh.param<double>("rate", control_frequency, 10.0);

            robot_hw = boost::make_shared<MotorDriverZLAC8015CANOpenInterface>();
            assert(robot_hw->init(nh, pnh));

            ros::Duration(0.5).sleep();
            ROS_INFO("[%s] ready. start controller...", ros::this_node::getName().c_str());

            cm = boost::make_shared<controller_manager::ControllerManager>(&(*robot_hw), nh);
            period = ros::Duration(1.0/control_frequency);

            loop_timer = nh.createTimer(period, &ControlNode::callback, this);
            loop_timer.start();
        }

        ~ControlNode()
        {
            loop_timer.stop();
        }

    private:
        void callback(const ros::TimerEvent& event)
        {
            ros::Time start_time = ros::Time::now();

            robot_hw->read(ros::Time::now(), period);
            cm->update(ros::Time::now(), period);
            robot_hw->write(ros::Time::now(), period);

            ROS_INFO("%f", (ros::Time::now() - start_time).toSec());
        }

    private:
        boost::shared_ptr<hardware_interface::RobotHW> robot_hw;
        boost::shared_ptr<controller_manager::ControllerManager> cm;
        ros::Duration period;
        ros::Timer loop_timer;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "thirabot_control_node");
    ros::AsyncSpinner spinner(3);

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ControlNode m(nh, pnh);

    spinner.start();
    ros::waitForShutdown();

    return 0;
}