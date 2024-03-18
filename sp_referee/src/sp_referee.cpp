#include "sp_referee/sp_referee.h"
#include "sp_referee/check.h"
#include "sp_referee/data.h"
#include "sp_referee/protocol.h"



namespace sp_referee
{
    bool Referee::init()
    {
        serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
        serial_.setPort("/dev/ttyUSB0");
        serial_.setBaudrate(115200);
        serial_.setTimeout(timeout);
        if (serial_.isOpen())
            return false;
        try
        {
            serial_.open();
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR_STREAM("Cannot open referee port");
        }

        nh_ = ros::NodeHandle("/referee");
        // nh_.getParam("use_image_transmission_link", use_image_transmission_link_);
        // if (use_image_transmission_link_)
        // {
        //     image_transmission_.setPort("/dev/ttyUSB1");
        //     image_transmission_.setBaudrate(115200);
        //     image_transmission_.setTimeout(timeout);
        // }
        

        XmlRpc::XmlRpcValue xml_rpc_value;
       
 

        nh_.getParam("read", xml_rpc_value);

        initCmd(xml_rpc_value, "read");
        nh_.getParam("write", xml_rpc_value);
        initCmd(xml_rpc_value, "write");
        nh_.getParam("ui/mode", xml_rpc_value);
        initUI(xml_rpc_value, "mode");

        robot_status_pub_ = nh_.advertise<sp_referee::RobotStatusMsg>("/robot_status", 1);
        power_heat_data_pub_ = nh_.advertise<sp_referee::PowerHeatDataMsg>("/power_heat_data", 1);
        shoot_data_pub_ = nh_.advertise<sp_referee::ShootDataMsg>("/shoot_data", 1);

        remote_control_pub_ = nh_.advertise<sp_referee::RemoteControlMsg>("/rc_data", 1);

        // chassis_cmd_sub_ = nh_.subscribe<sp_common::ChassisCmd>("/chassis_cmd", 1, &Referee::chassisCmdCallback, this);
        // gimbal_cmd_sub_ = nh_.subscribe<sp_common::GimbalCmd>("/gimbal_cmd", 1, &Referee::gimbalCmdCallback, this);
        manipulator_cmd_sub_ = nh_.subscribe<sp_common::ManipulatorCmd>("/cmd_manipulator", 1, &Referee::manipulatorCmdCallback, this);
        
        radar_cmd_sub_ = nh_.subscribe<sp_referee::RadarCmdMsg>("/radar_cmd", 1, &Referee::radarCmdCallback, this);
        map_robot_data_sub_ = nh_.subscribe<sp_referee::MapRobotDataMsg>("/map_robot_data", 1, &Referee::mapRobotDataCallback, this);
        map_data_sub_ = nh_.subscribe<sp_referee::MapDataMsg>("/map_data", 1, &Referee::mapDataCallback, this);
        return true;
    }

    void Referee::initCmd(XmlRpc::XmlRpcValue &cmd, std::string type)
    {
        if (type == "read")
        
        {     
            for (int i = 0; i < cmd.size(); ++i)
            {      
                //ROS_INFO_STREAM(std::hex<<static_cast<uint16_t>(static_cast<int>(cmd[i])));
                read_cmd_.push_back(static_cast<uint16_t>(static_cast<int>(cmd[i])));
            }
        }
        else if (type == "write")
        {
            
            for (int i = 0; i < cmd.size(); ++i)
            {      
                //ROS_INFO_STREAM(std::hex<<static_cast<uint16_t>(static_cast<int>(cmd[i])));
                write_cmd_.push_back(static_cast<uint16_t>(static_cast<int>(cmd[i])));
            }
        }
        else
        {
            ROS_ERROR_STREAM("Illegal type.");
            return; 
        }
    }

   

    bool Referee::getImageTrasmission()
    {
        return use_image_transmission_link_;
    }

    void Referee::manipulatorCmdCallback(const sp_common::ManipulatorCmd::ConstPtr &msg)
    {
        manipulator_cmd_ = *msg;
    }

    void Referee::radarCmdCallback(const sp_referee::RadarCmdMsg::ConstPtr &msg)
    {
        radar_cmd_ref_ = *msg;
    }

    void Referee::mapRobotDataCallback(const sp_referee::MapRobotDataMsg::ConstPtr &msg)
    {
        map_robot_data_ref_ = *msg;
    }

    void Referee::mapDataCallback(const sp_referee::MapDataMsg::ConstPtr &msg)
    {
        map_data_ref_ = *msg;
    }


} // namespace sp_referee