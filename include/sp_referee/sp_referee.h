#pragma once

#include <cstdint>
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <serial/serial.h>
#include "sp_referee/data.h"
#include "sp_referee/check.h"
#include "sp_referee/protocol.h"

#include "sp_referee/RobotStatusMsg.h"
#include "sp_referee/PowerHeatDataMsg.h"
#include "sp_referee/ShootDataMsg.h"
#include "sp_referee/RadarMarkDataMsg.h"
#include "sp_referee/SentryInfoMsg.h"
#include "sp_referee/RadarInfoMsg.h"
#include "sp_referee/RemoteControlMsg.h"

#include "sp_referee/RadarCmdMsg.h"
#include "sp_referee/MapRobotDataMsg.h"
#include "sp_referee/MapDataMsg.h"
// #include "rm_referee/common/data.h"
// #include "rm_referee/referee_base.h"
#include <sp_common/ManipulatorCmd.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
namespace sp_referee
{
    class Referee
    {
        public:
            Referee() = default;

            bool init();

            void read();

            void write();

            void sendUi();

            void sendGraphs();
            
            void sendString();

            void sendLines();

            void sendCorner();

            bool getImageTrasmission();

            void readImageTrasmission();

        private:

            void initCmd(XmlRpc::XmlRpcValue &cmd, std::string type);

            void initUI(XmlRpc::XmlRpcValue &ui, std::string type);

            int unpack(uint8_t* rx_data);

            void pack(uint8_t* tx_buffer, uint8_t* data, int cmd_id, int len);

            void getRobotInfo();

            void clearRxBuffer();

            void clearTxBuffer();

            sp_referee::GraphColor getColor(const std::string& color);

            sp_referee::GraphType getType(const std::string& type);

            void addUI(UIConfig &config);

            void updateUI(UIConfig &config);

            void deleteUI(UIConfig &config);

            void jointPosCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);

            void visionCornerCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);

            void cmdPumpCallback(const std_msgs::Bool::ConstPtr &msg);

            void cmdRodCallback(const std_msgs::Bool::ConstPtr &msg);

            void cmdDeflectionCallback(const std_msgs::Bool::ConstPtr &msg);

            void manipulatorCmdCallback(const sp_common::ManipulatorCmd::ConstPtr &msg);

            void radarCmdCallback(const sp_referee::RadarCmdMsg::ConstPtr &msg);
    
            void mapRobotDataCallback(const sp_referee::MapRobotDataMsgConstPtr &msg);

            void mapDataCallback(const sp_referee::MapDataMsgConstPtr &msg);

            ros::NodeHandle nh_;

            serial::Serial serial_;

            serial::Serial image_transmission_;

            bool use_image_transmission_link_{};

            std::vector<uint8_t> rx_buffer_;

            int rx_len_;

            bool referee_data_is_online_{};

            bool image_trasmission_data_is_online_{};

            ros::Time last_get_data_time_;
            
            ros::Time last_send_data_time_;

            const int frame_length_ = 128, frame_header_length_ = 5, cmd_id_length_ = 2, frame_tail_length_ = 2;

            const int k_unpack_buffer_length_ = 256;

            uint8_t unpack_buffer_[256]{};

            uint8_t tx_buffer_[128]{};

            int tx_len_;

            bool ui_generated{};

            RobotInfo robot_info_;

            Check check_;

            std::vector<UIConfig> string_ui_;

            std::vector<UIConfig> graphs_ui_;

            std::vector<UIConfig> lines_ui_;
            int frequency_num_ = 0;

            std::vector<uint16_t> read_cmd_;
            std::vector<uint16_t> write_cmd_;

            ros::Publisher robot_status_pub_;
            ros::Publisher power_heat_data_pub_;
            ros::Publisher shoot_data_pub_;
            ros::Publisher radar_mark_data_pub_;
            ros::Publisher remote_control_pub_;

            ros::Subscriber radar_cmd_sub_;
            ros::Subscriber map_robot_data_sub_;
            ros::Subscriber map_data_sub_;

            ros::Subscriber manipulator_cmd_sub_;
            ros::Subscriber joint_pos_sub_;
            ros::Subscriber vision_corner_sub_;
            ros::Subscriber cmd_pump_sub_;
            ros::Subscriber cmd_rod_sub_;
            ros::Subscriber cmd_deflection_sub_;
             

            std_msgs::Float64MultiArray joint_pos_;
            std_msgs::Float64MultiArray vision_corner_;
            std_msgs::Bool cmd_pump_;
            std_msgs::Bool cmd_rod_;
            std_msgs::Bool cmd_deflection_;

      

            sp_common::ManipulatorCmd manipulator_cmd_;

            sp_referee::RadarCmdMsg radar_cmd_ref_;
            sp_referee::MapRobotDataMsg map_robot_data_ref_;
            sp_referee::MapDataMsg map_data_ref_;

            Eigen::Matrix3d last_matrix{};
            Eigen::Matrix3d current_matrix{};
            ros::Time last_time{};
            ros::Time current_time{};
            ros::Publisher velocity_pub_;
            geometry_msgs::Twist cmd_velocity{};

            enum
            {
                MAUL,
                AUTO,
                JOINT,
                CALI
            };
    };
}