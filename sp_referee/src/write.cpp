#include "sp_referee/sp_referee.h"
#include "sp_referee/check.h"
#include "sp_referee/data.h"
#include "sp_referee/protocol.h"



namespace sp_referee
{
    
    void Referee::write()
    {
    
        for (int i = 0; i < write_cmd_.size(); i++)
        {
            int data_len = 0;
            int frame_len = 0;
            switch (write_cmd_[i])
            {
                case sp_referee::ROBOT_INTERACTIVE_DATA_CMD:
                {
                    // if (robot_info_.robot_type_ == "radar")
                    // {
                        data_len = static_cast<int>(sizeof(sp_referee::RadarCmd));
                        frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
                        sp_referee::RadarCmd radar_cmd;
                        radar_cmd.robot_interaction_data_header_.data_cmd_id_ = sp_referee::RADAR_CMD_CMD;
                        radar_cmd.robot_interaction_data_header_.sender_id_ = robot_info_.robot_id_;
                        radar_cmd.robot_interaction_data_header_.receiver_id_ = 0x8080;
                        radar_cmd.radar_cmd_ = radar_cmd_ref_.radar_cmd;
                        pack(reinterpret_cast<uint8_t*>(&tx_buffer_), reinterpret_cast<uint8_t*>(&radar_cmd), sp_referee::ROBOT_INTERACTIVE_DATA_CMD, data_len);
                        break;
                    // }
                    // else if (robot_info_.robot_type_ == "sentry")
                    // {
                    //     break;

                    // }
                    // else
                    //     break;
                    
                }
                // case sp_referee::MAP_ROBOT_DATA_CMD:
                // {
                       
                             
                //     data_len = sizeof(sp_referee::MapRobotData);
                //     uint8_t tx_buffer_[+ sizeof(sp_referee::MapRobotData)]{};  
                //     sp_referee::MapRobotData map_robot_data;
                //     map_robot_data.target_robot_id_ = map_robot_data_ref_.target_robot_id;
                //     map_robot_data.target_position_x_ = map_robot_data_ref_.target_position_x;
                //     map_robot_data.target_position_y_ = map_robot_data_ref_.target_position_y;
                //     pack(tx_buffer_, reinterpret_cast<uint8_t*>(&map_robot_data), sp_referee::MAP_ROBOT_DATA_CMD, data_len);
                //     break;
                // }
                case sp_referee::CUSTOM_CLIENT_DATA_CMD:
                {
                    data_len = sizeof(sp_referee::CustomClientData);
                    sp_referee::CustomClientData custom_client_data;
                    break;
                }
                case sp_referee::MAP_DATA_CMD:
                {
                    data_len = sizeof(sp_referee::MapData);
                    // sp_referee::MapData map_data;
                    // map_data.intention_ = map_data_ref_.intention;
                    // map_data.start_position_x_ = map_data_ref_.start_position_x;
                    // map_data.start_position_y_ = map_data_ref_.start_position_y;
                    // map_data.delta_x_ = map_data_ref_.delta_x;
                    // map_data.delta_y_ = map_data_ref_.delta_y;
                    // map_data.sender_id_ = map_data_ref_.sender_id;
                    // pack(tx_buffer_, reinterpret_cast<uint8_t*>(&map_data), sp_referee::MAP_DATA_CMD, data_len);
                    break;
                }
                case sp_referee::CUSTOM_INFO_CMD:
                {
                    data_len = sizeof(sp_referee::CustomInfo);
                    // sp_referee::CustomInfo custom_info;
                    // custom_info.sender_id_ = custom_info_ref_.sender_id;
                    // custom_info.receiver_id_ = custom_info_ref_.receiver_id;
                    // custom_info.user_data_ = custom_info_ref_.user_data;
                    // pack(tx_buffer_, reinterpret_cast<uint8_t*>(&custom_info), sp_referee::CUSTOM_INFO_CMD, data_len);
                    break;
                }
                default:
                {
                    ROS_WARN("Referee command ID not found.");
                    break;
                }
            }
           
            
            //ui_queue_.pop_back();
            last_send_data_time_ = ros::Time::now();
 

            try
            {
                ROS_INFO_STREAM("AAA"<<frame_len);
                serial_.write(tx_buffer_, 18);
            }
            catch (serial::PortNotOpenedException& e)
            {

            }
        }
       

        clearTxBuffer();

    }

    void Referee::pack(uint8_t* tx_buffer, uint8_t* data, int cmd_id, int len)
    {
        //memset(tx_buffer, 0, frame_length_);
        auto* frame_header = reinterpret_cast<sp_referee::FrameHeader*>(tx_buffer);

        frame_header->sof_ = 0xA5;
        frame_header->data_length_ = len;
        memcpy(&tx_buffer[frame_header_length_], reinterpret_cast<uint8_t*>(&cmd_id), cmd_id_length_);
      
        check_.appendCRC8CheckSum(tx_buffer, frame_header_length_);
       
        memcpy(&tx_buffer[frame_header_length_ + cmd_id_length_], data, len);
         
        check_.appendCRC16CheckSum(tx_buffer, frame_header_length_ + cmd_id_length_ + len + frame_tail_length_);
        for (int i = 0; i < frame_header_length_ + cmd_id_length_ + len + frame_tail_length_ + 2 ; i++)
                ROS_INFO_STREAM(std::hex<<int(tx_buffer[i]));
            ROS_INFO_STREAM("----------------------------------");
    }


    void Referee::clearTxBuffer()
    {
        for (int i = 0; i < 128; i++)
            tx_buffer_[i] = 0;
    }


} // namespace sp_referee