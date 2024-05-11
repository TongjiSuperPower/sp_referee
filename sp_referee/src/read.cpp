#include "sp_referee/sp_referee.h"
#include "sp_referee/check.h"
#include "sp_referee/data.h"
#include "sp_referee/protocol.h"



namespace sp_referee
{

    void Referee::read()
    {

        
        if (serial_.available())
        {
            rx_len_ = static_cast<int>(serial_.available());
            serial_.read(rx_buffer_, rx_len_);
        }
        else
        {
            return;
        }
    

        uint8_t temp_buffer[256] = { 0 };
        int frame_len;
        if (ros::Time::now() - last_get_data_time_ > ros::Duration(0.1))
            referee_data_is_online_ = false;
        if (rx_len_ < k_unpack_buffer_length_)
        {

            for (int k_i = 0; k_i < k_unpack_buffer_length_ - rx_len_; ++k_i)
                temp_buffer[k_i] = unpack_buffer_[k_i + rx_len_];
            for (int k_i = 0; k_i < rx_len_; ++k_i)
                temp_buffer[k_i + k_unpack_buffer_length_ - rx_len_] = rx_buffer_[k_i];
            for (int k_i = 0; k_i < k_unpack_buffer_length_; ++k_i)
                unpack_buffer_[k_i] = temp_buffer[k_i];
        }
        for (int k_i = 0; k_i < k_unpack_buffer_length_ - frame_length_; ++k_i)
        {
            if (unpack_buffer_[k_i] == 0xA5)
            {
                frame_len = unpack(&unpack_buffer_[k_i]);
                if (frame_len != -1)
                    k_i += frame_len;
            }
        }

        getRobotInfo();
        clearRxBuffer();
    }


    void Referee::readImageTrasmission()
    {
        if (image_transmission_.available())
        {
            rx_len_ = static_cast<int>(image_transmission_.available());
            image_transmission_.read(rx_buffer_, rx_len_);
        }
        else
        {
            return;
        }
    

        uint8_t temp_buffer[256] = { 0 };
        int frame_len;
        if (ros::Time::now() - last_get_data_time_ > ros::Duration(0.1))
            image_trasmission_data_is_online_ = false;
        if (rx_len_ < k_unpack_buffer_length_)
        {
            for (int k_i = 0; k_i < k_unpack_buffer_length_ - rx_len_; ++k_i)
                temp_buffer[k_i] = unpack_buffer_[k_i + rx_len_];
            for (int k_i = 0; k_i < rx_len_; ++k_i)
                temp_buffer[k_i + k_unpack_buffer_length_ - rx_len_] = rx_buffer_[k_i];
            for (int k_i = 0; k_i < k_unpack_buffer_length_; ++k_i)
                unpack_buffer_[k_i] = temp_buffer[k_i];
        }
        for (int k_i = 0; k_i < k_unpack_buffer_length_ - frame_length_; ++k_i)
        {
            if (unpack_buffer_[k_i] == 0xA5)
            {           
                frame_len = unpack(&unpack_buffer_[k_i]);
                if (frame_len != -1)
                    k_i += frame_len;
            }
        }

        clearRxBuffer();
    }

    // Data struct:
    // | frame_header:5 bytes | cmd_id:2 bytes | data: n bytes | frame_tail: 2 bytes |

    int Referee::unpack(uint8_t* rx_data)
    {                
        uint16_t cmd_id;
        int frame_len;
        sp_referee::FrameHeader frame_header;

        memcpy(&frame_header, rx_data, frame_header_length_);

        if (static_cast<bool>(check_.verifyCRC8CheckSum(rx_data, frame_header_length_)))
        {
       
            if (frame_header.data_length_ > 256)  // temporary and inaccurate value
            {
                //ROS_INFO_STREAM("discard possible wrong frames, appropriate data length: " << frame_header.data_length_);
                return 0;
            }
            frame_len = frame_header.data_length_ + frame_header_length_ + cmd_id_length_ + frame_tail_length_;
            
            if (check_.verifyCRC16CheckSum(rx_data, frame_len) == 1)
            {
               
                cmd_id = (rx_data[6] << 8 | rx_data[5]);
                ROS_INFO_STREAM(std::hex <<"CMD_ID:"<< cmd_id);

                auto it = std::find(read_cmd_.begin(), read_cmd_.end(), cmd_id);
                if ((it == read_cmd_.end()))
                    return frame_len;
                switch (cmd_id)
                {       
                    case sp_referee::GAME_STATUS_CMD:
                    {
                        sp_referee::GameStatus game_status_ref;
                        //rm_msgs::GameStatus game_status_data;
                        memcpy(&game_status_ref, rx_data + 7, sizeof(sp_referee::GameStatus));

                        // game_status_data.game_type = game_status_ref.game_type_;
                        // game_status_data.game_progress = game_status_ref.game_progress_;
                        // game_status_data.stage_remain_time = game_status_ref.stage_remain_time_;
                        // game_status_data.sync_time_stamp = game_status_ref.sync_time_stamp_;
                        // game_status_data.stamp = last_get_data_time_;

                        //referee_ui_.gameStatusDataCallBack(game_status_data, last_get_data_time_);
                        //game_status_pub_.publish(game_status_data);
                        break;
                    }
                    case sp_referee::GAME_RESULT_CMD:
                    {
                        sp_referee::GameResult game_result_ref;
                        memcpy(&game_result_ref, rx_data + 7, sizeof(sp_referee::GameResult));
                        break;
                    }
                    case sp_referee::GAME_ROBOT_HP_CMD:
                    {
                        sp_referee::GameRobotHp game_robot_hp_ref;
                    //sp_msgs::GameRobotHp game_robot_hp_data;
                        memcpy(&game_robot_hp_ref, rx_data + 7, sizeof(sp_referee::GameRobotHp));

                        // game_robot_hp_data.blue_1_robot_hp = game_robot_hp_ref.blue_1_robot_hp_;
                        // game_robot_hp_data.blue_2_robot_hp = game_robot_hp_ref.blue_2_robot_hp_;
                        // game_robot_hp_data.blue_3_robot_hp = game_robot_hp_ref.blue_3_robot_hp_;
                        // game_robot_hp_data.blue_4_robot_hp = game_robot_hp_ref.blue_4_robot_hp_;
                        // game_robot_hp_data.blue_5_robot_hp = game_robot_hp_ref.blue_5_robot_hp_;
                        // game_robot_hp_data.blue_7_robot_hp = game_robot_hp_ref.blue_7_robot_hp_;
                        // game_robot_hp_data.red_1_robot_hp = game_robot_hp_ref.red_1_robot_hp_;
                        // game_robot_hp_data.red_2_robot_hp = game_robot_hp_ref.red_2_robot_hp_;
                        // game_robot_hp_data.red_3_robot_hp = game_robot_hp_ref.red_3_robot_hp_;
                        // game_robot_hp_data.red_4_robot_hp = game_robot_hp_ref.red_4_robot_hp_;
                        // game_robot_hp_data.red_5_robot_hp = game_robot_hp_ref.red_5_robot_hp_;
                        // game_robot_hp_data.red_7_robot_hp = game_robot_hp_ref.red_7_robot_hp_;
                        // game_robot_hp_data.stamp = last_get_data_time_;

                        // game_robot_hp_pub_.publish(game_robot_hp_data);
                        break;
                    }
                    case sp_referee::EVENT_DATA_CMD:
                    {
                        sp_referee::EventData event_data_ref;
                        // sp_referee::EventDataMsg event_data;
                        memcpy(&event_data_ref, rx_data + 7, sizeof(sp_referee::EventData));

                        // event_data.event_data = event_data_ref.event_data_;
                        // event_data.stamp = last_get_data_time_;

                        // event_data_pub_.publish(event_data);
                        break;
                    }
                    case sp_referee::EXT_SUPPLY_PROJECTILE_ACTION_CMD:
                    {
                        sp_referee::ExtSupplyProjectileAction ext_supply_projectile_action_ref;
                        //sp_referee::ExtSupplyProjectileActionMsg ext_supply_projectile_action;
                        memcpy(&ext_supply_projectile_action_ref, rx_data + 7, sizeof(sp_referee::ExtSupplyProjectileAction));

                        // ext_supply_projectile_action.reserved = ext_supply_projectile_action_ref.reserved_;
                        // ext_supply_projectile_action.supply_robot_id = ext_supply_projectile_action_ref.supply_robot_id_;
                        // ext_supply_projectile_action.supply_projectile_step = ext_supply_projectile_action_ref.supply_projectile_step_;
                        // ext_supply_projectile_action.supply_projectile_num = ext_supply_projectile_action_ref.supply_projectile_num_; 
                        // ext_supply_projectile_action.stamp = last_get_data_time_;

                        // ext_supply_projectile_action_pub_.publish(ext_supply_projectile_action);
                        break;
                    }
                    case sp_referee::REFEREE_WARNING_CMD:
                    {
                        sp_referee::RefereeWarning referee_warning_ref;
                        memcpy(&referee_warning_ref, rx_data + 7, sizeof(sp_referee::RefereeWarning));
                        break;
                    }
                    case sp_referee::DART_INFO_CMD:
                    {
                        sp_referee::DartInfo dart_info_ref;
                        //sp_referee::DartInfoMsg dart_info;
                        memcpy(&dart_info_ref, rx_data + 7, sizeof(sp_referee::DartInfo));

                        // dart_info.dart_remaining_time = dart_info_ref.dart_remaining_time_;
                        // dart_info.dart_info = dart_info_ref.dart_info_;
                        // dart_info.stamp = last_get_data_time_;

                        // dart_info_pub_.publish(dart_info);
                        break;
                    }
                    case sp_referee::ROBOT_STATUS_CMD:
                    {
                        sp_referee::RobotStatus robot_status_ref;
                        sp_referee::RobotStatusMsg robot_status;
                        memcpy(&robot_status_ref, rx_data + 7, sizeof(sp_referee::RobotStatus));

                        robot_status.robot_id = robot_status_ref.robot_id_;
                        robot_status.robot_level = robot_status_ref.robot_level_;
                        robot_status.current_hp = robot_status_ref.current_hp_;
                        robot_status.maximum_hp = robot_status_ref.maximum_hp_;
                        robot_status.shooter_barrel_cooling_value = robot_status_ref.shooter_barrel_cooling_value_;
                        robot_status.shooter_barrel_heat_limit = robot_status_ref.shooter_barrel_cooling_value_;
                        robot_status.chassis_power_limit = robot_status_ref.shooter_barrel_cooling_value_;
                        robot_status.chassis_power_limit = robot_status_ref.chassis_power_limit_;
                        robot_status.power_management_chassis_output = robot_status_ref.power_management_chassis_output_;
                        robot_status.power_management_gimbal_output = robot_status_ref.power_management_gimbal_output_;
                        robot_status.power_management_shooter_output = robot_status_ref.power_management_shooter_output_;
                        
        
                        robot_info_.robot_id_ = robot_status_ref.robot_id_;
                        robot_status.stamp = last_get_data_time_;

                        // referee_ui_.robotStatusDataCallBack(robot_status, last_get_data_time_);
                        robot_status_pub_.publish(robot_status);
                        break;
                    }
                    case sp_referee::POWER_HEAT_DATA_CMD:
                    {
                        sp_referee::PowerHeatData power_heat_data_ref;
                        sp_referee::PowerHeatDataMsg power_heat_data;
                        memcpy(&power_heat_data_ref, rx_data + 7, sizeof(sp_referee::PowerHeatData));

                        power_heat_data.buffer_energy = power_heat_data_ref.buffer_energy_;
                        power_heat_data.chassis_power = power_heat_data_ref.chassis_power_;
                        power_heat_data.shooter_17mm_1_barrel_heat = power_heat_data_ref.shooter_17mm_1_barrel_heat_;
                        power_heat_data.shooter_17mm_2_barrel_heat = power_heat_data_ref.shooter_17mm_2_barrel_heat_;
                        power_heat_data.shooter_42mm_barrel_heat = power_heat_data_ref.shooter_42mm_barrel_heat_;
                        power_heat_data.chassis_voltage = static_cast<uint16_t>(power_heat_data_ref.chassis_voltage_ * 0.001);        // mV->V
                        power_heat_data.chassis_current = static_cast<uint16_t>(power_heat_data_ref.chassis_current_ * 0.001);  // mA->A

                        power_heat_data.stamp = last_get_data_time_;

                        power_heat_data_pub_.publish(power_heat_data);
                        break;
                    }
                    case sp_referee::ROBOT_POS_CMD:
                    {
                        sp_referee::RobotPos robot_pos_ref;
                        memcpy(&robot_pos_ref, rx_data + 7, sizeof(sp_referee::RobotPos));
                        break;
                    }
                    case sp_referee::BUFF_CMD:
                    {
                        sp_referee::Buff referee_buff;
                        memcpy(&referee_buff, rx_data + 7, sizeof(sp_referee::Buff));
                        break;
                    }
                    case sp_referee::AIR_SUPPORT_DATA_CMD:
                    {
                        sp_referee::AirSupportData air_support_data_ref;
                        memcpy(&air_support_data_ref, rx_data + 7, sizeof(sp_referee::AirSupportData));
                        break;
                    }
                    case sp_referee::HURT_DATA_CMD:
                    {
                        sp_referee::HurtData hurt_data_ref;
                        //sp_referee::HurtDataMsg hurt_data;
                        memcpy(&hurt_data_ref, rx_data + 7, sizeof(sp_referee::HurtData));

                        // hurt_data.armor_id = hurt_ref.armor_id_;
                        // hurt_data.hp_deduction_reason = hurt_ref.hp_deduction_reason_;
                        // hurt_data.stamp = last_get_data_time_;

                        // referee_ui_.HurtDataCallBack(hurt_data, last_get_data_time_);

                        // hurt_pub_.publish(hurt_data);
                        break;
                    }
                    case sp_referee::SHOOT_DATA_CMD:
                    {
                        sp_referee::ShootData shoot_data_ref;
                        sp_referee::ShootDataMsg shoot_data;

                        memcpy(&shoot_data_ref, rx_data + 7, sizeof(sp_referee::ShootData));

                        shoot_data.bullet_type = shoot_data_ref.bullet_type_;
                        shoot_data.shooter_number = shoot_data_ref.shooter_number_;
                        shoot_data.launching_frequency = shoot_data_ref.launching_frequency_;
                        shoot_data.initial_speed = shoot_data_ref.initial_speed_;  
                        shoot_data.stamp = last_get_data_time_;

                        shoot_data_pub_.publish(shoot_data);
                        break;
                    }
                    case sp_referee::PROJECTILE_ALLOWANCE_CMD:
                    {
                        sp_referee::ProjectileAllowance projectile_allowance_ref;
                        //sp_referee::ProjectileAllowanceMsg projectile_allowance;
                        memcpy(&projectile_allowance_ref, rx_data + 7, sizeof(sp_referee::ProjectileAllowance));

                        // projectile_allowance.projojectile_allowance_17mm = projectile_allowance_ref.projojectile_allowance_17mm_;
                        // projectile_allowance.projojectile_allowance_17mm = projectile_allowance_ref.projojectile_allowance_17mm_;
                        // projectile_allowance.remaining_gold_coin = projectile_allowance_ref.remaining_gold_coin_;
                        // projectile_allowance.stamp = last_get_data_time_;

                        // projectile_allowance_pub_.publish(projectile_allowance_data);
                        break;
                    }
                    case sp_referee::RFID_STATUS_CMD:
                    {
                        sp_referee::RfidStatus rfid_status_ref;
                        //sp_referee::RfidStatusMsg rfid_status;
                        memcpy(&rfid_status_ref, rx_data + 7, sizeof(sp_referee::RfidStatus));

                        // rfid_status.rfid_status = rfid_status_ref.rfid_status_;
                        // rfid_status.stamp = last_get_data_time_;

                        // rfid_status_pub_.publish(rfid_status);
                        break;
                    }
                    case sp_referee::DART_CLIENT_CMD:
                    {
                        sp_referee::DartClientCmd dart_client_cmd_ref;
                        //sp_referee::DartClientCmdMsg dart_client_cmd_data;
                        memcpy(&dart_client_cmd_ref, rx_data + 7, sizeof(sp_referee::DartClientCmd));

                        // dart_client_cmd_data.dart_launch_opening_status = dart_client_cmd_ref.dart_launch_opening_status_;
                        // dart_client_cmd_data.target_change_time = dart_client_cmd_ref.target_change_time_;
                        // dart_client_cmd_data.latestt_launch_cmd_time = dart_client_cmd_ref.latestt_launch_cmd_time_;
                        // dart_client_cmd_data.stamp = last_get_data_time_;

                        // dart_client_cmd_pub_.publish(dart_client_cmd_data);
                        break;
                    }
                    case sp_referee::GROUND_ROBOT_POSITION_CMD:
                    {
                        sp_referee::GroundRobotPosition ground_robot_position_ref;
                        //sp_referee::GroundRobotPositionMsg ground_robot_position;
                        memcpy(&ground_robot_position_ref, rx_data + 7, sizeof(sp_referee::GroundRobotPosition));

                        // ground_robot_position.hero_x; = ground_robot_position_ref.hero_x_;
                        // ground_robot_position.hero_y; = ground_robot_position_ref.hero_y_;
                        // ground_robot_position.engineer_x; = ground_robot_position_ref.engineer_x_;
                        // ground_robot_position.engineer_y; = ground_robot_position_ref.engineer_y_;
                        // ground_robot_position.standard_3_x; = ground_robot_position_ref.standard_3_x_;
                        // ground_robot_position.standard_3_y; = ground_robot_position_ref.standard_3_y_;
                        // ground_robot_position.standard_4_x; = ground_robot_position_ref.standard_4_x_;
                        // ground_robot_position.standard_4_y; = ground_robot_position_ref.standard_4_y_;
                        // ground_robot_position.standard_5_x; = ground_robot_position_ref.standard_5_x_;
                        // ground_robot_position.standard_5_y; = ground_robot_position_ref.standard_5_y_;
  
                        // ground_robot_position_ref.stamp = last_get_data_time_;

                        // ground_robot_position_pub_.publish(ground_robot_position);
                        break;
                    }
                    case sp_referee::RADAR_MARK_DATA_CMD:
                    {
                        // sp_referee::RadarMarkData radar_mark_data_ref;
                        // sp_referee::RadarMarkDataMsg radar_mark_data;
                        // memcpy(&radar_mark_data_ref, rx_data + 7, sizeof(sp_referee::RadarMarkData));

                        // radar_mark_data.mark_hero_progress = radar_mark_data_ref.mark_hero_progress_;
                        // radar_mark_data.mark_engineer_progress = radar_mark_data_ref.mark_engineer_progress_;
                        // radar_mark_data.mark_standard_3_progress = radar_mark_data_ref.mark_standard_3_progress_;
                        // radar_mark_data.mark_standard_4_progress = radar_mark_data_ref.mark_standard_4_progress_;
                        // radar_mark_data.mark_standard_5_progress = radar_mark_data_ref.mark_standard_5_progress_;
                        // radar_mark_data.mark_sentry_progress = radar_mark_data_ref.mark_sentry_progress_;
     
                        // radar_mark_data.stamp = last_get_data_time_;

                        //radar_mark_data_pub_.publish(radar_mark_data);
                        break;
                    }
                    case sp_referee::SENTRY_INFO_CMD:
                    {
                        // sp_referee::SentryInfo sentry_info_ref;
                        // sp_referee::SentryInfoMsg sentry_info;
                        // memcpy(&sentry_info_ref, rx_data + 7, sizeof(sp_referee::SentryInfo));

                        // sentry_info.sentry_info = sentry_info_ref.sentry_info_;
                        // sentry_info.stamp = last_get_data_time_;

                        //sentry_info_pub_.publish(sentry_info);
                        break;
                    }
                    case sp_referee::RADAR_INFO_CMD:
                    {
                        // sp_referee::RadarInfo radar_info_ref;
                        // sp_referee::RadarInfoMsg radar_info;
                        // memcpy(&radar_info_ref, rx_data + 7, sizeof(sp_referee::RadarInfo));

                        // radar_info.radar_info = radar_info_ref.radar_info_;
                        // radar_info.stamp = last_get_data_time_;

                        // radar_info_pub_.publish(radar_info);
                        break;
                    }
                    case sp_referee::ROBOT_INTERACTIVE_DATA_CMD:
                    {
                        // sp_referee::InteractiveData interactive_data_ref;  // local variable temporarily before moving referee data
                        // memcpy(&interactive_data_ref, rx_data + 7, sizeof(sp_referee::InteractiveData));
                        break;
                    }
                    case sp_referee::CUSTOM_ROBOT_DATA_CMD:
                    {
                        ROS_INFO_STREAM("0X302");
                        sp_referee::CustomRobotData custom_robot_data;
                        sp_referee::CustomRobotDataMsg custom_robot_data_ref;
                        memcpy(&custom_robot_data_ref, rx_data + 7, sizeof(sp_referee::CustomRobotDataMsg));
                        
                        last_matrix = current_matrix;
                        last_time = current_time;
                        
                        double w = 0.0001*(int16_t)((custom_robot_data_ref.data[0] << 8) | custom_robot_data_ref.data[1]);
                        double x = 0.0001*(int16_t)((custom_robot_data_ref.data[2] << 8) | custom_robot_data_ref.data[3]);
                        double y = 0.0001*(int16_t)((custom_robot_data_ref.data[4] << 8) | custom_robot_data_ref.data[5]);
                        double z = 0.0001*(int16_t)((custom_robot_data_ref.data[6] << 8) | custom_robot_data_ref.data[7]);
                        Eigen::Quaterniond quat(w, x, y, z);
                        quat.normalized(); 
                        current_matrix = quat.toRotationMatrix();
                        current_time = ros::Time::now();
                        ros::Duration duration = current_time - last_time;
                        double secs = duration.toSec();
                        Eigen::Matrix3d vel_matrix = current_matrix.inverse()*(current_matrix - last_matrix) /secs;
                        //ROS_INFO_STREAM("last_matrix: \n" << last_matrix);
                        //ROS_INFO_STREAM("current_matrix: \n" << current_matrix);
                        ROS_INFO_STREAM("vel_matrix: \n" << vel_matrix);
                        custom_robot_data.wx_vel_ = (vel_matrix(2, 1) -  vel_matrix(1, 2)) / 2;
                        custom_robot_data.wy_vel_ = (vel_matrix(0, 2) -  vel_matrix(2, 0)) / 2;
                        custom_robot_data.wz_vel_ = (vel_matrix(1, 0) -  vel_matrix(0, 1)) / 2;

                        cmd_velocity.angular.x = custom_robot_data.wx_vel_;
                        cmd_velocity.angular.y = custom_robot_data.wy_vel_;
                        cmd_velocity.angular.z = custom_robot_data.wz_vel_;
                        // sensor_msgs::Imu cmd_imu;
                        // cmd_imu.orientation = cmd_quat;
                        // imu_pub_.publish(cmd_imu);
                        velocity_pub_.publish(cmd_velocity);
                        // ROS_INFO_STREAM("收到了");
                        // map_robot_data.target_robot_id = map_robot_data_ref.target_robot_id_;
                        // map_robot_data.target_position_x = map_robot_data_ref.target_position_x_;
                        // map_robot_data.target_position_y = map_robot_data_ref.target_position_y_;
                        // map_robot_data.stamp = last_get_data_time_;
                        // map_robot_data_pub_.publish(map_robot_data);
                        break;
                    }
                    case sp_referee::MAP_COMMAND_CMD:
                    {
                        sp_referee::MapCommand map_command_ref;
                        //sp_referee::MapCommandMsg map_command;
                        memcpy(&map_command_ref, rx_data + 7, sizeof(sp_referee::MapCommand));

                        // map_command.target_position_x = map_command_ref.target_position_x_;
                        // map_command.target_position_y = map_command_ref.target_position_y_;
                        // map_command.cmd_keyboard = map_command_ref.cmd_keyboard_;
                        // map_command.target_robot_id = map_command_ref.target_robot_id_;
                        // map_command.cmd_source = map_command_ref.cmd_source_;
                        // map_command.stamp = last_get_data_time_;
                        // map_command_pub_.publish(map_command);
                        break;
                    }
                    case sp_referee::REMOTE_CONTROL_CMD:
                    {
                        sp_referee::RemoteControl remote_control_ref;
                        sp_referee::RemoteControlMsg remote_control;
                        memcpy(&remote_control_ref, rx_data + 7, sizeof(sp_referee::RemoteControl));
                                            
                        // remote_control.m_x = remote_control_ref.mouse_x_;
                        // remote_control.m_y = remote_control_ref.mouse_y_;
                        // remote_control.m_z = remote_control_ref.mouse_z_;
                        // remote_control.p_l = remote_control_ref.left_button_down_;
                        // remote_control.p_r = remote_control_ref.right_button_down_;

                        // remote_control.key_w = remote_control_ref.keyboard_value_ & 0x01 ? true : false;
                        // remote_control.key_s = remote_control_ref.keyboard_value_ & 0x02 ? true : false;
                        // remote_control.key_a = remote_control_ref.keyboard_value_ & 0x04 ? true : false;
                        // remote_control.key_d = remote_control_ref.keyboard_value_ & 0x08 ? true : false;
                        // remote_control.key_shift = remote_control_ref.keyboard_value_ & 0x10 ? true : false;
                        // remote_control.key_ctrl = remote_control_ref.keyboard_value_ & 0x02 ? true : false;
                        // remote_control.key_q = remote_control_ref.keyboard_value_ & 0x40 ? true : false;
                        // remote_control.key_e = remote_control_ref.keyboard_value_ & 0x80 ? true : false;
                        // remote_control.key_r = (remote_control_ref.keyboard_value_ >> 8) & 0x01 ? true : false;
                        // remote_control.key_f = (remote_control_ref.keyboard_value_ >> 8) & 0x02 ? true : false;
                        // remote_control.key_g = (remote_control_ref.keyboard_value_ >> 8) & 0x04 ? true : false;
                        // remote_control.key_z = (remote_control_ref.keyboard_value_ >> 8) & 0x08 ? true : false;
                        // remote_control.key_x = (remote_control_ref.keyboard_value_ >> 8) & 0x10 ? true : false;
                        // remote_control.key_c = (remote_control_ref.keyboard_value_ >> 8) & 0x20 ? true : false;
                        // remote_control.key_v = (remote_control_ref.keyboard_value_ >> 8) & 0x40 ? true : false;
                        // remote_control.key_b = (remote_control_ref.keyboard_value_ >> 8) & 0x80 ? true : false;

                        // remote_control.stamp = last_get_data_time_;
                        // remote_control_pub_.publish(remote_control);
                        break;
                    }
                    default:
                    {
                        ROS_WARN("Referee command ID not found.");
                        break;
                    }
                }
                referee_data_is_online_ = true;
                last_get_data_time_ = ros::Time::now();
                return frame_len;
            }
        }
        return -1;
    }


    void Referee::getRobotInfo()
    {
        robot_info_.robot_color_ = robot_info_.robot_id_ >= 100 ? "blue" : "red";
        switch (robot_info_.robot_id_)
        {
            case sp_referee::RED_HERO:
            case sp_referee::BLUE_HERO:
                robot_info_.robot_type_ = "hero";
                break;
            case sp_referee::RED_ENGINEER:
            case sp_referee::BLUE_ENGINEER:
                robot_info_.robot_type_ = "engineer";
                break;
            case sp_referee::RED_INFANTRY_3:
            case sp_referee::RED_INFANTRY_4:
            case sp_referee::RED_INFANTRY_5:
            case sp_referee::BLUE_INFANTRY_3:
            case sp_referee::BLUE_INFANTRY_4:
            case sp_referee::BLUE_INFANTRY_5:
                robot_info_.robot_type_ = "infantry";
                break;
            case sp_referee::RED_AERIAL:
            case sp_referee::BLUE_AERIAL:
                robot_info_.robot_type_ = "aerial";
                break;
            case sp_referee::RED_SENTRY:
            case sp_referee::BLUE_SENTRY:
                robot_info_.robot_type_ = "sentry";
                break;
            case sp_referee::RED_DART:
            case sp_referee::BLUE_DART:
                robot_info_.robot_type_ = "dart";
                break;
            case sp_referee::RED_RADAR:
            case sp_referee::BLUE_RADAR:
                robot_info_.robot_type_ = "radar";
                break;
            

        }

        switch (robot_info_.robot_id_)
        {
            case sp_referee::BLUE_HERO:
                robot_info_.client_id_ = sp_referee::BLUE_HERO_CLIENT;
                break;
            case sp_referee::BLUE_ENGINEER:
                robot_info_.client_id_ = sp_referee::BLUE_ENGINEER_CLIENT;
                break;
            case sp_referee::BLUE_INFANTRY_3:
                robot_info_.client_id_ = sp_referee::BLUE_INFANTRY_3_CLIENT;
                break;
            case sp_referee::BLUE_INFANTRY_4:
                robot_info_.client_id_ = sp_referee::BLUE_INFANTRY_4_CLIENT;
                break;
            case sp_referee::BLUE_INFANTRY_5:
                robot_info_.client_id_ = sp_referee::BLUE_INFANTRY_5_CLIENT;
                break;
            case sp_referee::RED_HERO:
                robot_info_.client_id_ = sp_referee::RED_HERO_CLIENT;
                break;
            case sp_referee::RED_ENGINEER:
                robot_info_.client_id_ = sp_referee::RED_ENGINEER_CLIENT;
                break;
            case sp_referee::RED_INFANTRY_3:
                robot_info_.client_id_ = sp_referee::RED_INFANTRY_3_CLIENT;
                break;
            case sp_referee::RED_INFANTRY_4:
                robot_info_.client_id_ = sp_referee::RED_INFANTRY_4_CLIENT;
                break;
            case sp_referee::RED_INFANTRY_5:
                robot_info_.client_id_ = sp_referee::RED_INFANTRY_5_CLIENT;
                break;
        }

        robot_info_.robot_id_ = sp_referee::RED_ENGINEER;
        robot_info_.client_id_ = sp_referee::RED_ENGINEER_CLIENT;
    
    }

    void Referee::clearRxBuffer()
    {
        rx_buffer_.clear();
        rx_len_ = 0;
    }


} // namespace sp_referee