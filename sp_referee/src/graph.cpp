#include "sp_referee/sp_referee.h"
#include <XmlRpcException.h>
namespace sp_referee
{
    void Referee::initUI(XmlRpc::XmlRpcValue &ui, std::string type)
    {
        if (type == "mode")
        {
            for (auto graph = ui.begin(); graph != ui.end(); graph++)
            {
                UIConfig config;
                int id = int(graph->second["id"]);
                config.figure_id_[0] = (id >> 0 & 0xFF);
                config.figure_id_[1] = (id >> 8 & 0xFF);
                config.figure_id_[2] = (id >> 16 & 0xFF);
              
                if (graph->second.hasMember("figure_type"))
                    config.figure_type_ =  getType(graph->second["figure_type"]);

                if (graph->second.hasMember("layer"))
                    config.layer_ = static_cast<int>(graph->second["layer"]);

                if (graph->second.hasMember("color"))
                    config.color_ = getColor(graph->second["color"]);


                if (graph->second.hasMember("width"))
                    config.width_ = static_cast<int>(graph->second["width"]);
                
                if (graph->second.hasMember("start_position")) // except circle
                {
                    config.start_x_ = static_cast<int>(graph->second["start_position"][0]);
                    config.start_y_ = static_cast<int>(graph->second["start_position"][1]);
                }

               if (graph->second.hasMember("center_position")) // circle
                {
                    config.center_x_ = static_cast<int>(graph->second["center_position"][0]);
                    config.center_y_ = static_cast<int>(graph->second["center_position"][1]);
                }

               if (graph->second.hasMember("end_position")) // for line and rectangle
                {
                    config.end_x_ = static_cast<int>(graph->second["end_position"][0]);
                    config.end_y_ = static_cast<int>(graph->second["end_position"][1]);
                }

                if (graph->second.hasMember("angle")) // for arc
                {
                    config.start_angle_ = static_cast<int>(graph->second["angle"][0]);
                    config.end_angle_ = static_cast<int>(graph->second["angle"][1]);
                }

                 if (graph->second.hasMember("semi_axis")) // for ellipse and arc
                {
                    config.semi_x_axis_ = static_cast<int>(graph->second["semi_axis"][0]);
                    config.semi_y_axis_ = static_cast<int>(graph->second["semi_axis"][1]);
                }

            
                if (graph->second.hasMember("size")) // for float, int and string
                    config.size_ = static_cast<int>(graph->second["size"]);

                if (graph->second.hasMember("length")) // for string
                    config.length_ = static_cast<int>(graph->second["length"]);

                if (graph->second.hasMember("radius")) // for circle
                    config.radius_ = static_cast<int>(graph->second["radius"]);
            

                // if (config.hasMember("delay"))
                //     delay_ = ros::Duration(static_cast<double>(config["delay"]));
                // if (config.hasMember("title"))
                //     title_ = static_cast<std::string>(config["title"]);
                if (graph->second.hasMember("context"))
                    config.context_ = static_cast<std::string>(graph->second["context"]);
                config.operate_type_ = sp_referee::GraphOperation::ADD;
                mode_ui_.push_back(config);
            }
        }

    }

    void Referee::sendUi()
    {
        int data_len = static_cast<int>(sizeof(sp_referee::InteractionFigureOne));
        int frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
        sp_referee::InteractionFigureOne figure_one{};
        figure_one.robot_interaction_data_header_.data_cmd_id_ = sp_referee::INTERACTION_FIGURE_CMD;
        figure_one.robot_interaction_data_header_.sender_id_ = robot_info_.robot_id_;
        figure_one.robot_interaction_data_header_.receiver_id_ = robot_info_.client_id_;
   
        //ROS_INFO_STREAM("aaa"<< robot_info_.robot_id_<< " " << figure_one.robot_interaction_data_header_.sender_id_);


        figure_one.interaction_figure_.figure_name_[0] = mode_ui_[0].figure_id_[0];
        figure_one.interaction_figure_.figure_name_[1] = mode_ui_[0].figure_id_[1];
        figure_one.interaction_figure_.figure_name_[2] = mode_ui_[0].figure_id_[2];
        figure_one.interaction_figure_.operate_type_ = mode_ui_[0].operate_type_;
        figure_one.interaction_figure_.figure_type_ = mode_ui_[0].figure_type_;
   

        figure_one.interaction_figure_.layer_ = mode_ui_[0].layer_;
        figure_one.interaction_figure_.color_ = mode_ui_[0].color_;

        figure_one.interaction_figure_.width_ = mode_ui_[0].width_;

        figure_one.interaction_figure_.start_x_ = mode_ui_[0].start_x_;
        figure_one.interaction_figure_.start_y_ = mode_ui_[0].start_y_;

        figure_one.interaction_figure_.details_c_ = mode_ui_[0].radius_;


        pack(reinterpret_cast<uint8_t*>(&tx_buffer_), reinterpret_cast<uint8_t*>(&figure_one), sp_referee::ROBOT_INTERACTIVE_DATA_CMD, data_len);


        try
        {
            serial_.write(tx_buffer_, frame_len);
        }
        catch (serial::PortNotOpenedException& e)
        {

        }

        clearTxBuffer();
    }

    void Referee::sendString()
    {
        for (int i = 0; i < mode_ui_.size(); i++)
        {
            int data_len = static_cast<int>(sizeof(sp_referee::ExtClientCustomCharacter));
            int frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
            sp_referee::ExtClientCustomCharacter figure_string{};
            figure_string.robot_interaction_data_header_.data_cmd_id_ = sp_referee::EXT_CLIENT_CUSTOM_CHARACTER_CMD;
            figure_string.robot_interaction_data_header_.sender_id_ = robot_info_.robot_id_;
            figure_string.robot_interaction_data_header_.receiver_id_ = robot_info_.client_id_;

            figure_string.interaction_figure_.figure_name_[0] = mode_ui_[i].figure_id_[0];
            figure_string.interaction_figure_.figure_name_[1] = mode_ui_[i].figure_id_[1];
            figure_string.interaction_figure_.figure_name_[2] = mode_ui_[i].figure_id_[2];
           
            figure_string.interaction_figure_.figure_type_ = mode_ui_[i].figure_type_;

                
            
  

            figure_string.interaction_figure_.operate_type_ = mode_ui_[i].operate_type_;
    
            figure_string.interaction_figure_.layer_ = mode_ui_[i].layer_;
            figure_string.interaction_figure_.color_ = mode_ui_[i].color_;

            figure_string.interaction_figure_.width_ = mode_ui_[i].width_;


            figure_string.interaction_figure_.start_x_ = mode_ui_[i].start_x_;
            figure_string.interaction_figure_.start_y_ = mode_ui_[i].start_y_;

            figure_string.interaction_figure_.details_a_ = mode_ui_[i].size_;
            figure_string.interaction_figure_.details_b_ = mode_ui_[i].length_;



            for (int j = 0; j < mode_ui_[i].length_; j++)
                figure_string.data_[j] = mode_ui_[i].context_[j];

                    

            // if (i == 1)
            // {
            //     if (manipulator_cmd_.control_mode = MAUL)
            //     {
            //         figure_string.data_[12] = 'M';
            //         figure_string.data_[13] = 'A';
            //         figure_string.data_[14] = 'U';
            //         figure_string.data_[15] = 'L';
            //         figure_string.interaction_figure_.details_b_ = 16;
            //     }
            //     else if (manipulator_cmd_.control_mode = AUTO)
            //     {
            //         figure_string.data_[12] = 'A';
            //         figure_string.data_[13] = 'U';
            //         figure_string.data_[14] = 'T';
            //         figure_string.data_[15] = 'O';
            //         figure_string.interaction_figure_.details_b_ = 16;
            //     }
            //     else if (manipulator_cmd_.control_mode = JOINT)
            //     {
            //         figure_string.data_[12] = 'J';
            //         figure_string.data_[13] = 'O';
            //         figure_string.data_[14] = 'I';
            //         figure_string.data_[15] = 'N';
            //         figure_string.data_[15] = 'T';
            //         figure_string.interaction_figure_.details_b_ = 17;
            //     }
            //     else if (manipulator_cmd_.control_mode = CALI)
            //     {
            //         figure_string.data_[12] = 'C';
            //         figure_string.data_[13] = 'A';
            //         figure_string.data_[14] = 'L';
            //         figure_string.data_[15] = 'I';
            //         figure_string.interaction_figure_.details_b_ = 16;
            //     }
            // }

            pack(reinterpret_cast<uint8_t*>(&tx_buffer_), reinterpret_cast<uint8_t*>(&figure_string), sp_referee::ROBOT_INTERACTIVE_DATA_CMD, data_len);


            try
            {
                serial_.write(tx_buffer_, frame_len);
            }
            catch (serial::PortNotOpenedException& e)
            {

            }

            clearTxBuffer();
        }
   
 
    }

    void Referee::addUI(UIConfig &config)
    {
        config.operate_type_ = sp_referee::GraphOperation::ADD;
    }

    void Referee::updateUI(UIConfig &config)
    {
        config.operate_type_ = sp_referee::GraphOperation::UPDATE;
    }

    void Referee::deleteUI(UIConfig &config)
    {
        config.operate_type_ = sp_referee::GraphOperation::DELETE;
    }

    sp_referee::GraphColor Referee::getColor(const std::string& color)
    {
        if (color == "main_color")
            return sp_referee::GraphColor::MAIN_COLOR;
        else if (color == "yellow")
            return sp_referee::GraphColor::YELLOW;
        else if (color == "green")
            return sp_referee::GraphColor::GREEN;
        else if (color == "orange")
            return sp_referee::GraphColor::ORANGE;
        else if (color == "purple")
            return sp_referee::GraphColor::PURPLE;
        else if (color == "pink")
            return sp_referee::GraphColor::PINK;
        else if (color == "cyan")
            return sp_referee::GraphColor::CYAN;
        else if (color == "black")
            return sp_referee::GraphColor::BLACK;
        else
            return sp_referee::GraphColor::WHITE;
    }

    sp_referee::GraphType Referee::getType(const std::string& type)
    {
        if (type == "rectangle")
            return sp_referee::GraphType::RECTANGLE;
        else if (type == "circle")
            return sp_referee::GraphType::CIRCLE;
        else if (type == "ellipse")
            return sp_referee::GraphType::ELLIPSE;
        else if (type == "arc")
            return sp_referee::GraphType::ARC;
        else if (type == "string")
            return sp_referee::GraphType::STRING;
        else
            return sp_referee::GraphType::LINE;
    }
} // namespace sp_referee