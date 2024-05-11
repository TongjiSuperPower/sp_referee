#include "sp_referee/sp_referee.h"
#include <XmlRpcException.h>
namespace sp_referee
{
    void Referee::initUI(XmlRpc::XmlRpcValue &ui, std::string type)
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

            if (graph->second.hasMember("frequency"))
                config.frequency_ = static_cast<int>(graph->second["frequency"]);
            if (type == "string")
                string_ui_.push_back(config);
            else if (type == "lines")
                lines_ui_.push_back(config);
            else if (type == "graphs")
                graphs_ui_.push_back(config);
        }

    }


    void Referee::sendGraphs()
    {
       if (frequency_num_ % 23 == 0)
        {
            int data_len = static_cast<int>(sizeof(sp_referee::InteractionFigureTwo));
            int frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
            sp_referee::InteractionFigureTwo figure_graphs{};
            figure_graphs.robot_interaction_data_header_.data_cmd_id_ = sp_referee::INTERACTION_FIGURE_TWO_CMD;
            figure_graphs.robot_interaction_data_header_.sender_id_ = sp_referee::RED_ENGINEER;
            figure_graphs.robot_interaction_data_header_.receiver_id_ = sp_referee::RED_ENGINEER_CLIENT;
            for (int i = 0; i < 2; i++)
            {
                figure_graphs.interaction_figure_[i].figure_name_[0] = graphs_ui_[i].figure_id_[0];
                figure_graphs.interaction_figure_[i].figure_name_[1] = graphs_ui_[i].figure_id_[1];
                figure_graphs.interaction_figure_[i].figure_name_[2] = graphs_ui_[i].figure_id_[2];
                
                figure_graphs.interaction_figure_[i].figure_type_ = graphs_ui_[i].figure_type_;

                if (i == 0)
                {
                    if (cmd_pump_.data)
                        addUI(graphs_ui_[i]);
                    else
                        deleteUI(graphs_ui_[i]);    
                }
                else if (i == 1)
                {
                    if (cmd_rod_.data)
                        addUI(graphs_ui_[i]);
                    else
                        deleteUI(graphs_ui_[i]);    
                }
                

                figure_graphs.interaction_figure_[i].operate_type_ = graphs_ui_[i].operate_type_;

                figure_graphs.interaction_figure_[i].layer_ = graphs_ui_[i].layer_;
                figure_graphs.interaction_figure_[i].color_ = graphs_ui_[i].color_;

                figure_graphs.interaction_figure_[i].width_ = graphs_ui_[i].width_;


                figure_graphs.interaction_figure_[i].start_x_ = graphs_ui_[i].center_x_;
                figure_graphs.interaction_figure_[i].start_y_ = graphs_ui_[i].center_y_;

                figure_graphs.interaction_figure_[i].details_c_ = graphs_ui_[i].radius_;
            }

    
            
            pack(reinterpret_cast<uint8_t*>(&tx_buffer_), reinterpret_cast<uint8_t*>(&figure_graphs), sp_referee::ROBOT_INTERACTIVE_DATA_CMD, data_len);

            try
            {
                serial_.write(tx_buffer_, frame_len);
            }
            catch (serial::PortNotOpenedException& e)
            {

            }
        }
    }

     void Referee::sendLines()
    {
        if (frequency_num_ % 51 == 0)
        {
            int data_len = static_cast<int>(sizeof(sp_referee::InteractionFigureSeven));
            int frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
            sp_referee::InteractionFigureSeven figure_lines{};
            figure_lines.robot_interaction_data_header_.data_cmd_id_ = sp_referee::INTERACTION_FIGURE_SEVEN_CMD;
            figure_lines.robot_interaction_data_header_.sender_id_ = sp_referee::RED_ENGINEER;
            figure_lines.robot_interaction_data_header_.receiver_id_ = sp_referee::RED_ENGINEER_CLIENT;
            for (int i = 0; i < lines_ui_.size(); i++)
            {
                figure_lines.interaction_figure_[i].figure_name_[0] = lines_ui_[i].figure_id_[0];
                figure_lines.interaction_figure_[i].figure_name_[1] = lines_ui_[i].figure_id_[1];
                figure_lines.interaction_figure_[i].figure_name_[2] = lines_ui_[i].figure_id_[2];
                
                figure_lines.interaction_figure_[i].figure_type_ = lines_ui_[i].figure_type_;

                    
                figure_lines.interaction_figure_[i].operate_type_ = lines_ui_[i].operate_type_;

                figure_lines.interaction_figure_[i].layer_ = lines_ui_[i].layer_;
                figure_lines.interaction_figure_[i].color_ = lines_ui_[i].color_;

                figure_lines.interaction_figure_[i].width_ = lines_ui_[i].width_;


                figure_lines.interaction_figure_[i].start_x_ = lines_ui_[i].start_x_;
                figure_lines.interaction_figure_[i].start_y_ = lines_ui_[i].start_y_;

                figure_lines.interaction_figure_[i].details_d_ = lines_ui_[i].end_x_;
                figure_lines.interaction_figure_[i].details_e_ = lines_ui_[i].end_y_;
            }

    
            
            pack(reinterpret_cast<uint8_t*>(&tx_buffer_), reinterpret_cast<uint8_t*>(&figure_lines), sp_referee::ROBOT_INTERACTIVE_DATA_CMD, data_len);

            try
            {
                serial_.write(tx_buffer_, frame_len);
            }
            catch (serial::PortNotOpenedException& e)
            {

            }
        }

    }

    void Referee::sendCorner()
    {
         int data_len = static_cast<int>(sizeof(sp_referee::InteractionFigureSeven));
            int frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
            sp_referee::InteractionFigureSeven figure_lines{};
            figure_lines.robot_interaction_data_header_.data_cmd_id_ = sp_referee::INTERACTION_FIGURE_SEVEN_CMD;
            figure_lines.robot_interaction_data_header_.sender_id_ = sp_referee::RED_ENGINEER;
            figure_lines.robot_interaction_data_header_.receiver_id_ = sp_referee::RED_ENGINEER_CLIENT;
            for (int i = 0; i < 4; i++)
            {
                figure_lines.interaction_figure_[i].figure_name_[0] = 0x05;
                figure_lines.interaction_figure_[i].figure_name_[1] = 0x00;
                figure_lines.interaction_figure_[i].figure_name_[2] = i;
                
                figure_lines.interaction_figure_[i].figure_type_ = sp_referee::GraphType::LINE;

                    
                figure_lines.interaction_figure_[i].operate_type_ = sp_referee::GraphOperation::UPDATE;

                figure_lines.interaction_figure_[i].layer_ = 0;
                figure_lines.interaction_figure_[i].color_ = sp_referee::GraphColor::WHITE;

                figure_lines.interaction_figure_[i].width_ = 2;
            }
            if (vision_corner_.data.empty())


                figure_lines.interaction_figure_[0].start_x_ = int(vision_corner_.data[0] / 2);
                figure_lines.interaction_figure_[0].start_y_ = int(vision_corner_.data[1] / 2);

                figure_lines.interaction_figure_[0].details_d_ = int(vision_corner_.data[2] / 2);
                figure_lines.interaction_figure_[0].details_e_ = int(vision_corner_.data[3] / 2);

                figure_lines.interaction_figure_[1].start_x_ = int(vision_corner_.data[2] / 2);
                figure_lines.interaction_figure_[1].start_y_ = int(vision_corner_.data[3] / 2);

                figure_lines.interaction_figure_[1].details_d_ = int(vision_corner_.data[4] / 2);
                figure_lines.interaction_figure_[1].details_e_ = int(vision_corner_.data[5] / 2);

                figure_lines.interaction_figure_[2].start_x_ = int(vision_corner_.data[4] / 2);
                figure_lines.interaction_figure_[2].start_y_ = int(vision_corner_.data[5] / 2);

                figure_lines.interaction_figure_[2].details_d_ = int(vision_corner_.data[6] / 2);
                figure_lines.interaction_figure_[2].details_e_ = int(vision_corner_.data[7] / 2);

                figure_lines.interaction_figure_[3].start_x_ = int(vision_corner_.data[6] / 2);
                figure_lines.interaction_figure_[3].start_y_ = int(vision_corner_.data[7] / 2);

                figure_lines.interaction_figure_[3].details_d_ = int(vision_corner_.data[0] / 2);
                figure_lines.interaction_figure_[3].details_e_ = int(vision_corner_.data[1] / 2);

    
            
            pack(reinterpret_cast<uint8_t*>(&tx_buffer_), reinterpret_cast<uint8_t*>(&figure_lines), sp_referee::ROBOT_INTERACTIVE_DATA_CMD, data_len);

            try
            {
                serial_.write(tx_buffer_, frame_len);
            }
            catch (serial::PortNotOpenedException& e)
            {

            }

    }

    void Referee::sendString()
    {
        frequency_num_++;  
        if (frequency_num_ == 1000)
            frequency_num_ = 0;
        for (int i = 0; i < string_ui_.size(); i++)
        {
            if (frequency_num_ % string_ui_[i].frequency_ == 0)
            {
                int data_len = static_cast<int>(sizeof(sp_referee::ExtClientCustomCharacter));
                int frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
                sp_referee::ExtClientCustomCharacter figure_string{};
                figure_string.robot_interaction_data_header_.data_cmd_id_ = sp_referee::EXT_CLIENT_CUSTOM_CHARACTER_CMD;
                figure_string.robot_interaction_data_header_.sender_id_ = sp_referee::RED_ENGINEER;
                figure_string.robot_interaction_data_header_.receiver_id_ = sp_referee::RED_ENGINEER_CLIENT;

                figure_string.interaction_figure_.figure_name_[0] = string_ui_[i].figure_id_[0];
                figure_string.interaction_figure_.figure_name_[1] = string_ui_[i].figure_id_[1];
                figure_string.interaction_figure_.figure_name_[2] = string_ui_[i].figure_id_[2];
                
                figure_string.interaction_figure_.figure_type_ = string_ui_[i].figure_type_;

                    
                if (frequency_num_ % (10 * string_ui_[i].frequency_) == 0)
                    addUI(string_ui_[i]);
                else    
                    updateUI(string_ui_[i]);

                figure_string.interaction_figure_.operate_type_ = string_ui_[i].operate_type_;

                figure_string.interaction_figure_.layer_ = string_ui_[i].layer_;
                figure_string.interaction_figure_.color_ = string_ui_[i].color_;

                figure_string.interaction_figure_.width_ = string_ui_[i].width_;


                figure_string.interaction_figure_.start_x_ = string_ui_[i].start_x_;
                figure_string.interaction_figure_.start_y_ = string_ui_[i].start_y_;

                figure_string.interaction_figure_.details_a_ = string_ui_[i].size_;
                figure_string.interaction_figure_.details_b_ = string_ui_[i].length_;

                for (int j = 0; j < string_ui_[i].length_; j++)
                    figure_string.data_[j] = string_ui_[i].context_[j];
                if (string_ui_[i].context_ == "PITCH" && !joint_pos_.data.empty())
                {
                    double pitch_pos = joint_pos_.data[5];
                   
                    uint8_t sign = (pitch_pos < 0)? '-': ' ';
                    
                    pitch_pos = abs(pitch_pos);
                    int32_t digit = (int(pitch_pos) % 10);
                    int32_t tenth = (int(pitch_pos * 10) % 10);
                    int32_t hunth = (int(pitch_pos * 100) % 10);
                
                    figure_string.data_[string_ui_[i].length_ + 0] = ':';
                    figure_string.data_[string_ui_[i].length_ + 1] = sign;
                    figure_string.data_[string_ui_[i].length_ + 2] = digit + '0';
                    figure_string.data_[string_ui_[i].length_ + 3] = '.';
                    figure_string.data_[string_ui_[i].length_ + 4] = tenth + '0';
                    figure_string.data_[string_ui_[i].length_ + 5] = hunth + '0';
                    figure_string.interaction_figure_.details_b_ = string_ui_[i].length_ + 6;
                }
                pack(reinterpret_cast<uint8_t*>(&tx_buffer_), reinterpret_cast<uint8_t*>(&figure_string), sp_referee::ROBOT_INTERACTIVE_DATA_CMD, data_len);

                try
                {
                    serial_.write(tx_buffer_, frame_len);
                }
                catch (serial::PortNotOpenedException& e)
                {

                }
            }
        }

        clearTxBuffer();
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