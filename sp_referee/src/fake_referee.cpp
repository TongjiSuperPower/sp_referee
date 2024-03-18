#include "sp_referee/sp_referee.h"

#include "sp_referee/GameRobotStatusMsg.h"
#include "sp_referee/PowerHeatDataMsg.h"
#include "sp_referee/ShootDataMsg.h"

int main(int argc, char** argv)
{
    std::string robot;
    ros::init(argc, argv, "sp_referee");  // sp_referee
    ros::NodeHandle nh("referee");

    ros::Publisher game_robot_status_pub;
    ros::Publisher power_heat_data_pub;
    ros::Publisher shoot_data_pub;

    sp_referee::GameRobotStatusMsg game_robot_status;
    sp_referee::PowerHeatDataMsg power_heat_data;
    sp_referee::ShootDataMsg shoot_data;


    while (ros::ok())
    {
        
        referee.pub();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}