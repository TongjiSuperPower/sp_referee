//
// Created by CherryBlossomNight on 2024/1/18
// Updated to be compatible with the referee system serial protocol manual v 1.6.1
//

#include "sp_referee/sp_referee.h"

int main(int argc, char** argv)
{
  std::string robot;
  ros::init(argc, argv, "sp_referee");  // sp_referee
  ros::NodeHandle nh("~");
  sp_referee::Referee referee;
  ros::Rate loop_rate(80);
  referee.init();
  while (ros::ok())
  {
    
    referee.read();
    // if (referee.getImageTrasmission())
    //   referee.readImageTrasmission();
    ros::spinOnce();
    //referee.write();
    //referee.sendUi(); 
    referee.sendString();
    loop_rate.sleep();
  }

  return 0;
}