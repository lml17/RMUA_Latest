#include <ros/ros.h>
#include "executor/chassis_executor.h"
#include "executor/gimbal_executor.h"
#include "example_behavior/gimbal_behavior.h"
#include "example_behavior/shoot_behavior.h"
#include "sel_behavior_node.h"
#include "roborts_msgs/ShootWhl.h"

void gimbalControlCallback(const roborts_msgs::GimbalAngle::ConstPtr &msg, roborts_decision::GimbalBehavior* gimbal_behavior);
void ShootControlCallback(const roborts_msgs::ShootWhl::ConstPtr &msg, roborts_decision::ShootBehavior* shoot_behavior);

int main(int argc, char **argv) {
  ros::init(argc, argv, "sel_behavior_node"); 
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/sel_behavior.prototxt";
  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto gimbal_executor = new roborts_decision::GimbalExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);
  roborts_decision::GimbalBehavior gimbal_behavior(chassis_executor,gimbal_executor, blackboard, full_path);
  roborts_decision::ShootBehavior shoot_behavior(chassis_executor, blackboard, full_path);

//  ros::Rate rate(10);
  ros::NodeHandle n;
//  ros::Subscriber sub = n.subscribe<roborts_msgs::GimbalAngle>("debug_gimbalAngle", 10, boost::bind(&gimbalControlCallback, _1, &gimbal_behavior));
  ros::Subscriber sub = n.subscribe<roborts_msgs::ShootWhl>("debug_Shoot", 10, boost::bind(&ShootControlCallback, _1, &shoot_behavior));  

  while(ros::ok()){
    ros::spinOnce();
  }

    
    // switch (command) {
    //   // test behavior
    //   case '0':
    //     ambush_behavior.Run();
    //     printf("State: SWING\n");
    //     break;
    //   case '5':
    //     ROS_FATAL("CASE5");
    //     gimbal_behavior.getYawandPitch();
    //     break; 
    //   case 27:
    //     if (command_thread.joinable()){
    //       command_thread.join();
    //     }
    //     return 0;
    //   default:
    //     break;
    // }
    // rate.sleep();



  return 0;
}
void gimbalControlCallback(const roborts_msgs::GimbalAngle::ConstPtr &msg, roborts_decision::GimbalBehavior* gimbal_behavior)
{  
    roborts_msgs::GimbalAngle temp = *msg;
    if((temp.yaw_angle == 2) && (temp.pitch_angle == 0.25)){   // 
	      roborts_msgs::GimbalAngle pos_now;
        gimbal_behavior->getYawandPitch(pos_now);
        ROS_FATAL("yaw=%f, pitch=%f",pos_now.yaw_angle, pos_now.pitch_angle);
  
    }
    else{
        gimbal_behavior->Run(temp);
    }
}
void ShootControlCallback(const roborts_msgs::ShootWhl::ConstPtr &msg, roborts_decision::ShootBehavior* shoot_behavior){
  roborts_msgs::ShootWhl temp = *msg;
  roborts_msgs::FricWhl fricwhl;
  roborts_msgs::ShootCmd shootcmd;
  fricwhl.request.open = temp.wheelopen;
  fricwhl.request.wheelspeed = temp.wheelspeed;
  shootcmd.request.mode = temp.mode;
  shootcmd.request.number = temp.number;
  shootcmd.request.shoot_freq = temp.shoot_freq;

  shoot_behavior->RunFricWhl(fricwhl);
  shoot_behavior->RunShoot(shootcmd);
  ROS_FATAL("wheelspeed=%d", temp.wheelspeed);
}



// void Command() {
//   while (command != 27) {
//     std::cout << "**************************************************************************************" << std::endl;
//     std::cout << "*********************************please send a command********************************" << std::endl;   
//     std::cout << "0:  behavior" << std::endl
//               << "1: gimbal_behavior" << std::endl
//               // << "2: back boot area behavior" << std::endl
//               // << "3: back_behavior" << std::endl
//               // << "4: search behavior" << std::endl
//               // << "5: turnToDetectedDirection behavior" << std::endl
//               // << "6: goal_behavior" << std::endl
//               // << "7: test_behavior behavior" << std::endl
//               // << "8: shield behavior" << std::endl
//               // << "9: attack behavior" << std::endl
//               // << "10: back boot area behavior" << std::endl
//               << "esc: exit program" << std::endl;
//     std::cout << "**************************************************************************************" << std::endl;
//     std::cout << "> ";
//     std::cin >> command;
//     // if (command != '0' &&  command != '1' && command != '2' && command != '3' && command != '4' && command != '5' && command != '6' && 
//     //     && command != "7" && command != "8" && command != "9" && command != 27) {
//     //   std::cout << "please input again!" << std::endl;
//     //   std::cout << "> ";
//     //   std::cin >> command;
//     // }

//   }
// }
