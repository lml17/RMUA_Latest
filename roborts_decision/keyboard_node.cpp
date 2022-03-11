#include <ros/ros.h>
#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/ShootWhl.h"

int main(int argc, char ** argv){
    ros::init(argc, argv, "keyboard");
    ros::NodeHandle nh;

    // roborts_msgs::GimbalAngle gimbal_angle;         // 云台位置指令
    // gimbal_angle.yaw_mode = false;
    // gimbal_angle.pitch_mode = false;
    // gimbal_angle.yaw_angle = 0.5;
    // gimbal_angle.pitch_angle = 0;

//    ros::Publisher pub = nh.advertise<roborts_msgs::GimbalAngle>("debug_gimbalAngle", 10);
    roborts_msgs::ShootWhl shootwhl;
    ros::Publisher pub = nh.advertise<roborts_msgs::ShootWhl>("debug_Shoot", 10);

    ros::Rate loop_rate(1.0);
    while (ros::ok())
    {
        std::cout << "Please Input: " << std::endl;
        // std::cin >> gimbal_angle.yaw_angle >> gimbal_angle.pitch_angle; // 读入信息
        int a,b,c,d,e;
    
        std::cin >> a >> b >> c >> d >> e;
        shootwhl.wheelopen = a;
        shootwhl.wheelspeed =  b;
        shootwhl.mode =  c;
        shootwhl.number =  d;
        shootwhl.shoot_freq =  e;
        // std::cout << "Input Over!" << std::endl;
        pub.publish(shootwhl);

        loop_rate.sleep();
    }
    
    return 0;
}