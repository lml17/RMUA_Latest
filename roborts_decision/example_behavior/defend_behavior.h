#ifndef ROBORTS_DECISION_DEFEND_BEHAVIOR_H
#define ROBORTS_DECISION_DEFEND_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../executor/gimbal_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"
#include "roborts_msgs/GimbalMode.h"
#include "roborts_msgs/GimbalAngle.h"
#include "line_iterator.h"

namespace roborts_decision {
class DefendBehavior {
 public:
  DefendBehavior(ChassisExecutor* &chassis_executor,
                GimbalExecutor* &gimbal_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       gimbal_executor_(gimbal_executor),
                                                       blackboard_(blackboard) {

  }
void Run() {
     printf("defend==================================================run");
     ros::NodeHandle n;
   // ros::ServiceClient client = n.serviceClient<roborts_msgs::GimbalMode>("set_gimbal_mode");
//
 //   roborts_msgs::GimbalMode gimbalMode;
    
   // if (!blackboard_->info.has_buff){
     geometry_msgs::PoseStamped my = blackboard_->GetRobotMapPose();
    
    blackboard_->info.last_position=my;
   

     ros::Rate rate(5);
     int i = 1;
      while (ros::ok())
      {
         /**
        ROS_INFO("======Gimbal Behavior.h============");
	roborts_msgs::GimbalAngle gimbal_angle;
	//#gimbal feedback angle data
	//bool yaw_mode
	//bool pitch_mode
	//float64 yaw_angle
	//float64 pitch_angle
      
      gimbal_angle.yaw_mode = false;
      gimbal_angle.pitch_mode = false;
      gimbal_angle.yaw_angle = 0.5;
      gimbal_angle.pitch_angle = 0;


      roborts_msgs::GimbalRate gimbal_rate;
      gimbal_rate.yaw_rate = 0;

        gimbal_executor_->Execute(gimbal_angle);
       **/
//#gimbal feedback angle data
//bool yaw_mode
//bool pitch_mode
//float64 yaw_angle
//float64 pitch_angle

     // gimbal_rate.pitch_rate = 0;
           /**
          auto quaternion = tf::createQuaternionMsgFromYaw(M_PI/3);
          auto quaternion1 = tf::createQuaternionMsgFromYaw(-M_PI/3);
         
        
        rate.sleep();
        ROS_INFO("Is swing");
        geometry_msgs::PoseStamped myPose;
         myPose.header.frame_id = "gimbal";
          myPose.header.stamp = ros::Time::now();
        myPose.pose.orientation = quaternion;
        chassis_executor_->Execute(myPose);
        rate.sleep();
        myPose.pose.orientation = quaternion1;
        chassis_executor_->Execute(myPose);
        rate.sleep();
        ROS_INFO("twist1.angular = end;");
         **/
        
        geometry_msgs::Twist twist;
	geometry_msgs::Vector3 linear;
	linear.x=0;
	linear.y=0;
	linear.z=0;
	geometry_msgs::Vector3 angular;
	angular.x=0;
	angular.y=0;
        if(i==1){
		angular.z= -M_PI/2;
        }else{
             angular.z= -M_PI;
        }
	//printf("twist1.angular = -3.14;\n");
	ROS_INFO("twist1.angular = -3.14;");
	twist.linear = linear;
	twist.angular = angular;

	chassis_executor_->Execute(twist);
	//sleep(1);
       rate.sleep();
       angular.z= M_PI;
       
	twist.angular = angular;
	ROS_INFO("twist1.angular = 3.14;");
	chassis_executor_->Execute(twist);
	rate.sleep();
    	i+=1;
        if(i>2){
               break;
        }
        
      }
  }
/**
  void Run() {
//printf("search_behavior=======run()\n");
      auto executor_state = Update();
      float move_dst = 0.20;               //车身左右移动的距离
      geometry_msgs::PoseStamped swing_goal;
      swing_goal.header.frame_id = "map";
      swing_goal = blackboard_->GetRobotMapPose();
      ROS_INFO("Got pose.");
      tf::Quaternion cur_q;
      tf::quaternionMsgToTF(swing_goal.pose.orientation, cur_q);
      double r, p, y;
      tf::Matrix3x3(cur_q).getRPY(r, p, y); // 从位置信息中获取车身的角度
      float move_x = asin(y) * move_dst;    // x轴方向上移动距离
      float move_y = -acos(y) * move_dst;   // y轴方向上移动距离
      swing_goal.pose.position.z = 0;
      ros::Rate rate(20);
      while (ros::ok() )
      {
        ROS_INFO("Is swing");
        // 更新 x, y 轴坐标
        ROS_INFO("position.x:%f",swing_goal.pose.position.x);
        ROS_INFO("position.y:%f",swing_goal.pose.position.y);
        ROS_INFO("move_x:%f",move_x);
        ROS_INFO("move_y:%f",move_y);
        swing_goal.pose.position.x += move_x;
        swing_goal.pose.position.y += move_y;
        chassis_executor_->Execute(swing_goal);
        rate.sleep();
        swing_goal.pose.position.x -= move_x;
        swing_goal.pose.position.y -= move_y;
        chassis_executor_->Execute(swing_goal);
        rate.sleep();
      } 
    
  }
**/
  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }
     
    for (int i=0; i<decision_config.search_region_1().size();i++)
        search_region_1_.push_back(blackboard_->Point2PoseStamped(decision_config.search_region_1(i)));

    return true;
  }

  void SetLastPosition(geometry_msgs::PoseStamped last_position) {
    last_position_ = last_position;
    search_count_ = 5;
  }

  ~DefendBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;
  GimbalExecutor* const gimbal_executor_;
  //! perception information
  Blackboard* const blackboard_;

  //! chase goal
  geometry_msgs::PoseStamped last_position_;

  //! search buffer
  std::vector<geometry_msgs::PoseStamped> search_region_1_;
  std::vector<geometry_msgs::PoseStamped> search_region_2_;
  std::vector<geometry_msgs::PoseStamped> search_region_3_;
  std::vector<geometry_msgs::PoseStamped> search_region_4_;
  std::vector<geometry_msgs::PoseStamped> search_region_;
  unsigned int search_count_;
  unsigned int search_index_;
  bool SearchBegin;

};
}

#endif //ROBORTS_DECISION_SWING_BEHAVIOR_H
