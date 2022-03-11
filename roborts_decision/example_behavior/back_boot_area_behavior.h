#ifndef ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H
#define ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class BackBootAreaBehavior {
 public:
  BackBootAreaBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {


    boot_position_.header.frame_id = "map";
    boot_position_.pose.orientation.x = 0;
    boot_position_.pose.orientation.y = 0;
    boot_position_.pose.orientation.z = 0;
    boot_position_.pose.orientation.w = 1;

    boot_position_.pose.position.x = 0;
    boot_position_.pose.position.y = 0;
    boot_position_.pose.position.z = 0;

    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

  }

  void Run() {

    auto executor_state = Update();
    geometry_msgs::PoseStamped my = blackboard_->GetRobotMapPose();
    geometry_msgs::PoseStamped goal = my;
    ROS_INFO("my.x:%f",my.pose.position.x);
    ROS_INFO("my.y:%f",my.pose.position.y);
    geometry_msgs::PoseStamped last_position = blackboard_->info.last_position;
    ROS_INFO("last_position.x:%f",last_position.pose.position.x);
    ROS_INFO("last_position.y:%f",last_position.pose.position.y);
    if (abs(last_position.pose.position.x - my.pose.position.x)>1||abs(last_position.pose.position.y - my.pose.position.y)>1){
            chassis_executor_->Cancel();
            ROS_INFO("stop back------------------");
            blackboard_->info.is_begin = true;
            
    }else{
	    ROS_INFO("run  back------------------");
	    
            blackboard_->info.last_position=my;
	    if (executor_state != BehaviorState::RUNNING) {
      geometry_msgs::PoseStamped boot_position_= blackboard_->info.start_pose;
       ROS_INFO("boot_position_.x:%f",boot_position_.pose.position.x);
       ROS_INFO("boot_position_.y:%f",boot_position_.pose.position.y);
       ROS_INFO("boot_position_.z:%f",boot_position_.pose.position.z);
      auto robot_map_pose = blackboard_->GetRobotMapPose();
      auto dx = boot_position_.pose.position.x - robot_map_pose.pose.position.x;
      auto dy = boot_position_.pose.position.y - robot_map_pose.pose.position.y;
      ROS_INFO("robot_map_pose.x:%f",robot_map_pose.pose.position.x);
       ROS_INFO("robot_map_pose.y:%f",robot_map_pose.pose.position.y);
       ROS_INFO("robot_map_pose.z:%f",robot_map_pose.pose.position.z);
      auto boot_yaw = tf::getYaw(boot_position_.pose.orientation);
      auto robot_yaw = tf::getYaw(robot_map_pose.pose.orientation);
      auto a = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
      tf::Quaternion rot1, rot2;
      tf::quaternionMsgToTF(boot_position_.pose.orientation, rot1);
      tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
      auto d_yaw =  rot1.angleShortestPath(rot2);
      ROS_INFO("p1:%f",a);
      ROS_INFO("p2:%f",d_yaw);
      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2 || d_yaw > 0.5) {
        ROS_INFO("==============move");
        chassis_executor_->Execute(boot_position_);
        blackboard_->SetMyGoal(boot_position_);

      }else{
        ROS_INFO("==============stay!!!!!!!!!!!!!!!!!!!");
        chassis_executor_->Cancel();
     }
    }
    
    
    }
    
  }

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

    boot_position_.header.frame_id = "map";

    float x, y, z, roll, pitch, yaw;
    if (decision_config.isblue()){
        if (decision_config.master()){
            x = decision_config.blue().master_bot().start_position().x();
            y = decision_config.blue().master_bot().start_position().y();
            z = decision_config.blue().master_bot().start_position().z();
            roll = decision_config.blue().master_bot().start_position().roll();
            pitch = decision_config.blue().master_bot().start_position().pitch();
            yaw = decision_config.blue().master_bot().start_position().yaw();
        }  
        else{
            x = decision_config.blue().wing_bot().start_position().x();
            y = decision_config.blue().wing_bot().start_position().y();
            z = decision_config.blue().wing_bot().start_position().z();
            roll = decision_config.blue().wing_bot().start_position().roll();
            pitch = decision_config.blue().wing_bot().start_position().pitch();
            yaw = decision_config.blue().wing_bot().start_position().yaw();
        }  
    }
    else{
       if (decision_config.master()){
            x = decision_config.red().master_bot().start_position().x();
            y = decision_config.red().master_bot().start_position().y();
            z = decision_config.red().master_bot().start_position().z();
            roll = decision_config.red().master_bot().start_position().roll();
            pitch = decision_config.red().master_bot().start_position().pitch();
            yaw = decision_config.red().master_bot().start_position().yaw();
       }
       else{
            x = decision_config.red().wing_bot().start_position().x();
            y = decision_config.red().wing_bot().start_position().y();
            z = decision_config.red().wing_bot().start_position().z();
            roll = decision_config.red().wing_bot().start_position().roll();
            pitch = decision_config.red().wing_bot().start_position().pitch();
            yaw = decision_config.red().wing_bot().start_position().yaw();

       }
    }


    boot_position_.pose.position.x = x;
    boot_position_.pose.position.z = z;
    boot_position_.pose.position.y = y;

    auto master_quaternion = tf::createQuaternionMsgFromRollPitchYaw(roll,
                                                                     pitch,
                                                                     yaw);
    boot_position_.pose.orientation = master_quaternion;

    return true;
  }

  ~BackBootAreaBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! boot position
  geometry_msgs::PoseStamped boot_position_;

  //! chase buffer
  std::vector<geometry_msgs::PoseStamped> chase_buffer_;
  unsigned int chase_count_;

};
}


#endif //ROBORTS_DECISION_BACK_BOOT_AREA_BEHAVIOR_H
