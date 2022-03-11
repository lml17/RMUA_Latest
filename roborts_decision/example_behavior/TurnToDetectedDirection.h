#ifndef ROBORTS_DECISION_TURNTODETECTEDDIRECTION_BEHAVIOR_H
#define ROBORTS_DECISION_TURNTODETECTEDDIRECTION_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class TurnToDetectedDirection {
 public:
  TurnToDetectedDirection(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {


    cancel_goal_ = true;
    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }
  }
	void Run() {
             printf("turntoDete-----------------run\n");
	if(blackboard_->IfTurn()){
		double q=1;   
		double d_yaw=0;
                int choose = blackboard_->ArmorDamageSource();
                ROS_INFO("TurnToDe.h========choose:%i",choose);
		switch (choose) {
		  case 0:  //前方
		      break;  
		  case 1:  //左方
		      d_yaw = M_PI / 2.;
		      break;
		  case 2:  //后方
		      d_yaw = M_PI;
		      q=0.5;
		      break;
		  case 3:  //右方
		      d_yaw = -M_PI / 2.;
		      break;
		  default:
		      return;
		} 
		try{ 
		  ros::Rate rate(q);
                  geometry_msgs::PoseStamped my = blackboard_->GetRobotMapPose();
		  geometry_msgs::PoseStamped hurt_pose;
                  hurt_pose = my;
                  ROS_INFO("TurnToDe.h========d_yaw:%f",d_yaw);
                  ROS_INFO("TurnToDe.h========M_PI:%f",M_PI);
		  auto quaternion = tf::createQuaternionMsgFromYaw(d_yaw);
                  my.pose.orientation = quaternion;
		  //hurt_pose.header.frame_id = "base_link";
		  //hurt_pose.header.stamp = ros::Time::now();
		  //hurt_pose.pose.orientation = quaternion;
                  ROS_INFO("TurnToDe.h========chassis==execute");
		  chassis_executor_->Execute(my);
                  blackboard_->damage_type_=1;
		  rate.sleep(); 
		}
		catch(std::exception& e){
		  ROS_ERROR("ERROR: %S",e.what());
		}
	   
	  }else{
               ROS_INFO("blackboard_->IfTurn()========NOT");

          }
      }
  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }



  ~TurnToDetectedDirection() = default;

 private:
  // load patrol as ambush points
  bool LoadParam(const std::string &proto_file_path) {
    roborts_decision::DecisionConfig decision_config;
    if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
      return false;
    }

    point_size_ = (unsigned int)(decision_config.blue().patrol().size());
    ambush_goals_.resize(point_size_);
    for (int i = 0; i != point_size_; i++) {
      if (blackboard_->info.team_blue){
          ambush_goals_[i] = blackboard_->Point2PoseStamped(decision_config.blue().patrol(i));
      }
      else{
          ambush_goals_[i] = blackboard_->Point2PoseStamped(decision_config.red().patrol(i));
      }
      
    }

    return true;
  }



  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

  //! attack goal
  std::vector<geometry_msgs::PoseStamped> ambush_goals_;

  int point_size_;
  //! cancel flag
  bool cancel_goal_;
};

}
#endif //ROBORTS_DECISION_AMBUSH_BEHAVIOR_H
