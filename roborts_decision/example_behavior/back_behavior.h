#ifndef ROBORTS_DECISION_BACK_BEHAVIOR_H
#define ROBORTS_DECISION_BACK_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class BackBehavior {
 public:
  BackBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {


    last_position_.header.frame_id = "map";
    last_position_.pose.orientation.x = 0;
    last_position_.pose.orientation.y = 0;
    last_position_.pose.orientation.z = 0;
    last_position_.pose.orientation.w = 1;

    last_position_.pose.position.x = 0;
    last_position_.pose.position.y = 0;
    last_position_.pose.position.z = 0;

    search_index_ = 0;
    search_count_ = 0;
    SearchBegin = true;
    if (!LoadParam(proto_file_path)) {
      ROS_ERROR("%s can't open file", __FUNCTION__);
    }

  }

  void Run() {
//printf("search_behavior=======run()\n");
    auto executor_state = Update();
    geometry_msgs::PoseStamped my = blackboard_->GetRobotMapPose();
    geometry_msgs::PoseStamped enemy_map_pose = blackboard_->GetEnemy();
    enemy_map_pose.pose.position.x = 4;
    enemy_map_pose.pose.position.y = 3;
    ROS_INFO("AMBUSH====RUN==my==position==x:%f",my.pose.position.x);
    ROS_INFO("AMBUSH====RUN==my==position==y:%f",my.pose.position.y);
  //  ROS_INFO("AMBUSH====RUN==my==position==z:%f",my.pose.position.z);
  //  ROS_INFO("AMBUSH====RUN==my==orientation==x:%f",my.pose.orientation.x);
  //  ROS_INFO("AMBUSH====RUN==my==orientation==y:%f",my.pose.orientation.y);
  //  ROS_INFO("AMBUSH====RUN==my==orientation==z:%f",my.pose.orientation.z);
  //  ROS_INFO("AMBUSH====RUN==my==orientation==w:%f",my.pose.orientation.w);
    ROS_INFO("AMBUSH====RUN==enemy_map_pose==position==x:%f",enemy_map_pose.pose.position.x);
    ROS_INFO("AMBUSH====RUN==enemy_map_pose==position==y:%f",enemy_map_pose.pose.position.y);
  //  ROS_INFO("AMBUSH====RUN==enemy_map_pose==position==z:%f",enemy_map_pose.pose.position.z);
  //  ROS_INFO("AMBUSH====RUN==enemy_map_pose==orientation==x:%f",enemy_map_pose.pose.orientation.x);
   // ROS_INFO("AMBUSH====RUN==enemy_map_pose==orientation==y:%f",enemy_map_pose.pose.orientation.y);
   // ROS_INFO("AMBUSH====RUN==enemy_map_pose==orientation==z:%f",enemy_map_pose.pose.orientation.z);
  //  ROS_INFO("AMBUSH====RUN==enemy_map_pose==orientation==w:%f",enemy_map_pose.pose.orientation.w);
    // if e.x<7
    if (executor_state != BehaviorState::RUNNING){
        if(enemy_map_pose.pose.position.x>=my.pose.position.x){
	    if(enemy_map_pose.pose.position.x<7){
		 my.pose.position.x = enemy_map_pose.pose.position.x +1;
		 my.pose.position.y = enemy_map_pose.pose.position.y;
	    }else{
		 if(enemy_map_pose.pose.position.y<1){
		    my.pose.position.x = enemy_map_pose.pose.position.x;
		    my.pose.position.y = enemy_map_pose.pose.position.y +1;
		 }else if (enemy_map_pose.pose.position.y>3.5){
		    my.pose.position.x = enemy_map_pose.pose.position.x;
		    my.pose.position.y = enemy_map_pose.pose.position.y -1;
		 }else {
		    my.pose.position.x = enemy_map_pose.pose.position.x;
		    my.pose.position.y = enemy_map_pose.pose.position.y -1;
		 }
	    }
        my.pose.orientation = blackboard_->GetRelativeQuaternion(enemy_map_pose, my);
        ROS_INFO("update orientation");
	chassis_executor_->Execute(my);
    	blackboard_->SetMyGoal(my);
        return;
       }
    }

   // if (blackboard_->GetCostMap2D()->GetCost(mx_,my_)>=253  && executor_state == BehaviorState::RUNNING){
   //     chassis_executor_->Cancel();
  //      search_index_ = (search_index_ + 1) % search_region_1_.size();

  //  }

    
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
     
    for (int i=0; i<decision_config.search_region_1().size();i++)
        search_region_1_.push_back(blackboard_->Point2PoseStamped(decision_config.search_region_1(i)));

    // may have more efficient way to search a region(enemy where disappear)
    // search_region_.resize((unsigned int)(decision_config.search_region_1().size()));

    // for (int i = 0; i != decision_config.search_region_1().size(); i++) {
    //   geometry_msgs::PoseStamped search_point;
    //   search_point.header.frame_id = "map";
    //   search_point.pose.position.x = decision_config.search_region_1(i).x();
    //   search_point.pose.position.y = decision_config.search_region_1(i).y();
    //   search_point.pose.position.z = decision_config.search_region_1(i).z();

    //   auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_1(i).roll(),
    //                                                             decision_config.search_region_1(i).pitch(),
    //                                                             decision_config.search_region_1(i).yaw());
    //   search_point.pose.orientation = quaternion;
    //   search_region_1_.push_back(search_point);
    // }

    // for (int i = 0; i != decision_config.search_region_2().size(); i++) {
    //   geometry_msgs::PoseStamped search_point;
    //   search_point.header.frame_id = "map";
    //   search_point.pose.position.x = decision_config.search_region_2(i).x();
    //   search_point.pose.position.y = decision_config.search_region_2(i).y();
    //   search_point.pose.position.z = decision_config.search_region_2(i).z();

    //   auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_2(i).roll(),
    //                                                             decision_config.search_region_2(i).pitch(),
    //                                                             decision_config.search_region_2(i).yaw());
    //   search_point.pose.orientation = quaternion;
    //   search_region_2_.push_back(search_point);
    // }

    // for (int i = 0; i != decision_config.search_region_3().size(); i++) {
    //   geometry_msgs::PoseStamped search_point;
    //   search_point.header.frame_id = "map";
    //   search_point.pose.position.x = decision_config.search_region_3(i).x();
    //   search_point.pose.position.y = decision_config.search_region_3(i).y();
    //   search_point.pose.position.z = decision_config.search_region_3(i).z();

    //   auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_3(i).roll(),
    //                                                             decision_config.search_region_3(i).pitch(),
    //                                                             decision_config.search_region_3(i).yaw());
    //   search_point.pose.orientation = quaternion;
    //   search_region_3_.push_back(search_point);
    // }

    // for (int i = 0; i != decision_config.search_region_4().size(); i++) {
    //   geometry_msgs::PoseStamped search_point;
    //   search_point.header.frame_id = "map";
    //   search_point.pose.position.x = decision_config.search_region_4(i).x();
    //   search_point.pose.position.y = decision_config.search_region_4(i).y();
    //   search_point.pose.position.z = decision_config.search_region_4(i).z();

    //   auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.search_region_4(i).roll(),
    //                                                             decision_config.search_region_4(i).pitch(),
    //                                                             decision_config.search_region_4(i).yaw());
    //   search_point.pose.orientation = quaternion;
    //   search_region_4_.push_back(search_point);
    // }
    return true;
  }

  void SetLastPosition(geometry_msgs::PoseStamped last_position) {
    last_position_ = last_position;
    search_count_ = 5;
  }

  ~BackBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

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

#endif //ROBORTS_DECISION_BACK_BEHAVIOR_H
