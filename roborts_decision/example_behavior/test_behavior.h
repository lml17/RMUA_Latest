#ifndef ROBORTS_DECISION_TEST_BEHAVIOR_H
#define ROBORTS_DECISION_TEST_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class TestBehavior {
 public:
    TestBehavior(ChassisExecutor* &chassis_executor,
                  Blackboard* &blackboard,
                  const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                         blackboard_(blackboard) {

     

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
            ROS_INFO("stop------------------");
            blackboard_->info.is_begin = true;
            
    }else{
	    ROS_INFO("run------------------");
	    geometry_msgs::PoseStamped enemy_map_pose = blackboard_->GetEnemy();
            blackboard_->info.last_position=my;
	    if (executor_state != BehaviorState::RUNNING){
		 double yaw = std::atan2(2,2);
		 goal.pose.position.x = 3;
		 goal.pose.position.y = 2;
		 chassis_executor_->Execute(goal);
	    	 blackboard_->SetMyGoal(goal);
                 
	    }
    
    
    }
    // // ambush points
    // if (
    //   blackboard_->info.has_my_enemy
    //   // && blackboard_->info.found_enemy   ? found_enemy always not valid.
    //   ){
    //     blackboard_->Shoot(1);
    // }
    // else{
    //     blackboard_->StopShoot();
    // }
    
    
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  

  ~TestBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  Blackboard* const blackboard_;

 

};
}

#endif //ROBORTS_DECISION_TEST_BEHAVIOR_H
