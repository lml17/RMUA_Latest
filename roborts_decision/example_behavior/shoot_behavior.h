#ifndef ROBORTS_DECISION_SHOOT_BEHAVIOR_H
#define ROBORTS_DECISION_SHOOT_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"
#include "roborts_msgs/ShootCmd.h"
#include "roborts_msgs/FricWhl.h"
#include "line_iterator.h"

namespace roborts_decision {
class ShootBehavior {
 public:
  ShootBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {


    cancel_goal_ = true;
    
  }
  
  void RunFricWhl(roborts_msgs::FricWhl &fricwhl){
    ros::NodeHandle n;
    ros::ServiceClient fric_client = n.serviceClient<roborts_msgs::FricWhl>("cmd_fric_wheel");
    if(fricwhl.request.wheelspeed < 1200){
      fricwhl.request.wheelspeed = 1200;
    }
    else if (fricwhl.request.wheelspeed > 1500){
      fricwhl.request.wheelspeed = 1500;
    }
    // https://github.com/RoboMaster/RoboRTS-Base/blob/icra2022/docs/node_api.md
    // 根据官方文档，摩擦轮的速度一般在1200~ 1500 之间
    if (fric_client.call(fricwhl)){
           ROS_INFO("Fricwhl Success");
    }
  }
  void RunShoot(roborts_msgs::ShootCmd &shootCmd) {
  //  auto executor_state = Update();
    ros::NodeHandle n;
    ros::ServiceClient shoot_client = n.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");

    // shootCmd.request.mode = 0;
    // shootCmd.request.number = 3;
    if (shoot_client.call(shootCmd)){
           ROS_INFO("Shoot Success");
    }
    // blackboard_->info.has_buff = true;
    // }else{
    //   fricwhl.request.open = false;
	  //   if (fric_client.call(fricwhl)){
		//    ROS_INFO("Feicwhl =false");
	  //   }
    // }
    
  }
  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }



  ~ShootBehavior() = default;

 private:
  // load patrol as ambush points
  



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
