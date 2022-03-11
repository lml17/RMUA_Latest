#ifndef ROBORTS_DECISION_GIMBALRATE_BEHAVIOR_H
#define ROBORTS_DECISION_GIMBALRATE_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../executor/gimbal_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class GimbalRateBehavior {
 public:
  GimbalRateBehavior(ChassisExecutor* &chassis_executor,
                GimbalExecutor* &gimbal_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       gimbal_executor_(gimbal_executor),
                                                       blackboard_(blackboard) {

  }


  void Run() {
      ROS_INFO("======Gimbal Behavior.h============");
      roborts_msgs::GimbalRate gimbal_rate;
//#gimbal feedback angle data
//bool yaw_mode
//bool pitch_mode
//float64 yaw_angle
//float64 pitch_angle

     // gimbal_rate.pitch_rate = 0;
      gimbal_rate.yaw_rate = 0.1;

      gimbal_executor_->Execute(gimbal_rate);                               
    
  }

  void Cancel() {
    gimbal_executor_->Cancel();
  }

  BehaviorState Update() {
    return gimbal_executor_->Update();
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

  ~GimbalRateBehavior() = default;

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
