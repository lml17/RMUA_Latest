#ifndef ROBORTS_DECISION_SWING_BEHAVIOR_H
#define ROBORTS_DECISION_SWING_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class SwingBehavior {
 public:
  SwingBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard) {

  }


  void Run() {
      printf("swing_behavior=======run()\n");
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

  ~SwingBehavior() = default;

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

#endif //ROBORTS_DECISION_SWING_BEHAVIOR_H
