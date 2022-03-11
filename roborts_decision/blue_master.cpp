#include <ros/ros.h>

#include "executor/chassis_executor.h"


#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"
#include "example_behavior/reload_behavior.h"
#include "example_behavior/shield_behavior.h"
#include "example_behavior/test_behavior.h"
#include "example_behavior/ambush_behavior.h"
#include "example_behavior/attack_behavior.h"
#include "example_behavior/TurnToDetectedDirection.h"
#include "example_behavior/swing_behavior.h"
#include "example_behavior/shoot_behavior.h"
#include "example_behavior/defend_behavior.h"
#include "example_behavior/gimbal_behavior.h"
#include "example_behavior/gimbal_rate_behavior.h"
#include "example_behavior/back_behavior.h"
#include "example_behavior/froze_behavior.h"
#include "sel_behavior_node.h"

enum BehaviorStateEnum{
     INIT = -1,
     BACKBOOT = 0,
     CHASE=1,
     SEARCH=2,
     ESCAPE=3,
     PATROL=4,
     RELOAD=5,
     SHIELD=6,
     AMBUSH=7,
     ATTACK=8,
     FROZE=9,
     TURN=10,
     DEFEND=11,
     TEST=12,
     BACK=13

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "blue_master");
  
  
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/blue_master.prototxt";

  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);
  auto gimbal_executor = new roborts_decision::GimbalExecutor;
  // Behavior State Enum
  BehaviorStateEnum last_state, cur_state;
  last_state = BehaviorStateEnum::INIT;
  cur_state = BehaviorStateEnum::INIT;

  
  roborts_decision::FrozeBehavior froze_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::BackBootAreaBehavior back_boot_area_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::AmbushBehavior    ambush_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::AttackBehavior    attack_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::SearchBehavior       search_behavior(chassis_executor, blackboard, full_path);

  roborts_decision::ChaseBehavior        chase_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::EscapeBehavior       escape_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::PatrolBehavior       patrol_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::GoalBehavior       goal_behavior(chassis_executor, blackboard);
  roborts_decision::ReloadBehavior     reload_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::ShieldBehavior     shield_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::TestBehavior      test_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::TurnToDetectedDirection    turnToDetectedDirection(chassis_executor, blackboard, full_path);
  roborts_decision::DefendBehavior    defend_behavior(chassis_executor,gimbal_executor, blackboard, full_path);
  roborts_decision::BackBehavior    back_behavior(chassis_executor, blackboard, full_path);
  ros::Rate rate(5);
  
  // for filter noise command
  unsigned int count=0;
  const unsigned int count_bound = 3;
  printf("Only One==============================================================================\n");
  ros::Time start_time = ros::Time::now();
  geometry_msgs::PoseStamped cur_pose = blackboard->GetRobotMapPose();
  blackboard->info.start_pose = cur_pose;
  ROS_INFO("start_pose.x:%f",cur_pose.pose.position.x);
  ROS_INFO("start_pose.y:%f",cur_pose.pose.position.y);
  ROS_INFO("start_pose.z:%f",cur_pose.pose.position.z);
  while(ros::ok()){
    ros::spinOnce();
    ros::Time now_time = ros::Time::now();
    
    // static bool click = true;
    // if (blackboard->info.remaining_time % 60 >2 )  click = true;
    // if (blackboard->info.remaining_time % 60 <= 2 && click) {blackboard->info.times_to_supply = 2; blackboard->info.times_to_buff=1; click=false;}

    if (blackboard->info.remaining_time % 60 <= 1) {blackboard->info.times_to_supply = 2; blackboard->info.times_to_buff=1;}
    printf("-----------------------------------------\n");
    printf("start_time:%d\n",start_time.toSec());
    printf("now_time:%d\n",now_time.toSec());
    //printf("game_status:%i\n",blackboard->info.game_status);
   // printf("Remaining Time:%d and is begin:%d\n", blackboard->info.remaining_time, blackboard->info.is_begin);
  //  printf("hp:%d, bullet:%d,  pose:(%f, %f)\n", blackboard->info.remain_hp, blackboard->info.remain_bullet, blackboard->GetRobotMapPose().pose.position.x, blackboard->GetRobotMapPose().pose.position.y); 
    /**
    printf("Times to supply:%d, to buff:%d\n", blackboard->info.times_to_supply, blackboard->info.times_to_buff);
    printf("\n");
    printf("Ally hp:%d, bullet:%d,  pose:(%f, %f)\n", blackboard->info.ally_remain_hp, blackboard->info.ally_remain_bullet, blackboard->info.ally.pose.position.x, blackboard->info.ally.pose.position.y);
    printf("has_ally_enemy:%d, has_ally_first_enemy:%d, has_ally_second_endmy:%d\n", blackboard->info.has_ally_enemy, blackboard->info.has_ally_first_enemy, blackboard->info.has_ally_second_enemy);
    printf("ally first enemy pose:(%f, %f), ally second enemy pose:(%f, %f)\n", blackboard->info.ally_first_enemy.pose.position.x, blackboard->info.ally_first_enemy.pose.position.y, blackboard->info.ally_second_enemy.pose.position.x, blackboard->info.ally_second_enemy.pose.position.y);
    printf("\n");
    printf("My: has_buff:%d\n", blackboard->info.has_buff);
    printf("hp:%d, bullet:%d,  pose:(%f, %f)\n", blackboard->info.remain_hp, blackboard->info.remain_bullet, blackboard->GetRobotMapPose().pose.position.x, blackboard->GetRobotMapPose().pose.position.y);    
    printf("has_my_enemy:%d, has_first_enemy:%d, has_second_enemy:%d\n", blackboard->info.has_my_enemy, blackboard->info.has_first_enemy, blackboard->info.has_second_enemy);  
    printf("my first enemy pose:(%f, %f), my second enemy pose:(%f, %f)\n", blackboard->info.first_enemy.pose.position.x, blackboard->info.first_enemy.pose.position.y, blackboard->info.second_enemy.pose.position.x, blackboard->info.second_enemy.pose.position.y);
    printf("my goal:(%f, %f), ally goal:(%f, %f)\n", blackboard->info.my_goal.pose.position.x, blackboard->info.my_goal.pose.position.y, blackboard->info.ally_goal.pose.position.x, blackboard->info.ally_goal.pose.position.y);
    printf("Is Stuck:%d\n", blackboard->IsInStuckArea());
     **/
    blackboard->IsInStuckArea();
    // shoot and dodge command when game is on!
    // game start
  //  if(blackboard->info.game_status==4){
    geometry_msgs::PoseStamped rt_pose = blackboard->GetRobotMapPose();
    if(ros::Duration(15)>=(now_time-start_time)&&(now_time-start_time)>=ros::Duration(0)){ 
    // printf("===========start game=============================\n");
     int damage_source = blackboard->info.damage_source_info; 
   //  printf("damage_source:%i\n",damage_source);
   // if (last_state != BehaviorStateEnum::ESCAPE){
        
   //     if (blackboard->CanDodge()) {
   //         blackboard->StartDodge(); 
            // printf("In Dodge!\n"); 
      //  }
        // else 
        //   printf("Not In Dodge!\n");
   // }
   
    if (blackboard->CanShoot()) blackboard->Shoot(blackboard->info.shoot_hz);
    // my pose
    geometry_msgs::PoseStamped mypose = blackboard->GetRobotMapPose();
   // cur_state = BehaviorStateEnum::FROZE; 
   
    if(abs(rt_pose.pose.position.x-5)<0.5&&abs(rt_pose.pose.position.y-3)<0.5){
            if((now_time-start_time)>=ros::Duration(10)){
                    cur_state = BehaviorStateEnum::AMBUSH;
             }else{
                    cur_state = BehaviorStateEnum::DEFEND;
             }  
     }else{
          cur_state = BehaviorStateEnum::BACK;
     }
    
    // add 
    //  血量不足
    /**
    if(blackboard->info.ally_remain_hp <=800 || blackboard->info.remain_hp<=800){
             // 有获胜优势 
             if(blackboard->WithAdvantage()<2.0){
                     //射击
                     cur_state = BehaviorStateEnum::TEST;
             } else{  // 没有获胜优势
                 // buff 已经刷新
                 if(blackboard->info.buff_refresh){
                         // 距离加成区更近
                         if (blackboard->NearBuff()){
                                // 获取buff
				cur_state = BehaviorStateEnum::TEST;

                          }else if (blackboard->info.remain_hp<=800){
                                // 撤退
                                cur_state = BehaviorStateEnum::TEST;
                           }

              // }else if(blackboard->IsNearRefresh() && blackboard_ptr->OnBuffLocation()){   // buff 即将 刷新
               }else if(blackboard->IsNearRefresh() ){   // buff 即将 刷新
                     cur_state = BehaviorStateEnum::TEST;

               }else{  // buff 还有很久才 刷新
                   cur_state = BehaviorStateEnum::TEST;
               }

            }
    }else{ // 双方血量都大于800
          //  受到攻击
          if (((ros::Time::now()-blackboard->GetLastArmorAttackTime()).toSec()<0.5)){
                // 没有转向
                if (blackboard->IfTurn()){
			cur_state = BehaviorStateEnum::TEST;
                }else{ // 已经转向
                     if (blackboard->HurtedPerSecond()>=60){
			cur_state = BehaviorStateEnum::TEST;
                     } else {
                        cur_state = BehaviorStateEnum::TEST;
                     }

                }
                
          }else { //  没有受到攻击
              // 队友受到惩罚且受到攻击
              if (blackboard->IfTeammatePunished() && blackboard->TeammateAttacked()){ 
                      // 跟随
                      cur_state = BehaviorStateEnum::TEST;
              }else {  // 队友没有受到惩罚且受到攻击
                   // 有目标，则埋伏射击
                   if (blackboard->info.has_my_enemy || blackboard->info.valid_camera_armor ){
                         cur_state = BehaviorStateEnum::TEST;
                    } else { // 没有目标，则搜索
                        cur_state = BehaviorStateEnum::TEST;  
                    }

              }
          }
     
    }
     **/
    //add end 
  
    
   }else if((now_time-start_time)>ros::Duration(15)){ 
  //} else if (blackboard->info.game_status==5){ 
     // game end
        printf("back to boot!!!!!!!!!!!!!!!!!!!!!!!\n");
        cur_state = BehaviorStateEnum::BACKBOOT;

   }
   else{ 
       // wait game start 
       cur_state = BehaviorStateEnum::TEST;
       printf("wait!!!!!!!!!!!!!!!!!!!!!!!\n");
   }
    
    
    // filter
    if (!(last_state == BehaviorStateEnum::RELOAD  ||  last_state == BehaviorStateEnum::SHIELD)){
        count = last_state != cur_state ? count + 1: 0;
        cur_state = count>=count_bound? cur_state: last_state;
    }
    
   
    // cancel last state
    if ( (last_state != BehaviorStateEnum::INIT && last_state != cur_state)  || blackboard->info.remain_hp<=0 ){
         switch (last_state){
            case BehaviorStateEnum::BACKBOOT:
                back_boot_area_behavior.Cancel();
                break;
            case BehaviorStateEnum::CHASE:
                chase_behavior.Cancel();
                break;
            case BehaviorStateEnum::ESCAPE:
                escape_behavior.Cancel();
                break;
            case BehaviorStateEnum::PATROL:
                patrol_behavior.Cancel();
                break;
            case BehaviorStateEnum::RELOAD:
                reload_behavior.Cancel();
                break;
            case BehaviorStateEnum::SHIELD:
                shield_behavior.Cancel();
                break;
            case BehaviorStateEnum::SEARCH:
                search_behavior.Cancel();
                break;
            case BehaviorStateEnum::AMBUSH:
                ambush_behavior.Cancel();
                break;
            case BehaviorStateEnum::ATTACK:
                attack_behavior.Cancel();
                break;
            case BehaviorStateEnum::TURN:
                turnToDetectedDirection.Cancel();
                break;
            case BehaviorStateEnum::DEFEND:
                defend_behavior.Cancel();
                break;
           case BehaviorStateEnum::TEST:
                test_behavior.Cancel();
                break;
           case BehaviorStateEnum::BACK:
                back_behavior.Cancel();
                break;
         }
    }

    // remain hp is 0, them dead!
    if (blackboard->info.remain_hp <=0){
        ros::shutdown();
        break;
    }


    switch (cur_state){
      case BehaviorStateEnum::BACKBOOT:
          back_boot_area_behavior.Run();
          std::cout<<"BackBoot" << std::endl;
          break;
      case BehaviorStateEnum::CHASE:
          chase_behavior.Run();
          std::cout<<"CHASE" << std::endl;
          break;
      case BehaviorStateEnum::ESCAPE:
          escape_behavior.Run();
          std::cout<<"ESCAPE" << std::endl;
          break;
      case BehaviorStateEnum::PATROL:
          patrol_behavior.Run();
          std::cout<<"PATROL" << std::endl;
          break;
      case BehaviorStateEnum::RELOAD:
          reload_behavior.Run();
          std::cout<<"RELOAD" << std::endl;
          break;
      case BehaviorStateEnum::SHIELD:
          shield_behavior.Run();
          std::cout<<"SHIELD" << std::endl;
          break;
      case BehaviorStateEnum::SEARCH:
          search_behavior.Run();
          std::cout<<"SEARCH" << std::endl;
          break;
      case BehaviorStateEnum::AMBUSH:
          ambush_behavior.Run();
          std::cout<<"AMBUSH" << std::endl;
          break;
      case BehaviorStateEnum::ATTACK:
          attack_behavior.Run();
          std::cout<<"ATTACK" << std::endl;
          break;
      case BehaviorStateEnum::FROZE: 
          froze_behavior.Run();
          std::cout<<"FROZE" << std::endl;
          break;
      case BehaviorStateEnum::TURN:
          turnToDetectedDirection.Run();
          std::cout<<"TURN" << std::endl;
          break;
     case BehaviorStateEnum::DEFEND:
          defend_behavior.Run();
          std::cout<<"DEFEND" << std::endl;
          break;
    case BehaviorStateEnum::TEST:
          test_behavior.Run();
          std::cout<<"TEST" << std::endl;
          break;
    case BehaviorStateEnum::BACK:
          back_behavior.Run();
          std::cout<<"BACK" << std::endl;
          break;

    }
    if(now_time-start_time>ros::Duration(30)||blackboard->info.is_begin){
              break;
    }
    last_state = cur_state;
    
    rate.sleep();
    
  }


  return 0;
}

