project(roborts_msgs)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  geometry_msgs
  actionlib
  message_generation
)

add_action_files(
  DIRECTORY action
  FILES
  LocalPlanner.action
  GlobalPlanner.action
  ArmorDetection.action
)

add_message_files(
  DIRECTORY msg
  FILES
  TwistAccel.msg
  GimbalAngle.msg
  GimbalRate.msg
  ObstacleMsg.msg
  ShootInfo.msg
  ShootState.msg
  ShooterCmd.msg
  SentryInfo.msg
  ArmorPos.msg
  ArmorsPos.msg
  AllyPose.msg
  Distance.msg
  GimbalPID.msg
  TargetInfo.msg
  FVector.msg
  FusionTarget.msg
  Target.msg
  LaserTarget.msg
  DodgeMode.msg
  GimbalInfo.msg
  BallCollision.msg
)

add_message_files(
    DIRECTORY msg/referee_system
    FILES
    BonusStatus.msg
    GameResult.msg
    GameRobotBullet.msg
    GameRobotHP.msg
    GameStatus.msg
    GameSurvivor.msg
    GameZone.msg
    GameZoneArray.msg
    LurkStatus.msg
    ProjectileSupply.msg
    RobotBonus.msg
    RobotDamage.msg
    RobotHeat.msg
    RobotShoot.msg
    RobotStatus.msg
    SupplierStatus.msg
)
add_message_files(
  DIRECTORY msg
  FILES
  ShootWhl.msg
)
add_service_files(
  DIRECTORY srv
  FILES
  GimbalMode.srv
  FricWhl.srv
  ShootCmd.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package()
