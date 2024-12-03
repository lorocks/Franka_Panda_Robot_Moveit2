
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/move_group/>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

void setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object1;
  object1.id = "cyl1";
  object1.header.frame_id = "world";
  object1.primitives.resize(1);
  object1.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object1.primitives[0].dimensions = { 0.25, 0.02 };

  geometry_msgs::msg::Pose pose1;
  pose1.position.x = 0.25;
  pose1.position.y = -0.5;
  pose1.position.z = 0.125;
  pose1.orientation.w = 1.0;
  object1.pose = pose1;

  moveit_msgs::msg::CollisionObject object2;
  object2.id = "cyl2";
  object2.header.frame_id = "world";
  object2.primitives.resize(1);
  object2.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object2.primitives[0].dimensions = { 0.25, 0.02 };

  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 0.5;
  pose2.position.y = -0.25;
  pose2.position.z = 0.125;
  pose2.orientation.w = 1.0;
  object2.pose = pose2;

  moveit_msgs::msg::CollisionObject object3;
  object3.id = "box";
  object3.header.frame_id = "world";
  object3.primitives.resize(1);
  object3.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object3.primitives[0].dimensions = { 0.4, 0.3, 0.05 }; // 0.05

  geometry_msgs::msg::Pose pose3;
  pose3.position.x = 0.375;
  pose3.position.y = -0.375;
  pose3.position.z = object2.primitives[0].dimensions[0] + (object3.primitives[0].dimensions[2] / 2) - object2.primitives[0].dimensions[0] / 2;
  pose3.orientation.w = 1.0;
  object3.pose = pose3;

  moveit_msgs::msg::CollisionObject object4;
  object4.id = "cyl3";
  object4.header.frame_id = "world";
  object4.primitives.resize(1);
  object4.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object4.primitives[0].dimensions = { 0.25, 0.02 };

  geometry_msgs::msg::Pose pose4;
  pose4.position.x = 0.5;
  pose4.position.y = -0.5;
  pose4.position.z = 0.125;
  pose4.orientation.w = 1.0;
  object4.pose = pose4;

  moveit_msgs::msg::CollisionObject object5;
  object5.id = "cyl4";
  object5.header.frame_id = "world";
  object5.primitives.resize(1);
  object5.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object5.primitives[0].dimensions = { 0.25, 0.02 };

  geometry_msgs::msg::Pose pose5;
  pose5.position.x = 0.25;
  pose5.position.y = -0.25;
  pose5.position.z = 0.125;
  pose5.orientation.w = 1.0;
  object5.pose = pose5;

  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.3, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.25;
  pose.position.y = -0.3;
  pose.position.z = object2.primitives[0].dimensions[0] + object3.primitives[0].dimensions[2] + (object.primitives[0].dimensions[0] / 2);
  pose.orientation.w = 1.0;
  object.pose = pose;



  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object1);
  psi.applyCollisionObject(object2);
  psi.applyCollisionObject(object3);
  psi.applyCollisionObject(object4);
  psi.applyCollisionObject(object5);
  psi.applyCollisionObject(object);
}


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  setupPlanningScene();

  // Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_arm = MoveGroupInterface(node, "panda_arm");
auto move_gripper = MoveGroupInterface(node, "hand");


// auto const target_pose = []{
//   geometry_msgs::msg::Pose msg;
//   msg.orientation.w = -0.45;
//   msg.orientation.x = -0.74;
//   msg.orientation.y = -0.25;
//   msg.orientation.z = -0.41;
//   msg.position.x = 0.25; // 25
//   msg.position.y = 0.0; // 3
//   msg.position.z = 0.45;
//   return msg;
// }();


move_gripper.setNamedTarget("open");




// Create a plan to that target pose
auto const [success_open, plan_open] = [&move_gripper]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_gripper.plan(msg));
  return std::make_pair(ok, msg);
}();

move_arm.setRPYTarget(1.57, 0.78, 0.0);
auto const [success_roll, plan_roll] = [&move_arm]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_arm.plan(msg));
  return std::make_pair(ok, msg);
}();
if(success_roll) {
  move_arm.execute(plan_roll);
} else {
  RCLCPP_ERROR(logger, "Planing failed!");
}
move_gripper.attachObject("object");


// Execute the plan

if(success_open) {
  move_gripper.execute(plan_open);
} else {
  RCLCPP_ERROR(logger, "Planing failed Open!");
}


// Create a plan to that target pose
auto const [success_move, plan_move] = [&move_arm]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_arm.plan(msg));
  return std::make_pair(ok, msg);
}();
if(success_move) {
  move_arm.execute(plan_move);
} else {
  RCLCPP_ERROR(logger, "Planing failed Move!");
}


move_gripper.setNamedTarget("close");
auto const [success_close, plan_close] = [&move_gripper]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_gripper.plan(msg));
  return std::make_pair(ok, msg);
}();

if(success_close) {
  move_gripper.execute(plan_close);
} else {
  RCLCPP_ERROR(logger, "Planing failed Close!");
}




  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}