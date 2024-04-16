#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit_msgs/msg/detail/attached_collision_object__struct.hpp>
#include <rclcpp/rclcpp.hpp>
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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask1(mtc::Stage* attach_object_stage);
  mtc::Task createTask2(mtc::Stage* attach_object_stage);
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
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
  object3.primitives[0].dimensions = { 0.4, 0.3, 0.05 };

  geometry_msgs::msg::Pose pose3;
  pose3.position.x = 0.375;
  pose3.position.y = -0.375;
  pose3.position.z = object2.primitives[0].dimensions[0] + (object3.primitives[0].dimensions[2] / 2);
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

  moveit_msgs::msg::AttachedCollisionObject aobject;
  aobject.set__object(object);

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object1);
  psi.applyCollisionObject(object2);
  psi.applyCollisionObject(object3);
  psi.applyCollisionObject(object4);
  psi.applyCollisionObject(object5);
  psi.applyCollisionObject(object);
}


void MTCTaskNode::doTask()
{
    // clang-format off
  mtc::Stage* attach_object_stage =
      nullptr;  // Forward attach_object_stage to place pose generator
  // clang-format on

  task_ = createTask1(attach_object_stage);

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed in init()");
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5 /* max_solutions */))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result1 = task_.execute(*task_.solutions().front());
  if (result1.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  task_ = createTask2(attach_object_stage);

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed in init()");
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5 /* max_solutions */))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result2 = task_.execute(*task_.solutions().front());
  if (result2.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }


  return;
}

mtc::Task MTCTaskNode::createTask1(mtc::Stage* attach_object_stage)
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  // // clang-format off
  // auto stage_initial =
  //     std::make_unique<mtc::stages::MoveTo>("initial hand", interpolation_planner);
  // // clang-format on
  // stage_initial->setGroup(arm_group_name);
  // stage_initial->setGoal("initial");
  // task.add(std::move(stage_initial)); 

  // {
  //   auto stage = std::make_unique<mtc::stages::MoveTo>("initial hand", interpolation_planner);
  //   // stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
  //   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  //   stage->setGoal("initial");
  //   task.add(std::move(stage));
  // }


  // {
  //   auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
  //   // stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
  //   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  //   stage->setGoal("ready");
  //   task.add(std::move(stage));
  // }



  // clang-format off
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  // clang-format on
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand)); 

  // clang-format off
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, interpolation_planner } });
  // clang-format on
  stage_move_to_pick->setTimeout(10.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  // ##### Me placing open here
  // task.add(std::move(stage_open_hand));

  // My implementation testing
  auto stage_motion = std::make_unique<mtc::stages::MoveTo>("motion", interpolation_planner);
  geometry_msgs::msg::PoseStamped location;
  location.header.frame_id = hand_frame;
  location.pose.orientation.w = 1.0;
  location.pose.position.x = 0.5;
  location.pose.position.y = 0.5;
  location.pose.position.z = 0.5;
  stage_motion->setGroup(arm_group_name);
  stage_motion->setIKFrame(hand_frame);
  stage_motion->setGoal(location);
  // task.add(std::move(stage_motion));
  //////////


  // This is an example of SerialContainer usage. It's not strictly needed here.
  // In fact, `task` itself is a SerialContainer by default.
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    // clang-format off
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    // clang-format on


    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      // clang-format on
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  ---- *               Generate Grasp Pose                *
     ***************************************************/
    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state

      // This is the transform from the object frame to the end-effector frame
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.1;

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      // clang-format on
      // grasp->insert(std::move(stage));
    }


    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup("panda_arm_hand")
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      // clang-format on
      grasp->insert(std::move(stage));
    }


    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("panda_finger_joint1",
                             "object",
                             true);
      // clang-format on
      // grasp->insert(std::move(stage));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(
                             "object",
                             true);
      // clang-format on
      // grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      // grasp->insert(std::move(stage));
    }

    

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      // clang-format on
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      // stage->setIKFrame("hand");
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      // vec.vector.x = 1.0;
      vec.vector.z = 0.5;
      stage->setDirection(vec);
      // stage->setGoal("ready");
      grasp->insert(std::move(stage));
    }
    task.add(std::move(grasp));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    // stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("ready");
    // task.add(std::move(stage));
  }

  {
    // clang-format off
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, interpolation_planner },
                                                  { hand_group_name, interpolation_planner } });
    // clang-format on
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    // task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    // clang-format off
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    // clang-format on

    /****************************************************
  ---- *               Generate Place Pose                *
     ***************************************************/
    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "object";
      // target_pose_msg.pose.position.
      target_pose_msg.pose.position.y = 0.5;
      target_pose_msg.pose.position.z = -0.3;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
      // clang-format on
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.x = -0.5;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    // task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    // stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("ready");
    // task.add(std::move(stage));
  }
  return task;
}



mtc::Task MTCTaskNode::createTask2(mtc::Stage* attach_object_stage)
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);


  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

 

  // {
  //   auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
  //   // stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
  //   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  //   stage->setGoal("ready");
  //   // task.add(std::move(stage));
  // }

  {
    // clang-format off
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, interpolation_planner },
                                                  { hand_group_name, interpolation_planner } });
    // clang-format on
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    // task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    // clang-format off
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    // clang-format on

    /****************************************************
  ---- *               Generate Place Pose                *
     ***************************************************/
    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "object";
      // target_pose_msg.pose.position.
      target_pose_msg.pose.position.y = 0.5;
      target_pose_msg.pose.position.z = -0.3;
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
      // clang-format on
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.x = -0.5;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    // task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    // stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("ready");
    // task.add(std::move(stage));
  }
  return task;
}



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();

  // rclcpp::init(argc, argv);
  // auto const node = std::make_shared<rclcpp::Node>(
  //   "hello_moveit",
  //   rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  // );
  // using moveit::planning_interface::MoveGroupInterface;
  // auto move_arm = MoveGroupInterface(node, "panda_arm");
  // auto move_gripper = MoveGroupInterface(node, "hand");
  // auto const logger = rclcpp::get_logger("hello_moveit");

  // move_arm.setNamedTarget("ready");
  // auto const [success_move, plan_move] = [&move_arm]{
  // moveit::planning_interface::MoveGroupInterface::Plan msg;
  // auto const ok = static_cast<bool>(move_arm.plan(msg));
  // return std::make_pair(ok, msg);
  // }();
  // if(success_move) {
  //   move_arm.execute(plan_move);
  // } else {
  //   RCLCPP_ERROR(logger, "Planing failed Move!");
  // }

  rclcpp::shutdown();
  return 0;
}