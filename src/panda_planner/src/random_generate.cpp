#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>
#include <moveit_msgs/msg/detail/collision_object__struct.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <unistd.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit_msgs/msg/detail/attached_collision_object__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
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

// MTC Node for task creation and execution
class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface2();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task task_;
  mtc::Task task_2;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr node_2;

  // Limits for random generation objects
  float coordinate_limits [6];
  int num_objects;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }, node_2{ std::make_shared<rclcpp::Node>("mtc_node2", options) }
{
    // Make x_max_limit, and y_limits as input params
    // For now initialized as standard value
    // Pass it as FloatArray and then for loop over it
    coordinate_limits[0] = 0.6;
    coordinate_limits[1] = 2.6;
    coordinate_limits[2] = -1.0;
    coordinate_limits[3] = 1.0;
    coordinate_limits[4] = 0.0;
    coordinate_limits[5] = 1.0;

    num_objects = 4;    
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface2()
{
  return node_2->get_node_base_interface();
}

// Add obstacles to RViz Scene
void MTCTaskNode::setupPlanningScene()
{
    RCLCPP_INFO(LOGGER, "Start Planning Setup");

    srand(time(0));

    moveit::planning_interface::PlanningSceneInterface psi;

    moveit_msgs::msg::CollisionObject obstacle;
    geometry_msgs::msg::Pose pose;
    int randNum;
    for (int i = 0; i < num_objects; i++){
        // RCLCPP_INFO(LOGGER, "Forming obstacle number %i", i);

        obstacle.id = "object" + std::to_string(i);
        obstacle.header.frame_id = "world";
        obstacle.primitives.resize(1);

        randNum = rand() % 4 + 1;
        obstacle.primitives[0].type = randNum;
        RCLCPP_INFO(LOGGER, "Generating object %i", randNum);

        if (randNum == 1)
            obstacle.primitives[0].dimensions = { 0.1 + ((double)rand() / RAND_MAX) * 0.1, 0.1 + ((double)rand() / RAND_MAX) * 0.1, 0.1 + ((double)rand() / RAND_MAX) * 0.1 };
        else if (randNum == 2)
            obstacle.primitives[0].dimensions = { 0.1 + ((double)rand() / RAND_MAX) * 0.3 };
        else
            obstacle.primitives[0].dimensions = { 0.1 + ((double)rand() / RAND_MAX) * 0.1, 0.1 + ((double)rand() / RAND_MAX) };
        

        pose.position.x = coordinate_limits[0] + ((double)rand() / RAND_MAX) * (coordinate_limits[1] - coordinate_limits[0]);
        pose.position.y = coordinate_limits[2] + ((double)rand() / RAND_MAX) * (coordinate_limits[3] - coordinate_limits[2]);
        pose.position.z = coordinate_limits[4] + ((double)rand() / RAND_MAX) * (coordinate_limits[5] - coordinate_limits[4]);
        pose.orientation.w = 1.0;
        obstacle.pose = pose;

        RCLCPP_INFO(LOGGER, "%f %f %f %f", pose.position.x, pose.position.y, pose.position.z, obstacle.primitives[0].dimensions[0]);

        psi.applyCollisionObject(obstacle);
    }

    obstacle.id = "object" + std::to_string(num_objects);
    obstacle.header.frame_id = "world";
    obstacle.primitives.resize(1);

    randNum = rand() % 4 + 1;
    obstacle.primitives[0].type = randNum;
    RCLCPP_INFO(LOGGER, "Generating object %i", randNum);

    if (randNum == 1)
        obstacle.primitives[0].dimensions = { 0.1 + ((double)rand() / RAND_MAX) * 0.1, 0.1 + ((double)rand() / RAND_MAX) * 0.1, 0.1 + ((double)rand() / RAND_MAX) * 0.1 };
    else if (randNum == 2)
        obstacle.primitives[0].dimensions = { 0.1 + ((double)rand() / RAND_MAX) * 0.3 };
    else
        obstacle.primitives[0].dimensions = { 0.1 + ((double)rand() / RAND_MAX) * 0.1, 0.1 + ((double)rand() / RAND_MAX) };
    

    pose.position.x = coordinate_limits[0] + ((double)rand() / RAND_MAX) * (coordinate_limits[1] - coordinate_limits[0]);
    pose.position.y = coordinate_limits[2] + ((double)rand() / RAND_MAX) * (coordinate_limits[3] - coordinate_limits[2]);
    pose.position.z = coordinate_limits[4] + ((double)rand() / RAND_MAX) * (coordinate_limits[5] - coordinate_limits[4]);
    pose.orientation.w = 1.0;
    obstacle.pose = pose;

    RCLCPP_INFO(LOGGER, "%f %f %f %f", pose.position.x, pose.position.y, pose.position.z, obstacle.primitives[0].dimensions[0]);

    psi.applyCollisionObject(obstacle);

    std_msgs::msg::ColorRGBA color;
    color.r = 1.0;
    color.b = 0.0;
    color.g = 0.0;
    color.a = 1.0;

    
    // planning_scene_monitor::PlanningSceneMonitor planScene;
    // planning_scene::PlanningScene::setObjectColor("object" + std::to_string(num_objects), color);

//   moveit_msgs::msg::CollisionObject object1;
//   object1.id = "cyl1";
//   object1.header.frame_id = "world";
//   object1.primitives.resize(1);
//   object1.primitives[0].type = 3;
//   object1.primitives[0].dimensions = { 0.25, 0.02 };

//   geometry_msgs::msg::Pose pose1;
//   pose1.position.x = 0.25;
//   pose1.position.y = -0.5;
//   pose1.position.z = 0.125;
//   pose1.orientation.w = 1.0;
//   object1.pose = pose1;

//   moveit_msgs::msg::CollisionObject object2;
//   object2.id = "cyl2";
//   object2.header.frame_id = "world";
//   object2.primitives.resize(1);
//   object2.primitives[0].type = 3;
//   object2.primitives[0].dimensions = { 0.25, 0.02 };

//   geometry_msgs::msg::Pose pose2;
//   pose2.position.x = 0.5;
//   pose2.position.y = -0.25;
//   pose2.position.z = 0.125;
//   pose2.orientation.w = 1.0;
//   object2.pose = pose2;

//   moveit_msgs::msg::CollisionObject object3;
//   object3.id = "box1";
//   object3.header.frame_id = "world";
//   object3.primitives.resize(1);
//   object3.primitives[0].type = 1;
//   object3.primitives[0].dimensions = { 0.4, 0.3, 0.05 };

//   geometry_msgs::msg::Pose pose3;
//   pose3.position.x = 0.375;
//   pose3.position.y = -0.375;
//   pose3.position.z = object2.primitives[0].dimensions[0] + (object3.primitives[0].dimensions[2] / 2);
//   pose3.orientation.w = 1.0;
//   object3.pose = pose3;

//   moveit_msgs::msg::CollisionObject object4;
//   object4.id = "cyl3";
//   object4.header.frame_id = "world";
//   object4.primitives.resize(1);
//   object4.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
//   object4.primitives[0].dimensions = { 0.25, 0.02 };

//   geometry_msgs::msg::Pose pose4;
//   pose4.position.x = 0.5;
//   pose4.position.y = -0.5;
//   pose4.position.z = 0.125;
//   pose4.orientation.w = 1.0;
//   object4.pose = pose4;

//   moveit_msgs::msg::CollisionObject object5;
//   object5.id = "cyl4";
//   object5.header.frame_id = "world";
//   object5.primitives.resize(1);
//   object5.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
//   object5.primitives[0].dimensions = { 0.25, 0.02 };

//   geometry_msgs::msg::Pose pose5;
//   pose5.position.x = 0.25;
//   pose5.position.y = -0.25;
//   pose5.position.z = 0.125;
//   pose5.orientation.w = 1.0;
//   object5.pose = pose5;

//   moveit_msgs::msg::CollisionObject object;
//   object.id = "object";
//   object.header.frame_id = "world";
//   object.primitives.resize(1);
//   object.primitives[0].type = shape_msgs::msg::SolidPrimitive::SPHERE;
//   object.primitives[0].dimensions = { 0.032 };

//   geometry_msgs::msg::Pose pose;
//   pose.position.x = 0.5;
//   pose.position.y = -0.3;
//   pose.position.z = object2.primitives[0].dimensions[0] + object3.primitives[0].dimensions[2] + (object.primitives[0].dimensions[0]);
//   pose.orientation.w = 1.0;
//   object.pose = pose;

//   moveit_msgs::msg::AttachedCollisionObject aobject;
//   aobject.set__object(object);

  
//   psi.applyCollisionObject(object1);
//   psi.applyCollisionObject(object2);
//   psi.applyCollisionObject(object3);
//   psi.applyCollisionObject(object4);
//   psi.applyCollisionObject(object5);
//   psi.applyCollisionObject(object);
}



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::executors::MultiThreadedExecutor executor2;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();

  spin_thread->join();

  rclcpp::shutdown();
  return 0;
}