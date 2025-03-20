#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
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

#include <string>

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
  mtc::Task createTask();
  mtc::Task createPickTask();
  mtc::Task createPlaceTask();
  mtc::Task task_;
  mtc::Task task2_;
  rclcpp::Node::SharedPtr node_;
  mtc::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mi_eeg_mtc", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(3);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = { 0.045, 0.045 , 0.045};

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = -0.25;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);

  // moveit_msgs::msg::CollisionObject object2;
  // object2.id = "object2";
  // object2.header.frame_id = "world";
  // object2.primitives.resize(3);
  // object2.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  // object2.primitives[0].dimensions = { 0.045, 0.045 , 0.045};

  // geometry_msgs::msg::Pose pose2;
  // pose2.position.x = 0.8;
  // pose2.position.y = -0.25;
  // pose2.orientation.w = 1.0;
  // object2.pose = pose2;
  // psi.applyCollisionObject(object2);

  double cube_xyz_locations[4][3] = {
      {0.5, -0.5, 0.0},
      {0.6, -0.5, 0.0},
      {0.6, -0.6, 0.0},
      {0.5, -0.6, 0.0},
  };
  std::string cube_names[4] = {"CubeA", "CubeB", "CubeC", "CubeD"};

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

  for (int i = 0; i < std::size(cube_names); i++)
  {
    std::cout << "Adding cube: " << cube_names[i] << " at "
              << cube_xyz_locations[i][0] << ", "
              << cube_xyz_locations[i][1] << std::endl;

    moveit_msgs::msg::CollisionObject object;
    object.id = cube_names[i];
    object.header.frame_id = "world";

    object.primitives.resize(1); // Only one primitive per object
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions = {0.045, 0.045, 0.045};

    geometry_msgs::msg::Pose pose;
    pose.position.x = cube_xyz_locations[i][0];
    pose.position.y = cube_xyz_locations[i][1];
    pose.orientation.w = 1.0;
    object.pose = pose;

    collision_objects.push_back(object); // Store the object in the vector
  }

  // Apply all objects at once
  psi.applyCollisionObjects(collision_objects);
}

void MTCTaskNode::doTask()
{
  // task_ = createTask();
  task_ = createPickTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5 /* max_solutions */))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }



  task2_ = createPlaceTask();
  try
  {
    task2_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task2_.plan(5 /* max_solutions */))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task2_.introspection().publishSolution(*task2_.solutions().front());

  auto result2 = task2_.execute(*task2_.solutions().front());
  if (result2.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

// mtc::Task MTCTaskNode::createTask()
// {
//   mtc::Task task;
//   task.stages()->setName("demo task");
//   task.loadRobotModel(node_);

//   const auto& arm_group_name = "panda_arm";
//   const auto& hand_group_name = "hand";
//   const auto& hand_frame = "panda_hand";

//   // Set task properties
//   task.setProperty("group", arm_group_name);
//   task.setProperty("eef", hand_group_name);
//   task.setProperty("ik_frame", hand_frame);

//   mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
//   auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
//   current_state_ptr = stage_state_current.get();
//   task.add(std::move(stage_state_current));

//   auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
//   auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

//   auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
//   cartesian_planner->setMaxVelocityScalingFactor(1.0);
//   cartesian_planner->setMaxAccelerationScalingFactor(1.0);
//   cartesian_planner->setStepSize(.01);

//   // clang-format off
//   auto stage_open_hand =
//       std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
//   // clang-format on
//   stage_open_hand->setGroup(hand_group_name);
//   stage_open_hand->setGoal("open");
//   task.add(std::move(stage_open_hand));

//   // clang-format off
//   auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
//       "move to pick",
//       mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
//   // clang-format on
//   stage_move_to_pick->setTimeout(5.0);
//   stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
//   task.add(std::move(stage_move_to_pick));

//   // clang-format off
//   mtc::Stage* attach_object_stage =
//       nullptr;  // Forward attach_object_stage to place pose generator
//   // clang-format on

//   // This is an example of SerialContainer usage. It's not strictly needed here.
//   // In fact, `task` itself is a SerialContainer by default.
//   {
//     auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
//     task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
//     // clang-format off
//     grasp->properties().configureInitFrom(mtc::Stage::PARENT,
//                                           { "eef", "group", "ik_frame" });
//     // clang-format on

//     {
//       // clang-format off
//       auto stage =
//           std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
//       // clang-format on
//       stage->properties().set("marker_ns", "approach_object");
//       stage->properties().set("link", hand_frame);
//       stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
//       stage->setMinMaxDistance(0.1, 0.15);

//       // Set hand forward direction
//       geometry_msgs::msg::Vector3Stamped vec;
//       vec.header.frame_id = hand_frame;
//       vec.vector.z = 1.0;
//       stage->setDirection(vec);
//       grasp->insert(std::move(stage));
//     }

//     /****************************************************
//   ---- *               Generate Grasp Pose                *
//      ***************************************************/
//     {
//       // Sample grasp pose
//       auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
//       stage->properties().configureInitFrom(mtc::Stage::PARENT);
//       stage->properties().set("marker_ns", "grasp_pose");
//       stage->setPreGraspPose("open");
//       stage->setObject("object");
//       // stage->setAngleDelta(M_PI / 12);
//       stage->setAngleDelta(M_PI);
//       stage->setMonitoredStage(current_state_ptr);  // Hook into current state

//       // This is the transform from the object frame to the end-effector frame
//       Eigen::Isometry3d grasp_frame_transform;
//       // Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
//       //                        Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
//       //                        Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
//       Eigen::Quaterniond q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
//                              Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
//                              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()); // Rotate wrist to point down
//       grasp_frame_transform.linear() = q.matrix();
//       grasp_frame_transform.translation().z() = 0.1;

//       // Compute IK
//       // clang-format off
//       auto wrapper =
//           std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
//       // clang-format on
//       wrapper->setMaxIKSolutions(8);
//       wrapper->setMinSolutionDistance(1.0);
//       wrapper->setIKFrame(grasp_frame_transform, hand_frame);
//       wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
//       wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
//       grasp->insert(std::move(wrapper));
//     }

//     {
//       // clang-format off
//       auto stage =
//           std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
//       stage->allowCollisions("object",
//                              task.getRobotModel()
//                                  ->getJointModelGroup(hand_group_name)
//                                  ->getLinkModelNamesWithCollisionGeometry(),
//                              true);
//       // clang-format on
//       grasp->insert(std::move(stage));
//     }

//     {
//       auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
//       stage->setGroup(hand_group_name);
//       stage->setGoal("close");
//       grasp->insert(std::move(stage));
//     }

//     {
//       auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
//       stage->attachObject("object", hand_frame);
//       attach_object_stage = stage.get();
//       grasp->insert(std::move(stage));
//     }

//     {
//       // clang-format off
//       auto stage =
//           std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
//       // clang-format on
//       stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
//       stage->setMinMaxDistance(0.1, 0.3);
//       stage->setIKFrame(hand_frame);
//       stage->properties().set("marker_ns", "lift_object");

//       // Set upward direction
//       geometry_msgs::msg::Vector3Stamped vec;
//       vec.header.frame_id = "world";
//       vec.vector.z = 1.0;
//       stage->setDirection(vec);
//       grasp->insert(std::move(stage));
//     }
//     task.add(std::move(grasp));
//   }

//   {
//     // clang-format off
//     auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
//         "move to place",
//         mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
//                                                   { hand_group_name, interpolation_planner } });
//     // clang-format on
//     stage_move_to_place->setTimeout(5.0);
//     stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
//     task.add(std::move(stage_move_to_place));
//   }

//   {
//     auto place = std::make_unique<mtc::SerialContainer>("place object");
//     task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
//     // clang-format off
//     place->properties().configureInitFrom(mtc::Stage::PARENT,
//                                           { "eef", "group", "ik_frame" });
//     // clang-format on

//     /****************************************************
//   ---- *               Generate Place Pose                *
//      ***************************************************/
//     {
//       // Sample place pose
//       auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
//       stage->properties().configureInitFrom(mtc::Stage::PARENT);
//       stage->properties().set("marker_ns", "place_pose");
//       stage->setObject("object");

//       geometry_msgs::msg::PoseStamped target_pose_msg;
//       target_pose_msg.header.frame_id = "object";
//       target_pose_msg.pose.position.y = 0.5;
//       target_pose_msg.pose.orientation.w = 1.0;
//       stage->setPose(target_pose_msg);
//       stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

//       // Compute IK
//       // clang-format off
//       auto wrapper =
//           std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
//       // clang-format on
//       wrapper->setMaxIKSolutions(2);
//       wrapper->setMinSolutionDistance(1.0);
//       wrapper->setIKFrame("object");
//       wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
//       wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
//       place->insert(std::move(wrapper));
//     }

//     {
//       auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
//       stage->setGroup(hand_group_name);
//       stage->setGoal("open");
//       place->insert(std::move(stage));
//     }

//     {
//       // clang-format off
//       auto stage =
//           std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
//       stage->allowCollisions("object",
//                              task.getRobotModel()
//                                  ->getJointModelGroup(hand_group_name)
//                                  ->getLinkModelNamesWithCollisionGeometry(),
//                              false);
//       // clang-format on
//       place->insert(std::move(stage));
//     }

//     {
//       auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
//       stage->detachObject("object", hand_frame);
//       place->insert(std::move(stage));
//     }

//     {
//       auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
//       stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
//       stage->setMinMaxDistance(0.1, 0.3);
//       stage->setIKFrame(hand_frame);
//       stage->properties().set("marker_ns", "retreat");

//       // Set retreat direction
//       geometry_msgs::msg::Vector3Stamped vec;
//       vec.header.frame_id = "world";
//       // vec.vector.x = -0.5;
//       vec.vector.z = 0.5;
//       stage->setDirection(vec);
//       place->insert(std::move(stage));
//     }
//     task.add(std::move(place));
//   }

//   {
//     auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
//     stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
//     stage->setGoal("ready");
//     task.add(std::move(stage));
//   }
//   return task;
// }

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
  rclcpp::shutdown();
  return 0;
}

mtc::Task MTCTaskNode::createPickTask()
{
  mtc::Task task;
  task.stages()->setName("pick block_A task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage* current_state_ptr = nullptr;  
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>("move to pick",
    mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  // mtc::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator

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
      // stage->setAngleDelta(M_PI / 12);
      stage->setAngleDelta(M_PI);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state

      // This is the transform from the object frame to the end-effector frame
      Eigen::Isometry3d grasp_frame_transform;
      // Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
      //                        Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
      //                        Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      Eigen::Quaterniond q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()); // Rotate wrist to point down
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.1;

      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
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
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    task.add(std::move(grasp));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("ready");
    task.add(std::move(stage));
  }
  return task;
}

mtc::Task MTCTaskNode::createPlaceTask()
{
  mtc::Task task;
  task.stages()->setName("place block_A task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage* current_state_ptr = nullptr;  
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  geometry_msgs::msg::PoseStamped target_pose_msg;
  target_pose_msg.header.frame_id = "world";  // Change to world frame
  target_pose_msg.pose.position.x = 0.5; // Set target position
  target_pose_msg.pose.position.y = 0.25;
  target_pose_msg.pose.position.z = 0.0;  // Place on table height
  target_pose_msg.pose.orientation.w = 1.0;

  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                  { hand_group_name, interpolation_planner } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          {"eef", "group", "ik_frame"});

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach target", cartesian_planner);
      // clang-format on
      stage->properties().set("marker_ns", "approach_target");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    /****************************************************
  ---- *               Generate Place Pose                *
    ***************************************************/
    // {
    //   auto stage =
    //       std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
    //   stage->allowCollisions("object",
    //                          task.getRobotModel()
    //                              ->getJointModelGroup(hand_group_name)
    //                              ->getLinkModelNamesWithCollisionGeometry(),
    //                          true);
    //   place->insert(std::move(stage));
    // }

    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state
      //stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      // Compute IK
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(5);
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
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
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
      // vec.vector.x = -0.5;
      vec.vector.z = 0.5;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("ready");
    task.add(std::move(stage));
  }

  return task;
}
