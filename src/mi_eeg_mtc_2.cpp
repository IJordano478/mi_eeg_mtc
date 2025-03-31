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
#include <map>

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
  mtc::Task createPickTask(std::string);
  mtc::Task createPlaceTask(std::string);
  mtc::Task task_get_A;
  mtc::Task task_get_B;
  mtc::Task task_get_C;
  mtc::Task task_get_D;
  mtc::Task task_place_A;
  mtc::Task task_place_B;
  mtc::Task task_place_C;
  mtc::Task task_place_D;
  double tile_xyz_locations[4][3]= {
    {0.5, 0.3, 0.0},
    {0.6, 0.3, 0.0},
    {0.6, 0.4, 0.0},
    {0.5, 0.4, 0.0},
  };
  std::map<std::string, int> object_index_map = {
    { "TileA", 0 },
    { "TileB", 1 },
    { "TileC", 2 },
    { "TileD", 3 },
    { "CubeA", 0 },
    { "CubeB", 1 },
    { "CubeC", 2 },
    { "CubeD", 3 },
};
  rclcpp::Node::SharedPtr node_;
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
  pose.position.y = -0.2;
  pose.position.z = 0.045/2;
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
      {0.5, -0.3, 0.0},
      {0.6, -0.3, 0.0},
      {0.6, -0.4, 0.0},
      {0.5, -0.4, 0.0},
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
    pose.position.z = 0.045/2;
    pose.orientation.w = 1.0;
    object.pose = pose;

    collision_objects.push_back(object); // Store the object in the vector
  }


  std::string tile_names[4] = {"TileA", "TileB", "TileC", "TileD"};
  for (int i = 0; i < std::size(tile_names); i++)
  {
    std::cout << "Adding cube: " << tile_names[i] << " at "
              << tile_xyz_locations[i][0] << ", "
              << tile_xyz_locations[i][1] << std::endl;

    moveit_msgs::msg::CollisionObject object;
    object.id = tile_names[i];
    object.header.frame_id = "world";

    object.primitives.resize(1); // Only one primitive per object
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions = {0.05, 0.05, 0.01};

    geometry_msgs::msg::Pose pose;
    pose.position.x = tile_xyz_locations[i][0];
    pose.position.y = tile_xyz_locations[i][1];
    pose.position.z = -0.01;
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
  mtc::Task task_get_A = createPickTask("CubeA");
  mtc::Task task_get_B = createPickTask("CubeB");
  mtc::Task task_get_C = createPickTask("CubeC");
  mtc::Task task_get_D = createPickTask("CubeD");

  // task_ = createTask();
  mtc::Task task_place_A = createPlaceTask("TileA");
  mtc::Task task_place_B = createPlaceTask("TileB");
  mtc::Task task_place_C = createPlaceTask("TileC");
  mtc::Task task_place_D = createPlaceTask("TileD");

  try
  {
    task_get_A.init();
    task_get_B.init();
    task_get_C.init();
    task_get_D.init();
    task_place_A.init();
    task_place_B.init();
    task_place_C.init();
    task_place_D.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_get_A.plan(5 /* max_solutions */))
  {RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");return;}
  task_get_A.introspection().publishSolution(*task_get_A.solutions().front());

  if (!task_get_B.plan(5 /* max_solutions */))
  {RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");return;}
  task_get_B.introspection().publishSolution(*task_get_B.solutions().front());

  if (!task_get_C.plan(5 /* max_solutions */))
  {RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");return;}
  task_get_C.introspection().publishSolution(*task_get_C.solutions().front());

  if (!task_get_D.plan(5 /* max_solutions */))
  {RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");return;}
  task_get_D.introspection().publishSolution(*task_get_D.solutions().front());

  if (!task_place_A.plan(5 /* max_solutions */))
  {RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");return;}
  task_place_A.introspection().publishSolution(*task_place_A.solutions().front());

  if (!task_place_B.plan(5 /* max_solutions */))
  {RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");return;}
  task_place_B.introspection().publishSolution(*task_place_B.solutions().front());

  if (!task_place_C.plan(5 /* max_solutions */))
  {RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");return;}
  task_place_C.introspection().publishSolution(*task_place_C.solutions().front());

  if (!task_place_D.plan(5 /* max_solutions */))
  {RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");return;}
  task_place_D.introspection().publishSolution(*task_place_D.solutions().front());
  

  auto result = task_get_A.execute(*task_get_A.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  result = task_get_A.execute(*task_get_A.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  result = task_get_A.execute(*task_place_A.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  result = task_get_A.execute(*task_get_A.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }



  // mtc::Task task2_ = createPlaceTask();
  // try
  // {
  //   task2_.init();
  // }
  // catch (mtc::InitStageException& e)
  // {
  //   RCLCPP_ERROR_STREAM(LOGGER, e);
  //   return;
  // }

  // if (!task2_.plan(5 /* max_solutions */))
  // {
  //   RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
  //   return;
  // }
  // task2_.introspection().publishSolution(*task2_.solutions().front());

  // auto result2 = task2_.execute(*task2_.solutions().front());
  // if (result2.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  // {
  //   RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
  //   return;
  // }

  // moveit::planning_interface::PlanningSceneInterface psi;
  // auto objs = psi.getObjects();
  // auto obj = objs["object"];
  // RCLCPP_INFO(LOGGER, "Object is at: x=%f y=%f z=%f",
  //             obj.pose.position.x, obj.pose.position.y, obj.pose.position.z);
  // try
  // {
  //   mtc::Task task_ = createPickTask(2);
  //   RCLCPP_INFO(LOGGER, "Created pick task");
  //   task_.init();
  //   RCLCPP_INFO(LOGGER, "Init");
  //   if (!task_.plan(5))
  //   {
  //     RCLCPP_ERROR(LOGGER, "Re-pick planning failed");
  //     return;
  //   }
  //   RCLCPP_INFO(LOGGER, "Planned");
  //   task_.introspection().publishSolution(*task_.solutions().front());
  //   RCLCPP_INFO(LOGGER, "Published");
  //   task_.execute(*task_.solutions().front());
  //   RCLCPP_INFO(LOGGER, "Executed");
  // }
  // catch (const mtc::InitStageException &e)
  // {
  //   RCLCPP_ERROR(LOGGER, "Init failed on re-pick: %s", e.what());
  // }
  // RCLCPP_INFO(LOGGER, "Done");

  return;
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
  rclcpp::shutdown();
  return 0;
}

mtc::Task MTCTaskNode::createPickTask(std::string cubename)
{
  mtc::Task task;
  // task.stages()->setName("pick block_A task");
  task.stages()->setName("pick " + cubename + " task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";
  RCLCPP_INFO(LOGGER, "Added panda parts");

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

  mtc::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator

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
      stage->setObject(cubename);
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
      stage->allowCollisions(cubename,
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

    // {
    //   auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    //   stage->attachObject(cubename, hand_frame);
    //   // attach_object_stage = stage.get();
    //   grasp->insert(std::move(stage));
    // }

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
  RCLCPP_INFO(LOGGER, "Finished generating task");
  return task;
}

mtc::Task MTCTaskNode::createPlaceTask(std::string tilename)
{
  mtc::Task task;
  task.stages()->setName("place " + tilename + " task");
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

  // Get the index from the map and then extract position
  int index = object_index_map.at(tilename);
  const auto& position = tile_xyz_locations[index];
  geometry_msgs::msg::PoseStamped target_pose_msg;
  target_pose_msg.header.frame_id = "world";  // Change to world frame
  target_pose_msg.pose.position.x = position[0];
  target_pose_msg.pose.position.y = position[1];
  target_pose_msg.pose.position.z = position[2];
  target_pose_msg.pose.orientation.w = 1.0;

  RCLCPP_INFO(LOGGER, "Placing '%s' at position: x=%.3f, y=%.3f, z=%.3f",
              tilename.c_str(), position[0], position[1], position[2]);

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
      stage->setObject(hand_frame);
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state
      //stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      // Compute IK
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(5);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(hand_frame);
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
      stage->allowCollisions(tilename,
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
      place->insert(std::move(stage));
    }

    // {
    //   auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    //   stage->detachObject("object", hand_frame);
    //   place->insert(std::move(stage));
    // }

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
