#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <chrono>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <control_msgs/action/gripper_command.hpp>

using GripperCommand = control_msgs::action::GripperCommand;
using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("demo2");

bool manipulator_plan_and_execute(moveit::planning_interface::MoveGroupInterface & move_group, 
                                  const std::vector<double> & target_pose, 
                                  const bool & left_arm);

void goal_response_callback(const GoalHandleGripperCommand::SharedPtr & goal_handle){
  if(!goal_handle){
    RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
  } else {
    RCLCPP_INFO(LOGGER, "Goal accepted by server, waiting for result");
  }
}
void feedback_callback(GoalHandleGripperCommand::SharedPtr, const std::shared_ptr<const GripperCommand::Feedback> feedback){
  RCLCPP_INFO(LOGGER, "Got Feedback: Current position is %f", feedback->position);
}
void result_callback(const GoalHandleGripperCommand::WrappedResult & result){
  switch (result.code)
  {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(LOGGER, "Goal was aborted");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(LOGGER, "Goal was canceled");
    return;
  default:
    RCLCPP_ERROR(LOGGER, "Unknown result code");
    return;
  }
  RCLCPP_INFO(LOGGER, "Goal is completed, current position is %f", result.result->position);
}

int main(int argc, char ** argv){
  /**
   * ROS2 init
  */
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto demo2_node = rclcpp::Node::make_shared("demo2", node_options);

  const std::vector<double> target_pose_1 = demo2_node->get_parameter("target_pose_1").as_double_array();
  const std::vector<double> target_pose_2 = demo2_node->get_parameter("target_pose_2").as_double_array();
  const double target_grasp_angle = demo2_node->get_parameter("target_grasp_angle").as_double();
  const std::vector<double> target_pose_3 = demo2_node->get_parameter("target_pose_3").as_double_array();
  const std::vector<double> target_pose_4 = demo2_node->get_parameter("target_pose_4").as_double_array();
  const std::string which_arm = demo2_node->get_parameter("which_arm").as_string();
  const bool left_arm = [&which_arm](){
    if(which_arm == "left"){
      return true;
    } else if(which_arm == "right"){
      return false;
    } else {
      RCLCPP_ERROR(LOGGER, "Invalid arm name");
      rclcpp::shutdown();
      return false;
    }
  }();

  // action
  const std::string gripper_action_name = 
    left_arm ? "left_gripper_controller/gripper_cmd" : "right_gripper_controller/gripper_cmd";
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client = 
    rclcpp_action::create_client<GripperCommand>(demo2_node, gripper_action_name);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(demo2_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  /**
   * MoveIt2 init
  */
  static const std::string PLANNING_GROUP = left_arm ? "left_ur_manipulator" : "right_ur_manipulator";

  // interface and joint group
  moveit::planning_interface::MoveGroupInterface move_group(demo2_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // const moveit::core::JointModelGroup * joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // get basic information 
  RCLCPP_INFO(LOGGER, "Planning (Reference) frame: %s", move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
  
  // add collision object (here is the table) ROS message for the robot to avoid 
  moveit_msgs::msg::CollisionObject collision_table;
  collision_table.header.frame_id = move_group.getPlanningFrame();
  collision_table.id = "table";
  // define box to add to the world
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 1.0;
  primitive.dimensions[primitive.BOX_Y] = 1.5;
  primitive.dimensions[primitive.BOX_Z] = 1.0;
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.5;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.5;
  collision_table.primitives.push_back(primitive);
  collision_table.primitive_poses.push_back(box_pose);
  collision_table.operation = collision_table.ADD;
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_table);
  RCLCPP_INFO(LOGGER, "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // step 1
  // move to target pose 1
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(3.0);
  
  bool success = manipulator_plan_and_execute(move_group, target_pose_1, left_arm);
  if(!success){
    rclcpp::shutdown();
    return 0;
  }
  // step 2
  // move to target pose 2 and grasp the object via action
  success = manipulator_plan_and_execute(move_group, target_pose_2, left_arm);
  if(!success){
    rclcpp::shutdown();
    return 0;
  }

  auto goal_msg = GripperCommand::Goal();
  goal_msg.command.position = target_grasp_angle;
  goal_msg.command.max_effort = 1.0;
  if(!gripper_client->wait_for_action_server(std::chrono::seconds(10))){
    RCLCPP_ERROR(LOGGER, "Action server not available after waiting");
    rclcpp::shutdown();
    return 0;
  }
  RCLCPP_INFO(LOGGER, "Sending gripper goal");
  auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&goal_response_callback, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&feedback_callback, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(&result_callback, std::placeholders::_1);
  gripper_client->async_send_goal(goal_msg, send_goal_options);
  
  std::this_thread::sleep_for(std::chrono::seconds(2)); // wait for grasping

  // step 3
  // move to target pose 3
  success = manipulator_plan_and_execute(move_group, target_pose_3, left_arm);
  if(!success){
    rclcpp::shutdown();
    return 0;
  }

  // step 4
  // move to target pose 4
  success = manipulator_plan_and_execute(move_group, target_pose_4, left_arm);
  if(!success){
    rclcpp::shutdown();
    return 0;
  }


  rclcpp::shutdown();
  return 0;
}

bool manipulator_plan_and_execute(moveit::planning_interface::MoveGroupInterface & move_group, 
                                  const std::vector<double> & target_pose, 
                                  const bool & left_arm){
  tf2::Quaternion target_quat;
  target_quat.setRPY(target_pose[3], target_pose[4], target_pose[5]);
  target_quat.normalize();
  geometry_msgs::msg::PoseStamped target_pose_stamped;
  target_pose_stamped.header.frame_id = left_arm ? "left_base_link" : "right_base_link";
  target_pose_stamped.pose.position.x = target_pose[0];
  target_pose_stamped.pose.position.y = target_pose[1];
  target_pose_stamped.pose.position.z = target_pose[2];
  target_pose_stamped.pose.orientation.w = target_quat.w();
  target_pose_stamped.pose.orientation.x = target_quat.x();
  target_pose_stamped.pose.orientation.y = target_quat.y();
  target_pose_stamped.pose.orientation.z = target_quat.z();
  move_group.setJointValueTarget(target_pose_stamped);
  // move_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if(success){
    RCLCPP_INFO(LOGGER, "Planning to target pose success");
    move_group.move();
  } else {
    RCLCPP_INFO(LOGGER, "Planning to target pose fail");
  }
  return success;
}