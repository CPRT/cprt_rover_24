#include "moveit_controller.h"

using moveit::planning_interface::MoveGroupInterface;
using std::placeholders::_1;

void executeTrajectory(moveit_msgs::msg::RobotTrajectory &traj,
                       moveit::planning_interface::MoveGroupInterfacePtr mgi) {
  RCLCPP_INFO(rclcpp::get_logger("moveit_controller"), "Starting thread");
  mgi->asyncExecute(traj);
  RCLCPP_INFO(rclcpp::get_logger("moveit_controller"), "Ending thread");
}

bool isEmpty(const geometry_msgs::msg::Pose &p) {
  return p.position.x == 0 && p.position.y == 0 && p.position.z == 0 &&
         p.orientation.x == 0 && p.orientation.y == 0 && p.orientation.z == 0 &&
         p.orientation.w == 0;
}

MoveitController::MoveitController(const rclcpp::NodeOptions &options)
    : Node("moveit_controller", options),
      node_ptr_(std::make_shared<rclcpp::Node>("example_moveit")),
      executor_ptr_(
          std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {
  subscription_ = this->create_subscription<interfaces::msg::ArmCmd>(
      "arm_base_commands", 10,
      std::bind(&MoveitController::topic_callback, this,
                std::placeholders::_1));

  move_group_ptr_ =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          node_ptr_, "rover_arm3");  // used to be rover_arm, then rover_arm2

  executor_ptr_->add_node(node_ptr_);
  executor_thread_ = std::thread([this]() { this->executor_ptr_->spin(); });

  // default pose, chosen to optimize starting movement
  default_pose_.position.x = ARM_DEFAULT_X;
  default_pose_.position.y = ARM_DEFAULT_Y;
  default_pose_.position.z = ARM_DEFAULT_Z;

  geometry_msgs::msg::Pose target_pose;
  target_pose.position = default_pose_.position;
  move_group_ptr_->setMaxVelocityScalingFactor(1.0);
  move_group_ptr_->setMaxAccelerationScalingFactor(1.0);
  move_group_ptr_->setPoseTarget(target_pose);

  // Create a plan to that target pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group_ptr_->plan(plan));

  // Execute the plan
  if (success) {
    move_group_ptr_->execute(plan);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Planing failed!");
  }
}

void MoveitController::topic_callback(const interfaces::msg::ArmCmd &armMsg) {
  geometry_msgs::msg::Pose poseMsg = armMsg.pose;
  double stepSize = armMsg.speed;
  RCLCPP_INFO(this->get_logger(), "I heard: %f %f %f %f %f %f %f",
              poseMsg.position.x, poseMsg.position.y, poseMsg.position.z,
              poseMsg.orientation.x, poseMsg.orientation.y,
              poseMsg.orientation.z,
              poseMsg.orientation.w);  //*/

  move_group_ptr_->stop();
  if (move_it_thread_.joinable()) {
    move_it_thread_.join();
  }
  if ((isEmpty(poseMsg) && !armMsg.reset &&
       armMsg.end_effector == interfaces::msg::ArmCmd::END_EFF_UNKNOWN &&
       !armMsg.query_goal_state) ||
      armMsg.estop) {
    RCLCPP_INFO(this->get_logger(), "Stopping the arm");
    return;
  }

  std::vector<geometry_msgs::msg::Pose> points;

  move_group_ptr_->setStartStateToCurrentState();
  geometry_msgs::msg::Pose current_pose =
      move_group_ptr_->getCurrentPose().pose;

  points.push_back(current_pose);

  // Print the current pose of the end effector
  RCLCPP_INFO(this->get_logger(), "Current pose: %f %f %f %f %f %f %f",
              current_pose.position.x, current_pose.position.y,
              current_pose.position.z, current_pose.orientation.x,
              current_pose.orientation.y, current_pose.orientation.z,
              current_pose.orientation.w);  //*/

  if (armMsg.end_effector != interfaces::msg::ArmCmd::END_EFF_UNKNOWN) {
    // TODO: Add end-effector features
  }
  if (armMsg.reset == true)  // reset to default position
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position = default_pose_.position;
    move_group_ptr_->setPoseTarget(target_pose);

    // Create a plan to that target pose
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_ptr_->plan(plan));

    // Execute the plan
    if (success) {
      move_group_ptr_->execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planing failed!");
    }
  } else {
    tf2::Vector3 dir;
  tf2::fromMsg(poseMsg.position, dir);
  tf2::Quaternion quaternion;
  tf2::fromMsg(current_pose.orientation, quaternion);
  
  tf2::Vector3 localTransform = quatRotate(quaternion, dir);
  /*auto const new_pose = [&]{
    geometry_msgs::msg::Pose msg = current_pose;
    msg.position.x += poseMsg.position.x*stepSize;
    msg.position.y += poseMsg.position.y*stepSize;
    msg.position.z += poseMsg.position.z*stepSize;
    
    
    return msg;
  }();*/
  
 
    geometry_msgs::msg::Pose msg = current_pose;
    msg.position.x += localTransform.getX()*stepSize;
    msg.position.y += localTransform.getY()*stepSize;
    msg.position.z += localTransform.getZ()*stepSize;
    tf2::Quaternion current;
    tf2::Quaternion desired;
    tf2::convert(msg.orientation, current);
    tf2::convert(poseMsg.orientation, desired);
    current *= desired;
    tf2::convert(current, msg.orientation);
    auto new_pose = msg;
    
   

    if (poseMsg.orientation.x != 0 || poseMsg.orientation.y != 0 ||
        poseMsg.orientation.z != 0 ||
        poseMsg.orientation.w != 0)  // rotation required
    {
      tf2::Quaternion q1;
      tf2::convert(current_pose.orientation, q1);
      tf2::Quaternion q2;
      q2.setRPY(poseMsg.orientation.x, poseMsg.orientation.y,
                poseMsg.orientation.z);
      tf2::Quaternion q3 = q1 * q2;
      geometry_msgs::msg::Quaternion q4 = tf2::toMsg(q3);

      new_pose.orientation = q4;
    }
    points.push_back(new_pose);
    move_group_ptr_->computeCartesianPath(points, EEF_STEP, JUMP_THRESHOLD,
                                          trajectory_);

    // launch thread
    move_it_thread_ =
        std::thread(executeTrajectory, std::ref(trajectory_), move_group_ptr_);
    RCLCPP_INFO(this->get_logger(), "Thread created");
  }
}

int main(int argc, char **argv) {
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveitController>(
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));
  rclcpp::spin(node);
}
