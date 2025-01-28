#include "moveit_controller.h"

using std::placeholders::_1;
using moveit::planning_interface::MoveGroupInterface;

void executeTrajectory(moveit_msgs::msg::RobotTrajectory &traj, moveit::planning_interface::MoveGroupInterfacePtr mgi)
{
  RCLCPP_INFO(rclcpp::get_logger("hello_moveit"), "Starting thread");
  mgi->asyncExecute(traj);
  RCLCPP_INFO(rclcpp::get_logger("hello_moveit"), "Ending thread");
}

void executePlan(moveit::planning_interface::MoveGroupInterface::Plan &rotationPlan, moveit::planning_interface::MoveGroupInterfacePtr mgi)
{
  RCLCPP_INFO(rclcpp::get_logger("hello_moveit"), "Starting thread");
  mgi->asyncExecute(rotationPlan);
  RCLCPP_INFO(rclcpp::get_logger("hello_moveit"), "Ending thread");
}

bool isEmpty(const geometry_msgs::msg::Pose &p)
{
  return p.position.x == 0 && p.position.y == 0 && p.position.z == 0 && p.orientation.x == 0 && p.orientation.y == 0 && p.orientation.z == 0 && p.orientation.w == 0;
}


TestNode::TestNode(const rclcpp::NodeOptions &options)
: Node("hello_moveit", options), node_ptr(std::make_shared<rclcpp::Node>("example_moveit")), executor_ptr(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
{
  node_name = ((std::string) this->get_namespace()).erase(0, 1);//"hello_moveit";
  //node_name.pop_back();
  
  RCLCPP_INFO(this->get_logger(), node_name.c_str());
  
  subscription_ = this->create_subscription<interfaces::msg::ArmCmd>(
  "arm_base_commands", 10, std::bind(&TestNode::topic_callback, this, std::placeholders::_1));
  
  publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("arm_trajectory", 11);
  
  move_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, "rover_arm3"); //used to be rover_arm, then rover_arm2
  
  executor_ptr->add_node(node_ptr);
  executor_thread = std::thread([this]() {this->executor_ptr->spin(); });
  
  geometry_msgs::msg::Pose target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 1.019404;
    msg.position.y = -0.028744;
    msg.position.z = -0.007406;
    return msg;
  }();
  move_group_ptr->setMaxVelocityScalingFactor(1.0);
  move_group_ptr->setMaxAccelerationScalingFactor(1.0);
  move_group_ptr->setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_ptr->plan(msg));
    return std::make_pair(ok, msg);
  }();
  
  publisher_->publish(plan.trajectory_.joint_trajectory);

  // Execute the plan
  if(success) {
    move_group_ptr->execute(plan);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Planing failed!");
  }
  
}


void TestNode::topic_callback(const interfaces::msg::ArmCmd & armMsg)
{
  geometry_msgs::msg::Pose poseMsg = armMsg.pose;
  double stepSize = armMsg.speed;
  RCLCPP_INFO(this->get_logger(), "I heard: %f %f %f %f %f %f %f",
    poseMsg.position.x,
    poseMsg.position.y,
    poseMsg.position.z,
    poseMsg.orientation.x,
    poseMsg.orientation.y,
    poseMsg.orientation.z,
    poseMsg.orientation.w);//*/
  
  
  move_group_ptr->stop();
  if (th.joinable())
  {
    th.join();
  }
  if ((isEmpty(poseMsg) && !armMsg.reset && armMsg.named_pose == 0 && !armMsg.query_goal_state) || armMsg.estop)
  {
    RCLCPP_INFO(this->get_logger(), "Stopping (for some reason)");
    return;
  }
  
  auto myid = std::this_thread::get_id();
  std::stringstream ss;
  ss << myid;
  std::string mystring = ss.str();
  
  
  RCLCPP_INFO(this->get_logger(), mystring.c_str());
  RCLCPP_INFO(this->get_logger(), std::to_string(this->get_clock()->now().seconds()).c_str());
  
  std::vector<geometry_msgs::msg::Pose> points;
  
  move_group_ptr->setStartStateToCurrentState();
  geometry_msgs::msg::Pose current_pose = move_group_ptr->getCurrentPose().pose;
  
  points.push_back(current_pose);
  
  // Print the current pose of the end effector
  RCLCPP_INFO(this->get_logger(), "Current pose: %f %f %f %f %f %f %f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.x,
    current_pose.orientation.y,
    current_pose.orientation.z,
    current_pose.orientation.w);//*/
  
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
  
  auto const new_pose = [&]{
    geometry_msgs::msg::Pose msg = current_pose;
    msg.position.x += localTransform.getX()*stepSize;
    msg.position.y += localTransform.getY()*stepSize;
    msg.position.z += localTransform.getZ()*stepSize;
    
    
    return msg;
  }();
  
  RCLCPP_INFO(this->get_logger(), "New pose: %f %f %f %f %f %f %f",
    new_pose.position.x,
    new_pose.position.y,
    new_pose.position.z,
    new_pose.orientation.x,
    new_pose.orientation.y,
    new_pose.orientation.z,
    new_pose.orientation.w);
  if (armMsg.reset == true) //reset something
  {
    geometry_msgs::msg::Pose target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.636922;
    msg.position.y = 0.064768;
    msg.position.z = 0.678810;
    return msg;
    }();
    move_group_ptr->setPoseTarget(target_pose);

    // Create a plan to that target pose
    auto const [success, plan] = [&]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_ptr->plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success) {
      RCLCPP_INFO(this->get_logger(), "Number of joint trajectory points: %li, number of multiDOFjointtrajectorypoints: %li", std::size(plan.trajectory_.joint_trajectory.points), std::size(plan.trajectory_.multi_dof_joint_trajectory.points));
      publisher_->publish(plan.trajectory_.joint_trajectory);
      move_group_ptr->execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planing failed!");
    }
  }
  else if (armMsg.named_pose != 0) //currently does not work
  {
    if (armMsg.named_pose == 1 || armMsg.named_pose == 2) //1 = closed, 2 = open
    {
      std::string s = "open"; //jjk reference?
      if (armMsg.named_pose == 1)
      {
        RCLCPP_INFO(this->get_logger(), "Furnace: close!");
        s = "closed";
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Furnace: open!");
      }
      //actually add stuff later
    }
  }
  else if (armMsg.query_goal_state)
  {
    std::vector<double> angles(6);
    for (unsigned long int i = 0; i < angles.size(); i++)
    {
      angles[i] = armMsg.goal_angles[i];
      RCLCPP_INFO(this->get_logger(), std::to_string(armMsg.goal_angles[i]).c_str());
    }
    move_group_ptr->setJointValueTarget(angles);
    auto const [success, plan] = [&]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_ptr->plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success) {
      RCLCPP_INFO(this->get_logger(), "Number of joint trajectory points: %li, number of multiDOFjointtrajectorypoints: %li", std::size(plan.trajectory_.joint_trajectory.points), std::size(plan.trajectory_.multi_dof_joint_trajectory.points));
      publisher_->publish(plan.trajectory_.joint_trajectory);
      move_group_ptr->execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planing failed!");
    }
  }
  else if (poseMsg.orientation.x != 0 || poseMsg.orientation.y != 0 || poseMsg.orientation.z != 0 || poseMsg.orientation.w != 0) //rotation required
  {
    tf2::Quaternion q1;
    tf2::convert(current_pose.orientation, q1);
    tf2::Quaternion q2;
    q2.setRPY(poseMsg.orientation.x, poseMsg.orientation.y, poseMsg.orientation.z);
    tf2::Quaternion q3 = q1 * q2;
    geometry_msgs::msg::Quaternion q4 = tf2::toMsg(q3);
    
    geometry_msgs::msg::Pose rotation_pose = current_pose;
    rotation_pose.orientation = q4;
    
    points.push_back(rotation_pose);
    
    const double jump_threshold = 0;
    const double eef_step = 0.01;
    //double fraction = move_group_interface.computeCartesianPath(points, eef_step, jump_threshold, trajectory);
    move_group_ptr->computeCartesianPath(points, eef_step, jump_threshold, trajectory);
    //RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
    //move_group_ptr->execute(trajectory);
    
    //launch thread
    RCLCPP_INFO(this->get_logger(), "What");
    if (th.joinable())
    {
      th.join();
    }
    RCLCPP_INFO(this->get_logger(), "What??");
    publisher_->publish(trajectory.joint_trajectory);
    th = std::thread(executeTrajectory, std::ref(trajectory), move_group_ptr);
    RCLCPP_INFO(this->get_logger(), "Why");
    
    //geometry_msgs::msg::Pose rotation_pose = current_pose;
    //rotation_pose.orientation = q4;
    
    /*move_group_ptr->setOrientationTarget(q4.x, q4.y, q4.z, q4.w);
    auto const ok = static_cast<bool>(move_group_ptr->plan(rotationPlan));
    
    if (ok)
    {
      RCLCPP_INFO(this->get_logger(), "Rotation");
      if (th.joinable())
      {
        th.join();
      }
      RCLCPP_INFO(this->get_logger(), "Rotation??");
      th = std::thread(executePlan, std::ref(rotationPlan), move_group_ptr);
      RCLCPP_INFO(this->get_logger(), "Rotation done");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Planing failed!");
    }*/
    
  }
  else //rotation not required
  {
    points.push_back(new_pose);
    const double jump_threshold = 0;
    const double eef_step = 0.01;
    //double fraction = move_group_interface.computeCartesianPath(points, eef_step, jump_threshold, trajectory);
    move_group_ptr->computeCartesianPath(points, eef_step, jump_threshold, trajectory);
    //RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
    //move_group_ptr->execute(trajectory);
    
    //launch thread
    RCLCPP_INFO(this->get_logger(), "What");
    RCLCPP_INFO(this->get_logger(), "What??");
    RCLCPP_INFO(this->get_logger(), "Number of joint trajectory points: %li, number of multiDOFjointtrajectorypoints: %li", std::size(trajectory.joint_trajectory.points), std::size(trajectory.multi_dof_joint_trajectory.points));
    for (int i = 0; i < (int)std::size(trajectory.joint_trajectory.points); i++)
    {
      RCLCPP_INFO(this->get_logger(), "Positions: %li, Velocity: %li, Accelerations: %li, effort: %li, duration: %u", std::size(trajectory.joint_trajectory.points[i].positions), std::size(trajectory.joint_trajectory.points[i].velocities), std::size(trajectory.joint_trajectory.points[i].accelerations), std::size(trajectory.joint_trajectory.points[i].effort), trajectory.joint_trajectory.points[i].time_from_start.nanosec);
      for (int j = 0; j < (int)std::size(trajectory.joint_trajectory.points[i].positions); j++)
      {
        RCLCPP_INFO(this->get_logger(), "positions: %f", trajectory.joint_trajectory.points[i].positions[j]);
      }
      for (int j = 0; j < (int)std::size(trajectory.joint_trajectory.points[i].positions); j++)
      {
        RCLCPP_INFO(this->get_logger(), "velocity: %f", trajectory.joint_trajectory.points[i].velocities[j]);
      }
      for (int j = 0; j < (int)std::size(trajectory.joint_trajectory.points[i].positions); j++)
      {
        RCLCPP_INFO(this->get_logger(), "acceleration: %f", trajectory.joint_trajectory.points[i].accelerations[j]);
      }
    }
    publisher_->publish(trajectory.joint_trajectory);
    th = std::thread(executeTrajectory, std::ref(trajectory), move_group_ptr);
    RCLCPP_INFO(this->get_logger(), "Why");
  }
  
  
  
}

int main(int argc, char ** argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestNode>(
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  rclcpp::spin(node);
}

