// MIT License. Copyright (c) 2026 Marcell Mor Szokedencsi (s5173191),
// University of Groningen. See ../LICENSE.
//
// xarm1s_link_attacher -- a minimal Gazebo Classic 11 world plugin that grasps
// an object by forming a rigid joint between the gripper link and the object ON
// COMMAND, so it is carried rigidly (no /set_entity_state teleport jitter), and
// released on command. This is the community-standard "fixed-joint attach" for
// stable grasping in Gazebo Classic.
//
// Implementation follows the two canonical references for this exact stack
// (pal-robotics/gazebo_ros_link_attacher and IFRA-Cranfield/IFRA_LinkAttacher,
// ROS 2 Humble + Gazebo Classic 11):
//   * a REVOLUTE joint with both limits pinned to 0 is the reliable fixed
//     constraint (Gazebo Classic's "fixed" joint type is historically unstable);
//   * the joint is created on the gripper model and REMOVED (not merely
//     detached) on release, so joints never accumulate.
// Objects are released by detaching and letting them settle on the surface --
// the standard approach; we deliberately do NOT pin objects to the static
// table (that destabilises ODE's whole-world solver).
//
// ROS 2 interface (std_msgs/String = the object's Gazebo model name):
//   /grasp/attach <model>  -> grasp that object with <gripper_model>::<gripper_link>
//   /grasp/detach <model>  -> release that object

#include <map>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace gazebo
{
class LinkAttacher : public WorldPlugin
{
public:
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = world;
    ros_node_ = gazebo_ros::Node::Get(sdf);
    if (sdf->HasElement("gripper_model")) {
      gripper_model_ = sdf->Get<std::string>("gripper_model");
    }
    if (sdf->HasElement("gripper_link")) {
      gripper_link_ = sdf->Get<std::string>("gripper_link");
    }
    attach_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
      "/grasp/attach", 10,
      std::bind(&LinkAttacher::OnAttach, this, std::placeholders::_1));
    detach_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
      "/grasp/detach", 10,
      std::bind(&LinkAttacher::OnDetach, this, std::placeholders::_1));
    RCLCPP_INFO(ros_node_->get_logger(),
      "xarm1s link_attacher ready (gripper %s::%s) -- /grasp/attach, /grasp/detach",
      gripper_model_.c_str(), gripper_link_.c_str());
  }

private:
  void OnAttach(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string & obj = msg->data;
    if (joints_.count(obj)) {return;}                         // already attached
    physics::ModelPtr gm = world_->ModelByName(gripper_model_);
    physics::ModelPtr om = world_->ModelByName(obj);
    if (!gm || !om) {
      RCLCPP_WARN(ros_node_->get_logger(),
        "attach: model not found (%s / %s)", gripper_model_.c_str(), obj.c_str());
      return;
    }
    physics::LinkPtr gl = gm->GetLink(gripper_link_);
    physics::LinkPtr ol = om->GetLinks().empty() ? nullptr : om->GetLinks()[0];
    if (!gl || !ol) {
      RCLCPP_WARN(ros_node_->get_logger(), "attach: link not found for %s", obj.c_str());
      return;
    }
    // Revolute joint with zero range = the proven, stable fixed constraint.
    // Created on the gripper model; named so it can be removed cleanly later.
    const std::string name = "graspjoint__" + obj;
    physics::JointPtr joint = gm->CreateJoint(name, "revolute", gl, ol);
    if (!joint) {
      RCLCPP_WARN(ros_node_->get_logger(), "attach: CreateJoint failed for %s", obj.c_str());
      return;
    }
    joint->Load(gl, ol, ignition::math::Pose3d());            // anchor = current pose
    joint->SetAxis(0, ignition::math::Vector3d(1, 0, 0));
    joint->SetUpperLimit(0, 0.0);
    joint->SetLowerLimit(0, 0.0);
    joint->SetEffortLimit(0, 0.0);
    // Form the constraint on a body AT REST: zero the object's velocity right
    // before Init so the joint captures no residual motion. A rigid joint that
    // forms while the body still has a little velocity (or settles against the
    // table) can release an impulse the instant it grasps -- intermittently
    // flinging the long banana at pickup. Doing this at ATTACH is safe (no
    // concurrent set_entity_state, unlike at release).
    ol->SetLinearVel(ignition::math::Vector3d::Zero);
    ol->SetAngularVel(ignition::math::Vector3d::Zero);
    joint->Init();
    joints_[obj] = name;
    RCLCPP_INFO(ros_node_->get_logger(), "attached '%s' to %s::%s",
      obj.c_str(), gripper_model_.c_str(), gripper_link_.c_str());
  }

  void OnDetach(const std_msgs::msg::String::SharedPtr msg)
  {
    auto it = joints_.find(msg->data);
    if (it == joints_.end()) {return;}
    physics::ModelPtr gm = world_->ModelByName(gripper_model_);
    if (gm) {
      physics::JointPtr j = gm->GetJoint(it->second);
      if (j) {j->Detach();}
      gm->RemoveJoint(it->second);                            // remove, don't leak
    }
    joints_.erase(it);
    RCLCPP_INFO(ros_node_->get_logger(), "detached '%s'", msg->data.c_str());
  }

  physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr attach_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr detach_sub_;
  std::map<std::string, std::string> joints_;   // object model name -> joint name
  std::string gripper_model_{"xarm1s"};
  std::string gripper_link_{"xarm_2_link"};
};

GZ_REGISTER_WORLD_PLUGIN(LinkAttacher)
}  // namespace gazebo
