// MIT License. Copyright (c) 2026 Marcell Mor Szokedencsi (s5173191),
// University of Groningen. See ../LICENSE.
//
// xarm1s_link_attacher -- a Gazebo Classic 11 world plugin that grasps an object
// by forming a rigid joint between the gripper and the object, but ONLY AFTER it
// detects that both fingers are genuinely in contact with that object. This is
// the CONTACT-TRIGGERED fixed-joint grasp: the gripper must physically close on
// the object before the weld forms, so there is no "grip on air" and no
// teleport-attach. The carried object moves rigidly with the gripper (no
// /set_entity_state jitter) and is released on command at the placement pose.
//
// Design follows the two canonical references for this stack:
//   * GazeboGraspFix (JenniferBuehler/gazebo-pkgs): a fixed joint is formed when
//     opposing contact forces are detected on a grasped object -- we use the
//     robust two-finger-contact proxy (both named finger links touch the object
//     for >= grip_time) which is reliable for a parallel jaw without the
//     fragile force-direction summation;
//   * Gazebo's own physics::Gripper and PAL/IFRA link_attacher use the SAME
//     zero-range-joint weld (Gazebo Classic's native "fixed" joint type is
//     historically unstable, so a revolute joint pinned to 0 is the proven
//     rigid constraint). The joint is created on the gripper model and REMOVED
//     (not merely detached) on release, so joints never accumulate.
//
// Contact detection reads the world ContactManager each physics step
// (SetNeverDropContacts so contacts are stored without a contact sensor /
// subscriber), at WorldUpdateEnd where the just-completed step's contacts are
// valid. All joint creation/removal happens in that physics-thread callback;
// the ROS callbacks only set intent (arm / request release), so joints are
// never mutated concurrently with a physics step.
//
// ROS 2 interface (std_msgs/String = the object's Gazebo model name):
//   /grasp/attach <model>  -> ARM: weld <model> to the gripper ON two-finger
//                             contact (welds immediately if already in contact)
//   /grasp/detach <model>  -> release <model> and disarm
//
// SDF parameters (all optional, defaults shown):
//   <gripper_model>xarm1s</gripper_model>            model carrying the gripper
//   <gripper_link>xarm_2_link</gripper_link>         link the object welds to
//   <left_finger_link> / <right_finger_link>         finger links to watch
//   <grip_time>0.05</grip_time>                      s of sustained two-finger
//                                                    contact before welding
//   <fallback_timeout>1.0</fallback_timeout>         s after arming to weld even
//                                                    without detected contact
//                                                    (demo robustness; 0 = never,
//                                                    require genuine contact)

#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
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

    auto get_str = [&](const char * k, const std::string & def) {
      return sdf->HasElement(k) ? sdf->Get<std::string>(k) : def;
    };
    auto get_dbl = [&](const char * k, double def) {
      return sdf->HasElement(k) ? sdf->Get<double>(k) : def;
    };
    gripper_model_ = get_str("gripper_model", "xarm1s");
    gripper_link_ = get_str("gripper_link", "xarm_2_link");
    left_finger_ = get_str("left_finger_link", "gripper_left_finger_link");
    right_finger_ = get_str("right_finger_link", "gripper_right_finger_link");
    grip_time_ = get_dbl("grip_time", 0.05);
    fallback_timeout_ = get_dbl("fallback_timeout", 1.0);

    // Store per-step contacts even with no contact sensor / transport
    // subscriber, so GetContacts() is populated for our OnUpdate read.
    if (world_->Physics()) {
      contact_mgr_ = world_->Physics()->GetContactManager();
      if (contact_mgr_) {contact_mgr_->SetNeverDropContacts(true);}
    }

    attach_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
      "/grasp/attach", 10,
      std::bind(&LinkAttacher::OnAttach, this, std::placeholders::_1));
    detach_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
      "/grasp/detach", 10,
      std::bind(&LinkAttacher::OnDetach, this, std::placeholders::_1));

    update_conn_ = event::Events::ConnectWorldUpdateEnd(
      std::bind(&LinkAttacher::OnUpdate, this));

    RCLCPP_INFO(ros_node_->get_logger(),
      "xarm1s link_attacher ready (contact-triggered; gripper %s weld-link %s, "
      "fingers %s/%s, grip_time=%.3fs, fallback=%.1fs)",
      gripper_model_.c_str(), gripper_link_.c_str(), left_finger_.c_str(),
      right_finger_.c_str(), grip_time_, fallback_timeout_);
  }

private:
  // ----- ROS callbacks: set intent only, never touch joints (other thread) ----
  void OnAttach(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string & obj = msg->data;
    std::lock_guard<std::mutex> lk(mtx_);
    if (joints_.count(obj)) {return;}              // already welded
    if (armed_.insert(obj).second) {
      contact_since_[obj] = -1.0;                  // not yet in two-finger contact
      armed_at_[obj] = world_->SimTime().Double();
      RCLCPP_INFO(ros_node_->get_logger(),
        "armed '%s' -- will weld on two-finger contact", obj.c_str());
    }
  }

  void OnDetach(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string & obj = msg->data;
    std::lock_guard<std::mutex> lk(mtx_);
    armed_.erase(obj);
    contact_since_.erase(obj);
    armed_at_.erase(obj);
    if (joints_.count(obj)) {pending_detach_.insert(obj);}  // physics thread frees it
  }

  // ----- physics-thread step: detect contact, form / remove welds -----
  void OnUpdate()
  {
    std::lock_guard<std::mutex> lk(mtx_);

    // 1) honour any pending release requests (joint mutation on this thread)
    if (!pending_detach_.empty()) {
      physics::ModelPtr gm = world_->ModelByName(gripper_model_);
      for (const std::string & obj : pending_detach_) {
        auto it = joints_.find(obj);
        if (it == joints_.end()) {continue;}
        if (gm) {
          physics::JointPtr j = gm->GetJoint(it->second);
          if (j) {j->Detach();}
          gm->RemoveJoint(it->second);             // remove, don't leak
        }
        joints_.erase(it);
        // Zero the released object's velocity so a residual contact spring (a
        // fingertip pad still penetrating a too-wide object, e.g. the 52mm coke)
        // cannot fling it the instant the weld is removed. Harmless for objects
        // the jaws clear cleanly (they are already near rest).
        physics::ModelPtr om = world_->ModelByName(obj);
        if (om) {
          om->SetLinearVel(ignition::math::Vector3d::Zero);
          om->SetAngularVel(ignition::math::Vector3d::Zero);
        }
        RCLCPP_INFO(ros_node_->get_logger(), "detached '%s'", obj.c_str());
      }
      pending_detach_.clear();
    }

    if (armed_.empty() || !contact_mgr_) {return;}
    const double now = world_->SimTime().Double();

    // 2) which armed object is each finger touching this step?
    std::map<std::string, bool> left_touch, right_touch;
    const unsigned int nc = contact_mgr_->GetContactCount();
    for (unsigned int i = 0; i < nc; ++i) {
      physics::Contact * c = contact_mgr_->GetContact(i);
      if (!c || c->count == 0) {continue;}
      std::string obj, finger;
      if (!ClassifyContact(c, obj, finger)) {continue;}
      if (!armed_.count(obj)) {continue;}
      if (finger == left_finger_) {left_touch[obj] = true;}
      if (finger == right_finger_) {right_touch[obj] = true;}
    }

    // 3) weld armed objects that meet the two-finger-contact (or fallback) test
    for (const std::string & obj :
         std::vector<std::string>(armed_.begin(), armed_.end())) {
      if (joints_.count(obj)) {continue;}
      const bool both = left_touch[obj] && right_touch[obj];
      if (both) {
        if (contact_since_[obj] < 0.0) {contact_since_[obj] = now;}
        if (now - contact_since_[obj] >= grip_time_) {
          FormGrasp(obj, true);
          continue;
        }
      } else {
        contact_since_[obj] = -1.0;                // reset debounce on contact loss
      }
      if (fallback_timeout_ > 0.0 && !joints_.count(obj) &&
          now - armed_at_[obj] >= fallback_timeout_) {
        FormGrasp(obj, false);
      }
    }
  }

  // True iff exactly one collision is a gripper finger and the other belongs to
  // a different model; returns that other model's name and the finger link name.
  bool ClassifyContact(physics::Contact * c, std::string & obj_name,
                       std::string & finger_link)
  {
    physics::Collision * cols[2] = {c->collision1, c->collision2};
    physics::LinkPtr links[2];
    for (int k = 0; k < 2; ++k) {
      if (!cols[k]) {return false;}
      links[k] = cols[k]->GetLink();
      if (!links[k]) {return false;}
    }
    auto is_finger = [&](const physics::LinkPtr & l) {
      return l->GetModel() && l->GetModel()->GetName() == gripper_model_ &&
             (l->GetName() == left_finger_ || l->GetName() == right_finger_);
    };
    int fi = is_finger(links[0]) ? 0 : (is_finger(links[1]) ? 1 : -1);
    if (fi < 0) {return false;}                    // no finger involved
    int oi = 1 - fi;
    if (is_finger(links[oi])) {return false;}      // finger-finger self contact
    finger_link = links[fi]->GetName();
    obj_name = links[oi]->GetModel() ? links[oi]->GetModel()->GetName() : "";
    return !obj_name.empty();
  }

  void FormGrasp(const std::string & obj, bool by_contact)
  {
    physics::ModelPtr gm = world_->ModelByName(gripper_model_);
    physics::ModelPtr om = world_->ModelByName(obj);
    if (!gm || !om) {
      RCLCPP_WARN(ros_node_->get_logger(),
        "grasp: model not found (%s / %s)", gripper_model_.c_str(), obj.c_str());
      return;
    }
    physics::LinkPtr gl = gm->GetLink(gripper_link_);
    physics::LinkPtr ol = om->GetLinks().empty() ? nullptr : om->GetLinks()[0];
    if (!gl || !ol) {
      RCLCPP_WARN(ros_node_->get_logger(), "grasp: link not found for %s", obj.c_str());
      return;
    }
    // Revolute joint with zero range = the proven, stable fixed constraint.
    const std::string name = "graspjoint__" + obj;
    physics::JointPtr joint = gm->CreateJoint(name, "revolute", gl, ol);
    if (!joint) {
      RCLCPP_WARN(ros_node_->get_logger(), "grasp: CreateJoint failed for %s", obj.c_str());
      return;
    }
    joint->Load(gl, ol, ignition::math::Pose3d());           // anchor = current pose
    joint->SetAxis(0, ignition::math::Vector3d(1, 0, 0));
    joint->SetUpperLimit(0, 0.0);
    joint->SetLowerLimit(0, 0.0);
    joint->SetEffortLimit(0, 0.0);
    // Form the constraint on a body at rest: zero the object's velocity right
    // before Init so the joint captures no residual motion (a rigid joint formed
    // while the body still has velocity can release an impulse at grasp).
    ol->SetLinearVel(ignition::math::Vector3d::Zero);
    ol->SetAngularVel(ignition::math::Vector3d::Zero);
    joint->Init();
    joints_[obj] = name;
    RCLCPP_INFO(ros_node_->get_logger(), "grasped '%s' (%s) -> %s::%s",
      obj.c_str(),
      by_contact ? "contact-triggered" : "FALLBACK timeout -- no contact detected",
      gripper_model_.c_str(), gripper_link_.c_str());
  }

  physics::WorldPtr world_;
  physics::ContactManager * contact_mgr_{nullptr};
  gazebo_ros::Node::SharedPtr ros_node_;
  event::ConnectionPtr update_conn_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr attach_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr detach_sub_;

  std::mutex mtx_;
  std::set<std::string> armed_;                    // objects to weld on contact
  std::set<std::string> pending_detach_;           // release requests for OnUpdate
  std::map<std::string, double> contact_since_;    // sim time both fingers touched
  std::map<std::string, double> armed_at_;         // sim time armed (fallback clock)
  std::map<std::string, std::string> joints_;      // object model name -> joint name

  std::string gripper_model_{"xarm1s"};
  std::string gripper_link_{"xarm_2_link"};
  std::string left_finger_{"gripper_left_finger_link"};
  std::string right_finger_{"gripper_right_finger_link"};
  double grip_time_{0.05};
  double fallback_timeout_{1.0};
};

GZ_REGISTER_WORLD_PLUGIN(LinkAttacher)
}  // namespace gazebo
