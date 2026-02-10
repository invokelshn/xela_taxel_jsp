#ifndef XELA_TAXEL_JOINT_STATE_PUBLISHER__XELA_TAXEL_JOINT_STATE_PUBLISHER_HPP_
#define XELA_TAXEL_JOINT_STATE_PUBLISHER__XELA_TAXEL_JOINT_STATE_PUBLISHER_HPP_

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace xela_taxel_joint_state_publisher
{
class XelaTaxelJointStatePublisher : public controller_interface::ControllerInterface
{
public:
  XelaTaxelJointStatePublisher();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct JointStateData
  {
    double pos = 0.0;
    double vel = 0.0;
    double eff = 0.0;
    bool has_pos = false;
    bool has_vel = false;
    bool has_eff = false;
  };

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);
  bool load_config();
  std::vector<std::string> build_output_names_locked() const;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> subs_;

  mutable std::mutex data_mutex_;
  std::unordered_map<std::string, JointStateData> last_states_;
  std::vector<std::string> last_input_order_;

  std::vector<std::string> keep_joints_;
  std::vector<std::string> ordered_keep_joints_;
  std::unordered_map<std::string, double> initial_positions_;

  std::string config_yaml_;
  std::vector<std::string> device_profiles_;
  std::string hand_side_;
  std::string output_topic_;
  std::vector<std::string> source_list_;
  double publish_rate_ = 30.0;
  bool preserve_input_order_ = true;

  rclcpp::Time last_publish_time_;
  rclcpp::Duration publish_period_{0, 0};
  bool configured_ = false;
};
}  // namespace xela_taxel_joint_state_publisher

#endif  // XELA_TAXEL_JOINT_STATE_PUBLISHER__XELA_TAXEL_JOINT_STATE_PUBLISHER_HPP_
