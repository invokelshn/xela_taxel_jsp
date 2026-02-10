#include "xela_taxel_joint_state_publisher/xela_taxel_joint_state_publisher.hpp"

#include <algorithm>
#include <filesystem>
#include <functional>
#include <unordered_set>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "controller_interface/helpers.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace xela_taxel_joint_state_publisher
{
namespace
{
constexpr char kDefaultOutputTopic[] = "/joint_states";
constexpr double kDefaultPublishRate = 30.0;
constexpr bool kDefaultPreserveInputOrder = true;
constexpr char kPackagePrefix[] = "package://";

std::string trim(const std::string & input)
{
  auto start = input.find_first_not_of(" \t\n\r");
  if (start == std::string::npos)
  {
    return "";
  }
  auto end = input.find_last_not_of(" \t\n\r");
  return input.substr(start, end - start + 1);
}

void append_unique(std::vector<std::string> & out, const std::vector<std::string> & in)
{
  std::unordered_set<std::string> seen(out.begin(), out.end());
  for (const auto & item : in)
  {
    const auto trimmed = trim(item);
    if (trimmed.empty())
    {
      continue;
    }
    if (seen.emplace(trimmed).second)
    {
      out.push_back(trimmed);
    }
  }
}

std::vector<std::string> parse_string_list(const YAML::Node & node)
{
  std::vector<std::string> out;
  if (!node || !node.IsSequence())
  {
    return out;
  }
  out.reserve(node.size());
  for (const auto & item : node)
  {
    if (item.IsScalar())
    {
      out.push_back(item.as<std::string>());
    }
  }
  return out;
}

std::vector<std::string> load_keep_joints_file(const std::filesystem::path & path)
{
  std::vector<std::string> out;
  YAML::Node file = YAML::LoadFile(path.string());
  YAML::Node list_node;

  if (file["keep_joints"])
  {
    list_node = file["keep_joints"];
  }
  else if (file.IsSequence())
  {
    list_node = file;
  }

  if (!list_node || !list_node.IsSequence())
  {
    return out;
  }

  out.reserve(list_node.size());
  for (const auto & item : list_node)
  {
    if (item.IsScalar())
    {
      out.push_back(item.as<std::string>());
    }
  }
  return out;
}

std::string resolve_package_path(const std::string & input)
{
  if (input.rfind(kPackagePrefix, 0) != 0)
  {
    return input;
  }

  std::string rest = input.substr(std::char_traits<char>::length(kPackagePrefix));
  auto slash = rest.find('/');
  if (slash == std::string::npos)
  {
    return input;
  }

  const std::string pkg_name = rest.substr(0, slash);
  const std::string rel_path = rest.substr(slash + 1);

  try
  {
    auto share_dir = ament_index_cpp::get_package_share_directory(pkg_name);
    return (std::filesystem::path(share_dir) / rel_path).string();
  }
  catch (const std::exception &)
  {
    return input;
  }
}

}  // namespace

XelaTaxelJointStatePublisher::XelaTaxelJointStatePublisher()
: publish_rate_(kDefaultPublishRate), preserve_input_order_(kDefaultPreserveInputOrder)
{
}

controller_interface::InterfaceConfiguration
XelaTaxelJointStatePublisher::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration
XelaTaxelJointStatePublisher::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::CallbackReturn XelaTaxelJointStatePublisher::on_init()
{
  try
  {
    auto_declare<std::string>("config_yaml", "");
    auto_declare<std::vector<std::string>>("device_profiles", {});
    auto_declare<std::string>("device_profile", "");
    auto_declare<std::string>("hand_side", "left");
    auto_declare<std::string>("output_topic", kDefaultOutputTopic);
    auto_declare<std::vector<std::string>>("source_list", {});
    auto_declare<double>("publish_rate", kDefaultPublishRate);
    auto_declare<bool>("preserve_input_order", kDefaultPreserveInputOrder);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn XelaTaxelJointStatePublisher::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  config_yaml_ = get_node()->get_parameter("config_yaml").as_string();
  device_profiles_ = get_node()->get_parameter("device_profiles").as_string_array();
  const std::string device_profile = get_node()->get_parameter("device_profile").as_string();
  hand_side_ = get_node()->get_parameter("hand_side").as_string();
  if (!device_profile.empty())
  {
    device_profiles_.clear();
    device_profiles_.push_back(device_profile);
    RCLCPP_INFO(get_node()->get_logger(), "device_profile override: %s", device_profile.c_str());
  }
  output_topic_ = get_node()->get_parameter("output_topic").as_string();
  source_list_ = get_node()->get_parameter("source_list").as_string_array();
  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
  preserve_input_order_ = get_node()->get_parameter("preserve_input_order").as_bool();

  keep_joints_.clear();
  ordered_keep_joints_.clear();
  initial_positions_.clear();
  last_states_.clear();
  last_input_order_.clear();

  if (!load_config())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (publish_rate_ <= 0.0)
  {
    publish_rate_ = kDefaultPublishRate;
  }
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
  last_publish_time_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());

  pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    output_topic_, rclcpp::SystemDefaultsQoS());

  subs_.clear();
  for (const auto & topic : source_list_)
  {
    if (topic == output_topic_)
    {
      RCLCPP_WARN(get_node()->get_logger(),
        "Skipping source_list topic '%s' because it matches output_topic.", topic.c_str());
      continue;
    }

    subs_.push_back(get_node()->create_subscription<sensor_msgs::msg::JointState>(
      topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&XelaTaxelJointStatePublisher::on_joint_state, this, std::placeholders::_1)));
  }

  configured_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn XelaTaxelJointStatePublisher::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  last_publish_time_ = get_node()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn XelaTaxelJointStatePublisher::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type XelaTaxelJointStatePublisher::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!configured_ || !pub_)
  {
    return controller_interface::return_type::OK;
  }

  if (publish_rate_ > 0.0)
  {
    if (last_publish_time_.nanoseconds() == 0)
    {
      last_publish_time_ = time;
    }
    if ((time - last_publish_time_) < publish_period_)
    {
      return controller_interface::return_type::OK;
    }
    last_publish_time_ = time;
  }

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = time;

  std::vector<std::string> out_names;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
  bool has_vel = false;
  bool has_eff = false;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (keep_joints_.empty())
    {
      return controller_interface::return_type::OK;
    }

    out_names = build_output_names_locked();
    if (out_names.empty())
    {
      return controller_interface::return_type::OK;
    }

    positions.reserve(out_names.size());
    velocities.reserve(out_names.size());
    efforts.reserve(out_names.size());

    for (const auto & name : out_names)
    {
      auto it = last_states_.find(name);
      double pos = 0.0;
      double vel = 0.0;
      double eff = 0.0;
      bool has_pos = false;
      bool local_has_vel = false;
      bool local_has_eff = false;

      if (it != last_states_.end())
      {
        pos = it->second.pos;
        vel = it->second.vel;
        eff = it->second.eff;
        has_pos = it->second.has_pos;
        local_has_vel = it->second.has_vel;
        local_has_eff = it->second.has_eff;
      }

      if (!has_pos)
      {
        auto init_it = initial_positions_.find(name);
        if (init_it != initial_positions_.end())
        {
          pos = init_it->second;
        }
      }

      positions.push_back(pos);
      velocities.push_back(vel);
      efforts.push_back(eff);
      has_vel = has_vel || local_has_vel;
      has_eff = has_eff || local_has_eff;
    }
  }

  msg.name = out_names;
  msg.position = positions;
  if (has_vel)
  {
    msg.velocity = velocities;
  }
  if (has_eff)
  {
    msg.effort = efforts;
  }

  pub_->publish(msg);

  return controller_interface::return_type::OK;
}

void XelaTaxelJointStatePublisher::on_joint_state(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!msg || msg->name.empty())
  {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  last_input_order_ = msg->name;

  for (size_t idx = 0; idx < msg->name.size(); ++idx)
  {
    const auto & name = msg->name[idx];
    auto & entry = last_states_[name];

    if (idx < msg->position.size())
    {
      entry.pos = msg->position[idx];
      entry.has_pos = true;
    }
    if (idx < msg->velocity.size())
    {
      entry.vel = msg->velocity[idx];
      entry.has_vel = true;
    }
    if (idx < msg->effort.size())
    {
      entry.eff = msg->effort[idx];
      entry.has_eff = true;
    }
  }
}

std::vector<std::string> XelaTaxelJointStatePublisher::build_output_names_locked() const
{
  std::vector<std::string> out_names;
  std::unordered_set<std::string> keep_set(keep_joints_.begin(), keep_joints_.end());
  std::unordered_set<std::string> out_set;

  if (!ordered_keep_joints_.empty())
  {
    for (const auto & name : ordered_keep_joints_)
    {
      if (!keep_set.count(name))
      {
        continue;
      }
      if (out_set.emplace(name).second)
      {
        out_names.push_back(name);
      }
    }
    return out_names;
  }

  if (preserve_input_order_ && !last_input_order_.empty())
  {
    for (const auto & name : last_input_order_)
    {
      if (!keep_set.count(name))
      {
        continue;
      }
      if (out_set.emplace(name).second)
      {
        out_names.push_back(name);
      }
    }
    for (const auto & name : keep_joints_)
    {
      if (out_set.emplace(name).second)
      {
        out_names.push_back(name);
      }
    }
    return out_names;
  }

  out_names = keep_joints_;
  return out_names;
}

bool XelaTaxelJointStatePublisher::load_config()
{
  if (config_yaml_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "config_yaml is required.");
    return false;
  }

  const std::string resolved_config = resolve_package_path(config_yaml_);

  YAML::Node cfg;
  try
  {
    cfg = YAML::LoadFile(resolved_config);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load config_yaml '%s': %s",
      resolved_config.c_str(), e.what());
    return false;
  }

  std::filesystem::path base_dir = std::filesystem::path(resolved_config).parent_path();

  if (output_topic_ == kDefaultOutputTopic && cfg["output_topic"])
  {
    output_topic_ = cfg["output_topic"].as<std::string>();
  }
  if (publish_rate_ == kDefaultPublishRate && cfg["publish_rate"])
  {
    publish_rate_ = cfg["publish_rate"].as<double>();
  }
  if (preserve_input_order_ == kDefaultPreserveInputOrder && cfg["preserve_input_order"])
  {
    preserve_input_order_ = cfg["preserve_input_order"].as<bool>();
  }
  if (source_list_.empty() && cfg["source_list"])
  {
    source_list_ = parse_string_list(cfg["source_list"]);
  }

  if (device_profiles_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "device_profiles must be provided.");
    return false;
  }

  YAML::Node profiles_node = cfg["device_profiles"];
  if (!profiles_node || !profiles_node.IsMap())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "device_profiles section missing in config.");
    return false;
  }

  for (const auto & profile_name : device_profiles_)
  {
    YAML::Node profile = profiles_node[profile_name];
    if (!profile)
    {
      RCLCPP_WARN(get_node()->get_logger(),
        "device_profiles entry '%s' not found in config.", profile_name.c_str());
      continue;
    }

    if (profile["keep_joints_files"] && profile["keep_joints_files"].IsSequence())
    {
      for (const auto & entry : profile["keep_joints_files"])
      {
        if (!entry.IsScalar())
        {
          continue;
        }
        std::string file_name = trim(entry.as<std::string>());
        if (file_name.empty())
        {
          continue;
        }

        std::filesystem::path file_path(file_name);
        if (file_path.is_relative())
        {
          file_path = base_dir / file_path;
        }

        try
        {
          auto joints = load_keep_joints_file(file_path);
          if (!joints.empty())
          {
            RCLCPP_INFO(get_node()->get_logger(),
              "Profile '%s' loaded keep_joints file '%s' (count=%zu, first=%s)",
              profile_name.c_str(), file_path.string().c_str(), joints.size(),
              joints.front().c_str());
          }
          append_unique(keep_joints_, joints);
        }
        catch (const std::exception & e)
        {
          RCLCPP_ERROR(get_node()->get_logger(),
            "Failed to load keep_joints file '%s': %s", file_path.string().c_str(), e.what());
        }
      }
    }

    if (profile["keep_joints"])
    {
      append_unique(keep_joints_, parse_string_list(profile["keep_joints"]));
    }

    if (profile["ordered_keep_joints"])
    {
      append_unique(ordered_keep_joints_, parse_string_list(profile["ordered_keep_joints"]));
    }

    if (profile["initial_positions"] && profile["initial_positions"].IsMap())
    {
      for (const auto & item : profile["initial_positions"])
      {
        if (!item.first.IsScalar())
        {
          continue;
        }
        const std::string key = item.first.as<std::string>();
        if (initial_positions_.count(key) > 0)
        {
          continue;
        }
        if (item.second.IsScalar())
        {
          initial_positions_[key] = item.second.as<double>();
        }
      }
    }
  }

  if (keep_joints_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "keep_joints resolved to empty list.");
    return false;
  }
  else
  {
    RCLCPP_INFO(get_node()->get_logger(), "Resolved keep_joints total: %zu (first=%s)",
      keep_joints_.size(), keep_joints_.front().c_str());
  }

  if (!ordered_keep_joints_.empty())
  {
    std::unordered_set<std::string> keep_set(keep_joints_.begin(), keep_joints_.end());
    std::vector<std::string> filtered;
    filtered.reserve(ordered_keep_joints_.size());
    for (const auto & name : ordered_keep_joints_)
    {
      if (keep_set.count(name))
      {
        filtered.push_back(name);
      }
    }
    ordered_keep_joints_.swap(filtered);
  }

  const std::string side = trim(hand_side_);
  if (side == "right" || side == "r") {
    for (auto & name : keep_joints_) {
      if (name.rfind("x_taxel_0_", 0) == 0) {
        name.replace(0, 9, "x_taxel_1_");
      }
    }
    for (auto & name : ordered_keep_joints_) {
      if (name.rfind("x_taxel_0_", 0) == 0) {
        name.replace(0, 9, "x_taxel_1_");
      }
    }
    RCLCPP_INFO(get_node()->get_logger(), "Applied hand_side=right mapping prefix x_taxel_1_.");
  } else if (side == "left" || side == "l") {
    for (auto & name : keep_joints_) {
      if (name.rfind("x_taxel_1_", 0) == 0) {
        name.replace(0, 9, "x_taxel_0_");
      }
    }
    for (auto & name : ordered_keep_joints_) {
      if (name.rfind("x_taxel_1_", 0) == 0) {
        name.replace(0, 9, "x_taxel_0_");
      }
    }
    RCLCPP_INFO(get_node()->get_logger(), "Applied hand_side=left mapping prefix x_taxel_0_.");
  }

  return true;
}

}  // namespace xela_taxel_joint_state_publisher

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  xela_taxel_joint_state_publisher::XelaTaxelJointStatePublisher,
  controller_interface::ControllerInterface)
