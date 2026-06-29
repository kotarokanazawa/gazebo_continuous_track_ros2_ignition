#include <gazebo_continuous_track_ros2_ignition/ignition_continuous_track_simple.hpp>

#include <cmath>
#include <chrono>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/common/Console.hh>

namespace gazebo_continuous_track_ros2_ignition
{
namespace gz = ignition::gazebo;

struct SegmentCommand
{
  gz::Entity joint{gz::kNullEntity};
  double sprocket_to_segment{1.0};
  double position{0.0};
  double translation_period{0.0};
  bool reset_position{false};
};

class IgnitionContinuousTrackSimplePrivate
{
public:
  std::string plugin_name{"continuous_track"};
  gz::Entity model_entity{gz::kNullEntity};
  gz::Entity sprocket_joint{gz::kNullEntity};
  double sprocket_pitch_diameter{0.0};
  double velocity_deadband{0.02};
  std::vector<SegmentCommand> segments;

  gz::Entity jointByName(gz::EntityComponentManager &_ecm, const std::string &_name) const
  {
    gz::Model model(this->model_entity);
    return model.JointByName(_ecm, _name);
  }

  static bool isLikelyRotationalSegment(const sdf::ElementPtr &_segment)
  {
    return _segment && _segment->HasElement("pitch_diameter");
  }

  static void setJointVelocity(gz::EntityComponentManager &_ecm, gz::Entity _joint, double _velocity)
  {
    auto vel_cmd = _ecm.Component<gz::components::JointVelocityCmd>(_joint);
    if (vel_cmd) {
      vel_cmd->Data() = {_velocity};
      return;
    }
    _ecm.CreateComponent(_joint, gz::components::JointVelocityCmd({_velocity}));
  }

  static double jointVelocity(gz::EntityComponentManager &_ecm, gz::Entity _joint)
  {
    const auto vel = _ecm.Component<gz::components::JointVelocity>(_joint);
    if (!vel || vel->Data().empty()) {
      return 0.0;
    }
    return vel->Data()[0];
  }

  static void resetJointPosition(
    gz::EntityComponentManager &_ecm, gz::Entity _joint, double _position)
  {
    auto reset = _ecm.Component<gz::components::JointPositionReset>(_joint);
    if (reset) {
      reset->Data() = {_position};
      return;
    }
    _ecm.CreateComponent(_joint, gz::components::JointPositionReset({_position}));
  }

  static double wrappedTranslation(double _position, double _period)
  {
    if (_period <= 1e-9) {
      return _position;
    }
    _position = std::fmod(_position, _period);
    if (_position < 0.0) {
      _position += _period;
    }
    return _position;
  }
};

IgnitionContinuousTrackSimple::IgnitionContinuousTrackSimple()
: data_(std::make_unique<IgnitionContinuousTrackSimplePrivate>())
{
}

IgnitionContinuousTrackSimple::~IgnitionContinuousTrackSimple() = default;

void IgnitionContinuousTrackSimple::Configure(
  const gz::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::EntityComponentManager &_ecm,
  gz::EventManager &)
{
  this->data_->model_entity = _entity;
  if (_sdf && _sdf->GetName() == "plugin" && _sdf->GetAttribute("name")) {
    this->data_->plugin_name = _sdf->GetAttribute("name")->GetAsString();
  }
  if (_sdf && _sdf->HasElement("track_name")) {
    this->data_->plugin_name = _sdf->Get<std::string>("track_name", this->data_->plugin_name).first;
  }

  if (!_sdf || !_sdf->HasElement("sprocket")) {
    ignerr << "[" << this->data_->plugin_name << "] missing <sprocket> element" << std::endl;
    return;
  }

  auto sdf = const_cast<sdf::Element *>(_sdf.get());
  const auto sprocket_elem = sdf->GetElement("sprocket");
  const auto sprocket_joint_name = sprocket_elem->Get<std::string>("joint", "").first;
  this->data_->sprocket_pitch_diameter =
    sprocket_elem->Get<double>("pitch_diameter", 0.0).first;

  this->data_->sprocket_joint = this->data_->jointByName(_ecm, sprocket_joint_name);
  if (this->data_->sprocket_joint == gz::kNullEntity ||
      this->data_->sprocket_pitch_diameter <= 0.0) {
    ignerr << "[" << this->data_->plugin_name << "] invalid sprocket joint or pitch_diameter"
           << std::endl;
    return;
  }

  this->data_->velocity_deadband = _sdf->Get<double>("velocity_deadband", 0.02).first;

  if (!sdf->HasElement("track")) {
    ignerr << "[" << this->data_->plugin_name << "] missing <track> element" << std::endl;
    return;
  }

  const auto track_elem = sdf->GetElement("track");
  for (auto segment_elem = track_elem->GetElement("segment"); segment_elem;
       segment_elem = segment_elem->GetNextElement("segment"))
  {
    const auto joint_name = segment_elem->Get<std::string>("joint", "").first;
    const auto joint = this->data_->jointByName(_ecm, joint_name);
    if (joint == gz::kNullEntity) {
      ignerr << "[" << this->data_->plugin_name << "] segment joint not found: " << joint_name
             << std::endl;
      continue;
    }

    SegmentCommand segment;
    segment.joint = joint;
    if (IgnitionContinuousTrackSimplePrivate::isLikelyRotationalSegment(segment_elem)) {
      const auto diameter = segment_elem->Get<double>("pitch_diameter", 0.0).first;
      if (diameter <= 0.0) {
        ignerr << "[" << this->data_->plugin_name << "] invalid segment pitch_diameter: "
               << joint_name << std::endl;
        continue;
      }
      segment.sprocket_to_segment = this->data_->sprocket_pitch_diameter / diameter;
      segment.reset_position = false;
    } else {
      segment.sprocket_to_segment = this->data_->sprocket_pitch_diameter / 2.0;
      segment.translation_period = segment_elem->Get<double>("translation_period", 0.0).first;
      segment.reset_position = true;
    }
    this->data_->segments.push_back(segment);
  }

  ignmsg << "[" << this->data_->plugin_name << "] loaded Ignition simple continuous track: "
         << this->data_->segments.size() << " segments" << std::endl;
}

void IgnitionContinuousTrackSimple::PreUpdate(
  const gz::UpdateInfo &_info,
  gz::EntityComponentManager &_ecm)
{
  if (_info.paused || this->data_->sprocket_joint == gz::kNullEntity) {
    return;
  }

  const double dt = std::chrono::duration<double>(_info.dt).count();

  double sprocket_vel =
    IgnitionContinuousTrackSimplePrivate::jointVelocity(_ecm, this->data_->sprocket_joint);
  if (std::abs(sprocket_vel) < this->data_->velocity_deadband) {
    sprocket_vel = 0.0;
  }

  for (auto &segment : this->data_->segments) {
    if (segment.joint == gz::kNullEntity) {
      continue;
    }
    const double segment_vel = sprocket_vel * segment.sprocket_to_segment;
    if (segment.reset_position) {
      segment.position = IgnitionContinuousTrackSimplePrivate::wrappedTranslation(
        segment.position + segment_vel * dt, segment.translation_period);
      IgnitionContinuousTrackSimplePrivate::resetJointPosition(
        _ecm, segment.joint, segment.position);
    }
    IgnitionContinuousTrackSimplePrivate::setJointVelocity(
      _ecm, segment.joint, segment_vel);
  }
}

}  // namespace gazebo_continuous_track_ros2_ignition

IGNITION_ADD_PLUGIN(
  gazebo_continuous_track_ros2_ignition::IgnitionContinuousTrackSimple,
  ignition::gazebo::System,
  gazebo_continuous_track_ros2_ignition::IgnitionContinuousTrackSimple::ISystemConfigure,
  gazebo_continuous_track_ros2_ignition::IgnitionContinuousTrackSimple::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
  gazebo_continuous_track_ros2_ignition::IgnitionContinuousTrackSimple,
  "gazebo_continuous_track_ros2_ignition::IgnitionContinuousTrackSimple")
