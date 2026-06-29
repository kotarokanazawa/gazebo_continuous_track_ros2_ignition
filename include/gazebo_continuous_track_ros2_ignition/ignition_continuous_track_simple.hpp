#ifndef GAZEBO_CONTINUOUS_TRACK_ROS2_IGNITION_CONTINUOUS_TRACK_SIMPLE_HPP_
#define GAZEBO_CONTINUOUS_TRACK_ROS2_IGNITION_CONTINUOUS_TRACK_SIMPLE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Types.hh>

namespace gazebo_continuous_track_ros2_ignition
{

class IgnitionContinuousTrackSimplePrivate;

class IgnitionContinuousTrackSimple:
  public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate
{
public:
  IgnitionContinuousTrackSimple();
  ~IgnitionContinuousTrackSimple() override;

  void Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &_event_mgr) override;

  void PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm) override;

private:
  std::unique_ptr<IgnitionContinuousTrackSimplePrivate> data_;
};

}  // namespace gazebo_continuous_track_ros2_ignition

#endif  // GAZEBO_CONTINUOUS_TRACK_ROS2_IGNITION_CONTINUOUS_TRACK_SIMPLE_HPP_
