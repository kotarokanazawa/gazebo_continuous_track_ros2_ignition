# gazebo_continuous_track_ros2_ignition

ROS 2 / Ignition Gazebo plugins and xacro macros for simulating
non-deformable continuous tracks with explicit track segment and optional
grouser geometry.

This is an unofficial ROS 2 / Ignition Gazebo port of the ROS 1 package
originally created by Okada et al. The original ROS 1 repository is:
https://github.com/yoshito-n-students/gazebo_continuous_track

This package currently installs one Ignition Gazebo System plugin:

- `libIgnitionContinuousTrackSimple.so`: drives existing straight/revolute
  track segment joints from a sprocket joint.

The plugin is equivalent in scope to the Gazebo Classic
`libContinuousTrackSimple.so` plugin. The full Gazebo Classic
`libContinuousTrack.so` runtime track-shoe generator is not implemented for
Ignition Gazebo yet.

The companion package `gazebo_continuous_track_example_ros2_ignition` contains
complete launch and model examples.

## Build

Ubuntu 22.04 / ROS 2 Humble with Ignition Fortress requires:

```bash
sudo apt install libignition-gazebo6-dev libignition-plugin-dev ignition-fortress
```

Build the package:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select gazebo_continuous_track_ros2_ignition --symlink-install
source install/setup.bash
```

The package installs environment hooks, so sourcing `install/setup.bash` adds
the plugin library and resource paths needed by Ignition Gazebo.

## Using `libIgnitionContinuousTrackSimple.so`

Use the simple plugin when the track is already represented by links and joints
and you want Ignition Gazebo to drive those joints from a sprocket speed.

```xml
<gazebo>
  <plugin name="gazebo_continuous_track_ros2_ignition::IgnitionContinuousTrackSimple"
          filename="libIgnitionContinuousTrackSimple.so">
    <track_name>track</track_name>
    <sprocket>
      <joint>sprocket_axle</joint>
      <pitch_diameter>0.24</pitch_diameter>
    </sprocket>
    <track>
      <segment>
        <joint>track_straight_segment_joint0</joint>
        <translation_period>0.0546</translation_period>
      </segment>
      <segment>
        <joint>track_arc_segment_joint0</joint>
        <pitch_diameter>0.24</pitch_diameter>
      </segment>
    </track>
    <velocity_deadband>0.02</velocity_deadband>
  </plugin>
</gazebo>
```

### `libIgnitionContinuousTrackSimple.so` Parameters

| Parameter | Required | Description |
| --- | --- | --- |
| `plugin/@name` | yes | Ignition System plugin class name. Use `gazebo_continuous_track_ros2_ignition::IgnitionContinuousTrackSimple`. |
| `plugin/@filename` | yes | Use `libIgnitionContinuousTrackSimple.so`. |
| `track_name` | no | Human-readable instance name used in log messages. |
| `sprocket/joint` | yes | Existing sprocket joint used as the speed source. |
| `sprocket/pitch_diameter` | yes | Sprocket pitch diameter in meters. |
| `track/segment` | yes | One or more joints to drive from the sprocket. |
| `track/segment/joint` | yes | Existing prismatic or revolute joint. |
| `track/segment/pitch_diameter` | required for revolute joints | Pitch diameter for a rotational segment. Omit for prismatic segments. |
| `track/segment/translation_period` | no | Wrap period in meters for a prismatic segment. The joint position is kept in the centred range `[-period/2, period/2)`. Use the repeated grouser or tread pitch when the straight segment should cycle visually. |
| `velocity_deadband` | no | Sprocket velocity threshold below which the track is treated as stopped. Default is `0.02`. |

## Xacro Macros

The package provides xacro helpers under `urdf_xacro/`.

### `make_track_simple_ignition`

Include:

```xml
<xacro:include filename="$(find gazebo_continuous_track_ros2_ignition)/urdf_xacro/macros_track_simple_ignition.urdf.xacro" />
```

Example:

```xml
<xacro:make_track_simple_ignition
  name="track"
  mass="0.4"
  length="0.7"
  radius="0.115"
  width="0.5"
  parent="body"
  sprocket_joint="sprocket_axle"
  pitch_diameter="0.23"
  grouser_count="40"
  grouser_height="0.02"
  grouser_width="0.018"
  enable_plugin="true">
  <material>
    <ambient>0.1 0.1 0.3 1</ambient>
    <diffuse>0.2 0.2 0.2 1</diffuse>
  </material>
</xacro:make_track_simple_ignition>
```

Parameters:

| Parameter | Default | Description |
| --- | --- | --- |
| `name` | required | Prefix for generated links, joints, and plugin instance. |
| `x y z` | `0` | Track center offset in the parent model frame. |
| `roll pitch yaw` | `0` | Track orientation. |
| `mass` | required | Total generated track mass. |
| `length` | required | Straight section length in meters. |
| `radius` | required | Pulley radius in meters. |
| `width` | required | Track width in meters. |
| `parent` | required | Parent link for generated segment joints. |
| `sprocket_joint` | required | Existing sprocket joint name. |
| `pitch_diameter` | required | Sprocket and arc segment pitch diameter used by the plugin. |
| `mu` | `0.5` | Contact friction coefficient. |
| `min_depth` | `0.01` | ODE contact min depth. |
| `implicit_spring_damper` | `1` | Joint spring/damper stabilization flag. |
| `enable_plugin` | `true` | Set `false` to generate only the geometry and joints without adding `libIgnitionContinuousTrackSimple.so`. |
| `velocity_deadband` | `0.02` | Sprocket velocity threshold used by the plugin. |
| `grouser_count` | `0` | Number of grousers around the full track. Set greater than `0` to generate grouser visual and collision geometry. |
| `grouser_height` | `0.02` | Grouser height in meters. |
| `grouser_width` | `0.02` | Grouser width along the belt path in meters. |
| `material` block | required | Material inserted into generated visuals. |

When `grouser_count` is greater than `0`, the macro computes a common tread
pitch from the full belt length and distributes straight and arc grousers with
the same pitch. The same pitch is also passed to prismatic segments as
`translation_period` so the straight belt animation cycles consistently with
the pulley sections.

### `make_track_simple_ignition_wheels`

Include the same `macros_track_simple_ignition.urdf.xacro` file.

This macro is used for comparison models made from multiple wheel segments. It
accepts the same parameters as `make_track_simple_ignition` and additionally
requires:

| Parameter | Default | Description |
| --- | --- | --- |
| `count` | required | Number of wheel arc segments to generate. |

`grouser_count` defaults to `0`, so smooth comparison models remain smooth
unless grousers are explicitly requested.

### Common Geometry Helpers

`macros_common_ignition.urdf.xacro` provides reusable geometry helpers such as
`make_box_element`, `make_polyline_element`, straight segment templates, arc
segment templates, and static grouser population helpers. These are mainly used
by the track macros but can also be reused by custom models.

## Notes

- Ignition Gazebo uses a different plugin API from Gazebo Classic. This package
  does not create nested runtime track-shoe models like Classic
  `libContinuousTrack.so`.
- The model must define the sprocket joint and all straight/revolute segment
  joints before the plugin is loaded.
- Prismatic track segments omit `pitch_diameter` and can use
  `translation_period` for wrapped straight-section animation.
- Revolute track segments require `pitch_diameter`.
- Grousers generated by the xacro macros have both visual and collision
  geometry.
- For ROS 2 launch files, comparison worlds, and `/cmd_vel` examples, see
  `gazebo_continuous_track_example_ros2_ignition`.

## License

This ROS 2 / Ignition Gazebo port keeps the original MIT License and copyright
notice. See `LICENSE` for the full license text. Keep that notice with
redistributed source or binary copies.

## Citation

Y. Okada, S. Kojima, K. Ohno and S. Tadokoro,
"Real-time Simulation of Non-Deformable Continuous Tracks with Explicit
Consideration of Friction and Grouser Geometry,"
2020 IEEE International Conference on Robotics and Automation (ICRA), 2020.
