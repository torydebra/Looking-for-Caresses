#Miro topic sub and
Published topics:
 * /miro/rob01/platform/mics [miro_msgs/platform_mics] 1 publisher
 * /miro/rob01/platform/caml/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription] 1 publisher
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher
 * /miro/rob01/platform/caml/compressedDepth [sensor_msgs/CompressedImage] 1 publisher
 * /miro/rob01/log [std_msgs/String] 1 publisher
 * /miro/rob01/platform/camr/theora/parameter_updates [dynamic_reconfigure/Config] 1 publisher
 * /miro/rob01/platform/camr/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription] 1 publisher
 * /miro/rob01/platform/caml/theora [theora_image_transport/Packet] 1 publisher
 * /miro/rob01/core/state [miro_msgs/core_state] 1 publisher
 * /miro/rob01/platform/caml/compressedDepth/parameter_descriptions [dynamic_reconfigure/ConfigDescription] 1 publisher
 * /miro/rob01/platform/sensors [miro_msgs/platform_sensors] 1 publisher
 * /miro/rob01/platform/camr/compressed [sensor_msgs/CompressedImage] 1 publisher
 * /miro/rob01/platform/camr/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription] 1 publisher
 * /miro/rob01/platform/caml/compressed [sensor_msgs/CompressedImage] 1 publisher
 * /miro/rob01/platform/caml/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription] 1 publisher
 * /miro/rob01/platform/caml/theora/parameter_updates [dynamic_reconfigure/Config] 1 publisher
 * /miro/rob01/platform/camr/compressedDepth/parameter_updates [dynamic_reconfigure/Config] 1 publisher
 * /rosout [rosgraph_msgs/Log] 1 publisher
 * /miro/rob01/platform/caml/compressedDepth/parameter_updates [dynamic_reconfigure/Config] 1 publisher
 * /miro/rob01/platform/camr/theora [theora_image_transport/Packet] 1 publisher
 * /miro/rob01/platform/camr/compressedDepth/parameter_descriptions [dynamic_reconfigure/ConfigDescription] 1 publisher
 * /miro/rob01/platform/camr/compressedDepth [sensor_msgs/CompressedImage] 1 publisher
 * /miro/rob01/platform/camr [sensor_msgs/Image] 1 publisher
 * /miro/rob01/platform/camr/compressed/parameter_updates [dynamic_reconfigure/Config] 1 publisher
 * /miro/rob01/platform/caml/compressed/parameter_updates [dynamic_reconfigure/Config] 1 publisher
 * /miro/rob01/platform/caml [sensor_msgs/Image] 1 publisher
 * /miro/rob01/platform/state [miro_msgs/platform_state] 1 publisher

Subscribed topics:
 * /miro/rob01/core/control [miro_msgs/core_control] 1 subscriber
 * /miro/rob01/platform/control [miro_msgs/platform_control] 1 subscriber
 * /rosout [rosgraph_msgs/Log] 1 subscriber
 * /miro/rob01/bridge/config [miro_msgs/bridge_config] 1 subscriber
 * /miro/rob01/platform/config [miro_msgs/platform_config] 1 subscriber
 * /miro/rob01/bridge/stream [miro_msgs/bridge_stream] 1 subscriber
 * /miro/rob01/core/config [miro_msgs/core_config] 1 subscriber


## type of [miro_msgs/platform_sensors]
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 msg_flags
int32 time_usec
float32 battery_voltage
sensor_msgs/Temperature temperature
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  float64 temperature
  float64 variance
sensor_msgs/Imu accel_head
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
  float64[9] orientation_covariance
  geometry_msgs/Vector3 angular_velocity
    float64 x
    float64 y
    float64 z
  float64[9] angular_velocity_covariance
  geometry_msgs/Vector3 linear_acceleration
    float64 x
    float64 y
    float64 z
  float64[9] linear_acceleration_covariance
sensor_msgs/Imu accel_body
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
  float64[9] orientation_covariance
  geometry_msgs/Vector3 angular_velocity
    float64 x
    float64 y
    float64 z
  float64[9] angular_velocity_covariance
  geometry_msgs/Vector3 linear_acceleration
    float64 x
    float64 y
    float64 z
  float64[9] linear_acceleration_covariance
nav_msgs/Odometry odometry
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  string child_frame_id
  geometry_msgs/PoseWithCovariance pose
    geometry_msgs/Pose pose
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
    float64[36] covariance
  geometry_msgs/TwistWithCovariance twist
    geometry_msgs/Twist twist
      geometry_msgs/Vector3 linear
        float64 x
        float64 y
        float64 z
      geometry_msgs/Vector3 angular
        float64 x
        float64 y
        float64 z
    float64[36] covariance
sensor_msgs/JointState joint_state
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  string[] name
  float64[] position
  float64[] velocity
  float64[] effort
float32 eyelid_closure
uint8 dip_state_phys
sensor_msgs/Range sonar_range
  uint8 ULTRASOUND=0
  uint8 INFRARED=1
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  uint8 radiation_type
  float32 field_of_view
  float32 min_range
  float32 max_range
  float32 range
uint8[4] light
uint8[4] touch_head
uint8[4] touch_body
uint8[2] cliff




