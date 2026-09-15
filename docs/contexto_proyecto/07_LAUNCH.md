# Launch: declaraciones y nodos

Índice AST para contrastar perfiles; no expande inclusiones ni simula ROS launch.

## src/rescue_bringup/launch/camera.launch.py

- L80: package=`'openni2_camera'`, executable=`'openni2_camera_node'`, name=`'openni2_camera'`
- L96: package=`'v4l2_camera'`, executable=`'v4l2_camera_node'`, name=`'rgb_camera'`
- L168: argumento `'driver'`; default_value='v4l2'
- L58: package=`'orbbec_camera'`, executable=`'orbbec_camera_node'`, name=`'orbbec_camera'`
- L125: package=`'astra_camera'`, executable=`'astra_camera_node'`, name=`'astra_camera'`
- L184: package=`'v4l2_camera'`, executable=`'v4l2_camera_node'`, name=`'rgb_camera'`

## src/rescue_bringup/launch/full_bringup.launch.py

- L66: package=`'joy'`, executable=`'joy_node'`, name=`'joy_node'`, condition=`IfCondition(teleop)`
- L75: package=`'rescue_command_station'`, executable=`'ps4_teleop_node'`, name=`'ps4_teleop_node'`, condition=`IfCondition(teleop)`
- L84: package=`'rescue_command_station'`, executable=`'dashboard_node'`, name=`'dashboard_node'`, condition=`IfCondition(teleop)`
- L93: argumento `'use_sim_time'`; default_value='false'
- L98: argumento `'serial_port'`; default_value='/dev/ttyUSB0'
- L103: argumento `'teleop'`; default_value='false'

## src/rescue_bringup/launch/lidar_ld19.launch.py

- L39: package=`'ldlidar_component'`, plugin=`'ldlidar::LdLidarComponent'`, name=`'ldlidar_node'`
- L88: argumento `'use_sim_time'`; default_value='false'
- L93: argumento `'serial_port'`; default_value='/dev/ttyAMA0'

## src/rescue_bringup/launch/logitech_vision.launch.py

- L28: package=`'rescue_bringup'`, executable=`'logitech_pub'`, name=`'logitech_pub'`
- L72: argumento `'device'`; default_value='2'
- L73: argumento `'hazmat_model'`; default_value=''
- L74: argumento `'enable_yolo'`; default_value='true'
- L75: argumento `'fps'`; default_value='15'
- L76: argumento `'output_dir'`; default_value=PathJoinSubstitution([EnvironmentVariable('HOME'), 'maps'])
- L44: package=`'rescue_bringup'`, executable=`'object_detector'`, name=`'object_detector'`

## src/rescue_bringup/launch/pedro_pc.launch.py

- L139: argumento `'network'`; default_value='auto'
- L141: argumento `'cable_interface'`; default_value='eno1'
- L142: argumento `'wifi_interface'`; default_value='wlp0s20f3'
- L143: argumento `'pi_cable_ip'`; default_value='10.42.0.240'
- L144: argumento `'pi_wifi_ip'`; default_value='192.168.231.137'
- L145: argumento `'pc_cable_ip'`; default_value='10.42.0.1'
- L146: argumento `'pc_wifi_ip'`; default_value='192.168.231.15'
- L147: argumento `'launch_slam'`; default_value='true'
- L148: argumento `'launch_rviz'`; default_value='true'
- L149: argumento `'launch_dashboard'`; default_value='true'
- L150: argumento `'launch_detector'`; default_value='false'
- L151: argumento `'hazmat_model'`; default_value=''
- L152: argumento `'output_dir'`; default_value=PathJoinSubstitution([EnvironmentVariable('HOME'), 'maps'])

## src/rescue_bringup/launch/pedro_pi.launch.py

- L213: package=`'rescue_robot_core'`, executable=`'motor_driver_node'`, name=`'motor_driver_node'`, condition=`IfCondition(launch_motors)`
- L234: argumento `'network'`; default_value='auto'
- L236: argumento `'cable_interface'`; default_value='eth0'
- L237: argumento `'wifi_interface'`; default_value='wlan0'
- L238: argumento `'pc_cable_ip'`; default_value='10.42.0.1'
- L239: argumento `'pc_wifi_ip'`; default_value='192.168.231.15'
- L240: argumento `'pi_cable_ip'`; default_value='10.42.0.240'
- L241: argumento `'pi_wifi_ip'`; default_value='192.168.231.137'
- L242: argumento `'launch_lidar'`; default_value='true'
- L243: argumento `'launch_camera'`; default_value='auto'
- L245: argumento `'launch_logitech'`; default_value='auto'
- L247: argumento `'launch_robot_description'`; default_value='true'
- L248: argumento `'launch_motors'`; default_value='true'
- L249: argumento `'launch_servos'`; default_value='true'
- L250: argumento `'camera_driver'`; default_value='astra_core'
- L251: argumento `'astra_depth_index'`; default_value='-1'
- L252: argumento `'astra_color_index'`; default_value='auto'
- L253: argumento `'astra_fps'`; default_value='30'
- L254: argumento `'jpeg_quality'`; default_value='80'
- L255: argumento `'logitech_device'`; default_value='auto'
- L256: argumento `'hazmat_model'`; default_value=''
- L257: argumento `'output_dir'`; default_value='/home/gardian/maps'
- L258: argumento `'max_pwm'`; default_value='0.85'
- L259: argumento `'pwm_frequency_hz'`; default_value='1000'
- L260: argumento `'cmd_timeout_seconds'`; default_value='1.0'

## src/rescue_bringup/launch/pi_sensors.launch.py

- L122: argumento `'launch_camera'`; default_value='true'
- L127: argumento `'launch_lidar'`; default_value='true'
- L132: argumento `'launch_logitech'`; default_value='true'
- L137: argumento `'launch_servos'`; default_value='true'
- L142: argumento `'hazmat_model'`; default_value=''
- L80: package=`'rescue_bringup'`, executable=`'astra_relay'`, name=`'astra_color_relay'`, condition=`IfCondition(launch_camera)`

## src/rescue_bringup/launch/robot_description.launch.py

- L31: package=`'robot_state_publisher'`, executable=`'robot_state_publisher'`, name=`'robot_state_publisher'`
- L43: package=`'joint_state_publisher'`, executable=`'joint_state_publisher'`, name=`'joint_state_publisher'`
- L54: package=`'tf2_ros'`, executable=`'static_transform_publisher'`, name=`'static_tf_odom_base'`
- L63: argumento `'use_sim_time'`; default_value='false'

## src/rescue_bringup/launch/save_map.launch.py

- L30: package=`'nav2_map_server'`, executable=`'map_saver_cli'`, name=`'map_saver'`
- L43: argumento `'map_name'`; default_value='rescue_map'
- L48: argumento `'map_dir'`; default_value=os.path.expanduser('~/maps')

## src/rescue_bringup/launch/slam.launch.py

- L265: argumento `'use_sim_time'`; default_value='false'
- L270: argumento `'serial_port'`; default_value='/dev/ttyUSB0'
- L275: argumento `'launch_rviz'`; default_value='true'
- L280: argumento `'launch_lidar'`; default_value='true'
- L285: argumento `'launch_camera'`; default_value='true'
- L290: argumento `'launch_robot_description'`; default_value='true'
- L295: argumento `'launch_detector'`; default_value='false'
- L300: argumento `'hazmat_model'`; default_value=''
- L305: argumento `'output_dir'`; default_value='/workspace/maps'
- L84: package=`'depthimage_to_laserscan'`, executable=`'depthimage_to_laserscan_node'`, name=`'depth_to_laserscan'`
- L110: package=`'rescue_bringup'`, executable=`'scan_merger'`, name=`'scan_merger'`
- L144: package=`'slam_toolbox'`, executable=`'async_slam_toolbox_node'`, name=`'slam_toolbox'`
- L179: package=`'rescue_bringup'`, executable=`'pointcloud_accumulator'`, name=`'pointcloud_accumulator'`
- L203: package=`'rescue_bringup'`, executable=`'geotiff_writer'`, name=`'geotiff_writer'`
- L224: package=`'rescue_bringup'`, executable=`'object_detector'`, name=`'object_detector'`, condition=`IfCondition(launch_detector)`
- L253: package=`'rviz2'`, executable=`'rviz2'`, name=`'rviz2'`, condition=`IfCondition(launch_rviz)`

## src/rescue_bringup/launch/vision.launch.py

- L173: argumento `'camera_driver'`; default_value='astra_core'
- L178: argumento `'hazmat_model'`; default_value=''
- L183: argumento `'enable_yolo'`; default_value='true'
- L188: argumento `'launch_rviz'`; default_value='false'
- L67: package=`'rescue_robot_core'`, executable=`'astra_rgbd_camera_node'`, name=`'astra_rgbd_camera_node'`, condition=`LaunchConfigurationEquals('camera_driver', 'astra_core')`
- L94: package=`'rescue_bringup'`, executable=`'object_detector'`, name=`'object_detector'`, condition=`LaunchConfigurationEquals('camera_driver', 'astra_sdk')`
- L125: package=`'rescue_bringup'`, executable=`'object_detector'`, name=`'object_detector'`, condition=`LaunchConfigurationEquals('camera_driver', 'astra_core')`
- L162: package=`'rviz2'`, executable=`'rviz2'`, name=`'rviz2'`, condition=`IfCondition(launch_rviz)`

## src/rescue_command_station/launch/arm_station.launch.py

- L37: package=`'rescue_command_station'`, executable=`'arm_gui_node'`, name=`'gui_control'`, condition=`IfCondition(LaunchConfiguration('gui'))`
- L45: argumento `'arm_config'`; default_value=arm_default
- L46: argumento `'servos_config'`; default_value=servos_default
- L47: argumento `'gui'`; default_value='true'
- L48: argumento `'managed_by_dashboard'`; default_value='false'
- L49: argumento `'sim'`; default_value='false'
- L54: package=`'rescue_robot_core'`, executable=`'dynamixel_sim_node'`, name=`'ax12a_driver'`, condition=`IfCondition(sim)`
- L59: package=`'rescue_robot_core'`, executable=`'dynamixel_sim_node'`, name=`'ex106_driver'`, condition=`IfCondition(sim)`
- L66: package=`'rescue_command_station'`, executable=`'cinematica_node'`, name=`'cinematica'`
- L70: package=`'rescue_command_station'`, executable=`'cartesian_node'`, name=`'cartesian'`

## src/rescue_command_station/launch/command_station.launch.py

- L19: argumento `'front_camera_topic'`; default_value='/robot/camera/front/image_raw/compressed'
- L23: argumento `'astra_color_topic'`; default_value='/robot/camera/astra/color/image_raw/compressed'
- L27: argumento `'astra_depth_topic'`; default_value='/robot/camera/astra/depth/image_raw/compressed'
- L31: argumento `'point_cloud_topic'`; default_value='/robot/camera/astra/points'
- L35: argumento `'joy_autorepeat_rate'`; default_value='20.0'
- L39: argumento `'joy_deadzone'`; default_value='0.05'
- L43: argumento `'cmd_publish_rate_hz'`; default_value='20.0'
- L47: argumento `'joy_timeout_seconds'`; default_value='0.7'
- L51: package=`'joy'`, executable=`'joy_node'`, name=`'joy_node'`
- L61: package=`'rescue_command_station'`, executable=`'ps4_teleop_node'`, name=`'ps4_teleop_node'`
- L71: package=`'rescue_command_station'`, executable=`'dashboard_node'`, name=`'drive_dashboard_node'`

## src/rescue_robot_core/launch/robot_core.launch.py

- L37: argumento `'logitech_index'`; default_value='0'
- L38: argumento `'logitech_width'`; default_value='640'
- L39: argumento `'logitech_height'`; default_value='480'
- L40: argumento `'logitech_fps'`; default_value='30'
- L41: argumento `'logitech_fourcc'`; default_value='MJPG'
- L42: argumento `'camera_buffer_size'`; default_value='1'
- L43: argumento `'jpeg_quality'`; default_value='80'
- L44: argumento `'astra_depth_index'`; default_value='-1'
- L45: argumento `'astra_color_index'`; default_value='2'
- L46: argumento `'astra_fps'`; default_value='30'
- L47: argumento `'astra_depth_fourcc'`; default_value='YUYV'
- L48: argumento `'astra_color_fourcc'`; default_value='MJPG'
- L49: argumento `'fx'`; default_value='525.0'
- L50: argumento `'fy'`; default_value='525.0'
- L51: argumento `'cx'`; default_value='319.5'
- L52: argumento `'cy'`; default_value='239.5'
- L53: argumento `'depth_scale'`; default_value='0.001'
- L54: argumento `'point_cloud_stride'`; default_value='8'
- L55: argumento `'point_cloud_every_n'`; default_value='3'
- L56: argumento `'point_cloud_max_depth_m'`; default_value='5.0'
- L57: argumento `'max_pwm'`; default_value='0.85'
- L58: argumento `'pwm_frequency_hz'`; default_value='1000'
- L59: argumento `'cmd_timeout_seconds'`; default_value='1.0'
- L60: package=`'rescue_robot_core'`, executable=`'motor_driver_node'`, name=`'motor_driver_node'`
- L71: package=`'rescue_robot_core'`, executable=`'logitech_camera_node'`, name=`'logitech_camera_node'`
- L86: package=`'rescue_robot_core'`, executable=`'astra_rgbd_camera_node'`, name=`'astra_rgbd_camera_node'`

## src/rescue_robot_core/launch/servos.launch.py

- L33: argumento `'servos_config'`; default_value=config_default
- L34: argumento `'log_level'`; default_value='info'
- L36: package=`'rescue_robot_core'`, executable=`'dynamixel_bus_node'`, name=`'ax12a_driver'`
- L41: package=`'rescue_robot_core'`, executable=`'ex106_driver_node'`, name=`'ex106_driver'`

