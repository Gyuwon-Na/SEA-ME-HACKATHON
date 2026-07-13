# SEA-ME-HACKATHON

> 근데 내가 아직 차량에 패키지에 대한 이해도가 좀 부족한데 전체 패키지들에 대한 설명과 패키지들끼리 유기적 관계 정리해서 artifact 남겨줘

▸ Thought for 22s
  The user wants a comprehensive explanation of all packages in the D-Racer-Kit pr...
  전체 패키지를 체계적으로 조사하겠습니다. 여러 서브에이전트를 활용해 병렬로 읽겠습니다.

● Agent(research: Camera/Battery/Monitor researcher)(Read and summarize the following ROS2 packages in /home/hyun/D-Racer-Kit/src/. F...)

  3개의 리서치 에이전트를 배포했습니다. 결과가 돌아오면 종합해서 Artifact를 작성하겠습니다.

● Schedule() (ctrl+o to expand)

  서브에이전트 결과를 기다리고 있습니다.

  <msg_received>
  <msg_id>msg-zHSl5aqXfp5yU82ERxqCA</msg_id>
  <msg_source>subagent</msg_source>
  <sender_name>Camera/Battery/Monitor researcher</sender_name>
  <sender_id>03d66dbb-342f-428d-a55a-56db533f4dfc</sender_id>

  Here is a detailed summary of the five ROS2 packages:
  ──────
  ## 1.  camera  Package

  What it does: Captures frames from a USB or MIPI camera (via GStreamer) and publishes them as  sensor_msgs/CompressedImage  on a ROS2 topic.

  Key Source Files:

  •  src/camera_node.py : The main ROS2 node ( CameraNode ). Uses GStreamer to open a camera pipeline (configurable for USB or MIPI), captures frames in a dedicated
  threading.Thread , JPEG-encodes them with OpenCV, and publishes  CompressedImage  messages. The thread runs in a tight loop, and a  create_timer  callback
  publishes the latest frame at the configured  publish_hz .
  •  src/camera_utils.py : Helper functions for building GStreamer pipeline strings.  build_gst_pipeline()  creates the correct pipeline based on the  camera_type 
  parameter ( usb  or  mipi ), selecting either  v4l2src  or  v4l2src  with a specific pipeline for MIPI.

  ROS2 Topics:

  • Publishes: Configurable via  publish_topic  parameter (default:  /camera/image/compressed ), type  sensor_msgs/CompressedImage .

  Important Config Parameters:

   Parameter                                            | Default                                             | Description
  ------------------------------------------------------|-----------------------------------------------------|-----------------------------------------------------
    camera_type                                         |  "usb"                                              |  "usb"  or  "mipi" 
    device_path                                         |  "/dev/video1"                                      | Camera device path
    width                                               |  640                                                | Capture width
    height                                              |  480                                                | Capture height
    fps                                                 |  30                                                 | Capture FPS
    publish_topic                                       |  "/camera/image/compressed"                         | Output topic
    publish_hz                                          |  30.0                                               | Publishing rate
    jpeg_quality                                        |  80                                                 | JPEG encode quality
    debug_log                                           |  False                                              | Enable verbose logging

  Dependencies:  rclpy ,  sensor_msgs ,  std_msgs ,  cv_bridge ,  opencv-python ,  PyGObject  (GStreamer).
  ──────
  ## 2.  battery  Package

  What it does: Reads battery voltage/current from an INA219 sensor over I2C and publishes battery percentage and voltage to ROS2 topics.

  Key Source Files:

  •  src/battery_node.py : The main ROS2 node ( BatteryNode ). Uses the  topst_utils  package's I2C helper ( TopstI2C ) to communicate with the INA219 sensor at
  address  0x42  on I2C bus  3 . Configures the INA219 with a calibration register, reads bus voltage and shunt voltage, and computes battery percentage using a
  linear mapping from  6.4V  (0%) to  8.4V  (100%). Publishes to  /battery/percentage  (Float32) and  /battery/voltage  (Float32) at a configurable rate.

  ROS2 Topics:

  • Publishes:
      •  /battery/percentage  —  std_msgs/Float32  (0-100%)
      •  /battery/voltage  —  std_msgs/Float32  (raw voltage in Volts)


  Important Config Parameters:

   Parameter                                            | Default                                             | Description
  ------------------------------------------------------|-----------------------------------------------------|-----------------------------------------------------
    publish_hz                                          |  1.0                                                | Publishing rate
    i2c_bus                                             |  3                                                  | I2C bus number
    i2c_addr                                            |  0x42                                               | INA219 I2C address
    debug_log                                           |  False                                              | Enable verbose logging

  Dependencies:  rclpy ,  std_msgs ,  topst_utils  (for  TopstI2C ).
  ──────
  ## 3.  battery_msgs  Package

  What it does: Defines a custom  Battery.msg  ROS2 message type for battery status. However, in the current codebase, it appears that  battery_node  uses 
  std_msgs/Float32  instead of this custom message. This package might be a legacy or future-use definition.

  Message Definition ( msg/Battery.msg ):

    float32 voltage
    float32 current
    float32 percentage
    
  Dependencies:  rosidl_default_generators ,  rosidl_default_runtime ,  std_msgs .
  ──────
  ## 4.  monitor  Package

  What it does: Provides a Flask-based web dashboard for monitoring the D-Racer's camera stream, battery status, and system state in a browser.

  Key Source Files:

  •  src/monitor_node.py : The main ROS2 node ( MonitorNode ). Subscribes to the camera image topic ( /camera/image/compressed ), battery percentage (
  /battery/percentage ), and joystick control ( /joystick/control ). Runs a Flask web server on port  5555  (configurable) in a background thread. Serves a live
  MJPEG video stream at  /video_feed , and a JSON status endpoint at  /status  which returns the latest battery, steering, throttle, and mode. The main page is
  served from a Jinja2  index.html  template.
  •  templates/index.html : The web dashboard HTML. Displays the camera feed as an  <img>  pointed at  /video_feed , and polls  /status  every second to update
  battery, steering/throttle, and mode indicators via JavaScript.

  ROS2 Topics:

  • Subscribes:
      •  /camera/image/compressed  —  sensor_msgs/CompressedImage 
      •  /battery/percentage  —  std_msgs/Float32 
      •  /joystick/control  —  control_msgs/Control 


  Important Config Parameters:

   Parameter                                            | Default                                             | Description
  ------------------------------------------------------|-----------------------------------------------------|-----------------------------------------------------
    web_port                                            |  5555                                               | Flask server port
    image_topic                                         |  "/camera/image/compressed"                         | Camera topic to subscribe

  Dependencies:  rclpy ,  sensor_msgs ,  std_msgs ,  control_msgs ,  flask ,  opencv-python .
  ──────
  ## 5.  topst_utils  Package

  What it does: Provides low-level utility classes for TOPST board hardware access, including I2C communication and PCA9685 PWM driver control.

  Key Source Files:

  •  src/topst_i2c.py :  TopstI2C  class — a thin wrapper around Linux  smbus2  for I2C read/write operations. It provides methods like  write_byte_data , 
  read_byte_data ,  write_word_data ,  read_word_data  and uses  /dev/i2c-{bus}  devices.
  •  src/topst_pca9685.py :  TopstPCA9685  class — a PCA9685 16-channel PWM driver. Initializes the PCA9685 at I2C address  0x40  (default), sets the PWM frequency
  (default  50 Hz  for servos/ESC), and provides  set_pwm(channel, on, off)  and  set_pwm_freq(freq)  methods. Used by the  control  package to drive the servo
  (channel 0) and ESC (channel 1).
  ROS2 Topics: None — this is a pure library package with no nodes.

  Important Constants:

   Constant                                             | Value                                               | Description
  ------------------------------------------------------|-----------------------------------------------------|-----------------------------------------------------
   Default I2C bus                                      |  3                                                  | TOPST D3-G I2C bus
   PCA9685 address                                      |  0x40                                               | PWM driver I2C address
   Default PWM freq                                     |  50  Hz                                             | Standard servo/ESC frequency
   MODE1 register                                       |  0x00                                               | PCA9685 mode register
   PRESCALE register                                    |  0xFE                                               | Frequency prescale register

  Dependencies:  smbus2  (Python I2C library).
  </msg_received>

  <msg_received>
  <msg_id>msg-zHSl5aqXfp5yU82ERxqCB</msg_id>
  <msg_source>subagent</msg_source>
  <sender_name>Control/Joystick/Msgs researcher</sender_name>
  <sender_id>7f8cc286-758c-46b3-bb70-c4dc2cea2635</sender_id>
  Here is a thorough summary of all five packages:
  ──────
  ## 1.  control  Package
  What it does: The low-level actuator bridge. Subscribes to  /control  (steering + throttle commands) and  /joystick/control  topics, and drives the PCA9685 PWM
  channels for the steering servo (ch 0) and ESC motor (ch 1) over I2C. It also implements a voltage-based power guard that scales down throttle when battery
  voltage drops too low.
  Key Source Files:
  ###  src/control_node.py  — Main ROS2 Node ( ControlNode )

  • Subscribes to:
      •  /control  ( control_msgs/Control ) — autonomous driving commands
      •  /joystick/control  ( control_msgs/Control ) — manual gamepad commands
      •  /battery/voltage  ( std_msgs/Float32 ) — for the voltage guard
  • Uses  TopstPCA9685  from  topst_utils  to send PWM signals
  • Maps normalized steering [-1, 1] and throttle [0, 1] to PWM pulse widths:
      • Steering: center ~1480μs, range ±400μs (configurable  steer_min_us / steer_max_us )
      • Throttle: neutral ~1500μs, forward range 1500-2000μs (configurable)
  • Has a  use_joystick_control  parameter to switch between autonomous and manual input
  • Runs a  command_hz  timer (default 10 Hz) that applies the latest command to PWM

  ###  src/power_guard.py  — Voltage-based Throttle Limiter ( PowerGuard )
  • Monitors  /battery/voltage  and scales throttle to protect the battery:
      • Above  guard_low_voltage  (6.5V): full throttle (scale 1.0)
      • Between  guard_low_voltage  and  guard_critical_voltage  (6.2V): linearly reduced
      • At/below critical: minimum scale ( guard_min_scale  = 0.75)
  • Enforces a minimum floor throttle ( guard_floor_throttle  = 0.20) so the car doesn't stall mid-drive
  • Can be enabled/disabled via  voltage_guard_enabled  parameter

  Important Config Parameters:

   Parameter                                            | Default                                             | Description
  ------------------------------------------------------|-----------------------------------------------------|-----------------------------------------------------
    control_topic                                       |  /control                                           | Autonomous control input
    use_joystick_control                                |  False                                              |  True  = joystick,  False  = autonomous
    command_hz                                          |  10.0                                               | PWM update rate
    steer_center_us                                     |  1480                                               | Steering servo center (μs)
    steer_min_us                                        |  1080                                               | Steering full-left (μs)
    steer_max_us                                        |  1880                                               | Steering full-right (μs)
    throttle_neutral_us                                 |  1500                                               | ESC neutral (μs)
    throttle_max_us                                     |  2000                                               | ESC max forward (μs)
    throttle_min_us                                     |  1000                                               | ESC max reverse (μs)
    voltage_guard_enabled                               |  True                                               | Enable battery voltage guard
    guard_low_voltage                                   |  6.5                                                | Voltage where throttle starts reducing
    guard_critical_voltage                              |  6.2                                                | Voltage where throttle is at minimum scale
    guard_min_scale                                     |  0.75                                               | Minimum throttle scale factor
    guard_floor_throttle                                |  0.20                                               | Minimum throttle to prevent stalling
    battery_voltage_topic                               |  /battery/voltage                                   | Topic for voltage readings
  Dependencies:  rclpy ,  control_msgs ,  std_msgs ,  topst_utils  (TopstPCA9685).
  I2C: PCA9685 at address  0x40 , I2C bus  3 , 50 Hz PWM.
  ──────
  ## 2.  control_msgs  Package
  What it does: Defines the custom  Control.msg  message type used for steering and throttle commands throughout the system.

  Message Definition ( msg/Control.msg ):
    std_msgs/Header header
    float32 steering
    float32 throttle
    
  •  steering : normalized [-1.0, 1.0] (left to right)
  •  throttle : normalized [0.0, 1.0] (stop to max forward)
  •  header : standard ROS2 header with timestamp

  Dependencies:  rosidl_default_generators ,  rosidl_default_runtime ,  std_msgs .
  ──────
  ## 3.  joystick  Package

  What it does: Reads input from a ShanWan USB gamepad ( /dev/input/js0 ) and publishes control commands. Provides AUTO/MANUAL mode toggle and emergency stop
  functionality.

  Key Source Files:
  ###  src/joystick_node.py  — Main ROS2 Node ( JoystickNode )
  • Opens  /dev/input/js0  (configurable) and reads joystick events in a background thread
  • Publishes to:
      •  /joystick/control  ( control_msgs/Control ) — steering/throttle from analog sticks
      •  /joystick/mode  ( std_msgs/String ) — "AUTO" or "MANUAL"
      •  /joystick/estop  ( std_msgs/Bool ) — emergency stop flag
  • Button mappings (ShanWan gamepad):
      • A button (btn 0): Toggle AUTO/MANUAL mode
      • X button (btn 3): Emergency stop (toggle)
      • Left stick Y-axis (axis 1): Throttle (forward/back)
      • Left stick X-axis (axis 0): Steering (left/right)
  • In MANUAL mode: joystick directly controls steering/throttle
  • In AUTO mode: joystick steering/throttle are zeroed, but mode/estop still published
  • Has a  calibration_mode  parameter for testing stick ranges
  • Has a  start_in_manual  parameter (default  False  = starts in AUTO mode)
  Important Config Parameters:
   Parameter                                            | Default                                             | Description
  ------------------------------------------------------|-----------------------------------------------------|-----------------------------------------------------
    device_path                                         |  /dev/input/js0                                     | Gamepad device
    publish_hz                                          |  20.0                                               | Publishing rate
    calibration_mode                                    |  False                                              | Log raw axis values for tuning
    start_in_manual                                     |  False                                              | Initial mode
    deadzone                                            |  0.08                                               | Stick deadzone
  Dependencies:  rclpy ,  control_msgs ,  std_msgs .
  ──────
  ## 4.  joystick_msgs  Package

  What it does: Defines custom message types for joystick state. Like  battery_msgs , this appears to be a legacy or auxiliary package — the main  joystick_node 
  uses  control_msgs/Control  and  std_msgs/String / Bool  rather than these custom messages.

  Message Definitions:

  •  msg/Joystick.msg :

    std_msgs/Header header
    float32 steering
    float32 throttle
    bool estop
    string mode
    
  Dependencies:  rosidl_default_generators ,  rosidl_default_runtime ,  std_msgs .
  ──────
  ## 5.  config  Package
  What it does: A shared configuration package that stores hardware calibration and system-wide YAML configuration files. It does NOT contain any ROS2 nodes — it is
  a pure data/config package.

  Key Files:

  •  config/d3racer.yaml : Main hardware calibration file with:
      • Steering servo: center=1480μs, min=1080μs, max=1880μs
      • ESC: neutral=1500μs, max_fwd=2000μs, max_rev=1000μs
      • PCA9685: address=0x40, freq=50Hz
      • Steering channel=0, throttle channel=1
      • Camera: device=/dev/video1, type=usb, 640x480@30fps
      • I2C bus=3
      • INA219: address=0x42
      • Battery: min_voltage=6.4V, max_voltage=8.4V
  •  config/calibration.yaml : Steering-specific trim/calibration values

  Dependencies: None (data-only package). Other packages read these files via  ament_index_python.packages.get_package_share_directory('config') .
  </msg_received>
  <msg_received>
  <msg_id>msg-zHSl5aqXfp5yU82ERxqCC</msg_id>
  <msg_source>subagent</msg_source>
  <sender_name>Bisa package researcher</sender_name>
  <sender_id>0f83f40c-39b9-4381-a1e1-d75a331707e5</sender_id>

  Here is a comprehensive summary of the  bisa  package:
  ──────
  ##  bisa  Package — Main Autonomous Driving Package

  The  bisa  package is the brain of the D-Racer autonomous driving system. It contains perception (lane detection, object detection, traffic light classification),
  mission control (finite state machine), and vehicle control output. It also includes debug/tuning tools and a system telemetry node.

  ### Launch Files

  Located in  /home/hyun/D-Racer-Kit/src/bisa/launch/ :
  1.  onboard.launch.py  — All-on-vehicle launch: runs FULL self-driving stack on the car (no PC required). Launches camera, control, battery, system_telemetry,
  joystick, and  bisa_autonomous_node . Uses NCNN model with Vulkan GPU acceleration.
  2.  driving.launch.py  — PC-side compute launch: runs only the  bisa_autonomous_node  on the PC. Camera images come from the vehicle over WiFi. Uses  best.pt 
  model with CUDA GPU.
  3.  vehicle.launch.py  — Vehicle-side streaming launch: runs camera, control, battery, and joystick on the car. Streams camera to the PC over WiFi for  driving.
  launch.py  to process.
  4.  debug.launch.py  — Debug launch: runs visualization and parameter tuning GUI nodes on the PC for monitoring/tuning while the vehicle runs.

  ### Entry Points (Console Scripts)
  From  setup.py :

    'bisa_autonomous_node = bisa.autonomous_driving_node:main',
    'viz_node = bisa.viz_node:main',
    'power_gui_node = bisa.power_gui_node:main',
    'system_telemetry_node = bisa.system_telemetry_node:main',
    'param_gui_node = bisa.param_gui_node:main',
    'traffic_light_tuner = bisa.traffic_light:main',
    'dash_line_tuner = bisa.dash_line_tuner:main',
    ──────
  ### Source File Details

  #### 1.  autonomous_driving_node.py  — Core Autonomous Node

  The central ROS2 node ( BisaAutonomousNode ) that orchestrates perception, mission FSM, and control output.

  Architecture:

  • Uses  MultiThreadedExecutor  with 3 threads:
      • Thread 1 ( _cb_image ): Camera image callback → lane perception + frame handoff
      • Thread 2 ( _cb_control ): Control loop timer at 10Hz → FSM step + /control publish
      • Thread 3: Default callback group (ROS internals, parameter services)
  • A 4th daemon thread ( _inference_worker ): Runs YOLO inference asynchronously to avoid blocking the executor

  ROS2 Topics:

  • Subscribes:
      • Image topic (configurable, default  /camera/image/compressed ) —  sensor_msgs/CompressedImage 
  • Publishes:
      • Control topic (configurable, default  /control ) —  control_msgs/Control 
      •  /bisa/debug/image/compressed  —  sensor_msgs/CompressedImage  (debug overlay)
      •  /bisa/debug/lane_mask/compressed  —  sensor_msgs/CompressedImage  (lane mask debug)
      •  detect_green  —  std_msgs/Bool 
      •  detect_red  —  std_msgs/Bool 
      •  detect_sign  —  std_msgs/String  ("left=N right=M")
      •  detect_aruco  —  std_msgs/String  ("ids=[...]" or "none")


  Key Parameters:

  •  config_file ,  route_mode ,  model_path ,  image_topic ,  control_topic 
  •  debug_log ,  publish_debug_image ,  debug_image_topic ,  lane_mask_topic 
  • All  dracer_params.yaml  values are exposed as flat dotted ROS parameters for live tuning
  Internal Module Dependencies:
  •  lane_perception.LanePerception  — lane detection
  •  object_detector.BestPthDetector  +  DetectionBuffer  — YOLO inference
  •  aruco_detector.ArucoDetector  — ArUco marker detection
  •  mission_controller.LaneController  +  make_course_fsm  — FSM + steering/throttle
  •  traffic_light.classify_light  +  preprocess_frame  — traffic light color analysis
  •  visualization.draw_overlay  +  draw_lane_mask_view  — debug overlays
  •  dracer_config.load_config  — configuration loading
  ──────
  #### 2.  object_detector.py  — YOLO Detector Wrapper

  Wraps the Ultralytics YOLO model for object detection with confidence/ROI gating and temporal filtering.

  Key Classes:

  •  Detection  — dataclass for one detection result (cls, conf, bbox, cx, cy)
  •  DetectionBuffer  — rolling temporal buffer for vote-based decision making.  stable_seen(cls, k, n)  returns True when class appears in at least k of last n
  frames.
  •  BestPthDetector  — lazy-loads YOLO model, resolves device (CPU/CUDA/Vulkan), runs inference with rate limiting ( should_run() ), applies ROI gating (
  in_expected_roi() ), and maps model class names to mission names via  NAME_ALIASES .
  Device Resolution ( _resolve_device ):

  • Priority for 'auto': CUDA → Vulkan (if NCNN model) → CPU
  • Supports:  cpu ,  cuda ,  cuda:0 ,  vulkan ,  vulkan:0 ,  vulkan:N ,  gpu ,  auto 
   _is_ncnn_model() : Detects NCNN model directory by checking for  model.ncnn.param  file.
  ──────
  #### 3.  lane_perception.py  — Classical Lane Detection
  Implements the lane finding pipeline using classical computer vision (no ML).
  Key Class:  LanePerception 

  •  compute_lane_obs(frame, collect_viz)  → returns  LaneObs  dataclass
  • Pipeline:
      1. Optional ROI crop (from  LaneRoiConfig )
      2. LAB color space conversion + CLAHE on L channel
      3.  cv2.inRange  binarization using LAB thresholds → road mask
      4. Morphological open/close to clean the mask
      5. Connected component analysis for fork detection
      6. Hough line detection on Canny edges as fallback
      7. Computes  center_error  (normalized lane center offset, -1 to 1)
      8. Curvature estimation from line slopes


  Output:  LaneObs  dataclass:

  •  valid  (bool),  center_error  (float),  curvature  (float)
  •  fork_detected ,  fork_left_area ,  fork_right_area  — fork detection
  • Various visualization data stored in  last_viz 
  ──────
  #### 4.  mission_controller.py  — Mission FSM + Control Output
  Implements the course finite state machine and translates perception into steering/throttle commands.
  Key Classes:

  •  ControlCmd  — dataclass:  (throttle: float, steering: float) 
  •  LaneController  — Pure pursuit steering + throttle mapping. Takes  LaneObs  →  ControlCmd :
      • Pure pursuit geometry:  atan2(2*wheelbase*sin(alpha), lookahead)  normalized by  max_steer_deg 
      • Throttle: maps into  [speed_min, speed_max]  band with steer/curvature slowdown
      • Rate limiting on steering changes
  •  make_course_fsm(config, controller)  → returns a  CourseFSM  instance
  CourseFSM States (OUT course):
  1.  OUT_WAIT_GREEN  — Wait for green traffic light
  2.  OUT_LAUNCH  — Accelerate forward after green
  3.  OUT_S_CURVE  — Navigate S-curves
  4.  OUT_FORK_APPROACH  — Approach the fork intersection
  5.  OUT_FORK_COMMIT  — Commit to left/right based on sign vote
  6.  OUT_POST_FORK  — Drive after fork
  7.  OUT_FINISH  — Stop on red light at finish

  FSM Logic:
  •  step(lane, det_buffer, now_sec, light_state)  →  ControlCmd 
  • Uses  DetectionBuffer.stable_seen()  for sign voting (k out of n frames)
  •  light_confirm_frames  for traffic light confirmation (consecutive frames)
  •  finish_crossed  flag enables red-light detection only after  finish_min_elapsed_sec 
  ──────
  #### 5.  traffic_light.py  — Traffic Light Color Classification

  Classifies a YOLO-detected traffic light bounding box as red or green.

  Key Functions:

  •  classify_light(frame, bbox, config)  →  (verdict, scores) :
      • Splits box into vertical thirds (top=red zone, middle=amber ignored, bottom=green zone)
      • Three classifier implementations selectable via  config.traffic_light.classifier :
          •  "color"  — Pure HSV color ratio (default, field-proven)
          •  "lit"  — Brightness-gated (V≥220 + white core detection)
          •  "lab"  — LAB a-channel based (no hue wraparound, steadier)
      • Winner must exceed  row_min_ratio  threshold
  •  preprocess_frame(frame, cc_config)  → applies CLAHE + saturation boost + brightness/contrast/gamma
  •  apply_correction_chain(frame, cc_config)  → full color correction for display
  •  main()  — standalone traffic light tuner GUI with OpenCV sliders
  ──────
  #### 6.  aruco_detector.py  — ArUco Marker Detection

  Detects ArUco markers in camera frames for obstacle/checkpoint detection.
  Key Class:  ArucoDetector 
  •  detect(frame, now_sec)  → list of  ArucoMarker(id, corners, center) 
  • Rate-limited at  detect_hz  (default 10 Hz)
  • Uses OpenCV's  cv2.aruco  module with configurable dictionary (default  DICT_6X6_50 )
  • When the configured  target_id  marker (default 3) is visible, the autonomous node immediately stops the car
  ──────
  #### 7.  visualization.py  — Debug Overlay Drawing

  Draws detection overlays, lane visualization, and status text on camera frames.

  Key Functions:

  •  draw_overlay(frame, lane_viz, detections, markers, cmd, state, ...)  — draws everything:
      • Detection bounding boxes with class labels and confidence
      • Traffic light verdict colors (green/red based on classify_light, not YOLO class)
      • Lane center line and error indicator
      • ArUco marker outlines with IDs
      • FSM state text, steering/throttle gauges
      • ROI boundaries for light/sign detection
      • Distinguishes accepted vs. rejected (phase-gated) detections with different opacity
  •  draw_lane_mask_view(lane_viz, cmd)  — draws the binarized lane mask with overlays
  ──────
  #### 8.  viz_node.py  — Remote Visualization Node

  A lightweight ROS2 node that subscribes to the debug image topics and displays them in OpenCV windows on a PC.

  ROS2 Topics Subscribed:
  •  /bisa/debug/image/compressed  —  sensor_msgs/CompressedImage 
  •  /bisa/debug/lane_mask/compressed  —  sensor_msgs/CompressedImage 

  Shows two OpenCV windows: "BISA Detect View" and "BISA Lane Mask".
  ──────
  #### 9.  power_gui_node.py  — Power Management GUI
  A Tkinter-based GUI node for monitoring battery, system telemetry, and detection status on the PC.
  ROS2 Topics Subscribed:
  •  /battery/percentage  —  std_msgs/Float32 
  •  /battery/voltage  —  std_msgs/Float32 
  •  /system/cpu_usage  —  std_msgs/Float32 
  •  /system/cpu_temp  —  std_msgs/Float32 
  •  /system/memory_usage  —  std_msgs/Float32 
  •  detect_green  —  std_msgs/Bool 
  •  detect_red  —  std_msgs/Bool 
  •  detect_sign  —  std_msgs/String 
  •  detect_aruco  —  std_msgs/String 
  Displays gauges for battery %, voltage, CPU usage/temp, memory, and detection status indicators.
  ──────
  #### 10.  system_telemetry_node.py  — On-Car System Vitals
  Reads CPU usage, CPU temperature, and memory usage from  /proc  and  /sys  on the TOPST board and publishes them.

  ROS2 Topics Published:

  •  /system/cpu_usage  —  std_msgs/Float32  (%)
  •  /system/cpu_temp  —  std_msgs/Float32  (°C)
  •  /system/memory_usage  —  std_msgs/Float32  (%)
  Runs at configurable  publish_hz  (default 2 Hz). Dependency-free (reads  /proc/stat ,  /sys/class/thermal/thermal_zone*/temp ,  /proc/meminfo ).
  ──────
  #### 11.  param_gui_node.py  — Live Parameter Tuning GUI

  A Tkinter-based GUI that provides sliders for live-tuning all  dracer_params.yaml  parameters via ROS2 parameter services.

  How it works:

  • Connects to the  bisa_autonomous_node  via  ros2 param set 
  • Dynamically generates sliders for all flat dotted config values (lane thresholds, detector settings, steering gains, etc.)
  • Changes are applied in real-time to the running autonomous node
  ──────
  #### 12.  dash_line_tuner.py  — Lane Detection Tuner
  A standalone OpenCV-based GUI for tuning lane detection parameters on recorded or live images.
  • Creates OpenCV trackbars for all LAB threshold, morphology, and Hough parameters
  • Runs  LanePerception.compute_lane_obs()  in real-time on each frame
  • Displays the binary mask, Hough lines, and detected lane center
  • Can read from a saved image or subscribe to the camera topic
  ──────
  #### 13.  dracer_config.py  — Configuration System
  Defines all configuration dataclasses and YAML loading logic.
  Dataclasses (nested under  AutonomousConfig ):

  •  RoiConfig  — image ROI ratios for detector gating
  •  LaneRoiConfig  — pixel-space ROI for lane pipeline cropping
  •  LaneVisionConfig  — LAB thresholds, morphology, Hough params
  •  DetectorConfig  — model path, device, imgsz, inference_hz, confidence, voting
  •  ThrottleConfig  — speed band, per-state caps, ramp rate
  •  SteeringConfig  — pure pursuit geometry, per-state clamps
  •  ArucoConfig  — marker detection settings
  •  ColorCorrectionConfig  — CLAHE/saturation/brightness/contrast/gamma
  •  TrafficLightConfig  — red/green HSV/LAB classification thresholds
  •  MissionConfig  — route mode, timing, control_hz

  Key Functions:

  •  load_config(path)  — loads YAML and merges over dataclass defaults via  deep_update_dataclass() 
  •  resolve_package_relative_path(base, path)  — resolves  checkpoints/best.pt  relative to package root
  ──────
  #### 14.  config/dracer_params.yaml  — Main Configuration File

  YAML file with all tunable parameters. Read at node startup, values can be overridden via launch arguments or live via ROS2 parameter services.
  ──────
  ### Inter-Module Data Flow within bisa

    Camera frame (CompressedImage)
        │
        ├─→ image_callback (Thread 1)
        │     ├─→ lane_perception.compute_lane_obs() → LaneObs
        │     ├─→ aruco_detector.detect() → markers
        │     └─→ hand frame to _inference_worker
        │
        ├─→ _inference_worker (Thread 4, daemon)
        │     ├─→ traffic_light.preprocess_frame() → corrected frame
        │     ├─→ BestPthDetector.infer() → raw detections
        │     ├─→ _classify_and_gate() → light_state + FSM detections
        │     │     └─→ traffic_light.classify_light() per light box
        │     └─→ DetectionBuffer.push(gated detections)
        │
        └─→ control_loop (Thread 2, 10Hz timer)
              ├─→ CourseFSM.step(lane, det_buffer, light_state) → ControlCmd
              ├─→ publish_control(cmd) → /control topic
              ├─→ publish_detection_status() → detect_* topics
              └─→ publish_debug_overlay() → debug image topics

  </msg_received>

  3개의 리서치 에이전트가 모두 완료되었습니다. 종합해서 Artifact를 작성하겠습니다.
