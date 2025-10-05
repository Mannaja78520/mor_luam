# mor_luam

ESP32 firmware now closes the steering/drive loop locally to avoid Wi-Fi
latency and maintains wheel odometry internally. The board publishes
`nav_msgs/Odometry` on `/mor_luam/odom/esp`; a matching ROS fallback integrator
(`ros2 launch mor_luam fallback_odom_launch.py`) keeps tracking the robot if the
firmware unexpectedly reboots.

Publish a single `geometry_msgs/Twist` command to steer and run:

- `linear.x` – wheel speed in RPM (positive = forward)
- `angular.z` – steering target in degrees `[0, 360)`

Helper CLI examples:

```bash
ros2 run mor_luam send_heading_speed.py 90 120   # 90° heading @ 120 RPM
ros2 run mor_luam send_heading_speed.py 45 0.25 --unit mps
ros2 run mor_luam drive_to_xy.py 2 0 --speed 0.25 --unit mps  # drive 20 m forward at 1 m/s
```

Tune wheel PID gains from ROS by publishing to the configuration topics. Each
message expects `[Kp, Ki, Kd, Kf, error_tol, i_min, i_max, out_min, out_max]`
with trailing values optional:

```bash
ros2 topic pub /mor_luam/config/spin_pid std_msgs/msg/Float32MultiArray '{data: [4.0, 0.3, 0.02, 2.0, 1.5]}'
ros2 topic pub /mor_luam/config/steer_pid std_msgs/msg/Float32MultiArray '{data: [1.5, 0.2, 0.0, 0.0, 4.0, -1200, 1200]}'
```
