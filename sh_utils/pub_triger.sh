# sh_utils/pub_triger.sh
#!/bin/bash
ros2 topic pub /pub_triger geometry_msgs/msg/PoseStamped \
"{header:
  {seq: 0, stamp: {sec: 0, nanosec: 0},
  frame_id: ''},
pose:
  {position: {x: 0.0, y: 0.0, z: 0.0},
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}"
