rostopic pub /triger geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0"



# #!/usr/bin/env bash
# # publish 10 Hz PoseStamped on /triger with valid header & quaternion

# while true; do
#   NOW_SECS=$(date +%s)
#   rostopic pub -r 10 /triger geometry_msgs/PoseStamped \
#     "{header: {stamp: {secs: ${NOW_SECS}, nsecs: 0}, frame_id: \"map\"}, \
#       pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \
#              orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
#   sleep 0.1
# done
