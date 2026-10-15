# Reference for the controllers: sample k is the reference at header.stamp + k * dt.
# World frame (ENU). Controllers interpolate it at their own times, so the publisher
# can run slower than they do. A hover setpoint is a trajectory with constant samples.
std_msgs/Header header
float64 dt
geometry_msgs/Point[] position
geometry_msgs/Vector3[] velocity
geometry_msgs/Vector3[] acceleration
float64[] yaw
