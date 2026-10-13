# Low-level command for the quadrotor: collective thrust and body angular rates.
# Same split as PX4's VehicleRatesSetpoint, but in SI units and the FLU body frame.
std_msgs/Header header
float64 thrust                      # collective thrust [N], along body +z
geometry_msgs/Vector3 body_rates    # desired angular velocity [rad/s], body frame (x fwd, y left, z up)
