# Local velocity setpoint
# Used to send setpoint to offboard controller
# Designed for the LIS-EPFL DroneCourse
# author Basil Huber LIS <basil.huber@gmail.com>

float32 vx			# in meters/sec
float32 vy			# in meters/sec
float32 vz			# in meters/sec
float32 yaw			# in radians NED -PI..+PI
bool 	yaw_valid	# indicates if yaw is valid