uint64 timestamp        # [us] Time since system start (microseconds) [2](@ref)

# ================== Basic Device Information ==================
uint8 frame             # [@enum FRAME_TYPE] Protocol frame type identifier (e.g., 0x00=AnchorFrame0, 0x01=TagFrame0)
uint8 role              # [@enum ROLE] Device role (e.g., anchor, tag, node)
uint8 id                # Unique device identifier ID
uint32 local_time       # [us] Device local timestamp (microseconds)
uint32 system_time      # [us] System global timestamp (microseconds)
float32 voltage         # [V] [@invalid NaN] Power supply voltage (Volts)

# ================== Distance Measurement Data ==================
float32[8] dis_arr      # [m] [@invalid NaN] Array of distance measurements (meters)

# ================== Position and Attitude Data ==================
float32[3] pos_3d       # [m] [@invalid NaN] 3D position coordinates (x, y, z, meters)
float32[3] eop_3d       # [m] [@invalid NaN] Estimated Position Error (meters)
float32[3] vel_3d       # [m/s] [@invalid NaN] 3D velocity vector (vx, vy, vz, meters/second)

# ================== Attitude Sensor Data ==================
float32[3] angle_3d     # [rad] [@invalid NaN] 3D Euler angles (roll, pitch, yaw, radians)
float32[4] quaternion   # [@invalid NaN] Quaternion attitude representation (w, x, y, z)

# ================== IMU Sensor Data ==================
float32[3] imu_gyro_3d  # [rad/s] [@invalid NaN] Gyroscope angular velocity (radians/second)
float32[3] imu_acc_3d   # [m/s^2] [@invalid NaN] Accelerometer data (meters/second^2)
