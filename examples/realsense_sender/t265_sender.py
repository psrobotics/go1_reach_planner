# realsense reading test
import sys
sys.path.append('../../')


import pyrealsense2 as rs
import numpy as np
import math as m

# lcm init
import lcm
from examples.lcm_type.exlcm import motion_t


# quaternion to rpy
def quaternion_to_rpy(quaternion):
    # Normalize the quaternion to ensure unit length
    quaternion /= np.linalg.norm(quaternion)
    # Extract individual components
    w, x, y, z = quaternion
    #w, y, x, z = quaternion
    # Compute roll (x-axis rotation)
    roll_x = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    # Compute pitch (y-axis rotation)
    sin_pitch = 2 * (w * y - z * x)
    # Ensure sin_pitch stays within valid range to avoid numerical issues
    sin_pitch = np.clip(sin_pitch, -1.0, 1.0)
    pitch_y = np.arcsin(sin_pitch)
    # Compute yaw (z-axis rotation)
    yaw_z = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    # Convert yaw to the range of -π to π
    yaw_z = np.arctan2(np.sin(yaw_z), np.cos(yaw_z))

    return roll_x, pitch_y, yaw_z

# relasense frame to state
def pose_to_state(pose):
    # pull out data
    t265_data = pose.get_pose_data()
    t265_frame_id = pose.frame_number
    t265_pos = t265_data.translation
    t265_quat = t265_data.rotation
    #t265_vel = t265_data.velocity
    #t265_acc = t265_data.acceleration
        
    # fix coordinate dir based on mounting dir
    x = -1 * t265_pos.x
    y = 1 * t265_pos.z
    z = 1 * t265_pos.y
    # get yaw angle
    q_w = t265_quat.w
    q_x = t265_quat.x
    q_y = t265_quat.y
    q_z = t265_quat.z
    quat = np.array([q_w, q_x, q_y, q_z])

    roll, pitch, yaw = quaternion_to_rpy(quat)
    # get continous yaw based on roll sign (untested)
    if roll<0:
        if pitch>0 and pitch<m.pi/2:
            pitch = m.pi-pitch
        else:
            pitch = -m.pi-pitch
    # pitch is yaw
    state_n = np.array([x, y, pitch])

    # send lcm message
    msg = motion_t()
    msg.timestamp = 0
    msg.position = (-1 * t265_pos.x, 1 * t265_pos.z, 1 * t265_pos.y)
    msg.orientation = (q_w, q_x, q_y, q_z)

    msg.name = "t265 pose"
    msg.enabled = True

    lc.publish("T265_LCM", msg.encode())

    return state_n


# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

# Start streaming with requested config
pipe.start(cfg)

# init state array, 3d state, x y yaw
t265_state = np.zeros(3)

lc = lcm.LCM()
print("LCM init done! \n")


try:
    while True:
        # wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()
        # fetch pose frame
        pose = frames.get_pose_frame()

        # pose to robot state arr, x, y, yaw in mounting dir
        if pose:
            t265_state = pose_to_state(pose)
  
        print("X: {0:.3f} Y: {1:.3f} Yaw: {2:.3f}\n".format(t265_state[0], t265_state[1], t265_state[2]))

finally:
    pipe.stop()

