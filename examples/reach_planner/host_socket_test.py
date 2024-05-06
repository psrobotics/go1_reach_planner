import socket
import numpy as np
import struct
import math
import time


def send_udp_message(message, ip, port):
    """
    Send a UDP message to the specified IP address and port.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(message, (ip, port))


# Example usage
ip = "192.168.123.15"  # destination IP of the jetson NX onboard
port = 8088  # replace with the destination port number

# command params, some with default trotting gait
cmd_x_vel = 0.0
cmd_y_vel = 0.0
cmd_yaw_vel = 0.0
cmd_height = 0.0 # -0.3~0.3, with 0 as normal height
cmd_freq = 2.0 # 2.0~4.0
cmd_phase = 0.5 # gait phase
cmd_offset = 0.0 # gait offset
cmd_bound = 0.0
cmd_duration = 0.5
cmd_footswing = 0.08 # foot swing height max(0,adj)*0.32+0.03
cmd_ori_pitch = 0.0 # -0.4~0.4
cmd_ori_roll = 0.0
cmd_stance_width = 0.33 # fpp width
cmd_stance_length = 0.40 # fpp len

t_s = 0

while True:
    # send low-level command
    t_s = t_s+0.003
    cmd_freq = 3.0+math.sin(t_s)
    cmd_yaw_vel = 0.5*math.sin(7*t_s)

    low_command = np.array([cmd_x_vel, cmd_y_vel, cmd_yaw_vel,
                            cmd_height, cmd_freq, cmd_phase, cmd_offset, cmd_bound, cmd_duration, cmd_footswing,
                            cmd_ori_pitch, cmd_ori_roll, cmd_stance_width, cmd_stance_length], dtype=np.float64)
    print('low level command - {}'.format(low_command))
    # Pack the NumPy array using tobytes()
    low_command_bytes = low_command.tobytes()
    send_udp_message(low_command_bytes, ip, port)

    # we need to keep loop within ~200hz frequency
    time.sleep(0.05)