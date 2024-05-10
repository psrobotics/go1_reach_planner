import sys
sys.path.append('../../')

#import sys
#sys.path.extend(['/path/to/subfolder1', '/path/to/subfolder2'])
#from package1 import module1
#from package2 import module2

import socket
import numpy as np
import struct
import math
import time

# lcm init
import lcm
from examples.lcm_type.exlcm import motion_t

import scipy.io
import h5py
import logging

import threading


def low_level_callback():
    
    print (time.ctime())
    # send to socket
    low_command = np.array([cmd_x_vel, cmd_y_vel, cmd_yaw_vel,
                            cmd_height, cmd_freq, cmd_phase, cmd_offset, cmd_bound, cmd_duration, cmd_footswing,
                            cmd_ori_pitch, cmd_ori_roll, cmd_stance_width, cmd_stance_length], dtype=np.float64)
    #print('low level command - {}'.format(low_command))
    # Pack the NumPy array using tobytes()
    low_command_bytes = low_command.tobytes()
    send_udp_message(low_command_bytes, ip, port)

    # save to csv, logger
    csv_row = "{0:d},{1:.6f},{2:.6f},{3:.6f},{4:.6f},{5:.6f},{6:.6f},{7:.6f}".format(global_opti_mode,cmd_yaw_vel,
                                                                                     global_xyz[0],global_xyz[1],global_xyz[2],
                                                                                     global_rpy[0],global_rpy[1],global_rpy[2])
    lgr.info(csv_row)
    
    # 100hz callback 0.025
    threading.Timer(0.025, low_level_callback).start()


# Send a UDP message to the specified IP address and port, low-level controller
def send_udp_message(message, ip, port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(message, (ip, port))

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
    
# lcm handle function, update state variables, get the log here
def lcm_handler(channel, data):

    msg = motion_t.decode(data)
    print("   Received message on channel \"%s\"" % channel)
    print("   timestamp   = %s" % str(msg.timestamp))
    print("   latency   = %s" % str(msg.latency))
    print("   position    = %s" % str(msg.position))
    print("   orientation = %s" % str(msg.orientation))
    print("   rb_name        = '%s'" % msg.rb_name)
    print("   enabled     = %s" % str(msg.enabled))
    print("")

    [roll, pitch, yaw] = quaternion_to_rpy(msg.orientation)
    print("roll - {0:.5f}, pitch - {1:.5f}, yaw - {2:.5f} \n".format(roll, pitch, yaw))

    # we use global variable here
    rbt_state[0] = 1*msg.position[0] + state_offset[0] 
    rbt_state[1] = 1*msg.position[1] + state_offset[1] 

    rbt_state[2] = (yaw+state_offset[2] + math.pi) % (2 * math.pi) - math.pi

    # update global variables for logging, some overlaid with offset
    global_xyz[0] = rbt_state[0]
    global_xyz[1] = rbt_state[1]
    global_xyz[2] = msg.position[2]
    global_rpy[0] = roll
    global_rpy[1] = pitch
    global_rpy[2] = rbt_state[2]

    print(rbt_state[2])

# setup lcm

lc = lcm.LCM()
subscription = lc.subscribe("VICON_LCM", lcm_handler)
print("lcm init done!")


# setup socket
ip = "192.168.123.15"  # destination IP of the jetson NX onboard
port = 8088  # replace with the destination port number
print("low level wtw socket init done!")

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

# mat states
rbt_state = np.zeros(3)
# state offset, x y yaw
state_offset = np.array([4.00-0.2, 4.00+0.4, 0.992])
# state index
state_index_f = np.array([0, 0, 0])
# target set thres, set to 0.7
tar_dist_thes = 0.2

# xyz, rpy
global_xyz = np.zeros(3)
global_rpy = np.zeros(3)

global_opti_mode = 1
global_opti_ctr = 0.0

# array for different gait mode setups
# turning rate k, we have 
omega_k_arr  = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 0.7, 0.6, 0.2])
# forward velocity k
x_vel_k_arr  = np.array([1.0, 0.8, 0.6, 0.4, 0.2, 1.0, 0.5, 3.0])
# body gait state
height_arr   = np.array([0.12, 0.12, 0.12, 0.12, 0.12, 0.12, -0.30, 0.3])
pitch_arr    = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
roll_arr     = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

phase_arr    = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.0])
offset_arr   = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
bound_arr    = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
duration_arr = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
freq_arr     = np.array([3.0, 3.0, 3.0, 3.0, 3.0, 2.7, 3.0, 2.7])

# low pass filter parameters
x_vel_old = 0.0
yaw_vel_old = 0.0
body_h_old = 0.12

# csv logger init
csv_path = '../../log_tmp/traj_record_1_rl.csv'
# logger init
# create logger
lgr = logging.getLogger('quad_logger')
lgr.setLevel(logging.DEBUG) # log all escalated at and above DEBUG
# add a file handler
fh = logging.FileHandler(csv_path)
fh.setLevel(logging.DEBUG) # ensure all messages are logged to file
# create a formatter and set the formatter for the handler.
frmt = logging.Formatter('%(asctime)s,%(name)s,%(levelname)s,%(message)s')
fh.setFormatter(frmt)
# add the Handler to the logger
lgr.addHandler(fh)
# start init indector
lgr.critical('New Traj Sample Start From Here')


# .mat file setup
mat_file_path = "../../../mat_file/quad_reachavoid_j_2.mat"
print('reading mat file')
with h5py.File(mat_file_path, 'r') as file:
    # list all variable keys
    print(list(file.keys()))
    # load variables
    data0 = np.array(file['tar_fcn']).astype(np.float64) # target set
    print('data0 read done')
    data_ctr = np.array(file['data_ctr']).astype(np.float64)
    print('data_ctr read done')
    data_f = np.array(file['data_f']).astype(np.float64) # value function
    print('data_f read done')
    data_op_mode = np.array(file['data_op_mode']).astype(np.int8)
    print('data_op_mode read done')

    grid = np.array(file['grid'])[0]
    grid_dim = np.array(file['grid_dim'])[0]
    grid_max = np.array(file['grid_max'])[0]
    grid_min = np.array(file['grid_min'])[0]
    grid_n = np.array(file['grid_n'])[0]
            
    tau = np.array(file['tau'])[0]
    tau_length = np.array(file['tau_length'])[0]

    print('.mat data read done\n')
    # print out basic grid info
    print('{} d system'.format(grid_dim))
    print('grid max - {}'.format(grid_max))
    print('grid min - {}'.format(grid_min))
    print('grid n - {}'.format(grid_n))
    print('with {} t slices\n'.format(tau_length))

    # prepare grid 
    grid_d = (grid_max - grid_min) / grid_n

    # timer callback to interact with low-level quadruped robot
    low_level_callback()

    lc_counter = 0


# main loop
try:
    while True:
        # check lcm call back in loop, update states

        lc.handle()

        # get state index
        state_index_f = (rbt_state - grid_min) / grid_d
        state_index_i = state_index_f.astype(np.int32)
        print("X_i: {0:d} Y_i: {1:d} Yaw_i: {2:d}".format(state_index_i[0], state_index_i[1], state_index_i[2]))
        
        # bound index i
        for s in range(0, 3, 1):
            if state_index_i[s]>grid_n[s]-1:
                state_index_i[s]=grid_n[s]-1
            elif state_index_i[s]<0:
                state_index_i[s]=0

        # get time index (earliset brs)
        _i_t_max = tau_length.astype(np.int32) - 1
        _i_t = None
    
        _upper = _i_t_max[0]
        _lower = 0

        while _upper>_lower:
            #print("u - {0:.3f}, l - {0:.3f}".format(_upper,_lower))
            _t_e = math.ceil((_upper+_lower)/2)
            _i_t = _t_e
            v_dist_t = data_f[_i_t, state_index_i[2], state_index_i[1], state_index_i[0]]
            if (v_dist_t > -0.001): # 0.001
                _lower = _i_t
            else:
                _upper = _i_t-1
        #print("upper - {0:d}, lower - {1:d}".format(_upper, _lower))
        _i_t = _upper

        # get opti mode, opti ctr
        # opti ctr, yaw velocity in rad/s
        opti_ctr = data_ctr[_i_t, state_index_i[2], state_index_i[1], state_index_i[0]]
        opti_mode = data_op_mode[_i_t, state_index_i[2], state_index_i[1], state_index_i[0]]
        opti_mode = opti_mode.astype(np.int32)
        print("opti_ctr - {0:.3f}, opti_mode - {1:d}".format(opti_ctr, opti_mode))

        # for csv logging
        global_opti_mode = opti_mode
        global_opti_ctr = opti_ctr

        # set walking mode and paras based on opti mode
        if opti_mode == 0:
            opti_mode = 1

        x_vel_n = x_vel_k_arr[opti_mode-1]*1.0
        yaw_vel_n = omega_k_arr[opti_mode-1]*opti_ctr
        body_h_n = height_arr[opti_mode-1]

        cmd_x_vel = (0.5*x_vel_old + 0.5*x_vel_n)*1.0
        cmd_yaw_vel = (0.7*yaw_vel_old + 0.3*yaw_vel_n)*0.95 # 0.6 is the k for heading angle
        cmd_height = 0.5*body_h_old + 0.5*body_h_n
        
        cmd_ori_pitch = pitch_arr[opti_mode-1]
        cmd_ori_roll = roll_arr[opti_mode-1]

        cmd_phase = phase_arr[opti_mode-1]
        cmd_offset = offset_arr[opti_mode-1]
        cmd_bound = bound_arr[opti_mode-1]
        cmd_duration = duration_arr[opti_mode-1]
        cmd_freq = freq_arr[opti_mode-1]

        x_vel_old = x_vel_n
        yaw_vel_old = yaw_vel_n
        body_h_old = body_h_n

        
        # check if reach the target set
        v_dist_tar = data0[state_index_i[2], state_index_i[1], state_index_i[0]]
        if v_dist_tar<tar_dist_thes:
            print('reach target set!\n')
            # set command velocity to 0 
            cmd_x_vel = 0.0
            cmd_yaw_vel = 0.0

        # ~200hz in loop
        #time.sleep(0.05)
        


except KeyboardInterrupt:
    pass
