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
from examples.lcm_type.exlcm import twist


import scipy.io
import h5py
import logging

import threading



def low_level_callback():

    print (time.ctime())

    low_command = np.array([cmd_x_vel, cmd_y_vel, cmd_yaw_vel,
                            cmd_height, cmd_freq, cmd_phase, cmd_offset, cmd_bound, cmd_duration, cmd_footswing,
                            cmd_ori_pitch, cmd_ori_roll, cmd_stance_width, cmd_stance_length], dtype=np.float64)
    #print('low level command - {}'.format(low_command))
    # Pack the NumPy array using tobytes()
    low_command_bytes = low_command.tobytes()
    send_udp_message(low_command_bytes, ip, port)

    # save to csv, logger
    csv_row = "{0:.6f},{1:.6f}".format(cmd_x_vel, cmd_yaw_vel)

    lgr.info(csv_row)
    #print("low level command sent\n")
    
    # 100hz callback 0.025 *0.050
    threading.Timer(0.050, low_level_callback).start()

# Send a UDP message to the specified IP address and port, low-level controller
def send_udp_message(message, ip, port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(message, (ip, port))


    
# lcm handle function, update state variables, get the log here
def lcm_handler(channel, data):
    global cmd_x_vel, cmd_yaw_vel

    msg = twist.decode(data)
    print("   Received message on channel \"%s\"" % channel)
    print("   timestamp   = %s" % str(msg.timestamp))
    print("   vx   = %s" % str(msg.v_x))
    print("   vyaw    = %s" % str(msg.v_yaw))
    print("")

    # set simple twist velocity
    #cmd.velocity = [1.1, 0.0]
    #cmd.yawSpeed = 9.0*msg.v_yaw
    if msg.v_x is not None and msg.v_yaw is not None:
        cmd_x_vel = msg.v_x
        cmd_yaw_vel = msg.v_yaw



# setup lcm
lc = lcm.LCM()
subscription = lc.subscribe("TWIST_PLANNER", lcm_handler)
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
cmd_stance_width = 0.25 # fpp width 0.33
cmd_stance_length = 0.40 # fpp len



# csv logger init
csv_path = '../../log_tmp/traj_record_1_rrt_rl.csv'
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



# timer callback to interact with low-level quadruped robot
low_level_callback()

# main loop
try:
    while True:
        # check lcm call back in loop, update states
        lc.handle()

except KeyboardInterrupt:
    pass
