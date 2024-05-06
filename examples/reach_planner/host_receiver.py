# lcm init
import sys
sys.path.append('../')

import lcm
from lcm_type.exlcm import motion_t

# socket init
from socket import socket, AF_INET, SOCK_DGRAM

# lcm handle function
def my_handler(channel, data):
    msg = motion_t.decode(data)
    print("Received message on channel \"%s\"" % channel)
    print("   timestamp   = %s" % str(msg.timestamp))
    print("   latency   = %s" % str(msg.latency))
    print("   position    = %s" % str(msg.position))
    print("   orientation = %s" % str(msg.orientation))
    print("   rb_name        = '%s'" % msg.rb_name)
    print("   enabled     = %s" % str(msg.enabled))
    print("")

lc = lcm.LCM()
subscription = lc.subscribe("VICON_LCM", my_handler)
print("lcm init done!")

socket_server = socket(AF_INET, SOCK_DGRAM)
ip_tmp = ('192.168.1.201', 9999)
socket_server.bind(ip_tmp)
print("socket init done!")


try:
    while True:
        # check lcm call back in loop
        lc.handle()
except KeyboardInterrupt:
    pass