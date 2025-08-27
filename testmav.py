import time
from pymavlink import mavutil

BRIDGE_IP = "192.168.1.200"   # <-- your bridge IP
UDP_OUT   = f"udpout:{BRIDGE_IP}:14550"
UDP_IN    = "udpin:0.0.0.0:14550"

outc = mavutil.mavlink_connection(UDP_OUT, source_system=255, source_component=190)
inc  = mavutil.mavlink_connection(UDP_IN,  source_system=255, source_component=191)

print("Sending HEARTBEATs to bridge and listening for echoed frames...")
while True:
    # send one HEARTBEAT
    outc.mav.heartbeat_send(
        type=mavutil.mavlink.MAV_TYPE_GCS,
        autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        base_mode=0, custom_mode=0,
        system_status=mavutil.mavlink.MAV_STATE_ACTIVE
    )
    # try to read anything that came back from the UART
    msg = inc.recv_match(blocking=False)
    if msg:
        print("RX:", msg)
    time.sleep(1.0)
