
from pymavlink import mavutil
import socket, json, math, time

mav = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
mav.wait_heartbeat()
print("Follower MAVLink connected")

UDP_IP = "0.0.0.0"
UDP_PORT = 14550
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

OFFSET_BACK = -5.0
OFFSET_RIGHT = 3.0

def offset_gps(lat, lon, yaw, x, y):
    R = 6378137.0
    dx = x * math.cos(yaw) - y * math.sin(yaw)
    dy = x * math.sin(yaw) + y * math.cos(yaw)
    dlat = dy / R
    dlon = dx / (R * math.cos(math.pi * lat / 180))
    return lat + dlat * 180 / math.pi, lon + dlon * 180 / math.pi

mav.set_mode_apm('GUIDED')

while True:
    data, _ = sock.recvfrom(1024)
    leader = json.loads(data.decode())

    tgt_lat, tgt_lon = offset_gps(
        leader["lat"], leader["lon"], leader["yaw"],
        OFFSET_BACK, OFFSET_RIGHT
    )

    mav.mav.set_position_target_global_int_send(
        0, mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b110111111000,
        int(tgt_lat * 1e7), int(tgt_lon * 1e7), leader["alt"],
        0,0,0, 0,0,0, 0,0
    )
    time.sleep(0.2)
