
from pymavlink import mavutil
import socket, json, time

mav = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
mav.wait_heartbeat()
print("Leader MAVLink connected")

UDP_IP = "192.168.4.255"
UDP_PORT = 14550
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

while True:
    msg = mav.recv_match(type=['GLOBAL_POSITION_INT', 'ATTITUDE'], blocking=True)

    if msg.get_type() == 'GLOBAL_POSITION_INT':
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.relative_alt / 1000.0

    if msg.get_type() == 'ATTITUDE':
        yaw = msg.yaw
        data = {
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "yaw": yaw,
            "time": time.time()
        }
        sock.sendto(json.dumps(data).encode(), (UDP_IP, UDP_PORT))
        time.sleep(0.1)
