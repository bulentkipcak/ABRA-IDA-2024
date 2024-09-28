"""
AbraIda Class for MAVLink Vehicle Control
Author: Bülent Kıpçak
Date: 2024-08-15
Description: This class provides methods for connecting to a MAVLink-compatible vehicle,
retrieving its current state, setting yaw and velocity targets, and handling JSON files
for vehicle mode and state information.
"""
import math
import json
import os
from pymavlink import mavutil

class AbraIda:

    def __init__(self, connection_string):
        self.master = mavutil.mavlink_connection(connection_string)
        self.master.wait_heartbeat()
        print("Araca bağlandı.")
    
    def get_current_vehicle_mode(self):
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True)
        if msg:
            vehicle_mode = mavutil.mode_string_v10(msg)
            print(f"Araç modu: {vehicle_mode}")
            return vehicle_mode
        else:
            print("Araç modu alınamadı!")
            return None
    
    def get_current_yaw(self):
        msg = self.master.recv_match(type='ATTITUDE', blocking=True)
        yaw_radians = msg.yaw
        yaw_degrees = math.degrees(yaw_radians)
        return yaw_degrees

    def set_yaw_target(self, yaw_angle, system_id=1, component_id=1):
        yaw_radians = math.radians(yaw_angle)
        msg = self.master.mav.set_position_target_local_ned_encode(
            0,  # time_boot_ms
            system_id,  # target_system
            component_id,  # target_component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # coordinate_frame
            0b100111111111,  # type_mask
            0, 0, 0,  # x, y, z
            0, 0, 0,  # vx, vy, vz
            0, 0, 0,  # afx, afy, afz
            yaw_radians,
            0  # Yaw velocity
        )
        self.master.mav.send(msg)
        print(f"Yaw açısı {yaw_angle} derece olarak ayarlandı")

    def set_velocity_target(self, forward_velocity, system_id=1, component_id=1):
        msg = self.master.mav.set_position_target_local_ned_encode(
            0,  # time_boot_ms
            system_id,  # target_system
            component_id,  # target_component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # coordinate_frame
            0b110111100111,  # type_mask
            0, 0, 0,  # x, y, z
            forward_velocity, 0, 0,  # vx, vy, vz
            0, 0, 0,  # afx, afy, afz
            0,  # Yaw degree
            0  # Yaw velocity
        )
        self.master.mav.send(msg)
        print(f"İleri hız {forward_velocity} m/s olarak ayarlandı")

    def read_error_yaw_from_json(self, filename='error_angle.json'):
        try:
            if os.path.getsize(filename) == 0:
                raise ValueError("JSON dosyası boş!")
            with open(filename, 'r') as file:
                data = json.load(file)
                return data['error_yaw']
        except ValueError as e:
            print(e)
            return None

    def update_json_mode(self, mode, filename='vehicle_mode.json'):
        data = {'vehicle_mode': mode}
        with open(filename, 'w') as f:
            json.dump(data, f, indent=4)

    def read_vehicle_state(self, filename='vehicle_state.json'):
        try:
            if os.path.getsize(filename) == 0:
                raise ValueError("JSON dosyası boş!")
            with open(filename, 'r') as file:
                data = json.load(file)
                return data['vehicle_state']
        except ValueError as e:
            print(e)
            return None
