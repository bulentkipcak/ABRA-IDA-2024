"""
Author: Bülent Kıpçak
Email: bulentkipcak04@gmail.com
Date: 2024-08-15
Description: This script captures video from a specified RTSP stream and detects colored buoys (red, yellow, green) in the frame.
It calculates distances and angles between buoys and updates the vehicle's state in a JSON file.
"""
import cv2
import numpy as np
import math
import json
import os
import time

# cap = cv2.VideoCapture("rtsp://192.168.31.60:8554/baseaxipcie120000rp1i2c88000imx7081a")
cap = cv2.VideoCapture("rtsp://192.168.31.181:8554/baseaxipcie120000rp1i2c88000imx7081a latency=0 is-live=True ! queue ! decodebin ! autovideosink", cv2.CAP_GSTREAMER)

class Duba:
    def __init__(self, color_name, lower_bound, upper_bound, color_bgr,min_y):
        self.color_name = color_name
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.color_bgr = color_bgr  
        self.objects = []
        self.min_y = min_y

    def detect_objects(self, frame, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.objects = []  
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  
                x, y, w, h = cv2.boundingRect(contour)
                cx = x + w // 2
                cy = y + h // 2
                if y >= self.min_y:
                    self.objects.append((x, y, w, h, cx, cy, area))  

        
        self.objects.sort(key=lambda obj: obj[5], reverse=True)  

    def draw_objects(self, frame):
        for idx, (x, y, w, h, cx, cy, area) in enumerate(self.objects, start=1):
            cv2.rectangle(frame, (x, y), (x + w, y + h), self.color_bgr, 2)
            cv2.putText(frame, f"ID: {idx}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color_bgr, 1)
            cv2.putText(frame, f"{self.color_name}", (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color_bgr, 1)
            cv2.putText(frame, f"({cx},{cy},{int(area)})", (x, y - 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color_bgr, 1)
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

def draw_reference_point(frame):
    height, width = frame.shape[:2]
    center_x, center_y = width // 2, height  

    
    cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)
    return (int(center_x), int(center_y))  


def calculate_distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def draw_lines_between_dubas(frame, red_duba, yellow_duba, green_duba):
    lines = []
    
    red_duba_coords = red_duba.objects[0] if red_duba.objects else None
    yellow_duba_coords = yellow_duba.objects[0] if yellow_duba.objects else None
    green_duba_coords = green_duba.objects[0] if green_duba.objects else None

    if yellow_duba_coords:
        if red_duba_coords:
            start_point = (red_duba_coords[4], red_duba_coords[5])
            end_point = (yellow_duba_coords[4], yellow_duba_coords[5])
            #!DERİNLİK SORUNU!!!!       
            # max_y = max(start_point[1], end_point[1])
            # start_point = (red_duba_coords[4],max_y)
            # end_point= (yellow_duba_coords[4],max_y)
            lines.append((start_point, end_point))
            cv2.line(frame, start_point, end_point, (0, 127, 255), 2)
      

        
        if green_duba_coords:
            start_point = (yellow_duba_coords[4], yellow_duba_coords[5])
            end_point = (green_duba_coords[4], green_duba_coords[5])
            # #!DERİNLİK SORUNU!!!!
            # max_y = max(start_point[1], end_point[1])
            # start_point = (yellow_duba_coords[4],max_y)
            # end_point= (green_duba_coords[4],max_y)
            lines.append((start_point, end_point))
            cv2.line(frame, start_point, end_point, (0, 127, 255), 2)
    
    if red_duba_coords and green_duba_coords:
            start_point = (red_duba_coords[4], red_duba_coords[5])
            end_point = (green_duba_coords[4], green_duba_coords[5])
            # #!DERİNLİK SORUNU!!!!
            # max_y = max(start_point[1], end_point[1])
            # start_point = (red_duba_coords[4],max_y)
            # end_point= (green_duba_coords[4],max_y)
            lines.append((start_point, end_point))
            cv2.line(frame, start_point, end_point, (0, 127, 255), 2)

    if lines:
        longest_line = max(lines, key=lambda line: calculate_distance(line[0], line[1]))
        shortest_line = min(lines, key=lambda line: calculate_distance(line[0], line[1]))

        start_point, end_point = longest_line
        distance = calculate_distance(start_point, end_point)
        longest_midpoint = ((start_point[0] + end_point[0]) // 2, (start_point[1] + end_point[1]) // 2)
        cv2.circle(frame, longest_midpoint, 5, (255, 0, 0), -1)
        ref_point = draw_reference_point(frame)
        cv2.line(frame, ref_point, longest_midpoint, (255, 0, 0), 2)
        cv2.putText(frame, f"{distance:.2f}px", (longest_midpoint[0] - 60, longest_midpoint[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        
        start_point, end_point = shortest_line
        distance = calculate_distance(start_point, end_point)
        shortest_midpoint = ((start_point[0] + end_point[0]) // 2, (start_point[1] + end_point[1]) // 2)
        cv2.circle(frame, shortest_midpoint, 5, (0, 255, 0), -1)
        cv2.line(frame, start_point, end_point, (0, 255, 0), 2)
        cv2.putText(frame, f"{distance:.2f}px", (shortest_midpoint[0] - 60, shortest_midpoint[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        height, width = frame.shape[:2]
        x_axis_start = (longest_midpoint[0], longest_midpoint[1])
        x_axis_end = (int(width/2), longest_midpoint[1])
        cv2.line(frame, x_axis_start, x_axis_end, (255, 255, 0), 2)

        ref_x = ref_point[0]
        ref_y = ref_point[1]
        height = frame.shape[0]
        y_axis_parallel_start = (ref_x, longest_midpoint[1])
        y_axis_parallel_end = (ref_x, height)
        cv2.line(frame, y_axis_parallel_start, y_axis_parallel_end, (255, 255, 0), 2)

        angle = calculate_angle((ref_x,ref_y),(longest_midpoint[0],longest_midpoint[1]), (ref_x,longest_midpoint[1]))
        cv2.putText(frame, f'{angle:.2f} degrees', (int(ref_x) + 30, int(ref_point[1]) - 20 ), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        update_json_angle(angle)



def calculate_angle(p1, p2, p3):
    v1 = np.array(p2) - np.array(p1)
    v2 = np.array(p3) - np.array(p1)
    angle = np.arctan2(v2[1], v2[0]) - np.arctan2(v1[1], v1[0])
    return -np.degrees(angle)


def update_json_angle(angle):
    data = {'target_yaw': angle}
    with open('target_angle.json', 'w') as f:
        json.dump(data, f, indent=4)
    

def read_vehicle_mode_from_json(filename='vehicle_mode.json'):
    try:
        if os.path.getsize(filename) == 0:
            raise ValueError("JSON dosyası boş!")
            
        with open(filename, 'r') as file:
            data = json.load(file)
            return data['vehicle_mode']
    except ValueError as e:
        print(e)
        return None  
    
def draw_vehicle_mode(frame):
    vehicle_mode= None
    while vehicle_mode is None:
        vehicle_mode = read_vehicle_mode_from_json()
        if vehicle_mode is None:
            print("JSON verisi bekleniyor...")
            time.sleep(0.1)  
        else:
            cv2.putText(frame, f'Arac Modu: {vehicle_mode}', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

def determine_vehicle_state(red_duba, yellow_duba, green_duba):
    if red_duba.objects and not green_duba.objects:
        return "KIRMIZI DUBA"
    elif green_duba.objects and not red_duba.objects:
        return "YESIL DUBA"
    elif yellow_duba.objects and not red_duba.objects and not green_duba.objects:
        return "SARI DUBA"
    elif not red_duba.objects and not yellow_duba.objects and not green_duba.objects:
        return "PARKUR BITTI, LIMANA YANASIYOR"
    else:
        return "GOREV YAPILIYOR"
    
def draw_vehicle_state(frame, vehicle_state):
    cv2.putText(frame, f'Arac Durumu: {vehicle_state}', (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

def update_json_vehicle_state(vehicle_state):
    """ Update the target_angle.json file with the given angle """
    data = {'vehicle_state': vehicle_state}
    with open('vehicle_state.json', 'w') as f:
        json.dump(data, f, indent=4)

def draw_abra_ida(frame):
    cv2.putText(frame, f'ABRA IDA', (20, 440), cv2.FONT_HERSHEY_SIMPLEX, 1, (128, 128, 0), 2)

#! SADECE UFUK CİZGİSİ GORUNTUSU, REFERANS ICIN COK ONEMLI 
def draw_ufuk_line(frame,min_y):
    cv2.line(frame, (0,min_y), (640,min_y), (0, 0, 255), 2)

#! min_y BURADA UFUK CIZGISI DRAW UFUKLINE CIZDIRIP REFERANS ALIN, RENK KODLARINI DA DEGIS
red_duba = Duba('Kirmizi', np.array([0, 50, 50]), np.array([10, 255, 255]), (0, 0, 255),min_y=150)
yellow_duba = Duba('Sari', np.array([20, 100, 100]), np.array([30, 255, 255]), (0, 255, 255),min_y=150)
green_duba = Duba('Yesil', np.array([40, 20, 50]), np.array([90, 255, 255]), (0, 255, 0),min_y=150)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    red_mask = cv2.inRange(hsv_frame, red_duba.lower_bound, red_duba.upper_bound)
    yellow_mask = cv2.inRange(hsv_frame, yellow_duba.lower_bound, yellow_duba.upper_bound)
    green_mask = cv2.inRange(hsv_frame, green_duba.lower_bound, green_duba.upper_bound)

    red_duba.detect_objects(frame, red_mask)
    yellow_duba.detect_objects(frame, yellow_mask)
    green_duba.detect_objects(frame, green_mask)

    red_duba.draw_objects(frame)
    yellow_duba.draw_objects(frame)
    green_duba.draw_objects(frame)

    draw_lines_between_dubas(frame, red_duba, yellow_duba, green_duba)
    
    draw_vehicle_mode(frame)

    vehicle_state = determine_vehicle_state(red_duba, yellow_duba, green_duba)
    draw_vehicle_state(frame, vehicle_state)
    update_json_vehicle_state(vehicle_state)

    draw_abra_ida(frame)
    draw_ufuk_line(frame,200)

    cv2.imshow('Frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
