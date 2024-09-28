"""
Author: Bülent Kıpçak
Email: bulentkipcak04@gmail.com
Date: 2024-08-15
Version: 1.0
Description: This project is designed to control a maritime vehicle using the MAVLink protocol.
"""
from abra_ida import AbraIda
import time

connection_string = "tcp:192.168.31.60:5760"
vehicle = AbraIda(connection_string)

while True:
    vehicle_mode = vehicle.get_current_vehicle_mode()
    vehicle.update_json_mode(vehicle_mode)

    error_yaw = None
    while error_yaw is None:
        error_yaw = vehicle.read_error_yaw_from_json()
        if error_yaw is None:
            print("JSON verisi bekleniyor...")
            time.sleep(0.1)

    vehicle_state = None
    while vehicle_state is None:
        vehicle_state = vehicle.read_vehicle_state()
        if vehicle_state is None:
            print("JSON verisi bekleniyor...")
            time.sleep(0.1)

    auto_pilot_error_angle = error_yaw / 4
    current_yaw = vehicle.get_current_yaw()
    target_yaw = current_yaw + auto_pilot_error_angle

    print(f"Şu anki açı: {current_yaw}, Hedef açı: {target_yaw}, Fark: {auto_pilot_error_angle}")

    if vehicle_state == "GOREV YAPILIYOR":
        if abs(error_yaw) <= 10:
            print("Yaw açısı hedef farkı ±10 dereceden küçük, 4m/s ileri hız ayarlanıyor.")
            vehicle.set_velocity_target(4)
        else:
            print("Yaw açısı hedef farkı ±10 dereceden büyük, sadece hedef açı ayarlanıyor.")
            vehicle.set_yaw_target(target_yaw)

        time.sleep(0.5)

    elif vehicle_state == "KIRMIZI DUBA":
        done = False  
        while not done:
            vehicle.set_velocity_target(4)
            print("4 m/s ile 3 saniye ileri gidiyor...")
            time.sleep(3)

            start_time = time.time()
            while time.time() - start_time < 4:
                current_yaw = vehicle.get_current_yaw()
                vehicle.set_yaw_target(current_yaw + 10)
                time.sleep(0.1)

            print("Diğer dubalar aranıyor, sağa dönülüyor...")
            done = True

    elif vehicle_state == "YESIL DUBA":
        vehicle.set_yaw_target(current_yaw - 10)
        print("Diğer dubalar aranıyor, sola dönülüyor...")

    elif vehicle_state == "SARI DUBA":
        vehicle.set_yaw_target(current_yaw - 10)
        print("Diğer dubalar aranıyor, sola dönülüyor...")
        time.sleep(5)
        vehicle.set_yaw_target(current_yaw + 20)
        print("Diğer dubalar aranıyor, sağa dönülüyor...")

    elif vehicle_state == "PARKUR BITTI, LIMANA YANASIYOR":
        print("Limana yanaşıyor...")
        vehicle.set_velocity_target(5)
        print("5 m/s ile ileri gidiyor.")
        time.sleep(4)
        vehicle.set_velocity_target(5)
        print("5 m/s ile ileri gidiyor.")
        time.sleep(4)
        current_yaw = vehicle.get_current_yaw()
        aci = current_yaw + 45
        vehicle.set_yaw_target(aci)
        print("Sağa dönüyor.")
        time.sleep(4)
        current_yaw = vehicle.get_current_yaw()
        aci = current_yaw + 45
        vehicle.set_yaw_target(aci)
        print("Sağa dönüyor.")
        time.sleep(4)

    else:
        vehicle.set_velocity_target(0)
        vehicle.set_yaw_target(vehicle.get_current_yaw())
        print("Bilinmeyen durum.")

    time.sleep(0.5)
