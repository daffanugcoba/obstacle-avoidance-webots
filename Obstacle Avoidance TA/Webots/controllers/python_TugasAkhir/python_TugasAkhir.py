from controller import Robot
import matplotlib.pyplot as plt
import numpy as np 
import skfuzzy as fuzz
import time 
import os
import csv
# create the Robot instance.
robot = Robot()
    # Mendapatkan nilai timestep dari lingkungan simulasi
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

    # Mengatur motor agar berputar tanpa batas dan memiliki kecepatan awal 0
left_motor.setPosition(float('inf'))    
left_motor.setVelocity(0.0)
right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)

    # definisi sensor
sensor_ps0 = robot.getDevice('ps0')
sensor_ps1 = robot.getDevice('ps1')
sensor_ps6 = robot.getDevice('ps6')
sensor_ps7 = robot.getDevice('ps7')

sensor_ps0.enable(timestep)
sensor_ps1.enable(timestep)
sensor_ps6.enable(timestep)
sensor_ps7.enable(timestep)

    #Waktu awal
waktuMulai = time.time()
plotkiri = []
plotkanan = []

    #definisi array fuzzy
x_sensor = np.arange(0, 41, 1)
x_motor = np.arange(-51, 51, 1)

    #alat konversi
def map_value(value):
    return int((value + 100) * 1)
    
def calculate_motor(signal):
    return (signal/10)

kecepatan_motor_kanan_last = 0
kecepatan_motor_kiri_last = 0

gps = robot.getDevice('gps')
gps.enable(timestep)

csv_file_name = 'data_value.csv'
if os.path.exists(csv_file_name):
    os.remove(csv_file_name)

def save_to_csv(data):
    with open(csv_file_name, mode='a', newline='') as file:
        writer = csv.writer(file)
     
        if file.tell() == 0:
            writer.writerow(['gps value x','gps value y']) 
        writer.writerow(data)
while robot.step(timestep) != -1:
           # Memproses data sensor
    value_ps0 = sensor_ps0.getValue()
    value_ps1 = sensor_ps1.getValue()
    value_ps6 = sensor_ps6.getValue()
    value_ps7 = sensor_ps7.getValue()
    
    sensorkanan = (value_ps0 + value_ps1)/2
    sensortengah = (value_ps0 + value_ps7)/2
    sensorkiri = (value_ps6 + value_ps7)/2
        #membatasi nilai sensor menjadi 100
    if sensorkiri >= 100:
        sensorkiri = 100
    if sensortengah >= 100:
        sensortengah = 100
    if sensorkanan >= 100:
        sensorkanan = 100
        #konversi nilai sensor kedalam fuzzy
    #sensorkanan_minus 
    sensorkanan_konversi = map_value(-sensorkanan)
    sensortengah_konversi = map_value(-sensortengah)
    sensorkiri_konversi = map_value(-sensorkiri)

    sensor_right = sensorkanan_konversi
    sensor_mid = sensortengah_konversi
    sensor_left = sensorkiri_konversi
    
        #fuzzyy
    left_sensor_nier = fuzz.trapmf(x_sensor, [0, 0, 5, 12])
    left_sensor_medium = fuzz.trapmf(x_sensor, [8, 15, 30, 37])
    left_sensor_far = fuzz.trapmf(x_sensor, [33, 40, 40, 40])
    
    mid_sensor_nier = fuzz.trapmf(x_sensor, [0, 0, 5, 12])
    mid_sensor_medium = fuzz.trapmf(x_sensor, [8, 15, 30, 37])
    mid_sensor_far = fuzz.trapmf(x_sensor, [33, 40, 40, 40])
    
    right_sensor_nier = fuzz.trapmf(x_sensor, [0, 0, 5, 12])
    right_sensor_medium = fuzz.trapmf(x_sensor, [8, 15, 30, 37])
    right_sensor_far = fuzz.trapmf(x_sensor, [33, 40, 40, 40])
    
    right_motor_negatif = fuzz.trapmf(x_motor, [-50, -50, -50, 0])
    right_motor_positif = fuzz.trapmf(x_motor, [0, 50, 50, 50])
    
    left_motor_negatif = fuzz.trapmf(x_motor, [-50, -50, -50, 0])
    left_motor_positif = fuzz.trapmf(x_motor, [0, 50, 50, 50])
    
        #fuzz
    left_sensor_nier_fuzz = fuzz.interp_membership(x_sensor, left_sensor_nier, sensor_left)
    left_sensor_medium_fuzz = fuzz.interp_membership(x_sensor, left_sensor_medium, sensor_left)
    left_sensor_far_fuzz = fuzz.interp_membership(x_sensor, left_sensor_far, sensor_left)
    
    mid_sensor_nier_fuzz = fuzz.interp_membership(x_sensor, mid_sensor_nier, sensor_mid)
    mid_sensor_medium_fuzz = fuzz.interp_membership(x_sensor, mid_sensor_medium, sensor_mid)
    mid_sensor_far_fuzz = fuzz.interp_membership(x_sensor, mid_sensor_far, sensor_mid)
    
    right_sensor_nier_fuzz = fuzz.interp_membership(x_sensor, right_sensor_nier, sensor_right)
    right_sensor_medium_fuzz = fuzz.interp_membership(x_sensor, right_sensor_medium, sensor_right)
    right_sensor_far_fuzz = fuzz.interp_membership(x_sensor, right_sensor_far, sensor_right)
    
        #Rule
    rules_1 = np.fmin(left_sensor_nier_fuzz, np.fmin(mid_sensor_nier_fuzz, right_sensor_nier_fuzz))#NEGATIF,NEGATIF
    rules_2 = np.fmin(left_sensor_nier_fuzz, np.fmin(mid_sensor_nier_fuzz, right_sensor_medium_fuzz))#POSITIF,NEGATIF
    rules_3 = np.fmin(left_sensor_nier_fuzz, np.fmin(mid_sensor_medium_fuzz, right_sensor_nier_fuzz))#POSITIF,POSITIF
    rules_4 = np.fmin(left_sensor_nier_fuzz, np.fmin(mid_sensor_medium_fuzz, right_sensor_medium_fuzz))#POSITIF,NEGATIF
    rules_5 = np.fmin(left_sensor_medium_fuzz, np.fmin(mid_sensor_nier_fuzz, right_sensor_nier_fuzz))#NEGATIF,POSITIF
    rules_6 = np.fmin(left_sensor_medium_fuzz, np.fmin(mid_sensor_medium_fuzz, right_sensor_nier_fuzz))#NEGATIF,POSITIF
    rules_7 = np.fmin(left_sensor_medium_fuzz, np.fmin(mid_sensor_nier_fuzz, right_sensor_medium_fuzz))#POSITIF,NEGATIF
    rules_8 = np.fmin(left_sensor_medium_fuzz, np.fmin(mid_sensor_medium_fuzz, right_sensor_medium_fuzz))#POSITIF,POSITIF
    rules_9 = np.fmin(left_sensor_nier_fuzz, np.fmin(mid_sensor_nier_fuzz, right_sensor_far_fuzz))#POSITIF,NEGATIF
    rules_10 = np.fmin(left_sensor_nier_fuzz, np.fmin(mid_sensor_far_fuzz, right_sensor_nier_fuzz))#POSITIF,POSITIF
    rules_11 = np.fmin(left_sensor_nier_fuzz, np.fmin(mid_sensor_far_fuzz, right_sensor_far_fuzz))#POSITIF,NEGATIF
    rules_12 = np.fmin(left_sensor_far_fuzz, np.fmin(mid_sensor_nier_fuzz, right_sensor_nier_fuzz))#NEGATIF,POSITIF
    rules_13 = np.fmin(left_sensor_far_fuzz, np.fmin(mid_sensor_far_fuzz, right_sensor_nier_fuzz))#NEGATIF,POSITIF
    rules_14 = np.fmin(left_sensor_far_fuzz, np.fmin(mid_sensor_nier_fuzz, right_sensor_far_fuzz))#POSITIF,NEGATIF
    rules_15 = np.fmin(left_sensor_far_fuzz, np.fmin(mid_sensor_far_fuzz, right_sensor_far_fuzz))#POSITI,POSITIF
    rules_16 = np.fmin(left_sensor_medium_fuzz, np.fmin(mid_sensor_far_fuzz, right_sensor_far_fuzz))#POSITIF,POSITIF
    rules_17 = np.fmin(left_sensor_medium_fuzz, np.fmin(mid_sensor_far_fuzz, right_sensor_medium_fuzz))#POSITIF,POSITIF
    rules_18 = np.fmin(left_sensor_medium_fuzz, np.fmin(mid_sensor_medium_fuzz, right_sensor_far_fuzz))#POSITIF,POSITIF
    rules_19 = np.fmin(left_sensor_far_fuzz, np.fmin(mid_sensor_far_fuzz, right_sensor_medium_fuzz))#POSITIF,POSITIF
    rules_20 = np.fmin(left_sensor_far_fuzz, np.fmin(mid_sensor_medium_fuzz, right_sensor_far_fuzz))#POSITIF,POSITIF
    rules_21 = np.fmin(left_sensor_far_fuzz, np.fmin(mid_sensor_medium_fuzz, right_sensor_medium_fuzz))#POSITIF,POSITIF
    rules_22 = np.fmin(left_sensor_nier_fuzz, np.fmin(mid_sensor_far_fuzz, right_sensor_medium_fuzz))#POSITIF,NEGATIF
    rules_23 = np.fmin(left_sensor_nier_fuzz, np.fmin(mid_sensor_medium_fuzz, right_sensor_far_fuzz))#POSITIF,NEGATIF
    rules_24 = np.fmin(left_sensor_far_fuzz, np.fmin(mid_sensor_nier_fuzz, right_sensor_medium_fuzz))#NEGATIF,POSITIF
    rules_25 = np.fmin(left_sensor_far_fuzz, np.fmin(mid_sensor_medium_fuzz, right_sensor_nier_fuzz))#NEGATIF,POSITIF
    rules_26 = np.fmin(left_sensor_medium_fuzz, np.fmin(mid_sensor_nier_fuzz, right_sensor_far_fuzz))#POSITIF,NEGATIF
    rules_27 = np.fmin(left_sensor_medium_fuzz, np.fmin(mid_sensor_far_fuzz, right_sensor_nier_fuzz))#NEGATIF,POSITIF 
    
        #Defuzz
    negatif_left = np.fmin(np.fmax(rules_1,np.fmax(rules_5,np.fmax(rules_6,np.fmax(rules_12,np.fmax(rules_13,np.fmax(rules_24,np.fmax(rules_25,rules_27))))))),left_motor_negatif)
    positif_left= np.fmin(np.fmax(rules_2,np.fmax(rules_3,np.fmax(rules_4,np.fmax(rules_7,np.fmax(rules_8,np.fmax(rules_9,np.fmax(rules_10,np.fmax(rules_11,np.fmax(rules_14,np.fmax(rules_15,np.fmax(rules_16,np.fmax(rules_17,np.fmax(rules_18,np.fmax(rules_19,np.fmax(rules_20,np.fmax(rules_21,np.fmax(rules_22,np.fmax(rules_23,rules_26)))))))))))))))))),left_motor_positif)
    
    negatif_right= np.fmin(np.fmax(rules_1,np.fmax(rules_2,np.fmax(rules_4,np.fmax(rules_7,np.fmax(rules_9,np.fmax(rules_11,np.fmax(rules_14,np.fmax(rules_22,np.fmax(rules_23,rules_26))))))))),right_motor_negatif)
    positif_right= np.fmin(np.fmax(rules_3,np.fmax(rules_5,np.fmax(rules_6,np.fmax(rules_8,np.fmax(rules_10,np.fmax(rules_12,np.fmax(rules_13,np.fmax(rules_15,np.fmax(rules_16,np.fmax(rules_17,np.fmax(rules_18,np.fmax(rules_19,np.fmax(rules_20,np.fmax(rules_21,np.fmax(rules_24,np.fmax(rules_25,rules_27)))))))))))))))),right_motor_positif)
    
        # Aggregate all three output membership functions together
    aggregated_left = np.fmax(positif_left, negatif_left)
    kecepatan_motor_kiri = fuzz.defuzz(x_motor, aggregated_left, 'MOM')

        # Aggregate all three output membership functions together
    aggregated_right = np.fmax(negatif_right, positif_right)
    kecepatan_motor_kanan = fuzz.defuzz(x_motor, aggregated_right, 'MOM')

    kecepatan_motor_kiri_last = calculate_motor(kecepatan_motor_kiri)
    kecepatan_motor_kanan_last = calculate_motor(kecepatan_motor_kanan)
    
    
    #print(f"SKa : {sensorkanan:.2f} || ST : {sensortengah:.2f} || SKi : {sensorkiri:.2f} || SDKi : {sensorkiri_konversi:.2f} || SDT : {sensortengah_konversi:.2f} || SDKa : {sensorkanan_konversi:.2f} || MKa : {kecepatan_motor_kanan_last:.2f} || MKi : {kecepatan_motor_kiri_last:.2f}")
    #print(f"{sensorkanan:.2f}")
    #print(f"{sensortengah:.2f}")
    #print(f"{sensorkiri:.2f}")
    #print(f"{kecepatan_motor_kanan_last:.2f}")
    #print(f"{kecepatan_motor_kiri_last:.2f}")
    
    #print(f"{sensorkiri_konversi:.2f}")
    #print(f"{sensortengah:.2f}")
    #print(f"{sensorkiri:.2f}")
    gps_value = gps.getValues()
    msg = " "
    for each_val in gps_value:
        msg += "{0:.2f}".format(each_val)
    #print(msg)
    gps_x = gps_value[0]
    gps_y = gps_value[1]
    print(f"{gps_value[1]:.2f}")
    
    left_motor.setVelocity(kecepatan_motor_kiri_last)
    right_motor.setVelocity(kecepatan_motor_kanan_last)
    waktu = time.time()
    # left_motor.setVelocity(6)
    # right_motor.setVelocity(6)
    plotkiri.append(kecepatan_motor_kiri)
    plotkanan.append(kecepatan_motor_kanan)
    # sensorplot.append(value_ps5)
    data = [gps_x,gps_y]
    save_to_csv(data)
    waktuBerjalan = waktu - waktuMulai
    if waktuBerjalan >= 80:
         break

plt.subplot(2,1,1)
plt.plot(plotkiri, label ="motorkiri", color ="blue")
plt.plot(plotkanan, label ="motorkanan", color ="red")
plt.ylabel("Keceparan motor")
plt.xlabel("Waktu")
plt.legend()
   
plt.show()