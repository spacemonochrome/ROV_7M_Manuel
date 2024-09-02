import smbus
import time
import os
import time

MPU6050_ADDRESS = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47
ACCEL_CONFIG = 0x1C
TEMP_OUT_H = 0x41
TEMP_OUT_L = 0x42
DLPF_REGISTER = 0x1A
DLPF_SETTING = 0x06

bus = smbus.SMBus(1)

def mpu6050_init():
    bus.write_byte_data(MPU6050_ADDRESS, PWR_MGMT_1, 0)    
    bus.write_byte_data(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00)

def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDRESS, addr)
    low = bus.read_byte_data(MPU6050_ADDRESS, addr+1)
    value = ((high << 8) | low)
    if value > 32768:
        value = value - 65536
    return value

def set_dlpf(setting):
    bus.write_byte_data(MPU6050_ADDRESS, DLPF_REGISTER, setting)

def read_temperature():
    temp_high = bus.read_byte_data(MPU6050_ADDRESS, TEMP_OUT_H)
    temp_low = bus.read_byte_data(MPU6050_ADDRESS, TEMP_OUT_L)
    temp_raw = (temp_high << 8) | temp_low
    if temp_raw > 32767:
        temp_raw -= 65536
    temp_celsius = (temp_raw / 340.0) + 36.53
    return temp_celsius

def MPU_AccelXYZ():
    accel_x = read_raw_data(ACCEL_XOUT_H)
    accel_y = read_raw_data(ACCEL_YOUT_H)
    accel_z = read_raw_data(ACCEL_ZOUT_H)
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)
    temperature = read_temperature()    
    return accel_x, accel_y, accel_z, temperature, gyro_x, gyro_y, gyro_z


if __name__ == "__main__":
    mpu6050_init()
    set_dlpf(DLPF_SETTING)
    
    # Kalibrasyon için boş değerler
    gyro_bias = {'x': 0, 'y': 0, 'z': 0}
    accel_bias = {'x': 0, 'y': 0, 'z': 0}
    calibration_samples = 100

    print("Kalibrasyon Başlatılıyor...")

    # Kalibrasyon örneklerini topla
    for i in range(calibration_samples):
        accel_x = read_raw_data(ACCEL_XOUT_H)
        accel_y = read_raw_data(ACCEL_YOUT_H)
        accel_z = read_raw_data(ACCEL_ZOUT_H)
        temperature = read_temperature()
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)
        
        # Jiroskop verilerini toplama
        gyro_bias['x'] += gyro_x
        gyro_bias['y'] += gyro_y
        gyro_bias['z'] += gyro_z
        
        # İvmeölçer verilerini toplama
        accel_bias['x'] += accel_x
        accel_bias['y'] += accel_y
        accel_bias['z'] += accel_z
        
        time.sleep(0.05)

    # Ortalama alarak sapmaları hesapla
    gyro_bias['x'] /= calibration_samples
    gyro_bias['y'] /= calibration_samples
    gyro_bias['z'] /= calibration_samples

    accel_bias['x'] /= calibration_samples
    accel_bias['y'] /= calibration_samples
    accel_bias['z'] /= calibration_samples

    print("Kalibrasyon Tamamlandı.")
    print(f"Jiroskop Sapmaları: {gyro_bias}")
    print(f"İvmeölçer Sapmaları: {accel_bias}")

    gyro_bias = int(gyro_bias)
    accel_bias = int(accel_bias)
    time.sleep(3)
    
    while True:
        accel_x = read_raw_data(ACCEL_XOUT_H)
        accel_y = read_raw_data(ACCEL_YOUT_H)
        accel_z = read_raw_data(ACCEL_ZOUT_H)
        temperature = read_temperature()
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)
        #print(f"Accel X: {accel_x} | Accel Y: {accel_y} | Accel Z: {accel_z} | Sıcaklık: {temperature:.2f} °C")
        #print(f"Gyro X: {gyro_x} | Gyro Y: {gyro_y} | Gyro Z: {gyro_z}")
        print(f"Accel X: {accel_bias['x'] - accel_x} | Accel Y: {accel_bias['y']- accel_y} | Accel Z: {accel_bias['y'] - accel_z}")
        print(f"Gyro X: {accel_bias['x'] - gyro_x} | Gyro Y: {accel_bias['y'] - gyro_y} | Gyro Z: {accel_bias['z'] - gyro_z}")

        time.sleep(0.1)
