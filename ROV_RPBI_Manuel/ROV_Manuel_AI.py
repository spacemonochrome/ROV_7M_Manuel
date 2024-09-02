#!/usr/bin/env python
import time
import os
import threading
import serial

delaysuresi = 15
delayy = delaysuresi/1000
dogrulamabyti = 95
durmasarti = False

TxData = [125,125,125,125,125,125,125,dogrulamabyti,65,65,65,65,65,65,65,dogrulamabyti]
RxData = [None]
TxEkBilgi = [65,65,65,65,65,65,65]

TxMotorverisi = [125,125,125,125,125,125,125] #Gonderilen Motor degeri
MotorDeger =    [125,125,125,125,125,125,125] #Ulasilmak istenen motor degeri



A  = [125,125,125,125,125,125,125] # İleri Yön
B  = [125,125,125,125,125,125,125] # Geri Yön
C  = [125,125,125,125,125,125,125] # Sağa Dönme
D  = [125,125,125,125,125,125,125] # Sola Dönme
E  = [125,125,125,125,125,125,125] # Sağa Git
F  = [125,125,125,125,125,125,125] # Sola Git
G  = [125,125,125,125,125,125,125] # Batma
H  = [125,125,125,125,125,125,125] # Çıkma
I  = [125,125,125,125,125,125,125] # Torpido
J  = [125,125,125,125,125,125,125] # Boşta durma

'''
Durmasarti degiscek ve TxMotorverisi degisir
Örnek
TxMotorverisi = A ileri gider
TxMotorverisi = B geri gider
haberleşmeyi durdurmak istersen durmasarti = false yap

'''

ser = serial.Serial(
        port='/dev/ttyS0',
        baudrate = 115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=100
)

# 25 ve 225 değerleri arasında değer alır

def bosaAlma():
        global TxMotorverisi, dogrulamabyti, TxData, TxEkBilgi, durmasarti
        for i in range(0, 50):
                TxData = TxMotorverisi + [dogrulamabyti] + TxEkBilgi + [dogrulamabyti]
                ser.write(bytearray(TxData))

def MotorDegerUpdate():
        global durmasarti, MotorDeger, TxMotorverisi
        bosaAlma()
        while durmasarti:
                for i in range(0,len(TxMotorverisi)):
                        if MotorDeger[i] > TxMotorverisi[i]:
                                TxMotorverisi[i] = TxMotorverisi[i] + 1
                        elif MotorDeger[i] == TxMotorverisi[i]:
                                pass
                        elif MotorDeger[i] < TxMotorverisi[i]:
                                TxMotorverisi[i] = TxMotorverisi[i] - 1
                time.sleep(delayy)

def UartWrite():
        global TxData, durmasarti, TxMotorverisi, TxEkBilgi, dogrulamabyti
        while durmasarti:
                TxData = TxMotorverisi + [dogrulamabyti] + TxEkBilgi + [dogrulamabyti]
                ser.write(bytearray(TxData))

def UartRead():
        global RxData
        while durmasarti:
                RxData = ser.read(1)
                if(RxData == "**********"):
                        os.system("sudo reboot")

def EkranaYaz():
        global TxData, RxData, durmasarti
        while durmasarti:
                time.sleep(0.1)
                os.system("clear")
                print("Giden Veri " + str(TxData) + " Gelen veri " + str(RxData))
                
if __name__ == '__main__':
        t1 = threading.Thread(target = MotorDegerUpdate)
        t2 = threading.Thread(target = UartWrite)
        t3 = threading.Thread(target = UartRead)
        t4 = threading.Thread(target = EkranaYaz)
        t1.start()
        t2.start()
        t3.start()
        t4.start()
        t1.join()
        t2.join()
        t3.join()
        t4.join()
        print("durdu")
