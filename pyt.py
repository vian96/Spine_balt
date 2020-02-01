import serial
import bpy
from math import radians
from threading import Thread

deltas=[obj.delta_rotation_euler for obj in bpy.data.objects]

def movcord(n:int, x, y, z):
    delt=deltas[n]
    delt.x=x
    delt.y=y
    delt.z=z

def sread(port:str):
    ser = serial.Serial(port)
    try:
        global kil
        while 1:
                sx,y,sz=0,0,0
                if kil:
                    raise Exception('stopped')
                s = str(ser.read_until())[2:-7]
                #s=input()
                print(s)
                n=8
                s=s.split(" ")[2:]
                while s:
                    x,z = radians(float(s[0])), -radians(float(s[1]))
                    yield n,x-sx,y,z-sz
                    sx, sz=x, z
                    s=s[2:]
                    n-=1
    except Exception as e:
        ser.close()
        raise e

def todo(port_name:str):
    for n,x,y,z in sread(port_name):
        #print(n,x,y,z)
        movcord(n,x,y,z)

def kil_inp():
    global kil
    input()
    kil=True

kil=False
t1=Thread(target=todo, args=(input('Started, input port:'),))
t1.start()
t2=Thread(target=kil_inp)
t2.start()