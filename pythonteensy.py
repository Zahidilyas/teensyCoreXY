#!/usr/bin/env python3

import serial
import time
import random as rnd

class PiXY:
    def __init__(self,ser):
        self.ser  = ser
        self.ser.flush()
        self.ser.write("\r\n\r\n".encode('utf-8')) # Hit enter a few times to wake the Printrbot
        time.sleep(2)   # Wait for Printrbot to initialize
        self.ser.flushInput()  # Flush startup text in self.serial input
        
        self.ser.readline()
        self.ser.readline()
        self.ser.readline()
        self.ser.readline()
        self.ser.readline()
        self.ser.readline()
        #Initiailizing
        print("Initiailizing...")
        print("Caliberation might take few secs")
        print("---Caliberating")
        self.calibrate()
        time.sleep(2)
        self.moveto(10,10)
        print("---Grip Up")
        self.up()
        print("---Grip Close")
        #self.grip_close()
        print("---Moving to center")
        self.moveto(120,120)
        time.sleep(2)
        print("---Rotate to 0")
        self.rotate(0)
        print("Initialization Completed")
    
    def calibrate(self):
        self.ser.write( ('T1' + '\n').encode('utf-8'))
        
    def up(self):
        self.ser.write(('T3' + '\n').encode('utf-8'))
        time.sleep(2)
        
    def down(self):
        self.ser.write(('T4' + '\n').encode('utf-8'))
        time.sleep(2)
        
    def grip_open(self):
        self.ser.write(('Z1' + '\n').encode('utf-8'))
        time.sleep(2)

    def grip_close(self):
        self.ser.write(('Z2' + '\n').encode('utf-8'))
        time.sleep(2)

    def grip_open_close(self, val):
        self.ser.write('O'+str(val)+ '/n')
        
    def rotate(self,angle):
        self.ser.write(('R'+str(angle) + '\n').encode('utf-8'))
        time.sleep(2)
        
    def moveto(self,x,y):
        self.ser.write( ('G00 X'+str(x)+' Y'+str(y) + '\n').encode('utf-8'))
       

if __name__=="__main__":
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    grip = PiXY(ser)
    grip.down()
    grip.moveto(50,50)

    while(1):
        Xcor = rnd.randint(0,150)
        Ycor = rnd.randint(0,200)
        print("X = ",Xcor)
        #print("  ")
        print("Y = ",Ycor)
        #print("   ")
        grip.moveto(Xcor,Ycor)
        time.sleep(1)

        grip.grip_open_close(90)
        grip.grip_open_close(40)
