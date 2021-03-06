import pynmea2
import pyproj as proj
import math
import threading
import numpy as np
from time import sleep

import serial

class GpsThreadReadings (threading.Thread):
    def __init__(self,GpsReadings):
        super(GpsThreadReadings , self).__init__(name="GPS thread")
        self.serialCom = serial.Serial(port='/dev/ttyACM0',baudrate=115200)
        #self.serialCom = serial.Serial(port='COM3',baudrate=115200)
        self.GpsReadings = GpsReadings
        self.GpsRun = True
        print("GPS Created")
        

    def getGpsReadings(self):
        return self.GpsReadings

    def setDeltaForRover(self,defLat,defLong):
        self.defLat = defLat
        self.defLong = defLong
    def run(self):
        while self.GpsRun:
            #self.GpsReadings[1] = self.GpsReadings[1] - self.defLat*0.5
            #self.GpsReadings[2] = self.GpsReadings[2] - self.defLong*0.5
            #sleep(0.01)
            self.GpsReadings = self.readGPS(self.GpsReadings[0],self.GpsReadings[1],self.GpsReadings[2])

    def calAngle(self,lat1,long1,lat2,long2):
        dy = lat2 - lat1

        dx = math.cos(math.pi/180*lat1)*(long2 - long1)
        angle = math.atan2(dy, dx)*180/math.pi-90
        if angle <-180:
            angle = angle+360
            
        return -angle

    def deg2rad(self,deg):
        return deg * (math.pi/180)

    def getDistanceFromLatLonInKm(self,lat1,lon1,lat2,lon2) :
        R = 6371; # Radius of the earth in km
        dLat = self.deg2rad(lat2-lat1);  # deg2rad below
        dLon = self.deg2rad(lon2-lon1); 
        a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(self.deg2rad(lat1)) * math.cos(self.deg2rad(lat2)) *  math.sin(dLon/2) * math.sin(dLon/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a)); 
        d = R * c; # Distance in km
        return d*1000


    def readGPS(self,angle,latAv2,longAv2):
        data = self.serialCom.readline()
        GPS_Read = str(data)[2:-5]

        try:
            msg = pynmea2.parse(GPS_Read)
            
                
            if msg.sentence_type == 'GGA' :
                
                #latAv1 =  msg.latitude
                #longAv1 =  msg.longitude
                
                latAv2 =  msg.latitude
                longAv2 =  msg.longitude
                #distance = self.getDistanceFromLatLonInKm(latAv1,longAv1,latAv2,longAv2)
                

                #print("Distance: %s" % (distance))
                
                #if distance>0.19 :
                #    angle = self.calAngle(latAv1,longAv1,latAv2,longAv2)
                    
                #    latAv2 =latAv1*0.6+0.4*latAv2
                #    longAv2 =longAv1*0.6+0.4*longAv2
                    
                    
                    #print("Angle: %s" % (angle))
                    
                    
                    #plt.scatter(longAv2, latAv2)
                    #plt.pause(0.001)

                
        except:
            pass
        #return [angle,latAv2,longAv2]
        return [0,latAv2,longAv2]



##plt.axis([31.1790, 31.1792, 30.01407, 30.0142])

##ser = serial.Serial(port='COM3',baudrate=115200)
##i = 0
##GpsReadings = [0,0,0]
##GpsReadings = readGPS(ser,GpsReadings[0],GpsReadings[1],GpsReadings[2])
##print("Angle: %s, latitude: %s,  Longitude: %s" % (GpsReadings[0], GpsReadings[1], GpsReadings[2]))
##plt.show()
##ser.close();


