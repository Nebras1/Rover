# -*- coding: utf-8 -*-
"""
Created on Mon Dec  2 14:32:15 2019

@author: Ahmed.Dakrory
"""
import math
import utm

import pyproj
geodesic = pyproj.Geod(ellps='WGS84')

def convertNumberIntoAsciValue(valueToConvert):
    latLongList = list(str(valueToConvert))
    latLongAsciBytes = []
    for i in range(0,len(latLongList)):
        latLongAsciBytes.append(ord(latLongList[i]))
    return latLongAsciBytes

def sendArrayOfBytes(address,data,bus):
    for i in range(0,len(data)):
        bus.write_byte(address,data[i])



def deg2num(lat_deg, lon_deg, zoom):
  lat_rad = math.radians(lat_deg)
  n = 2.0 ** zoom
  xtile = ((lon_deg + 180.0) / 360.0 * n)
  ytile = ((1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
  return (xtile, ytile)


def calAngle(point1_StartGPS,point2_EndDist):
    lat1 = point1_StartGPS[0]
    long1 = point1_StartGPS[1]
    lat2 = point2_EndDist[0]
    long2 = point2_EndDist[1]
    fwd_azimuth,back_azimuth,distance = geodesic.inv(lat1, long1, lat2, long2)

    angle = 90-fwd_azimuth
    if angle > 180:
        angle = angle - 360
    return angle


def getDistanceFromLatLonInMeter(point1,point2) :
    try:
        R = 6371; # Radius of the earth in km
        dLat = deg2rad(point2[0]-point1[0]);  # deg2rad below
        dLon = deg2rad(point2[1]-point1[1]); 
        a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(deg2rad(point1[0])) * math.cos(deg2rad(point2[0])) *  math.sin(dLon/2) * math.sin(dLon/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a)); 
        d = R * c; # Distance in km
    except:
        pass
    return d*1000


def deg2rad(deg):
    return deg * (math.pi/180)

def mainLoopForSendTheNeededLengthAndAngle(KpDistance,KpAngle,KpRate,Gps,routingClass,listOfPoints,bus,addr,imu):
    #get the first Point which is the nabour Point to me
    #get the GPS Point Here
    currPointGPS = routingClass.node(Gps.getGpsReadings()[1],Gps.getGpsReadings()[2])
    indexFirstTarget = len(listOfPoints) - 1
    indexSecondTarget = goToNextTargetOrNot(listOfPoints,Gps,indexFirstTarget)
    
    notReachEndPoint = True
    
    indexCurrentTargetPoint = (len(listOfPoints)-1)

    while notReachEndPoint:
        

        if imu.Readings !=None:
            angleRover = imu.Readings['Yaw']

        if imu.Rates !=None:
            gyroRover = imu.Rates['gz']
       
        
        [actionDistance, angleAction,actionRate] = calculateControlAction(KpDistance,KpAngle,KpRate,Gps,listOfPoints,indexCurrentTargetPoint,angleRover,gyroRover)
        

        #remove this after set Gps
        #simulateRoverGPS(Gps,listOfPoints,indexCurrentTargetPoint)
        
        indexCurrentTargetPoint = goToNextTargetOrNot(listOfPoints,Gps,indexCurrentTargetPoint)
        notReachEndPoint = checkIfNotReachedEndPoint(indexCurrentTargetPoint)
        
        sendActionsToMicroController(actionDistance, angleAction,actionRate,addr,bus)
        print("Distance: %f, Angle: %f, GPS: %f, i: %f" %(actionDistance, angleAction,Gps.getGpsReadings()[1],indexCurrentTargetPoint))

def sendActionsToMicroController(actionDistance, angleAction,actionRate,addr,bus):
    sendArrayOfBytes(addr,convertNumberIntoAsciValue(actionDistance),bus)
    sendArrayOfBytes(addr,convertNumberIntoAsciValue(':'),bus)
    sendArrayOfBytes(addr,convertNumberIntoAsciValue(angleAction),bus)
    sendArrayOfBytes(addr,convertNumberIntoAsciValue('$'),bus)
    sendArrayOfBytes(addr,convertNumberIntoAsciValue(actionRate),bus)
    sendArrayOfBytes(addr,convertNumberIntoAsciValue(';'),bus)
    
def goToNextTargetOrNot(listOfPoints,Gps,indexOfCurrentTarget):
    targetPoint = [listOfPoints[indexOfCurrentTarget][1],listOfPoints[indexOfCurrentTarget][2]]
    #Adjust this Parameter to get the best Performance
    if getDistanceFromLatLonInMeter([Gps.getGpsReadings()[1],Gps.getGpsReadings()[2]],targetPoint) < 0.1:
        del listOfPoints[indexOfCurrentTarget]

    return len(listOfPoints)-1
    
def calculateControlAction(KpDistance,KpAngle,KpRate,Gps,listOfPoints,indexCurrentTargetPoint,angleRover,gyroRover):
    currPointGPS = [Gps.getGpsReadings()[1],Gps.getGpsReadings()[2]]
    currentTarget = [listOfPoints[indexCurrentTargetPoint][1],listOfPoints[indexCurrentTargetPoint][2]]
    distance = getDistanceFromLatLonInMeter(currentTarget,currPointGPS)
    angle = deg2rad(calAngle(currPointGPS,currentTarget))
    
    errorAngle = angle-angleRover
    angleAction = KpAngle * errorAngle
    
    actionRate = KpRate * gyroRover
    
    
    if errorAngle == 0:
        errorAngle = 0.1
    errorDistance = distance/errorAngle
    
    actionDistance = KpDistance * errorDistance
    
    
    return [actionDistance, angleAction,actionRate]


def simulateRoverGPS(Gps,listOfPoints,indexCurrentTargetPoint):
    
    currPointGPS = [Gps.getGpsReadings()[1],Gps.getGpsReadings()[2]]
    #ThoseWillBeRemoved
    defLat = (currPointGPS[0] - listOfPoints[indexCurrentTargetPoint][1])
    defLong = (currPointGPS[1] - listOfPoints[indexCurrentTargetPoint][2])

    Gps.setDeltaForRover(defLat,defLong)
   

    

    
def checkIfNotReachedEndPoint(indexCurrentTargetPoint):
    if indexCurrentTargetPoint == -1:
       return False
    else:
        return True
   
def mostNabourPointAndDeleteNewPoint(listOfPoints,cPoint):
    indexMin=0
    lengthMin = 99999999999999.99999999

    for n in range(0,len(listOfPoints)):
        listPoint = [listOfPoints[n][1],listOfPoints[n][2]]
        d = getDistanceFromLatLonInMeter(cPoint,listPoint)
        if d<lengthMin:
            lengthMin=d
            indexMin = n
    pointToReturn = [listOfPoints[indexMin][1],listOfPoints[indexMin][2]]
    del listOfPoints[indexMin]
    return [indexMin,lengthMin,pointToReturn]
