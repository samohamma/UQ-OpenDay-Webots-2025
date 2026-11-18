# Copyright 1996-2019 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#
# CACC car-following model based on [1], [2].
# [1] Milanes, V., and S. E. Shladover. Handling Cut-In Vehicles in Strings
#    of Cooperative Adaptive Cruise Control Vehicles. Journal of Intelligent
#     Transportation Systems, Vol. 20, No. 2, 2015, pp. 178-191.
# [2] Xiao, L., M. Wang and B. van Arem. Realistic Car-Following Models for
#    Microscopic Simulation of Adaptive and Cooperative Adaptive Cruise
#     Control Vehicles. Transportation Research Record: Journal of the
#     Transportation Research Board, No. 2623, 2017. (DOI: 10.3141/2623-01).




"""vehicle_driver controller."""

from vehicle import Driver
from controller import Supervisor, Robot, Node, RadarTarget, Radar, Receiver, Emitter, LED
import array
import struct
import time
import numpy as np
import math
from playsound2 import playsound
import threading

###################  define road geometric parameters ######################################
numLane=4
laneWidth=3.65
xoffsetLane = [-3500-laneWidth*2, -3398.6-laneWidth*2, -3297.2-laneWidth*2]
zoffsetRoad = [-18000, -9007.6, -15.2] 
startLength = 1000
mergeLength = 500
hdriveLength =1500
cadriveLength =4000
takeoverLength = 2000
divergeLength = 500

lanePositions=[[xoffsetLane[0]+laneWidth/2, xoffsetLane[0]+3*laneWidth/2, xoffsetLane[0]+5*laneWidth/2, xoffsetLane[0]+7*laneWidth/2],\
[xoffsetLane[1]+laneWidth/2, xoffsetLane[1]+3*laneWidth/2, xoffsetLane[1]+5*laneWidth/2, xoffsetLane[1]+7*laneWidth/2],\
[xoffsetLane[2]+laneWidth/2, xoffsetLane[2]+3*laneWidth/2, xoffsetLane[2]+5*laneWidth/2, xoffsetLane[2]+7*laneWidth/2]]

ringRoadLength = startLength+mergeLength+hdriveLength+cadriveLength+takeoverLength+divergeLength

zoffsetRoadEnd = [zoffsetRoad[0]+ringRoadLength, zoffsetRoad[1]+ringRoadLength, zoffsetRoad[2]+ringRoadLength] 

################################
WAITING_TIME_FOR_RUN = 1000

################### define vehicle parameters ###################################
vehLength = 4.9
gpsZOffset= 1
myMaxSpeed = 30
vehNumber_Lane = 11
vehNumber_Lane = 15

################### define steady state of traffic flow  ###################################
myMinGap = 2
myOriginGap = 30

ampDisturbance=0
myDesSpeed = [22.2222, 19.4444, 16.6667]

#myHeadwayTime = [1, 1.3, 1.6, 1.9]
myHeadwayTime = [1.9,1.6,1.3,1]

mySteadyGap = [[0 for i in range(4)] for j in range(3)]
for i in range(0,3):
    for j in range(0,4):
        mySteadyGap[i][j] = myDesSpeed[i]*myHeadwayTime[j] + vehLength + myMinGap

###################  define lane-change parameters ####################
myMinRespGap = 6
myfastSpdGap = 16
#defines the slope of the cut-in transition function
Ktg = 0.2
#cut-in transition duration 4 second
cutinDuration = 2

### participant start time for acceleration before the traffic light
startTime = 7

## define cut_in state
cutIn_action = ('CUT_NO', 'CUT_IN', 'CUT_OUT')
#################### define controller (CACC and FVDM)parameters   #######################################

myDesSpeedGain = 1.2

mySpeedControlGain=-0.4

myGapControlGainGap = 0.45
myGapControlGainGapDot = 0.0125
myCollisionAvoidanceGainGap = 0.45
myCollisionAvoidanceGainGapDot = 0.05
myGapClosingControlGainGap = 0.005
myGapClosingControlGainGapDot = 0.05
CACC_ControlMode=[0]

T_adpt = 5
Lamda = 0.65







def apply_PID(position, targetPosition, curAngle):
    """Apply the PID controller and return the angle command."""
    P = 0.005
    I = 0.0000002
    D = 0.12
    #P = 2
    #I = 0
    #D = 0
    K = 0    
    diff = position - targetPosition
    if apply_PID.previousDiff is None:
        apply_PID.previousDiff = diff
    # anti-windup mechanism
    if diff > 0 and apply_PID.previousDiff < 0:
        apply_PID.integral = 0
    if diff < 0 and apply_PID.previousDiff > 0:
        apply_PID.integral = 0
    apply_PID.integral += diff
    # compute angle
    angle = P * diff + I * apply_PID.integral + D * (diff - apply_PID.previousDiff)+K*curAngle
    apply_PID.previousDiff = diff
    if angle >0.1:
        angle = 0.1
    if angle <-0.1:
        angle = -0.1

    return angle


apply_PID.integral = 0
apply_PID.previousDiff = None


def getLeaderInfo(radar, singleGap, mySpeed, lane_index, myOriginSeq):
    numVeh=radar.getNumberOfTargets()
    preVeh=False
    gap=77
    speedDiff=myMaxSpeed-mySpeed
    if numVeh>0:
        preObj=radar.getTargets()
        for i in range(0,numVeh):
            #print("preObj[i].distance",preObj[i].distance, "preObj[i].azimuth", preObj[i].azimuth, "lane_index", lane_index,"myOriginSeq",myOriginSeq)
            if (preObj[i].distance<gap and abs(preObj[i].azimuth) < 0.04) or preObj[i].distance<singleGap:
            #if preObj[i].distance<gap:
                preVeh=True
                gap = preObj[i].distance
                speedDiff=preObj[i].speed
                
    return preVeh, gap, speedDiff


def speedSpeedControl(speed, vErr, lane_change_action):
    sclAccel = mySpeedControlGain * vErr
    newSpeed = speed + sclAccel
    return newSpeed
    

def speedGapControl(preVeh, gap2pred, predSpeed, speed, vErr, accel, lane_change_action, time_step, lane_index):
    if preVeh:
        desHeadwayTime = myHeadwayTime[lane_index]
        if lane_change_action == 'CUT_IN':
            #calculate based on equation (2), reference [1], but optimise in order to smooth the deceleration start phase.
            if speed>0.001:
                cutinHeadwayTime = gap2pred/speed
            else:
                cutinHeadwayTime = 5
            desHeadwayTime = cutinHeadwayTime + time_step/(1000*cutinDuration)*(myHeadwayTime[lane_index]-cutinHeadwayTime)
            #print("In cut_in, gap ", gap2pred, ", myspeed", speed, ", preSpeed ", predSpeed, "desired headway", desHeadwayTime)
        
        desSpacing = desHeadwayTime * speed
        gap = gap2pred - myMinGap
        spacingErr = gap - desSpacing
        #######original spacingErr1 = predSpeed - speed + myHeadwayTime * accel;##########
        #spacingErr1 = predSpeed - speed + myHeadwayTime * accel
        spacingErr1 = -vErr - desHeadwayTime * accel
        #print("desHeadwayTime", desHeadwayTime,"desSpacing", desSpacing, "gap",gap)

        if spacingErr > 0 and spacingErr < 0.2 and vErr < 0.1:#gap mode newSpeed = speed + 0.45 * spacingErr + 0.0125 *spacingErr1;
            newSpeed = speed + myGapControlGainGap * spacingErr + myGapControlGainGapDot * spacingErr1
        elif spacingErr < 0:# collision avoidance mode newSpeed = speed + 0.45 * spacingErr + 0.05 *spacingErr1;
            newSpeed = speed + myCollisionAvoidanceGainGap * spacingErr + myCollisionAvoidanceGainGapDot * spacingErr1
        else:# gap closing mode 
            newSpeed = speed + myGapClosingControlGainGap * spacingErr + myGapClosingControlGainGapDot * spacingErr1
        
    else:
        newSpeed = speedSpeedControl(speed, vErr, lane_change_action)
    
    
    return newSpeed



def speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action, time_step, lane_index):
    if preVeh:
        desHeadwayTime = myHeadwayTime[lane_index]
        if lane_change_action == 'CUT_IN':
            #calculate based on equation (2), reference [1], but optimise in order to smooth the deceleration start phase.
            if speed>0.001:
                cutinHeadwayTime = gap2pred/speed
            else:
                cutinHeadwayTime = 10
            desHeadwayTime = cutinHeadwayTime + time_step/(1000*cutinDuration)*(myHeadwayTime[lane_index]-cutinHeadwayTime)
            #print("In cut_in, gap ", gap2pred, ", myspeed", speed, ", preSpeed ", predSpeed, "desired headway", desHeadwayTime)
            
        gap = gap2pred - myMinGap
        optSpeed = max(0, min(myMaxSpeed,gap/desHeadwayTime))
        
        accel = (optSpeed-speed)/T_adpt + Lamda*(predSpeed-speed)
        
        newSpeed = speed + accel*time_step/1000
        
      
    else:
        newSpeed = speedSpeedControl(speed, vErr, lane_change_action)
    
    
    return newSpeed



def speed_CACC(preVeh, gap2pred, predSpeed, speed, desSpeed, accel,CACC_ControlMode, lane_change_action,time_step, lane_index):
    newSpeed = 0.0
    if preVeh:
        vErr = speed - predSpeed
    else:
        vErr = speed - myMaxSpeed
    #print("speedDiff", speedDiff, "vErr", vErr)
        
    if speed >0.001:
        time_gap = gap2pred / speed
    else:
        time_gap = 5
    if time_gap > 3:
        #newSpeed = speedSpeedControl(speed, vErr, lane_change_action)
        newSpeed = speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action,time_step, lane_index)

        CACC_ControlMode[0] = 0
    elif time_gap <= 2:
        #newSpeed = speedGapControl(preVeh, gap2pred, predSpeed, speed, vErr, accel,lane_change_action,time_step, lane_index)
        newSpeed = speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action,time_step, lane_index)
        CACC_ControlMode[0] = 1
    else:
        if CACC_ControlMode[0]==0:
            #newSpeed = speedSpeedControl(speed, vErr,lane_change_action)
            newSpeed = speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action,time_step, lane_index)
            
        else:
            #newSpeed = speedGapControl(preVeh, gap2pred, predSpeed, speed, vErr, accel, lane_change_action,time_step, lane_index)
            newSpeed = speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action,time_step, lane_index)


        
    return newSpeed


def get_filtered_speed(speed):
    """Filter the speed ommand to avoid abrupt speed changes."""
    get_filtered_speed.previousSpeeds.append(speed)
    if len(get_filtered_speed.previousSpeeds) > 100:  # keep only 80 values
        get_filtered_speed.previousSpeeds.pop(0)
    return sum(get_filtered_speed.previousSpeeds) / float(len(get_filtered_speed.previousSpeeds))

get_filtered_speed.previousSpeeds = []


driver = Driver()
veh = driver.getSelf()
controllerName = veh.getField("controller")
print("current controller is ",controllerName.getSFString())
myPosition = veh.getField("translation")




time_step = int(driver.getBasicTimeStep()) 

radar = driver.getRadar("radar")
radar.enable(time_step)

#print("radar", radar)

gps = driver.getGPS("gps")
gps.enable(time_step)

recv = driver.getReceiver("receiver")
recv.enable(time_step)

driver.step()

#ring road number 
ringNum = 0
cutIn_on = False                
cutinHeadwayTime0 = 0
numStep=0
lane_index = 0 
#obtain original position of the first ringroad.
isOriginPos = True
myOriginPos_x = 0
myOriginPos_z = 0
myOriginSeq = 0

myPreSpeed = 0


## set initial traffic light with red
trafficLight = driver.getFromDef("TL_" + str(ringNum))
tlPosition=trafficLight.getField("translation").getSFVec3f()[2]
#print("traffic light location is ", tlPosition)
isTLinitialized =False
## define logic if participant is allowed to enter the main road. (in case the first vehicle overtakes the participant)
isReadyToEnter = False


numStep_required=200000
Time_waiting_hreshold = WAITING_TIME_FOR_RUN/time_step   
if numStep <numStep_required:
    numStep += 1
else:  
    while driver.step() != -1:

            myCurSpeed = gps.getSpeed()
            if myCurSpeed<200 and myCurSpeed>=0:
                #here myGps_z is the GPS postion of vehicle, different from vehicle position in Webots. GPS offset is needed.
                myGps_x = gps.getValues()[0]
                myGps_z = gps.getValues()[2]
                lane_index = int((myGps_x - xoffsetLane[ringNum])/laneWidth)   
                if isOriginPos:
                    myOriginPos_x = myGps_x
                    myOriginPos_z = myGps_z -gpsZOffset
                    #obtain the sequence of CAV.
                    myOriginSeq = int(round((myOriginPos_z - zoffsetRoad[0])/myOriginGap))
                    isOriginPos = False

                if (not isTLinitialized) and myOriginSeq == 0 and lane_index == 0:
                    trafficLight.getField("controller").setSFString("traffic_light_red")
                    trafficLight.getField("controllerArgs").setSFString("2000 0.1 r")
                    #trafficLight.getField("controller").setSFString("traffic_light_red")

                    isTLinitialized=True

            


                if (not isReadyToEnter) and myOriginSeq == vehNumber_Lane-1 and lane_index == 0 and myGps_z >= tlPosition-startTime*myCurSpeed:
               
                    trafficLight.getField("controller").setSFString("traffic_light_green")
                    trafficLight.getField("controllerArgs").setSFString("0.1 2000 g")
                    #trafficLight.getField("controller").setSFString("traffic_light_green")

                    isReadyToEnter = True
                    print("ready to enter the main road")
                    #threading.Thread(target=playsound, args=('Auto.mp3',), daemon=True).start() #AutO
                    
                #decide which lane the vehicle is located in.
                singleGap = myDesSpeed[ringNum]*myHeadwayTime[lane_index] + myMinGap
                preVeh, gap2pred, speedDiff = getLeaderInfo(radar, singleGap, myCurSpeed,lane_index,  myOriginSeq)
                #print("preVeh", preVeh,"gap2pred", gap2pred, "speedDiff",speedDiff,"lane_index", lane_index, "myOriginSeq",myOriginSeq )
                gap2pred = gap2pred - 0.5*vehLength
                predSpeed = speedDiff + myCurSpeed
                myAccel = (myCurSpeed-myPreSpeed)/(time_step/1000)

                myPreSpeed = myCurSpeed

                #myDesSpeed = myDesSpeed+ampDisturbance*math.sin(numStep/100)
                #if (driver.getTime() == 40):
                    #myDesSpeed =0
                lane_change_action = cutIn_action[0]
                # check the message and implement cut in response
                if recv.getQueueLength()>0:
                    message=recv.getData()
                    dataList=struct.unpack(b"4s4d",message)
                    #str=''.join(dataList)
                    recv.nextPacket()

                    if cutIn_on and (message.find(b"LCLL")>=0 or message.find(b"LCRL")>=0):
                        cutIn_on = False                
                        print("Lane change finished at position", myGps_z)

                    ### judge if received the first LCL message in the target area
                    if (not cutIn_on) and (message.find(b"LCLF")>=0 or message.find(b"LCLN")>=0) and dataList[4]<=(myGps_z+gap2pred+1.5*vehLength)\
                    and ((dataList[4]> myGps_z+vehLength+myfastSpdGap) or (dataList[4]> myGps_z+vehLength+myMinRespGap and dataList[1]-myCurSpeed<2) )\
                    and dataList[2]>=lanePositions[ringNum][lane_index]-0.9*laneWidth and dataList[2]<=lanePositions[ringNum][lane_index]-0.4*laneWidth:
                        cutIn_on = True
                        cutinHeadwayTime0 = (dataList[4]-myGps_z-vehLength)/myCurSpeed
                        print("Receive message of LC to left at position", myGps_z, ", position of participant vehicle is ", dataList[4])

                    
                    if cutIn_on and message.find(b"LCL")>=0:
                        if dataList[2]<lanePositions[ringNum][lane_index]+0.5*laneWidth:
                            ## CUT-IN: HD vehicle at the right side of egovehicle, and wants to do left lane-change. additional 0.5laneWidth is to avoid over-lane-change.
                            ## The hd vehicle is the leader vehicle of egovehicle
                            lane_change_action = cutIn_action[1]
                            preVeh = True
                            gap2pred = dataList[4]-myGps_z-vehLength
                            speedDiff = dataList[1]-myCurSpeed
                            predSpeed = dataList[1]
                        else:
                            ## CUT-OUT: HD vehicle has left the current lane and is doing left lane-change, cut_in action finished at the current lane.    
                            lane_change_action = cutIn_action[2]
                            cutIn_on = False  
                    

                    ### judge if received the first LCR message in the target area
                    if (not cutIn_on) and (message.find(b"LCRF")>=0 or message.find(b"LCRN")>=0) and dataList[4]<=(myGps_z+gap2pred+1.5*vehLength) \
                    and ((dataList[4]> myGps_z+vehLength+myfastSpdGap) or (dataList[4]> myGps_z+vehLength+myMinRespGap  and dataList[1]-myCurSpeed<2 ))\
                    and dataList[2]>=lanePositions[ringNum][lane_index]+0.4*laneWidth and dataList[2]<=lanePositions[ringNum][lane_index]+0.9*laneWidth:
                        cutIn_on = True
                        cutinHeadwayTime0 = (dataList[4]-myGps_z-vehLength)/myCurSpeed
                        print("Receive message of LC to Right at position", myGps_z, ", position of participant vehicle is ", dataList[4])

                    if cutIn_on and message.find(b"LCR")>=0:
                        if dataList[2]>lanePositions[ringNum][lane_index]-0.5*laneWidth:
                            ## CUT-IN: HD vehicle at the right side of egovehicle, and wants to do left lane-change. additional 0.5laneWidth is to avoid over-lane-change.
                            ## The hd vehicle is the leader vehicle of egovehicle
                            lane_change_action = cutIn_action[1]
                            preVeh = True
                            gap2pred = dataList[4]-myGps_z-vehLength
                            speedDiff = dataList[1]-myCurSpeed
                            predSpeed = dataList[1]
                        else:
                            ## CUT-OUT: HD vehicle  has left the current lane and is doing right lane-change, cut_in action finished at the current lane.    
                            lane_change_action = cutIn_action[2]
                            cutIn_on = False  

                    #dataList[4]=0
                    #dataList[3]=0
                    #dataList[2]=0
                    #dataList[1]=0  

                #implement desired longitudinal speed.
                newSpeed=speed_CACC(preVeh, gap2pred, predSpeed, myCurSpeed, myDesSpeed, myAccel,CACC_ControlMode, lane_change_action, time_step, lane_index)
                # desired speed of leader is constant.
                if myOriginSeq ==vehNumber_Lane-1:
                    newSpeed = myDesSpeed[ringNum]
                
                if myGps_z <= zoffsetRoadEnd[ringNum]+gpsZOffset+mySteadyGap[ringNum][numLane-1]*(vehNumber_Lane-1)-(vehNumber_Lane-myOriginSeq-1)*mySteadyGap[ringNum][lane_index] and ringNum <=2 and numStep>=Time_waiting_hreshold:
                    driver.setCruisingSpeed(newSpeed*3.6)
                
                
                # calculate desired angle 
                curAngle=round(driver.getSteeringAngle(), 3)
                desAngle = apply_PID(round(myGps_x, 4), lanePositions[ringNum][lane_index], curAngle)
                #print("current angle is ",curAngle,"desired angle is ", desAngle,"myGps_x is ", round(myGps_x, 2),"myPosition_Z is ", round(myGps_z, 2),  "lane postion is ", lanePositions[lane_index])
                if abs(desAngle-curAngle)>0.000001:
                    driver.setSteeringAngle(desAngle)
         
                if myGps_z > zoffsetRoadEnd[ringNum]+gpsZOffset+mySteadyGap[ringNum][numLane-1]*(vehNumber_Lane-1)-(vehNumber_Lane-myOriginSeq-1)*mySteadyGap[ringNum][lane_index] and ringNum <2: #\
                #and myOriginSeq == 0 and lane_index == numLane-1 and ringNum <2:
                    driver.setCruisingSpeed(0)
                    if myCurSpeed <= 0.1:
                        ringNum += 1
                        myPosition.setSFVec3f([lanePositions[ringNum][lane_index], 0.4, zoffsetRoad[ringNum]+myOriginSeq*myOriginGap])
                        trafficLight = driver.getFromDef("TL_" + str(ringNum))
                        tlPosition = trafficLight.getField("translation").getSFVec3f()[2]

                        trafficLight.getField("controller").setSFString("traffic_light_red")
                        trafficLight.getField("controllerArgs").setSFString("2000 0.1 r")
                        isReadyToEnter = False

            numStep += 1
            
           
            #print("position", gps.getValues()[2],"gap", gap, "speed difference", speedDiff)

