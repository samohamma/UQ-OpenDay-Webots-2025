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
import sys
sys.path.append(r'c:\users\uqsmoha8\appdata\roaming\python\python37\site-packages')

from vehicle import Driver
from controller import Supervisor, Robot, Node, RadarTarget, Radar, Receiver, Emitter, LED
import array
import struct
import time
import numpy as np
import math
#from playsound2 import playsound
import threading
import csv
import sys 
sys.path.append("..") 
from ContrlPara import ContrlPara


def apply_PID(position, targetPosition, curAngle, ctrlPara):
    """Apply the PID controller and return the angle command."""

    P = ctrlPara.LaneChg_PID_P
    I = ctrlPara.LaneChg_PID_I
    D = ctrlPara.LaneChg_PID_D
    K = ctrlPara.LaneChg_PID_K    
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



def getLeaderInfo(radar, singleGap, mySpeed,  ctrlPara):
    numVeh=radar.getNumberOfTargets()
    preVeh=False
    gap=ctrlPara.maxDetectRange
    speedDiff=ctrlPara.myMaxSpeed-mySpeed
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


def speedSpeedControl(speed, vErr, mySpeedControlGain):
    sclAccel = mySpeedControlGain * vErr
    newSpeed = speed + sclAccel
    return newSpeed
    

def speedGapControl(preVeh, gap2pred, predSpeed, speed, vErr, accel, lane_change_action, time_step, myHeadwayTime, ctrlPara):
    if preVeh:
        desHeadwayTime = myHeadwayTime
        if lane_change_action == 'CUT_IN':
            #calculate based on equation (2), reference [1], but optimise in order to smooth the deceleration start phase.
            if speed>0.001:
                cutinHeadwayTime = gap2pred/speed
            else:
                cutinHeadwayTime = 5
            desHeadwayTime = cutinHeadwayTime + time_step/(1000*ctrlPara.cutinDuration)*(myHeadwayTime-cutinHeadwayTime)
            #print("In cut_in, gap ", gap2pred, ", myspeed", speed, ", preSpeed ", predSpeed, "desired headway", desHeadwayTime)
        
        desSpacing = desHeadwayTime * speed
        gap = gap2pred - myMinGap
        spacingErr = gap - desSpacing
        #######original spacingErr1 = predSpeed - speed + myHeadwayTime * accel;##########
        #spacingErr1 = predSpeed - speed + myHeadwayTime * accel
        spacingErr1 = -vErr - desHeadwayTime * accel
        #print("desHeadwayTime", desHeadwayTime,"desSpacing", desSpacing, "gap",gap)

        if spacingErr > 0 and spacingErr < 0.2 and vErr < 0.1:#gap mode newSpeed = speed + 0.45 * spacingErr + 0.0125 *spacingErr1;
            newSpeed = speed + ctrlPara.myGapControlGainGap * spacingErr + ctrlPara.myGapControlGainGapDot * spacingErr1
        elif spacingErr < 0:# collision avoidance mode newSpeed = speed + 0.45 * spacingErr + 0.05 *spacingErr1;
            newSpeed = speed + ctrlPara.myCollisionAvoidanceGainGap * spacingErr + ctrlPara.myCollisionAvoidanceGainGapDot * spacingErr1
        else:# gap closing mode 
            newSpeed = speed + ctrlPara.myGapClosingControlGainGap * spacingErr + ctrlPara.myGapClosingControlGainGapDot * spacingErr1
        
    else:
        newSpeed = speedSpeedControl(speed, vErr, ctrlPara.mySpeedControlGain)
    
    
    return newSpeed



def speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action, time_step, myHeadwayTime, ctrlPara):
    if preVeh:
        desHeadwayTime = myHeadwayTime
        if lane_change_action == 'CUT_IN':
            #calculate based on equation (2), reference [1], but optimise in order to smooth the deceleration start phase.
            if speed>0.001:
                cutinHeadwayTime = gap2pred/speed
            else:
                cutinHeadwayTime = 10
            desHeadwayTime = cutinHeadwayTime + time_step/(1000*ctrlPara.cutinDuration)*(myHeadwayTime-cutinHeadwayTime)
            #print("In cut_in, gap ", gap2pred, ", myspeed", speed, ", preSpeed ", predSpeed, "desired headway", desHeadwayTime)
            
        gap = gap2pred - ctrlPara.myMinGap
        optSpeed = max(0, min(ctrlPara.myMaxSpeed,gap/desHeadwayTime))
        
        accel = (optSpeed-speed)/ctrlPara.T_adpt + ctrlPara.Lamda*(predSpeed-speed)
        if accel>ctrlPara.maxAccel:
            accel = ctrlPara.maxAccel
        if accel<ctrlPara.minAccel:
            accel = ctrlPara.minAccel
            
        newSpeed = speed + accel*time_step/1000
        
      
    else:
        newSpeed = speedSpeedControl(speed, vErr, ctrlPara.mySpeedControlGain)
    
    
    return newSpeed



def speed_CACC(preVeh, gap2pred, predSpeed, speed, accel,CACC_ControlMode, lane_change_action,time_step, myHeadwayTime, ctrlPara):
    newSpeed = 0.0
    if preVeh:
        vErr = speed - predSpeed
    else:
        vErr = speed - ctrlPara.myMaxSpeed
    #print("speedDiff", speedDiff, "vErr", vErr)
        
    if speed >0.001:
        time_gap = gap2pred / speed
    else:
        time_gap = 5
    if time_gap > 3:
        #newSpeed = speedSpeedControl(speed, vErr, lane_change_action)
        newSpeed = speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action,time_step, myHeadwayTime, ctrlPara)

        CACC_ControlMode[0] = 0
    elif time_gap <= 2:
        #newSpeed = speedGapControl(preVeh, gap2pred, predSpeed, speed, vErr, accel,lane_change_action,time_step, myHeadwayTime,ctrlPara)
        newSpeed = speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action,time_step, myHeadwayTime, ctrlPara)
        CACC_ControlMode[0] = 1
    else:
        if CACC_ControlMode[0]==0:
            #newSpeed = speedSpeedControl(speed, vErr,lane_change_action)
            newSpeed = speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action,time_step, myHeadwayTime,ctrlPara)
            
        else:
            #newSpeed = speedGapControl(preVeh, gap2pred, predSpeed, speed, vErr, accel, lane_change_action,time_step, myHeadwayTime, ctrlPara)
            newSpeed = speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action,time_step, myHeadwayTime,ctrlPara)


        
    return newSpeed


def get_filtered_speed(speed):
    """Filter the speed ommand to avoid abrupt speed changes."""
    get_filtered_speed.previousSpeeds.append(speed)
    if len(get_filtered_speed.previousSpeeds) > 100:  # keep only 80 values
        get_filtered_speed.previousSpeeds.pop(0)
    return sum(get_filtered_speed.previousSpeeds) / float(len(get_filtered_speed.previousSpeeds))





time.sleep(2)


###import system parameters

ctrlPara = ContrlPara()
paraFilename = '../Controller_input_param.csv'
#read csv, and split on "," the line
P_File = open(paraFilename, "r")
paraFile = csv.reader(P_File, delimiter=",")
ctrlPara.loadVehInfo(paraFile)
P_File.seek(0)
paraFile = csv.reader(P_File, delimiter=",")
ctrlPara.loadGeoPara(paraFile)
P_File.seek(0)
paraFile = csv.reader(P_File, delimiter=",")
ctrlPara.loadVehPosPara(paraFile)
P_File.seek(0)
paraFile = csv.reader(P_File, delimiter=",")
ctrlPara.load_TL_PosPara(paraFile)
P_File.seek(0)
paraFile = csv.reader(P_File, delimiter=",")
ctrlPara.loadTimeSyn(paraFile)
P_File.seek(0)
paraFile = csv.reader(P_File, delimiter=",")
ctrlPara.loadCtrlPara(paraFile)





driver = Driver()
veh = driver.getSelf()
controllerName = veh.getField("controller")
print("current controller is ",controllerName.getSFString())
myPosition = veh.getField("translation")


### save results to file
filename = time.strftime("%Y%m%d-%H%M")
id = veh.getField("name")
recFile = '../'+ 'AV'+'-'+filename+'.csv'
fp = open(recFile, 'a')


time_step = int(driver.getBasicTimeStep()) 

radar = driver.getRadar("radar")
radar.enable(time_step)

#print("radar", radar)

gps = driver.getGPS("gps")
gps.enable(time_step)

recv = driver.getReceiver("receiver")
recv.enable(time_step)

driver.step()



#######initialize status

apply_PID.integral = 0
apply_PID.previousDiff = None
## define cut_in state
cutIn_action = ('CUT_NO', 'CUT_IN', 'CUT_OUT')
CACC_ControlMode=[0]
get_filtered_speed.previousSpeeds = []
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
trafficLight = driver.getFromDef("TL_img_StnGo" + str(ringNum))
tlPosition=trafficLight.getField("translation").getSFVec3f()[2]
print("traffic light location is ", tlPosition)
isTLinitialized =False
## define logic if participant is allowed to enter the main road. (in case the first vehicle overtakes the participant)
isReadyToEnter = False

avInfo = []
av_brake_act = 0
av_reaction_pv = 0

Veh_StnGo_turnGreen = False
Veh_StnGo_readtoCount = False
Veh_StnGo_Timer = 0

StnGo_originPos = [[[ctrlPara.RR1_Stop_nGoVeh_Lane1_trans_x,0.51,ctrlPara.RR1_Stop_nGoVeh_Lane1_trans_z],[ctrlPara.RR1_Stop_nGoVeh_Lane2_trans_x,0.51,ctrlPara.RR1_Stop_nGoVeh_Lane2_trans_z],[ctrlPara.RR1_Stop_nGoVeh_Lane3_trans_x,0.51,ctrlPara.RR1_Stop_nGoVeh_Lane3_trans_z],[ctrlPara.RR1_Stop_nGoVeh_Lane4_trans_x,0.51,ctrlPara.RR1_Stop_nGoVeh_Lane4_trans_z]],\
                   [[ctrlPara.RR2_Stop_nGoVeh_Lane1_trans_x,0.51,ctrlPara.RR2_Stop_nGoVeh_Lane1_trans_z],[ctrlPara.RR2_Stop_nGoVeh_Lane2_trans_x,0.51,ctrlPara.RR2_Stop_nGoVeh_Lane2_trans_z],[ctrlPara.RR2_Stop_nGoVeh_Lane3_trans_x,0.51,ctrlPara.RR2_Stop_nGoVeh_Lane3_trans_z],[ctrlPara.RR2_Stop_nGoVeh_Lane4_trans_x,0.51,ctrlPara.RR2_Stop_nGoVeh_Lane4_trans_z]],\
                   [[ctrlPara.RR3_Stop_nGoVeh_Lane1_trans_x,0.51, ctrlPara.RR3_Stop_nGoVeh_Lane1_trans_z],[ctrlPara.RR3_Stop_nGoVeh_Lane2_trans_x,0.51, ctrlPara.RR3_Stop_nGoVeh_Lane2_trans_z],[ctrlPara.RR3_Stop_nGoVeh_Lane3_trans_x,0.51, ctrlPara.RR3_Stop_nGoVeh_Lane3_trans_z],[ctrlPara.RR3_Stop_nGoVeh_Lane4_trans_x,0.51, ctrlPara.RR3_Stop_nGoVeh_Lane4_trans_z]]]


while driver.step() != -1:
    
    if not Veh_StnGo_turnGreen:
        if trafficLight.getField("state").getSFString() == 'green':
            Veh_StnGo_readtoCount = True
            Veh_StnGo_turnGreen = True
        else:
            numStep += 1
            continue
            
    if Veh_StnGo_readtoCount:
        if Veh_StnGo_Timer*time_step/1000<ctrlPara.DeltT_TL_stopngo:
            Veh_StnGo_Timer += 1
            numStep += 1
            continue 
        else:
            Veh_StnGo_Timer = 0
            Veh_StnGo_readtoCount = False
            
        
    myCurSpeed = gps.getSpeed()
    if myCurSpeed<200 and myCurSpeed>=0:
        #here myGps_z is the GPS postion of vehicle, different from vehicle position in Webots. GPS offset is needed.
        myGps_x = gps.getValues()[0]
        myGps_z = gps.getValues()[2]
        lane_index = int((myGps_x - ctrlPara.xoffsetLane[ringNum])/ctrlPara.laneWidth)   
        if isOriginPos:
            myOriginPos_x = myGps_x
            myOriginPos_z = myGps_z -ctrlPara.gpsZOffset
            #obtain the sequence of CAV.
            myOriginSeq = int(round((myOriginPos_z - ctrlPara.RR1_Stop_nGoVeh_Lane1_trans_z)/ctrlPara.myOriginGap))
            #print("myOriginSeq",myOriginSeq)
            isOriginPos = False



        #threading.Thread(target=playsound, args=('Auto.mp3',), daemon=True).start() #AutO
            
        #decide which lane the vehicle is located in.
        singleGap = ctrlPara.myDesSpeed[ringNum]*ctrlPara.myHeadwayTime[ringNum][lane_index] + ctrlPara.myMinGap
        preVeh, gap2pred, speedDiff = getLeaderInfo(radar, singleGap, myCurSpeed, ctrlPara)
        #print("preVeh", preVeh,"gap2pred", gap2pred, "speedDiff",speedDiff,"lane_index", lane_index, "myOriginSeq",myOriginSeq )
        gap2pred = gap2pred - 0.5*ctrlPara.vehLength
        predSpeed = speedDiff + myCurSpeed
        myAccel = (myCurSpeed-myPreSpeed)/(time_step/1000)
        if myAccel<0:
            av_brake_act = 1
        else:
            av_brake_act = 0
            
        myPreSpeed = myCurSpeed


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
            if (not cutIn_on) and (message.find(b"LCLF")>=0 or message.find(b"LCLN")>=0) and dataList[4]<=(myGps_z+gap2pred+1.5*ctrlPara.vehLength)\
            and ((dataList[4]> myGps_z+ctrlPara.vehLength+ctrlPara.myfastSpdGap) or (dataList[4]> myGps_z+ctrlPara.vehLength+ctrlPara.myMinRespGap and dataList[1]-myCurSpeed<2) )\
            and dataList[2]>=ctrlPara.lanePositions[ringNum][lane_index]-0.9*ctrlPara.laneWidth and dataList[2]<=ctrlPara.lanePositions[ringNum][lane_index]-0.4*ctrlPara.laneWidth:
                cutIn_on = True
                cutinHeadwayTime0 = (dataList[4]-myGps_z-ctrlPara.vehLength)/myCurSpeed
                av_reaction_pv = 1
                print("Receive message of LC to left at position", myGps_z, ", position of participant vehicle is ", dataList[4])

            
            if cutIn_on and message.find(b"LCL")>=0:
                if dataList[2]<ctrlPara.lanePositions[ringNum][lane_index]+0.5*ctrlPara.laneWidth:
                    ## CUT-IN: HD vehicle at the right side of egovehicle, and wants to do left lane-change. additional 0.5laneWidth is to avoid over-lane-change.
                    ## The hd vehicle is the leader vehicle of egovehicle
                    lane_change_action = cutIn_action[1]
                    preVeh = True
                    gap2pred = dataList[4]-myGps_z-ctrlPara.vehLength
                    speedDiff = dataList[1]-myCurSpeed
                    predSpeed = dataList[1]
                else:
                    ## CUT-OUT: HD vehicle has left the current lane and is doing left lane-change, cut_in action finished at the current lane.    
                    lane_change_action = cutIn_action[2]
                    cutIn_on = False  
            

            ### judge if received the first LCR message in the target area
            if (not cutIn_on) and (message.find(b"LCRF")>=0 or message.find(b"LCRN")>=0) and dataList[4]<=(myGps_z+gap2pred+1.5*ctrlPara.vehLength) \
            and ((dataList[4]> myGps_z+ctrlPara.vehLength+ctrlPara.myfastSpdGap) or (dataList[4]> myGps_z+ctrlPara.vehLength+ctrlPara.myMinRespGap  and dataList[1]-myCurSpeed<2 ))\
            and dataList[2]>=ctrlPara.lanePositions[ringNum][lane_index]+0.4*ctrlPara.laneWidth and dataList[2]<=ctrlPara.lanePositions[ringNum][lane_index]+0.9*ctrlPara.laneWidth:
                cutIn_on = True
                cutinHeadwayTime0 = (dataList[4]-myGps_z-ctrlPara.vehLength)/myCurSpeed
                av_reaction_pv = 1
                print("Receive message of LC to Right at position", myGps_z, ", position of participant vehicle is ", dataList[4])

            if cutIn_on and message.find(b"LCR")>=0:
                if dataList[2]>ctrlPara.lanePositions[ringNum][lane_index]-0.5*ctrlPara.laneWidth:
                    ## CUT-IN: HD vehicle at the right side of egovehicle, and wants to do left lane-change. additional 0.5laneWidth is to avoid over-lane-change.
                    ## The hd vehicle is the leader vehicle of egovehicle
                    lane_change_action = cutIn_action[1]
                    preVeh = True
                    gap2pred = dataList[4]-myGps_z-ctrlPara.vehLength
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
        newSpeed=speed_CACC(preVeh, gap2pred, predSpeed, myCurSpeed, myAccel,CACC_ControlMode, lane_change_action, time_step, ctrlPara.myHeadwayTime[ringNum][lane_index], ctrlPara)
        # desired speed of leader is constant.
        if myOriginSeq ==ctrlPara.Num_row_vehs_stopngo-1:
            newSpeed = ctrlPara.myDesSpeed[ringNum]
        
        if myGps_z <= ctrlPara.zoffsetRoadEnd[ringNum]+ctrlPara.gpsZOffset+ctrlPara.mySteadyGap[ringNum][ctrlPara.numLane-1]*(ctrlPara.vehNumber_Lane-1)-(ctrlPara.vehNumber_Lane-myOriginSeq-1)*ctrlPara.mySteadyGap[ringNum][lane_index] and ringNum <=ctrlPara.numRingRoad-1:
            driver.setCruisingSpeed(newSpeed*3.6)
        

        
        # calculate desired angle 
        curAngle=round(driver.getSteeringAngle(), 3)
        desAngle = apply_PID(round(myGps_x, 4), ctrlPara.lanePositions[ringNum][lane_index], curAngle, ctrlPara)
        #print("current angle is ",curAngle,"desired angle is ", desAngle,"myGps_x is ", round(myGps_x, 2),"myPosition_Z is ", round(myGps_z, 2),  "lane postion is ", lanePositions[lane_index])
        if abs(desAngle-curAngle)>0.00001:
            #print("current angle is ",curAngle,"desired angle is ", desAngle,"myGps_x is ", round(myGps_x, 2),"myPosition_Z is ", round(myGps_z, 2),  "lane postion is ", ctrlPara.lanePositions[ringNum][lane_index])
            driver.setSteeringAngle(desAngle)
 
        if myGps_z > ctrlPara.zoffsetRoadEnd[ringNum]+ctrlPara.gpsZOffset+ctrlPara.mySteadyGap[ringNum][ctrlPara.numLane-1]*(ctrlPara.vehNumber_Lane-1)-(ctrlPara.vehNumber_Lane-myOriginSeq-1)*ctrlPara.mySteadyGap[ringNum][lane_index] and ringNum <ctrlPara.numRingRoad-1: #\
        #and myOriginSeq == 0 and lane_index == numLane-1 and ringNum <2:
            driver.setCruisingSpeed(0)
            if myCurSpeed <= 0.1:
                ringNum += 1
                myPosition.setSFVec3f([StnGo_originPos[ringNum][lane_index][0], 0.51, StnGo_originPos[ringNum][lane_index][2]+myOriginSeq*ctrlPara.myOriginGap])
                trafficLight = driver.getFromDef("TL_img_StnGo" + str(ringNum))
                tlPosition = trafficLight.getField("translation").getSFVec3f()[2]

                Veh_StnGo_turnGreen = False

    

        # avInfo.append( round(driver.getTime(),4))
        # avInfo.append( lane_index)
        # avInfo.append( 'roadId')
        # avInfo.append( round(myGps_x,4))
        # avInfo.append(  round(myGps_z-ctrlPara.gpsZOffset,4))
        # avInfo.append(  round(myCurSpeed,4))
        # avInfo.append( id.getSFString())
        # avInfo.append( 'leaderId')
        # avInfo.append('followerId')
        # avInfo.append(av_brake_act)
        # avInfo.append(av_reaction_pv)



        # fp.write(','.join(str(line) for line in avInfo))
        # fp.write('\n')
        # #print(avInfo, file=fp)
        # avInfo = []
    
    numStep += 1
    
   
    #print("position", gps.getValues()[2],"gap", gap, "speed difference", speedDiff)

