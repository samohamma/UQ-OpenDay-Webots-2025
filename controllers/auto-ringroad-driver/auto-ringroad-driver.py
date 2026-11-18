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




import sys
sys.path.append(r'c:\users\uqsmoha8\appdata\roaming\python\python37\site-packages')
import sys
print("Python version:", sys.version)
print("Python executable:", sys.executable)
print("Python path:", sys.path)
#from playsound2 import playsound
from playsound import playsound

"""vehicle_driver controller."""
import sys
print(sys.executable)
from vehicle import Driver
from controller import Supervisor, Robot, Node, RadarTarget, Radar, Receiver, Emitter, LED, Joystick, Display, ImageRef
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


import os

def clear_console():
    # For Windows
    if os.name == 'nt':
        os.system('cls')
    # For Linux and MacOS
    else:
        os.system('clear')

clear_console()
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


def getLeaderInfo(radar, singleGap, mySpeed, ctrlPara):
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
        gap = gap2pred - ctrlPara.myMinGap
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
        #print("fvdm-newspeed", newSpeed)
      
    else:
        #newSpeed = speedSpeedControl(speed, vErr, lane_change_action,ctrlPara.mySpeedControlGain)
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
        #print("fvdm-newspeed", newSpeed)
    
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
        newSpeed = speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action,time_step, myHeadwayTime,ctrlPara)

        CACC_ControlMode[0] = 0
    elif time_gap <= 2:
        #newSpeed = speedGapControl(preVeh, gap2pred, predSpeed, speed, vErr, accel,lane_change_action,time_step, myHeadwayTime,ctrlPara)
        newSpeed = speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action,time_step, myHeadwayTime,ctrlPara)
        CACC_ControlMode[0] = 1
    else:
        if CACC_ControlMode[0]==0:
            #newSpeed = speedSpeedControl(speed, vErr,lane_change_action)
            newSpeed = speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action,time_step, myHeadwayTime,ctrlPara)
            
        else:
            #newSpeed = speedGapControl(preVeh, gap2pred, predSpeed, speed, vErr, accel, lane_change_action,time_step, myHeadwayTime, ctrlPara)
            newSpeed = speedGapFVDM(preVeh, gap2pred, predSpeed, speed, vErr, lane_change_action,time_step, myHeadwayTime,ctrlPara)


        
    return newSpeed
    

def display_update(display, speed, pic):
    display.imagePaste(pic, 0, 0, False)
    display.drawText("{:.1f}".format(speed*3.6)+' Km/h', 150, 30)


# play message and return messageId& switchHD
def playMessage(myGps_z, ctrlPara, toPlayMessageId, ringNum):

    messageId = ''
    nextMessageId = toPlayMessageId
    if  toPlayMessageId == 'Msg0' and myGps_z >= ctrlPara.RR1_TLonrmp_trans_z+(ctrlPara.zoffsetRoad[1]-ctrlPara.zoffsetRoad[0])*ringNum-490:
        messageId = 'Msg0'
        nextMessageId = 'Msg1'

    if  toPlayMessageId == 'Msg4' and myGps_z >= ctrlPara.pos_hdTocad[ringNum]-ctrlPara.DeltaZ_Msg4:
        messageId = 'Msg4'
        if ctrlPara.RRX_NDRT[ringNum] ==1:
            nextMessageId = 'Msg5b'        
        else:
            nextMessageId = 'Msg5a'        
    if  toPlayMessageId == 'Msg5a' and myGps_z >= ctrlPara.pos_hdTocad[ringNum] + ctrlPara.DeltaZ_Msg5a:
        #messageId = 'Msg5a'
        nextMessageId = 'Msg6'        
    if  toPlayMessageId == 'Msg5b' and myGps_z >= ctrlPara.pos_hdTocad[ringNum] + ctrlPara.DeltaZ_Msg5b:
       # messageId = 'Msg5b'
        nextMessageId = 'Msg6'        
    if  toPlayMessageId == 'Msg6' and myGps_z >= ctrlPara.pos_cadTohd[ringNum] - ctrlPara.DeltaZ_Msg6:
        messageId = 'Msg6'
        print("Msg 6 should be played")
        nextMessageId = 'Msg7'        
    if  toPlayMessageId == 'Msg7' and myGps_z >= ctrlPara.pos_cadTohd[ringNum] + ctrlPara.DeltaZ_Msg7:
        messageId = 'Msg7'
        nextMessageId = 'Msg8' 
    if  toPlayMessageId == 'Msg8' and myGps_z >= ctrlPara.pos_diverge[ringNum]- ctrlPara.DeltaZ_Msg8:
        messageId = 'Msg8'


    if messageId != '':
        threading.Thread(target=playsound, args=('../soundPlay/'+ messageId +'.mp3',), daemon=True).start()
    
    
    return messageId, nextMessageId



def get_filtered_speed(speed):
    """Filter the speed ommand to avoid abrupt speed changes."""
    get_filtered_speed.previousSpeeds.append(speed)
    if len(get_filtered_speed.previousSpeeds) > 100:  # keep only 80 values
        get_filtered_speed.previousSpeeds.pop(0)
    return sum(get_filtered_speed.previousSpeeds) / float(len(get_filtered_speed.previousSpeeds))




##### program start here #########
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



## define cut_in state
cutIn_action = ('CUT_NO', 'CUT_IN', 'CUT_OUT')

#################### define controller (CACC and FVDM)parameters   #######################################

CACC_ControlMode=[0]


#################### define display information  #######################################
TurningLeftIcon_file = '../icon/left-arrow.png'
TurningRightIcon_file = '../icon/right-arrow.png'
StayonIcon_file = '../icon/up-arrow.png'

apply_PID.integral = 0
apply_PID.previousDiff = None

get_filtered_speed.previousSpeeds = []

###########define drivng mode switch button###############
SwitchMode = 23


driver = Driver()


veh = driver.getSelf()
controllerName = veh.getField("controller")
print("current controller is ",controllerName.getSFString())
myPosition = veh.getField("translation")


filename = time.strftime("%Y%m%d-%H%M%S")
id = veh.getField("name").getSFString()
print(id)
recFile = '../'+ 'PT'+'-'+filename+'.csv'
fp = open(recFile, 'a')

time_step = int(driver.getBasicTimeStep()) 


driver.setSteeringAngle(0.0)
driver.setCruisingSpeed(0)


joystick = driver.getJoystick()
joystick.enable(time_step)



radar = driver.getRadar("radar")
radar.enable(time_step)

#print("radar", radar)

gps = driver.getGPS("gps")
gps.enable(time_step)

recv = driver.getReceiver("receiver")
recv.enable(time_step)


display = driver.getDisplay("display")
display.setColor(0xFF0000)
display.setFont("Arial", 20, True)
StayonIcon = display.imageLoad(StayonIcon_file)
display.imagePaste(StayonIcon, 0, 0, False)

myTrafficLight = None


## select which traffic light should be controlled by participant vehicle
myTLzPos = [[ctrlPara.RR1_TLonrmp_trans_z, ctrlPara.RR1_TLImgnry_StnGo_trans_z, ctrlPara.RR1_TLImgnry_TakeOver_trans_z],\
					[ctrlPara.RR2_TLonrmp_trans_z, ctrlPara.RR2_TLImgnry_StnGo_trans_z, ctrlPara.RR2_TLImgnry_TakeOver_trans_z],\
					[ctrlPara.RR3_TLonrmp_trans_z, ctrlPara.RR3_TLImgnry_StnGo_trans_z, ctrlPara.RR3_TLImgnry_TakeOver_trans_z]]

TL_Name = [['TL_onrmp0', 'TL_img_StnGo0', 'TL_img_takeOver0'],\
									['TL_onrmp1', 'TL_img_StnGo1', 'TL_img_takeOver1'],\
									['TL_onrmp2', 'TL_img_StnGo2', 'TL_img_takeOver2']]
                                    
                
driver.step()





## initialize onramp traffic light with red state
if gps.getValues()[2]<myTLzPos[0][0]:
    for i in range (0,3):
        for j in range (0, 3):
            print ("myTLzPos",myTLzPos[i][j])
            myTrafficLight = driver.getFromDef(TL_Name[i][j])
            myTrafficLight.getField("controller").setSFString("traffic_light_red")
            myTrafficLight.getField("controllerArgs").setSFString("2000 0.1 r")
        
## set initial traffic light with red






toCtrlTLIndex = [3,3] 

for i in range(0,3):
    for j in range(0,3):
        if gps.getValues()[2]<myTLzPos[i][j]:
            myTrafficLight = driver.getFromDef(TL_Name[i][j])
            if myTrafficLight.getField("state").getSFString() == "red":
                toCtrlTLIndex[0]=i
                toCtrlTLIndex[1]=j
                print("hi",TL_Name[i][j],"",toCtrlTLIndex[0],toCtrlTLIndex[1],",", myTLzPos[toCtrlTLIndex[0]][toCtrlTLIndex[1]])
                break
    break            

if joystick.isConnected():
    print(joystick.getModel(),"detected")
    
    
#decide which ring road number is based on the current location of participant. 
ringNum = int((gps.getValues()[2]-ctrlPara.zoffsetRoad[0])/(ctrlPara.zoffsetRoad[1]-ctrlPara.zoffsetRoad[0]))

##### play message autodriving reminder #########
toPlayMessageId = ''

if gps.getValues()[2]<myTLzPos[0][0]:
    threading.Thread(target=playsound, args=('../soundPlay/Msg4.mp3',), daemon=True).start() #AutO
    toPlayMessageId = 'Msg0'
else:
    toPlayMessageId = 'Msg4'

### initialize messageId to be played first.






isReadyToAutodriving = False


numStep=0

myPreSpeed = 0

## initialise take over warning
takeoverWarning = False

mTimer=0
Time_Threshold = ctrlPara.switchWaitingTime/time_step


pvInfo = []
pv_brake_act = 0
pv_reaction_pv = 0


### take over driving 
isOnramp_takeover = False
isOnbroken_takeover = False



while driver.step() != -1:

    ## first step is used for sensor warming up 
    if numStep==0:
        preObj0=radar.getTargets()
        myCurSpeed = gps.getSpeed()
        myPreSpeed = myCurSpeed
        myGps_x = gps.getValues()[0]
        myGps_z = gps.getValues()[2]
    else:
        myCurSpeed = gps.getSpeed()
        if numStep%(ctrlPara.infoDispTime/time_step) ==0:
            display_update(display, myCurSpeed, StayonIcon)
        #here myGps_z is the GPS postion of vehicle, different from vehicle position in Webots. GPS offset is needed.
        myGps_x = gps.getValues()[0]
        myGps_z = gps.getValues()[2]
        ### decide the current ring road number
        if not isReadyToAutodriving:
            ringNum = int((gps.getValues()[2]-ctrlPara.zoffsetRoad[0])/(ctrlPara.zoffsetRoad[1]-ctrlPara.zoffsetRoad[0]))
            if ringNum>2:
                ringNum = 2
            if ringNum<0:
                ringNum = 0
            isReadyToAutodriving = True
           
        lane_index = int((myGps_x - ctrlPara.xoffsetLane[ringNum])/ctrlPara.laneWidth)
        
        if lane_index>3:
            lane_index = 3
        if lane_index<0:
            lane_index = 0

            
        #decide which lane the vehicle is located in.
        singleGap = ctrlPara.myDesSpeed[ringNum]*ctrlPara.myHeadwayTime[ringNum][lane_index] + ctrlPara.myMinGap
        preVeh, gap2pred, speedDiff = getLeaderInfo(radar, singleGap, myCurSpeed, ctrlPara)
        #print("preVeh", preVeh,"gap2pred", gap2pred, "speedDiff",speedDiff)
        gap2pred = gap2pred - 0.5*ctrlPara.vehLength
        predSpeed = speedDiff + myCurSpeed
        myAccel = (myCurSpeed-myPreSpeed)/(time_step/1000)

        myPreSpeed = myCurSpeed

        lane_change_action = cutIn_action[0]



        #implement desired longitudinal speed.
        newSpeed=speed_CACC(preVeh, gap2pred, predSpeed, myCurSpeed, myAccel,CACC_ControlMode, lane_change_action, time_step, ctrlPara.myHeadwayTime[ringNum][lane_index], ctrlPara)
        if (not math.isnan(newSpeed)):
            driver.setCruisingSpeed(newSpeed*3.6)
        
 
        
        
        
        #print(round(driver.getTime(),4),',', round(myGps_z, 4),',' , round(myCurSpeed,4),',',  round(myAccel,4),',' , round(gap2pred,4),',', round(speedDiff,4), file=fp)    

        
        # calculate desired angle 
        curAngle=round(driver.getSteeringAngle(), 3)
        desAngle = apply_PID(round(myGps_x, 4), ctrlPara.lanePositions[ringNum][lane_index], curAngle, ctrlPara)
        if abs(desAngle-curAngle)>0.000001 and myGps_z >= ctrlPara.zoffsetRoad[0]+1000:
            driver.setSteeringAngle(desAngle)

        #print("newSpeed", newSpeed,"desAngle", desAngle, "curAngle",curAngle)    



        messageId, toPlayMessageId = playMessage(myGps_z, ctrlPara, toPlayMessageId, ringNum)

        if toPlayMessageId == 'Msg1':
            mTimer += 1
        if joystick.getPressedButton() == SwitchMode or mTimer>=Time_Threshold:
           isOnramp_takeover = True
        if messageId == 'Msg6':
            #threading.Thread(target=playsound, args=('Takeover.mp3',), daemon=True).start()
            #sOnramp_takeover = True
            print("ok")    
        if toPlayMessageId == 'Msg6':
            mTimer += 1

###########################################


        # pvInfo.append(round(driver.getTime(),4))
        # pvInfo.append(lane_index)
        # pvInfo.append('Road ID')
        # pvInfo.append( round(myGps_x,4))
        # pvInfo.append(  round(myGps_z-ctrlPara.gpsZOffset,4))
        # pvInfo.append(round(myCurSpeed,4))
        # pvInfo.append(id)
        # pvInfo.append('Leader id')
        # pvInfo.append('follower id')
        # pvInfo.append('neighbour leader id')        
        # pvInfo.append('neighbour id')        
        # pvInfo.append('neighbour follower id')       
        # pvInfo.append(messageId)   


        if myGps_z >= ctrlPara.pos_cadTohd[ringNum]-ctrlPara.DeltaZ_Msg6:   
            mTimer += 1
            if not takeoverWarning:
                threading.Thread(target=playsound, args=('Msg6.mp3',), daemon=True).start()
   #             print("this piece of code is activated")
                takeoverWarning = True
            if joystick.getPressedButton() == SwitchMode or mTimer>=Time_Threshold:                   
                isOnbroken_takeover=True
#                fp.close()
#               controllerName.setSFString("racing_wheel_com")   
                
    #        
      #          threading.Thread(target=playsound, args=('Takeover.mp3',), daemon=True).start()
       #         takeoverWarning = True


        #update traffic light status
        if(toCtrlTLIndex[1]==0 and myGps_z >= myTLzPos[toCtrlTLIndex[0]][0]-ctrlPara.DeltaZ_TL_onramp)\
        or(toCtrlTLIndex[1]==1 and myGps_z>=myTLzPos[toCtrlTLIndex[0]][1]-ctrlPara.DeltaZ_TL_stopngo)\
        or(toCtrlTLIndex[1]==2 and myGps_z>=myTLzPos[toCtrlTLIndex[0]][2]-ctrlPara.DeltaZ_TL_takeover):
            print ("myGps_z:",myGps_z, "myTLzPos",myTLzPos[toCtrlTLIndex[0]][toCtrlTLIndex[1]], "DeltaZ_TL_takeover",ctrlPara.DeltaZ_TL_takeover)
            myTrafficLight.getField("controller").setSFString("traffic_light_green")
            myTrafficLight.getField("controllerArgs").setSFString("0.1 2000 g")
            temp = toCtrlTLIndex[0]*3+toCtrlTLIndex[1]+1
            toCtrlTLIndex[0] = int(temp/3)
            toCtrlTLIndex[1] = int(temp%3)
            myTrafficLight = driver.getFromDef(TL_Name[toCtrlTLIndex[0]][toCtrlTLIndex[1]])
    
    

        
        if isOnbroken_takeover or isOnramp_takeover:
        
            pvInfo.append(2)## button id is 2

            print(pvInfo, file=fp)    
            fp.close()

            controllerName.setSFString("racing_wheel_com")
        else:
        
            pvInfo.append(0)## button id is 0
            print(pvInfo, file=fp)        
    
    pvInfo = []
    numStep += 1