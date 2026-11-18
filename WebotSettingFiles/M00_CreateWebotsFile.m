close all 
clearvars
clc

% M0_CreateParticipant_ID
M01_CreateRoadGeometry
M01b_CreateTrafficlights
M02_ParticipantsVehicle
M03_CreateWebotsCAVs
M04_CreateGraphicalFeatures


Current_directory=cd;
File='Geometry.txt';
file = fullfile(Current_directory, File);
[tempDir, tempFile] = fileparts(file)
status = copyfile(file, fullfile(tempDir, [tempFile, '.wbt']))


% fileID = fopen('A01-WebotsWorld.txt','w');

% Combine all components
system('copy A01-WorldSetting.txt + A01-Geometry.txt + A01-Trafficlights.txt + A01-ParticipantsVehicle.txt + A01-WebotsCAVs.txt A01-WebotsWorld.txt')


% Make it a readable file for webots

% st01- Remove the last line SUB created in the above command
fid = fopen('A01-WebotsWorld.txt','r');
i = 1;
tline = fgetl(fid);
A={};
A{i} = tline;
while ischar(tline)
    i = i+1;
    tline = fgetl(fid);
    A{i} = tline;
end
fclose(fid);
fid = fopen('A00-WebotsWorld.txt', 'w');
for i = 1:numel(A)-2
    if A{i+1} == -1
        fprintf(fid,'%s', A{i});
        break
    else
        fprintf(fid,'%s\n', A{i});
    end
end
fclose(fid)

Current_cd= cd
cd ../
main_cd =cd
new_cd=[main_cd,'\worlds']
cd (Current_cd)
copyfile('A00-WebotsWorld.wbt',new_cd)


% st02- Change file extension to .wbt
copyfile A00-WebotsWorld.txt A00-WebotsWorld.wbt

% Copy the new file to the worlds folder
% open A00-WebotsWorld.txt

%%
Current_cd= cd
cd ../
main_cd =cd
new_cd=[main_cd,'\worlds']
cd (Current_cd)
copyfile('A00-WebotsWorld.wbt',new_cd)

% new
%  cd 'C:\Program Files\MATLAB\R2017a\examples\matlab_featured'

% number of vehicle for each lane e.g. 
% 44 CAVs in total for 11 numVehicle_Lane with 4 lanes

%%% ###################### geometric parameter ######################

numRingRoad=Num_road;
numberOfLanes=prelim_info.numberOfLanes(prelim_info.CAV_starts==1);
laneWidth=prelim_info.Lane_width(prelim_info.CAV_starts==1);
startLength=prelim_info.Section_length(prelim_info.CAV_starts==1);
mergeLength=prelim_info.Section_length(strcmp(prelim_info.Type,...
    'LaneSeparation_merge')==1)+prelim_info.Section_length(strcmp(prelim_info.Type,...
    'LaneDrop')==1);
hdriveLength=prelim_info.Section_length(strcmp(prelim_info.Experiment_task,...
    'ParticipantDrivesSteadily')==1);
cadriveLength=prelim_info.Section_length(strcmp(prelim_info.Experiment_task,...
    'ParticipantAutoDriving')==1);
takeoverLength=prelim_info.Section_length(strcmp(prelim_info.Experiment_task,...
    'ParticipantTakeOverAndLC')==1);
divergeLength=prelim_info.Section_length(strcmp(prelim_info.Type,...
    'LaneExpansion')==1)+prelim_info.Section_length(strcmp(prelim_info.Type,...
    'LaneSeparation_diverge')==1);
Variable_name1={'numRingRoad';'numberOfLanes';'laneWidth';'startLength';...
    'mergeLength';'hdriveLength';'cadriveLength';'takeoverLength';'divergeLength'};
 


%% stable traffic speed (m/s) for three ringroads
%################### traffic flow parameter #########################
% original and minimum of inter-vehicle distance 
% (i.e. coordinate difference between two adjacent vehicles, unit: meter)
numVehicle_Lane=NumVehs;
OriginGap = 30;
minGap = 2;
%## stable traffic speed (m/s) for three ringroads

DesiredSpeed = [22.2222 19.4444 16.6667, 16.6667];
% Time headway for each lane from right to left

%%### Time ahead (second) for traffic light turning green before the arrival of the first CAV
clear HeadwayTime
HeadwayTime_Lane = [1.9 1.6 1.3 1];
HeadwayTime_RR=[];
for k=1:Num_road
HeadwayTime_RR=[HeadwayTime_RR,HeadwayTime_Lane];
end
startTime = 7;
%#define waiting time interval for participant to switch driving mode (unit: ms)

switchWaitingTime = 20000;

%############# display information update frequency (unit: ms, must be multiple basic time step) ############

infoDispTime = 200;
Variable_name1={'numRingRoad';'numberOfLanes';'laneWidth';'startLength';...
    'mergeLength';'hdriveLength';'cadriveLength';'takeoverLength';...
    'divergeLength';'numVehicle_Lane';'OriginGap';'minGap';...
    'switchWaitingTime';'infoDispTime'};
Variable_value1=[numRingRoad;numberOfLanes;laneWidth;startLength;...
    mergeLength;hdriveLength;cadriveLength;takeoverLength;...
    divergeLength;numVehicle_Lane;OriginGap;minGap;...
    switchWaitingTime;infoDispTime];

Variable_name2={};
Variable_value2=reshape(RR_translation,[size(RR_translation,1)*size(RR_translation,2),1]);
Variable_name3={};
Variable_value3=[DesiredSpeed';HeadwayTime_RR'];
for k=1:Num_road     
Variable_name2=[Variable_name2;...
   strrep(['RR',num2str(k),'_','translation_x'],' ','');...
   strrep(['RR',num2str(k),'_','translation_y'],' ','');...
   strrep(['RR',num2str(k),'_','translation_z'],' ','');...
   strrep(['DesiredSpeed_RR',num2str(k)],' ','')];
   for j=1:numberOfLanes
   Variable_name3=[Variable_name3;strrep(['HeadwayRR',num2str(k),'_','Lane_',num2str(j)],' ','')];
   end
end

% Variable_name3={'DesiredSpeed_RR1';'DesiredSpeed_RR2';'DesiredSpeed_RR3';...
%     'HeadwayRR1_Lane_1';'HeadwayRR1_Lane_1';'HeadwayRR1_Lane_1';'HeadwayRR1_Lane_1';...
%      'HeadwayRR2_Lane_1';'HeadwayRR2_Lane_1';'HeadwayRR2_Lane_1';'HeadwayRR2_Lane_1';...
%      'HeadwayRR3_Lane_1';'HeadwayRR3_Lane_1';'HeadwayRR3_Lane_1';'HeadwayRR3_Lane_1'};

Variable_name=[Variable_name1;Variable_name2;Variable_name3];
Variable_value=[Variable_value1;Variable_value2;Variable_value3];

T1=table(Variable_name,Variable_value);
writetable(T1,'Controller_input_param.csv','Delimiter',',');

Current_cd= cd
cd ../
main_cd =cd
new_cd=[main_cd,'\controllers']
cd (Current_cd)
copyfile('Controller_input_param.csv',new_cd)

%-----------------Create the variables needed for the auto-driving.py controller
Variable_name1={'numRingRoad';'numberOfLanes';'laneWidth';'startLength';...
    'mergeLength';'hdriveLength';'cadriveLength';'takeoverLength';...
    'divergeLength'};
xoffsetRoad=[];
zoffsetRoad=[];
xoffsetRoad_name={};
zoffsetRoad_name={};
for k=1:Num_road     
xoffsetRoad=[xoffsetRoad,RR_translation(k,1)];
zoffsetRoad=[zoffsetRoad,RR_translation(k,3)];

xoffsetRoad_name=[xoffsetRoad_name,...
   strrep(['xoffsetRoad_', num2str(k)],' ','')];
zoffsetRoad_name=[zoffsetRoad_name,...
   strrep(['zoffsetRoad_', num2str(k)],' ','')]
end

GeometricalParam_name=[Variable_name1;xoffsetRoad_name';zoffsetRoad_name'];
GeometricalParam_value=[numRingRoad,numberOfLanes,laneWidth,startLength,...
                        mergeLength,hdriveLength,cadriveLength,takeoverLength,...
                        divergeLength,xoffsetRoad,zoffsetRoad]';

T2=table(GeometricalParam_name,GeometricalParam_value);

% T2=table(Variable_name,Variable_value);
writetable(T2,'GeometricalParam.csv','Delimiter',',');

Current_cd= cd
cd ../
main_cd =cd
new_cd=[main_cd,'\controllers']
cd (Current_cd)
copyfile('GeometricalParam.csv',new_cd)

open A01-WebotsWorld.txt



% cd ..\
% 
% cd  worlds\