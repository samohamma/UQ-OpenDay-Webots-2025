close all
clearvars
clc

cd_world= cd;
cd ../;
cd_main =cd;
cd_participants_info=[cd_main,'\Participants_info'];
cd_matlabcodes=[cd_main,'\WebotSettingFiles'];
cd_controllers=[cd_main,'\controllers'];

cd(cd_matlabcodes);
M01_CreateRoadGeometry;
M01b_CreateTrafficlights;
M02_ParticipantsVehicle;
M03_CreateWebotsCAVs;
M04_CreateGraphicalFeatures;

File='Geometry.txt';
file = fullfile(cd, File);
[tempDir, tempFile] = fileparts(file);
status = copyfile(file, fullfile(tempDir, [tempFile, '.wbt']));

% Combine all components
system(['copy A01-WorldSetting.txt + A01-ParticipantsVehicle.txt+A01-WebotsCAVs.txt+ A01-Trafficlights.txt+ A01-Geometry.txt+ A01-Graphicalfeature.txt  A01-WebotsWorld.txt']);

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
copyfile A00-WebotsWorld.txt A00-WebotsWorld.wbt
copyfile('A00-WebotsWorld.wbt',cd_world);

cd(cd_world);

% open A00-WebotsWorld.wbt


N_participant=50;
DesiredSpeed = [22.2222 19.4444 16.6667, 12.6667]; % m/s
HeadwayTime_Lane = [1.9 1.6 1.3 1];


numVehicle_Lane=NumVehs;
switchWaitingTime = 20000; % time step each during 0.05s
infoDispTime = 200;

%- Traffic condition paramters:
OriginGap = 30;
minGap = 2;


%- FVDMs_paramters
Order_headway_each=zeros(Num_road,numberOfLanes);

random_participant=floor(1+N_participant*rand(1));


for i_participant=1:N_participant



    for i_experiment=1:2

        % define parameters that are dynamics through the controller parameters-
        Order_Speead_each=randperm(Num_road)
        rng(i_participant);
        for i_road=1:Num_road
            Order_Headway_each(i_road,:)=randperm(numberOfLanes);
        end

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



        % stable traffic speed (m/s) for three ringroads
        %################### traffic flow parameter #########################
        % original and minimum of inter-vehicle distance
        % (i.e. coordinate difference between two adjacent vehicles, unit: meter)

        %## stable traffic speed (m/s) for three ringroads

        % Time headway for each lane from right to left

        %%### Time ahead (second) for traffic light turning green before the arrival of the first CAVr
        HeadwayTime_RR=[];
        for i_road=1:Num_road

        end
        startTime = 7;
        %#define waiting time interval for participant to switch driving mode (unit: ms)
        Stop_and_Go_length=prelim_info.Section_length(strcmp(prelim_info.Experiment_task,...
            'Participant_StopAndGod')==1);

        %############# display information update frequency (unit: ms, must be multiple basic time step) ############

        Variable_name1={'numRingRoad';'numberOfLanes';'laneWidth';'startLength';...
            'mergeLength';'Stop_and_Go_length';'hdriveLength';'cadriveLength';'takeoverLength';...
            'divergeLength';'numVehicle_Lane';'OriginGap';'minGap';...
            'switchWaitingTime';'infoDispTime'};
        Variable_value1=[numRingRoad;numberOfLanes;laneWidth;startLength;...
            mergeLength;Stop_and_Go_length;hdriveLength;cadriveLength;takeoverLength;...
            divergeLength;numVehicle_Lane;OriginGap;minGap;...
            switchWaitingTime;infoDispTime];

        Variable_name2={};
        Variable_value2=[];
        Variable_name3={};
        Variable_value3=[];
        for k=1:Num_road
            Variable_value2=[Variable_value2;RR_translation(k,:)'];
            Variable_name2=[Variable_name2;...
                strrep(['RR',num2str(k),'_','translation_x'],' ','');...
                strrep(['RR',num2str(k),'_','translation_y'],' ','');...
                strrep(['RR',num2str(k),'_','translation_z'],' ','')];
            for j=1:numberOfLanes
                Variable_name3=[Variable_name3;...
                    strrep(['HeadwayRR',num2str(k),'_','Lane_',num2str(j)],' ','')];
            end
            Variable_name3=[Variable_name3;strrep(['DesiredSpeed',...
                '_','RR',num2str(k)],' ','')]
            Order_Headway_each=randperm(numberOfLanes);

            Variable_value3=[Variable_value3;[HeadwayTime_Lane(Order_Headway_each),DesiredSpeed(Order_Speead_each(k))]'];
        end

        Msg_Length=[]
        NumMsg=8;

        LMsg_red_ahead=200;
        LMsg_slow_ahead=200;
        LMsg_activate_Auto=200;
        LMsg_On_Auto=0;
        LMsg_During_Auto=500;
        LMsg_PrepareTakeOver=500;
        LMsg_Prepare_Diverge=1800;
        LMsg_Diverge=500;


        % ------------- Define How the NDTRs are distributed

        n = Num_road*2; % Total length must be even to have even # of 0s and 1s.
        numberOfOnes = n/2
        % Get a list of random locations, with no number repeating.
        indexes = randperm(n)
        % Start off with all zeros.
        x = zeros(1, n);
        % Now make half of them, in random locations, a 1.
        x(indexes(1:numberOfOnes)) = 1

        NDRT_1=x(1:length(x)/2);
        NDRT_2=x(length(x)/2+1:length(x));

        if i_experiment==1
            NDRT_value=NDRT_1';
        else
            NDRT_value=NDRT_2';
        end

        Msg_Length_val=[LMsg_red_ahead;LMsg_slow_ahead;LMsg_activate_Auto;
            LMsg_On_Auto;LMsg_During_Auto;LMsg_PrepareTakeOver;LMsg_Prepare_Diverge;
            LMsg_Diverge];

        NDRT_name={};
        Msg_Length_name={};
        for j=1:NumMsg
            Msg_Length_name=[Msg_Length_name;...
                strrep(['L_','Msg',num2str(j)],' ','')];
        end

        for k=1:Num_road
            NDRT_name=[NDRT_name;strrep(['RR',num2str(k),'_NDRT'],' ','')];
        end

        T_start_surr=4;
        DeltaZ_stop_nGo=200;

        Variable_name4={'T_start_surr';'DeltaZ_stop_nGo'}
        Variable_value4=[T_start_surr;DeltaZ_stop_nGo];

        Variable_name=[Variable_name1;Variable_name2;Variable_name3;...
            Msg_Length_name;NDRT_name;Variable_name4;Start_vehs_names;...
            TrafficLight_names];

        Variable_value=[Variable_value1;Variable_value2;Variable_value3;
            Msg_Length_val;NDRT_value;Variable_value4;Start_vehs_values;...
            TrafficLight_values];

        T1=table(Variable_name,Variable_value);

        % Controller_Filename=['Controller_input_param_round_',num2str(i_experiment),'.csv']

        Controller_Filename=['Controller_param',num2str(i_participant),'_round',num2str(i_experiment),'.csv']

        cd(cd_participants_info)
        writetable(T1,Controller_Filename,'Delimiter',',');
        if i_participant==random_participant
% %             copyfile(Controller_Filename,'Controller_input_param.csv');
% %             copyfile('Controller_input_param.csv',cd_controllers);

        end
        cd(cd_matlabcodes);

        % Controller_Filename=['Controller_input_param_round_',num2str(i_experiment),'.csv']
        % copyfile(Controller_Filename,new_cd)

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






    end
end
cd(cd_world)

