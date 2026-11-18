% This script automatically creates all the AVs that are used in the
% different parts of the experiment. % The initial position of each vehicle
% (at the start of the simulation) is stored in the 'translation' property
% of each AV [x,y,z], which stands for:
%             x= the lateral positon (increasing from right to left)
%             y= the vertical position (increasing from the bottom to the top)
%             z= the longitudinal position (increasing with the driving direction)

% AVs considered are of four types:
    % 1) Backup: Those at the very end, designed to ensure the lines of
    % behind and surrounding the participants are always suffient so that
    % the participant doesn't miss them

    % 2) Surrounding vehicles in the merge area, designed to ensure the
    % the participant enters each freeway without following any AV, but at
    % the same time is surrounded by AVs from behind and the adacent lanes

    % 3) Stop-and-go vehicles, these vehicles are waiting for the
    % participant to arrive at some point, and then start having NGSIM
    % vehicle trajectories for their trajectory


    % 4) Broken vehicle which, designed to issue the take-over request for
    % the participants 

% Also, this script automatically records the starting points for the first
% row of AVs, in each category, for each road.



fileID = fopen('A01-WebotsCAVs.txt','w');

BMW_rotation= [0 1 0 0];



NumVehs_backup=3; % Per row.
NumVehs_surr=3; % Surrounding vehicles at start  % Per row.
numVehicle_Stop_nGo=1; % Stop-and-go vehicles  % Per row.

%%  1) Create back up vehicles in the mainstream

% numbering goes like :i= the current row increase in the driving direction
                    % :j= the current lane- it increases from the right to left 
x_veh=zeros(NumVehs_backup,numberOfLanes);
y_veh=zeros(NumVehs_backup,numberOfLanes);
z_veh=zeros(NumVehs_backup,numberOfLanes);

   
x_shift=(floor((numberOfLanes-1)/2)+0.5)*(Specs_info.LaneWidth); %

% Position of the first AV in the mainstream:
        % The AV in the Rightmost lane at z start.
    
x_start=RR_translation(1,1)+x_shift;
y_start=RR_translation(1,2)+0.38;
z_start=RR_translation(1,3);

InitialGap=30; % Gap between CAVs


for i=1:NumVehs_backup
    for j=1:numberOfLanes

        Col=rand(1,3);

        x_veh(i,j)=x_start-(j-1)*LaneWidth;
        y_veh(i,j)=y_start;
        z_veh(i,j)=z_start+(i-1)*InitialGap;

        BMW_line= ['BmwX5 {'];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line = [strjoin({' ','translation',...
            num2str(x_veh(i,j)),num2str(y_veh(i,j)),num2str(z_veh(i,j))})] ;
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({' ','rotation',...
            num2str(BMW_rotation(1)),num2str(BMW_rotation(2)),...
            num2str(BMW_rotation(3)),num2str(BMW_rotation(4))})] ;
        fprintf(fileID,'%s\r', BMW_line)


        BMW_line= [strjoin({' ','color',...
            num2str(Col(1)),num2str(Col(2)),num2str(Col(3))})] ;
        fprintf(fileID,'%s\r', BMW_line)


        BMW_name=strrep(strjoin({'"Veh_row',num2str(i),'_Lane_',num2str(j),'"'}),' ','')
        BMW_line = [strjoin({' ','name '}),BMW_name];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({' ','controller','"auto_backup"'})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({' ','supervisor','TRUE'})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({' ','sensorsSlotFront','['})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({'  ','Radar','{'})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({'    ','minRange','2'})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({'    ','maxRange','80'})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({'    ','horizontalFieldOfView','0.05'})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({'    ','verticalFieldOfView','0.002'})];
        fprintf(fileID,'%s\r', BMW_line)


        BMW_line= [strjoin({'  ','}'})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({' ',']'})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({' ','sensorsSlotTop','['})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({'  ','GPS','{'})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({'  ','}',})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({'  ','Receiver','{'})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({'  ','}',})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({'  ','Emitter','{'})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({'  ','}',})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({' ',']'})];
        fprintf(fileID,'%s\r', BMW_line)
        fprintf(fileID,'%s\r',' interior FALSE')
        fprintf(fileID,'%s\r',' dynamicSpeedDisplay FALSE')
        fprintf(fileID,'%s\r',' indicatorLevers FALSE')
        fprintf(fileID,'%s\r',' completeInterior FALSE')
        fprintf(fileID,'%s\r',' rightWingMirror FALSE')
        fprintf(fileID,'%s\r',' leftWingMirror FALSE')
        fprintf(fileID,'%s\r',' rearMirror FALSE')

        LineText ='}' ;
        fprintf(fileID,'%s\r', LineText)
    end
end

% Variable_name={};
Variable_name2={};
Variable_value2=[];
Variable_name3={};
Variable_value3=[];

% Extract the relative position of the first row of vehicles in the first
% road
Pos_veh=[]
for j=1:numberOfLanes
Pos_veh=[Pos_veh;[x_veh(1,j),y_veh(1,j),z_veh(1,j)]-RR_translation(1,:)];
end

% Derive the global position of first row vehicles in each road
Pos_veh_RR=[]
for k=1:Num_road
Pos_veh_RR=[Pos_veh_RR;[RR_translation(k,:)+Pos_veh]];
end

Backup_veh_name={};
Backup_veh_value=[];

for k=1:Num_road*numberOfLanes
   Backup_veh_value=[Backup_veh_value;[Pos_veh_RR(k,:)]'];
end

for k=1:Num_road
    for j=1:numberOfLanes
   Backup_veh_name=[Backup_veh_name;...
   [strrep(['RR',num2str(k),'_Backup_Lane',num2str(j),'_trans_x'],' ','')];...
   [strrep(['RR',num2str(k),'_Backup_Lane',num2str(j),'_trans_y'],' ','')];...
   [strrep(['RR',num2str(k),'_Backup_Lane',num2str(j),'_trans_z'],' ','')]]
    end
end

%% 2) Create surrounding vehicles at the merge area
LSurr=200;
z_start_surr=RR_translation(1,3)+Specs_info.L_start-LSurr;

% numbering goes like :i= the current row increase in the driving direction
                    % :j= the current lane- it increases from the right to left 
x_veh=zeros(NumVehs_surr,numberOfLanes);
y_veh=zeros(NumVehs_surr,numberOfLanes);
z_veh=zeros(NumVehs_surr,numberOfLanes);

for i=1:NumVehs_surr
    for j=1:numberOfLanes

        Col=rand(1,3);

        x_veh(i,j)=x_start-(j-1)*LaneWidth;
        y_veh(i,j)=y_start;
        z_veh(i,j)=z_start_surr+(i-1)*InitialGap;

        BMW_line= ['BmwX5 {'];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line = [strjoin({' ','translation',...
            num2str(x_veh(i,j)),num2str(y_veh(i,j)),num2str(z_veh(i,j))})] ;
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({' ','rotation',...
            num2str(BMW_rotation(1)),num2str(BMW_rotation(2)),...
            num2str(BMW_rotation(3)),num2str(BMW_rotation(4))})] ;
        fprintf(fileID,'%s\r', BMW_line)


        BMW_line= [strjoin({' ','color',...
            num2str(Col(1)),num2str(Col(2)),num2str(Col(3))})] ;
        fprintf(fileID,'%s\r', BMW_line)


        BMW_name=strrep(strjoin({'"Veh_row',num2str(NumVehs_backup+i),'_Lane_',num2str(j),'"'}),' ','')
        BMW_line = [strjoin({' ','name '}),BMW_name];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({' ','controller','"auto_surrounding_merge"'})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({' ','supervisor','TRUE'})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({' ','sensorsSlotFront','['})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({'  ','Radar','{'})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({'    ','minRange','2'})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({'    ','maxRange','80'})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({'    ','horizontalFieldOfView','0.05'})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({'    ','verticalFieldOfView','0.002'})];
        fprintf(fileID,'%s\r', BMW_line)


        BMW_line= [strjoin({'  ','}'})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({' ',']'})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({' ','sensorsSlotTop','['})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({'  ','GPS','{'})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({'  ','}',})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({'  ','Receiver','{'})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({'  ','}',})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({'  ','Emitter','{'})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({'  ','}',})];
        fprintf(fileID,'%s\r', BMW_line)

        BMW_line= [strjoin({' ',']'})];
        fprintf(fileID,'%s\r', BMW_line)
        fprintf(fileID,'%s\r',' interior FALSE')
        fprintf(fileID,'%s\r',' dynamicSpeedDisplay FALSE')
        fprintf(fileID,'%s\r',' indicatorLevers FALSE')
        fprintf(fileID,'%s\r',' completeInterior FALSE')
        fprintf(fileID,'%s\r',' rightWingMirror FALSE')
        fprintf(fileID,'%s\r',' leftWingMirror FALSE')
        fprintf(fileID,'%s\r',' rearMirror FALSE')

        LineText ='}' ;
        fprintf(fileID,'%s\r', LineText)
    end
end

% RR1_SurrVeh_Lane1_trans_x


% Extract the relative position of the first row of vehicles in the first
% road
Pos_veh=[]
for j=1:numberOfLanes
Pos_veh=[Pos_veh;[x_veh(1,j),y_veh(1,j),z_veh(1,j)]-RR_translation(1,:)];
end

% Derive the global position of first row vehicles in each road
Pos_veh_RR=[]
for k=1:Num_road
Pos_veh_RR=[Pos_veh_RR;[RR_translation(k,:)+Pos_veh]];
end

Surr_veh_RR_name={};
Surr_veh_RR_value=[];

for k=1:Num_road*numberOfLanes
   Surr_veh_RR_value=[Surr_veh_RR_value;[Pos_veh_RR(k,:)]'];
end

for k=1:Num_road
    for j=1:numberOfLanes
   Surr_veh_RR_name=[Surr_veh_RR_name;...
   [strrep(['RR',num2str(k),'_SurrVeh_Lane',num2str(j),'_trans_x'],' ','')];...
   [strrep(['RR',num2str(k),'_SurrVeh_Lane',num2str(j),'_trans_y'],' ','')];...
   [strrep(['RR',num2str(k),'_SurrVeh_Lane',num2str(j),'_trans_z'],' ','')]]
    end
end


%% 3) Create the stop-and-go vehicles

% numbering goes like :i= the current row increase in the driving direction
                    % :j= the current lane- it increases from the right to left 
x_veh=zeros(numVehicle_Stop_nGo,numberOfLanes);
y_veh=zeros(numVehicle_Stop_nGo,numberOfLanes);
z_veh=zeros(numVehicle_Stop_nGo,numberOfLanes);

% Calculate the distance from the start coordinate in each RR

z_start=RR_translation(1,3)+prelim_info.Section_length(...
    strcmp(prelim_info.Experiment_task,'CAVsStartExperiment')==1)+...
    sum(prelim_info.Section_length(...
    strcmp(prelim_info.Experiment_task,'MergeInHighway')==1))+Specs_info.DeltaZ_stop_nGo
     
% ;

for i=1:numVehicle_Stop_nGo
    for j=1:numberOfLanes
        Col=rand(1,3);

        x_veh(i,j)=x_start-(j-1)*LaneWidth;
        y_veh(i,j)=y_start;

        z_veh(i,j)=z_start+(i-1)*InitialGap;
        
        BMW_line= ['BmwX5 {'];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line = [strjoin({' ','translation',...
            num2str(x_veh(i,j)),num2str(y_veh(i,j)),num2str(z_veh(i,j))})] ;
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({' ','rotation',...
            num2str(BMW_rotation(1)),num2str(BMW_rotation(2)),...
            num2str(BMW_rotation(3)),num2str(BMW_rotation(4))})] ;
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({' ','color',...
            num2str(Col(1)),num2str(Col(2)),num2str(Col(3))})] ;
        fprintf(fileID,'%s\r', BMW_line)
        BMW_name=strrep(strjoin({'"Veh_row',num2str(NumVehs_backup+NumVehs_surr+i),'_Lane_',num2str(j),'"'}),' ','')
        BMW_line = [strjoin({' ','name '}),BMW_name];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({' ','controller','"auto_stop_and_go"'})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({' ','supervisor','TRUE'})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({' ','sensorsSlotFront','['})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({'  ','Radar','{'})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({'    ','minRange','2'})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({'    ','maxRange','80'})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({'    ','horizontalFieldOfView','0.05'})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({'    ','verticalFieldOfView','0.002'})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({'  ','}'})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({' ',']'})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({' ','sensorsSlotTop','['})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({'  ','GPS','{'})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({'  ','}',})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({'  ','Receiver','{'})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({'  ','}',})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({'  ','Emitter','{'})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({'  ','}',})];
        fprintf(fileID,'%s\r', BMW_line)
        BMW_line= [strjoin({' ',']'})];
        fprintf(fileID,'%s\r', BMW_line)
        fprintf(fileID,'%s\r',' interior FALSE')
        fprintf(fileID,'%s\r',' dynamicSpeedDisplay FALSE')
        fprintf(fileID,'%s\r',' indicatorLevers FALSE')
        fprintf(fileID,'%s\r',' completeInterior FALSE')
        fprintf(fileID,'%s\r',' rightWingMirror FALSE')
        fprintf(fileID,'%s\r',' leftWingMirror FALSE')
        fprintf(fileID,'%s\r',' rearMirror FALSE')
        LineText ='}' ;
        fprintf(fileID,'%s\r', LineText)
    end
end

% Variable_name={};
Variable_name2={};
Variable_value2=[];
Variable_name3={};
Variable_value3=[];


% Extract the relative position of the first row of vehicles in the first
% road
Pos_veh=[]
for j=1:numberOfLanes
Pos_veh=[Pos_veh;[x_veh(1,j),y_veh(1,j),z_veh(1,j)]-RR_translation(1,:)];
end

% Derive the global position of first row vehicles in each road
Pos_veh_RR=[]
for k=1:Num_road
Pos_veh_RR=[Pos_veh_RR;[RR_translation(k,:)+Pos_veh]];
end

Stop_nGo_veh_name={};
Stop_nGo_veh_value=[];

for k=1:Num_road*numberOfLanes
   Stop_nGo_veh_value=[Stop_nGo_veh_value;[Pos_veh_RR(k,:)]'];
end

for k=1:Num_road
    for j=1:numberOfLanes
   Stop_nGo_veh_name=[Stop_nGo_veh_name;...
   [strrep(['RR',num2str(k),'_Stop_nGoVeh_Lane',num2str(j),'_trans_x'],' ','')];...
   [strrep(['RR',num2str(k),'_Stop_nGoVeh_Lane',num2str(j),'_trans_y'],' ','')];...
   [strrep(['RR',num2str(k),'_Stop_nGoVeh_Lane',num2str(j),'_trans_z'],' ','')]]
    end
end

%% 4) Create the broken vehicle at the take-over area
x_veh=x_start-3*LaneWidth;
y_veh=y_start;
z_veh=RR_translation(1,3)+(Specs_info.L_tot_BF_TakeOver+Specs_info.DeltaZ_Msg6);

Col=rand(1,3);

BMW_line= ['BmwX5 {'];
fprintf(fileID,'%s\r', BMW_line)

BMW_line = [strjoin({' ','translation',...
    num2str(x_veh),num2str(y_veh),num2str(z_veh)})] ;
fprintf(fileID,'%s\r', BMW_line);

BMW_line= [strjoin({' ','rotation',...
    num2str(BMW_rotation(1)),num2str(BMW_rotation(2)),...
    num2str(BMW_rotation(3)),num2str(BMW_rotation(4))})] ;
fprintf(fileID,'%s\r', BMW_line);

BMW_line= [strjoin({' ','color',...
    num2str(Col(1)),num2str(Col(2)),num2str(Col(3))})] ;
fprintf(fileID,'%s\r', BMW_line)


BMW_name=strrep(strjoin({'"Broken_TakeOver','"'}),' ','')
BMW_line = [strjoin({' ','name '}),BMW_name];
fprintf(fileID,'%s\r', BMW_line)

BMW_line= [strjoin({' ','controller','"auto_broken"'})];
fprintf(fileID,'%s\r', BMW_line)

BMW_line= [strjoin({' ','supervisor','TRUE'})];
fprintf(fileID,'%s\r', BMW_line)

BMW_line= [strjoin({' ','sensorsSlotFront','['})];
fprintf(fileID,'%s\r', BMW_line)

BMW_line= [strjoin({'  ','Radar','{'})];
fprintf(fileID,'%s\r', BMW_line)

BMW_line= [strjoin({'    ','minRange','2'})];
fprintf(fileID,'%s\r', BMW_line)

BMW_line= [strjoin({'    ','maxRange','80'})];
fprintf(fileID,'%s\r', BMW_line)

BMW_line= [strjoin({'    ','horizontalFieldOfView','0.05'})];
fprintf(fileID,'%s\r', BMW_line)

BMW_line= [strjoin({'    ','verticalFieldOfView','0.002'})];
fprintf(fileID,'%s\r', BMW_line)


BMW_line= [strjoin({'  ','}'})];
fprintf(fileID,'%s\r', BMW_line)

BMW_line= [strjoin({' ',']'})];
fprintf(fileID,'%s\r', BMW_line)

BMW_line= [strjoin({' ','sensorsSlotTop','['})];
fprintf(fileID,'%s\r', BMW_line)

BMW_line= [strjoin({'  ','GPS','{'})];
fprintf(fileID,'%s\r', BMW_line)
BMW_line= [strjoin({'  ','}',})];
fprintf(fileID,'%s\r', BMW_line)

BMW_line= [strjoin({'  ','Receiver','{'})];
fprintf(fileID,'%s\r', BMW_line)
BMW_line= [strjoin({'  ','}',})];
fprintf(fileID,'%s\r', BMW_line)

BMW_line= [strjoin({'  ','Emitter','{'})];
fprintf(fileID,'%s\r', BMW_line)
BMW_line= [strjoin({'  ','}',})];
fprintf(fileID,'%s\r', BMW_line)

BMW_line= [strjoin({' ',']'})];
fprintf(fileID,'%s\r', BMW_line)
fprintf(fileID,'%s\r',' interior FALSE')
fprintf(fileID,'%s\r',' dynamicSpeedDisplay FALSE')
fprintf(fileID,'%s\r',' indicatorLevers FALSE')
fprintf(fileID,'%s\r',' completeInterior FALSE')
fprintf(fileID,'%s\r',' rightWingMirror FALSE')
fprintf(fileID,'%s\r',' leftWingMirror FALSE')
fprintf(fileID,'%s\r',' rearMirror FALSE')

LineText ='}' ;
fprintf(fileID,'%s\r', LineText)


% Extract the relative position of the first row of vehicles in the first
% road
Pos_veh=[[x_veh(1,1),y_veh(1,1),z_veh(1,1)]-RR_translation(1,:)];


% Derive the global position of first row vehicles in each road
Pos_veh_RR=[]
for k=1:Num_road
Pos_veh_RR=[Pos_veh_RR;[RR_translation(k,:)+Pos_veh]];
end


BrokenVeh_name={};
BrokenVeh_value=[];

for k=1:Num_road
BrokenVeh_value=[BrokenVeh_value;[Pos_veh_RR(k,:)]'];
end

for k=1:Num_road
   BrokenVeh_name=[BrokenVeh_name;...
   [strrep(['RR',num2str(k),'_BrokenVeh','_trans_x'],' ','')];...
   [strrep(['RR',num2str(k),'_BrokenVeh','_trans_y'],' ','')];...
   [strrep(['RR',num2str(k),'_BrokenVeh','_trans_z'],' ','')]]
end

fclose(fileID);
%% 5) Record the data relevant for the position Controller_input_param.csv in the controllers folder

Start_vehs_names=[Backup_veh_name;Surr_veh_RR_name;Stop_nGo_veh_name;BrokenVeh_name];
Start_vehs_names=[Start_vehs_names;...
    'NumVehs_backup';'NumVehs_surr';'numVehicle_Stop_nGo'];
Start_vehs_values=[Backup_veh_value;Surr_veh_RR_value;Stop_nGo_veh_value;BrokenVeh_value];
Start_vehs_values=[Start_vehs_values;NumVehs_backup;NumVehs_surr;numVehicle_Stop_nGo];

