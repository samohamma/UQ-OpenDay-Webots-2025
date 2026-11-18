fileID = fopen('A01-TrafficLights.txt','w');
NumVehs=12;

translation =[-3500 0.5 -18000];
BMW_rotation= [0 1 0 0];

X_veh=0.4+zeros(numberOfLanes,NumVehs);
Y_veh=0.4+zeros(numberOfLanes,NumVehs);
Z_veh=0.4+zeros(numberOfLanes,NumVehs);

X_shif=-5.5;
X_start=translation(1)+X_shif;
Y_start=translation(2);
Z_Start=translation(3);
Shift=2*LaneWidth+RoadBarrierWidth/2;

Pos_loc=zeros(Num_road,3);
for k=1:Num_road
    Pos_loc(k,3)=RR_Lstart;
    Pos_loc(k,1)=+Shift;
end

Pos_TL=RR_translation+Pos_loc;
Pos_TL(:,2)=-5;
% Traffic lights on the Onramps at the begining of each freeway.
for i=1:Num_road
    TL_Line= ['DEF TL_onrmp',num2str(i-1), ' GenericTrafficLight {'];
    fprintf(fileID,'%s\r', TL_Line);
    TL_Line = [strjoin({' ','translation',...
        num2str(Pos_TL(i,1)),num2str(Pos_TL(i,2)),num2str(Pos_TL(i,3))})] ;
    fprintf(fileID,'%s\r', TL_Line);
    fprintf(fileID,'%s\r', ['  ','name ', '"','imaginary trafficlight',num2str(i-1),'"']);
    fprintf(fileID,'%s\r', ['  ','state ', '"','red','"']);
    fprintf(fileID,'%s\r', ['  ','controller ', '"traffic_light_red"']);
    fprintf(fileID,'%s\r', ['  ','controllerArgs ', '"2000 0.1 r"']);
    fprintf(fileID,'%s\r', '}');
end


% Imaginary traffic lights on the mainstream raffic lights on the


% Calculate the distance from the start coordinate in each RR
L_stopnGo=200;
z_TL_img=prelim_info.Section_length(...
    strcmp(prelim_info.Experiment_task,'CAVsStartExperiment')==1)+...
    sum(prelim_info.Section_length(...
    strcmp(prelim_info.Experiment_task,'MergeInHighway')==1))+Specs_info.DeltaZ_stop_nGo
y_TL_img=-5; % Burry this traffic light deep so that vehicles don't recongnize it
x_TL_img=0;

Pos_TL_imaginary=RR_translation+[x_TL_img,y_TL_img,z_TL_img]

for i=1:Num_road
    TL_Line= ['DEF TL_img_StnGo',num2str(i-1), ' GenericTrafficLight {'];
    fprintf(fileID,'%s\r', TL_Line);
    TL_Line = [strjoin({' ','translation',...
        num2str(Pos_TL_imaginary(i,1)),num2str(Pos_TL_imaginary(i,2)),num2str(Pos_TL_imaginary(i,3))})] ;
    fprintf(fileID,'%s\r', TL_Line);
    fprintf(fileID,'%s\r', ['  ','name ', '"','imaginary trafficlight',num2str(i-1),'"']);
    fprintf(fileID,'%s\r', ['  ','state ', '"','red','"']);
    fprintf(fileID,'%s\r', ['  ','controller ', '"traffic_light_red"']);
    fprintf(fileID,'%s\r', ['  ','controllerArgs ', '"2000 0.1 r"']);
    fprintf(fileID,'%s\r', '}');
end

%% Imaginary traffic lights on the mainstream raffic lights on the take-over

z_TL_img=Specs_info.L_tot_BF_TakeOver+Specs_info.DeltaZ_Msg6
y_TL_img=-5; % Burry this traffic light deep so that vehicles don't recongnize it
x_TL_img=0;

Pos_TL_imaginary_takOver=RR_translation+[x_TL_img,y_TL_img,z_TL_img]

for i=1:Num_road
    TL_Line= ['DEF TL_img_takeOver',num2str(i-1), ' GenericTrafficLight {'];
    fprintf(fileID,'%s\r', TL_Line);
    TL_Line = [strjoin({' ','translation',...
        num2str(Pos_TL_imaginary_takOver(i,1)),num2str(Pos_TL_imaginary_takOver(i,2)),num2str(Pos_TL_imaginary_takOver(i,3))})] ;
    fprintf(fileID,'%s\r', TL_Line);
    fprintf(fileID,'%s\r', ['  ','name ', '"','imaginary trafficlight',num2str(i-1),'"']);
    fprintf(fileID,'%s\r', ['  ','state ', '"','red','"']);
    fprintf(fileID,'%s\r', ['  ','controller ', '"traffic_light_red"']);
    fprintf(fileID,'%s\r', ['  ','controllerArgs ', '"2000 0.1 r"']);
    fprintf(fileID,'%s\r', '}');
end


%% Record the essential information for the controller file
TrafficLight_names={};
TrafficLight_values=[];

for k=1:Num_road
TrafficLight_values=[TrafficLight_values;[Pos_TL(k,:)]'];
end
for k=1:Num_road
TrafficLight_values=[TrafficLight_values;[Pos_TL_imaginary(k,:)]'];
end
for k=1:Num_road
TrafficLight_values=[TrafficLight_values;[Pos_TL_imaginary_takOver(k,:)]'];
end


for k=1:Num_road
   TrafficLight_names=[TrafficLight_names;...
   [strrep(['RR',num2str(k),'_TLonrmp','','_trans_x'],' ','')];...
   [strrep(['RR',num2str(k),'_TLonrmp','','_trans_y'],' ','')];...
   [strrep(['RR',num2str(k),'_TLonrmp','','_trans_z'],' ','')]]
end

for k=1:Num_road
   TrafficLight_names=[TrafficLight_names;...
   [strrep(['RR',num2str(k),'_TLImgnry_StnGo','','_trans_x'],' ','')];...
   [strrep(['RR',num2str(k),'_TLImgnry_StnGo','','_trans_y'],' ','')];...
   [strrep(['RR',num2str(k),'_TLImgnry_StnGo','','_trans_z'],' ','')]]
end

for k=1:Num_road
   TrafficLight_names=[TrafficLight_names;...
   [strrep(['RR',num2str(k),'_TLImgnry_TakeOver','','_trans_x'],' ','')];...
   [strrep(['RR',num2str(k),'_TLImgnry_TakeOver','','_trans_y'],' ','')];...
   [strrep(['RR',num2str(k),'_TLImgnry_TakeOver','','_trans_z'],' ','')]]
end


fclose(fileID);