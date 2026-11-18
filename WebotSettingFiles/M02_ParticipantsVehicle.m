%Get initial position
Participant_veh=flip(RR1_onramp_Sec1_Trans_pts(1,:));
Participant_veh(1)=abs(Participant_veh(1))+prelim_info.Lane_width(1)/2;
Start_onramp_coord=Participant_veh+RR1_translation;

X_veh=Start_onramp_coord(1)+0.2;
Y_veh=Start_onramp_coord(2)+Y_Geometry;
Z_veh=Start_onramp_coord(3);

%Get initial rotation
Xrot_veh=0;
Yrot_veh=1;
Zrot_veh=0;
theta_veh=-Specs_info.Total_angle_circle;

fid = fopen('A01-ParticipantsVehicle.txt','r');
i = 1;
tline = fgetl(fid);
A{i} = tline;
while ischar(tline)
    i = i+1;
    tline = fgetl(fid);
    A{i} = tline;
end
fclose(fid);



A{2} = [strjoin({' ','translation',...
            num2str(X_veh),num2str(Y_veh),num2str(Z_veh)})] ;
A{3} = [strjoin({' ','rotation',...
            num2str(Xrot_veh),num2str(Yrot_veh),num2str(Zrot_veh),num2str(theta_veh)})] ;
% Write cell A into txt
fid = fopen('A01-ParticipantsVehicle.txt', 'w');
for i = 1:numel(A)
    if A{i+1} == -1
        fprintf(fid,'%s', A{i});
        break
    else
        fprintf(fid,'%s\n', A{i});
    end
end
fclose(fid)