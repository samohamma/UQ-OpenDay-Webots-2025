% The following code is the back up of the lines we used for upramp:
% After deciding to change the design, because of the inherent difficulties
% of having upramps (e.g., like bumping the road, ect). However, I decided
% to save these pieces of codes for future just in case because we put so
% much efforts into it.

% The below code is calculating end-points:
Trans_coord_RR_exit    

Height_upramp=Specs_exit.Height_upramp;
Length_upramp=Specs_exit.L_upramp;

RR1_upramp_loc=[(2*LaneWidth+LaneWidth+2*RoadBarrierWidth),...
    0,...
    Length_mainroad+Specs_exit.L_upramp];
RR2_start_x=RR1_end_x-Shift_Onramp...
    -(Specs_exit.Radius_curve)*(1-cos(Specs_exit.Angle_curve))...
    -Specs_exit.L_downramp*(sin(Specs_exit.Angle_curve))...
    -L_onramp(2)*sin(Theta_circle)...
    -Rcircle_Onramp*(1-cos(Theta_circle));

RR2_start_Z=RR1_end_z...
    +Specs_exit.L_upramp...
    +Specs_exit.L_upStraight...
    +Specs_exit.Radius_curve*sin(Specs_exit.Angle_curve)...
    +Specs_exit.L_downramp*(cos(Specs_exit.Angle_curve))...
    +L_onramp(2)*cos(Theta_circle)...
    +Rcircle_Onramp*(sin(Theta_circle))...
    -RR_Lstart;

% RR_upramp=RR_translation+[RR1_upramp_loc;RR1_upramp_loc;RR1_upramp_loc;RR1_upramp_loc]

% Shift_downramp_x=-Specs_exit.Radius_curve*(1-cos(Specs_exit.Angle_curve));
% Shift_downramp_z=Specs_exit.L_upStraight+Specs_exit.Radius_curve*sin(Specs_exit.Angle_curve);

% The below code is for calculating and printing stuff within the
% experiment

    %     % Upramp
    %     fprintf(fileID,'%s\r',[Spc2,'DEF SLOPE Road', '{']);
    %     fprintf(fileID,'%s\r',[Spc3,'translation',Spc1,'0',Spc1,'0',...
    %         Spc1,num2str(-Length_upramp)]);
    %     fprintf(fileID,'%s\r',[Spc3,'rotation 0 1 0 0']);
    %     fprintf(fileID,'%s\r',[Spc3,'name "road(1)"']);
    %     fprintf(fileID,'%s\r',[Spc3,'width',Spc1,num2str(2*LaneWidth)]);
    %     fprintf(fileID,'%s\r',[Spc3,'bottom TRUE']);
    %     fprintf(fileID,'%s\r',[Spc3,'wayPoints [']);
    %
    %     fprintf(fileID,'%s\r',[Spc4,'0 0 0']);
    %     fprintf(fileID,'%s\r',[Spc4,'0',...
    %         Spc1,num2str(Height_upramp),Spc1,num2str(Length_upramp)]);
    %     fprintf(fileID,'%s\r',[Spc3,']']);
    %
    %     fprintf(fileID,'%s\r',[Spc3,'roadBoundingObject TRUE']);
    %     fprintf(fileID,'%s\r',[Spc3,'rightBorderBoundingObject TRUE']);
    %     fprintf(fileID,'%s\r',[Spc3,'leftBorderBoundingObject TRUE']);
    %     fprintf(fileID,'%s\r',[Spc3,'castShadows TRUE']);
    %     fprintf(fileID,'%s\r',[Spc2,'}']);
    %
    %     % Straight up there
    %     fprintf(fileID,'%s\r',[Spc2,'DEF MIDDLE StraightRoadSegment {']);
    %     fprintf(fileID,'%s\r',[Spc3,'translation ',['0',Spc1,...
    %         num2str(Height_upramp),Spc1,'0']]); %Manual
    %     fprintf(fileID,'%s\r',[Spc3,'width ',num2str(2*LaneWidth)]);
    %     fprintf(fileID,'%s\r',[Spc3,'lines [']);
    %     fprintf(fileID,'%s\r',[Spc4,'RoadLine {']);
    %     fprintf(fileID,'%s\r',[Spc5,'type "continuous"']);
    %     fprintf(fileID,'%s\r',[Spc4,'}']);
    %     fprintf(fileID,'%s\r',[Spc3,']']);
    %     fprintf(fileID,'%s\r',[Spc3,'bottom TRUE']);
    %     fprintf(fileID,'%s\r',[Spc3,'length 170']); %Manual
    %     fprintf(fileID,'%s\r',[Spc3,'rightBorderBoundingObject TRUE']);
    %     fprintf(fileID,'%s\r',[Spc3,'leftBorderBoundingObject TRUE']);
    %     fprintf(fileID,'%s\r',[Spc3,'castShadows TRUE']);
    %     fprintf(fileID,'%s\r',[Spc3,'roadBoundingObject TRUE']);
    %     fprintf(fileID,'%s\r',[Spc2,'}']);
    %
    %
    %     % Curve
    %     fprintf(fileID,'%s\r',[Spc2,'DEF EST_CURVE CurvedRoadSegment {']);
    %     fprintf(fileID,'%s\r',[Spc3,'translation ',Spc1,...
    %         num2str(-Specs_exit.Radius_curve),Spc1,num2str(Height_upramp),...
    %         Spc1,num2str(Specs_exit.L_upStraight)]); %Manual
    %     fprintf(fileID,'%s\r',[Spc3,'name ',Spc1,'"road(5)"']);
    %     fprintf(fileID,'%s\r',[Spc3,'width ',num2str(2*LaneWidth)]);
    %     fprintf(fileID,'%s\r',[Spc3,'bottom TRUE']);
    %     fprintf(fileID,'%s\r',[Spc3,'curvatureRadius',Spc1,num2str(Specs_exit.Radius_curve)]);
    %     fprintf(fileID,'%s\r',[Spc3,'totalAngle',Spc1,num2str(Specs_exit.Angle_curve)]);
    %     fprintf(fileID,'%s\r',[Spc3,'subdivision',Spc1,'32']); %Manual
    %     fprintf(fileID,'%s\r',[Spc3,'tilt',Spc1,'0.3']); %Manual
    %     fprintf(fileID,'%s\r',[Spc3,'roadBoundingObject TRUE']);
    %     fprintf(fileID,'%s\r',[Spc3,'rightBorderBoundingObject TRUE']);
    %     fprintf(fileID,'%s\r',[Spc3,'leftBorderBoundingObject TRUE']);
    %     fprintf(fileID,'%s\r',[Spc3,'castShadows TRUE']);
    %     fprintf(fileID,'%s\r',[Spc2,'}']);
    %     %
    %
    %    % Downramp
    %     fprintf(fileID,'%s\r',[Spc2,'DEF SLOPE Road',Spc1,'{']);
    %     fprintf(fileID,'%s\r',[Spc3,'translation',Spc1,...
    %         num2str(Shift_downramp_x),Spc1,num2str(Height_upramp),Spc1,...
    %         num2str(Shift_downramp_z)]);
    %     fprintf(fileID,'%s\r',[Spc3,'rotation',Spc1,'0 1 0',Spc1,...
    %         num2str(-Specs_exit.Angle_curve)]);
    %     fprintf(fileID,'%s\r',[Spc3,'name "road(1)"']);
    %     fprintf(fileID,'%s\r',[Spc3,'width',Spc1,num2str(2*LaneWidth)]);
    %     fprintf(fileID,'%s\r',[Spc3,'bottom TRUE']);
    %
    %     fprintf(fileID,'%s\r',[Spc3,'wayPoints [']);
    %     fprintf(fileID,'%s\r',[Spc4,'0 0 0']);
    %     fprintf(fileID,'%s\r',[Spc4,'0',...
    %         Spc1,num2str(-Height_upramp),Spc1,num2str(Length_upramp)]);
    %     fprintf(fileID,'%s\r',[Spc3,']']);
    %
    %     fprintf(fileID,'%s\r',[Spc3,'roadBoundingObject TRUE']);
    %     fprintf(fileID,'%s\r',[Spc3,'rightBorderBoundingObject TRUE']);
    %     fprintf(fileID,'%s\r',[Spc3,'leftBorderBoundingObject TRUE']);
    %     fprintf(fileID,'%s\r',[Spc3,'castShadows TRUE']);
    %     fprintf(fileID,'%s\r',[Spc2,'}'])
