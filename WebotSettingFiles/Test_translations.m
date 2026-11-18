clearvars
clc
close all

format bank

Lstart=1000;
Theta_circle_Onramp=0.3;
L_onramp=400;
Rcircle_Onramp=1500;
LaneWidth=3.65


Xstart_Onramp=Lstart-(L_onramp*cos(Theta_circle_Onramp)+Rcircle_Onramp*sin(Theta_circle_Onramp));
Zstart_Onramp=0+(L_onramp*sin(Theta_circle_Onramp)+Rcircle_Onramp*(1-cos(Theta_circle_Onramp)));



% Circle section
Rcircle_Onramp=1500
Roadroation=-1.5708;
CircleRotation=-Roadroation+Theta_circle_Onramp;
RoadBarrierWidth=0.8;

Shift_circle=(2*LaneWidth+RoadBarrierWidth)+(LaneWidth+RoadBarrierWidth);
Shift_Onramp=(2*LaneWidth+RoadBarrierWidth)+(LaneWidth+RoadBarrierWidth)+RoadBarrierWidth/2

Rcircle_Onramp=Rcircle_Onramp;
Xstart_Onramp=Lstart-(L_onramp*cos(Theta_circle_Onramp)+Rcircle_Onramp*sin(Theta_circle_Onramp)+0.2)
Zstart_Onramp=0+(L_onramp*sin(Theta_circle_Onramp)+Rcircle_Onramp*(1-cos(Theta_circle_Onramp))+Shift_Onramp-0.2)

174.385
197.953
% Circle Offramp

RCircle_diverge=500;
C_rule=120;
Theta_circle_Offramp=Theta_circle_Onramp

sumLmain=9500;


Rotatation_Circle_diverge=-Roadroation+Theta_circle_Offramp;
Zstart_Offramp_Circle_diverge=-(RCircle_diverge+Shift_circle);
Xstart_Offramp_Circle=sumLmain;


Zstart_Offramp_Circle_diverge=-(RCircle_diverge+Shift_circle);
Xstart_Offramp_Circle=sumLmain;

Zend_Offramp_Circle_diverge=Shift_circle+RCircle_diverge*(1-cos(Theta_circle_Offramp));
Xend_Offramp_Circle_diverge=Xstart_Offramp_Circle+RCircle_diverge*sin(Theta_circle_Offramp)



Corrd_end_RR=[-3500 -18000]+[Zend_Offramp_Circle_diverge Xend_Offramp_Circle_diverge]

XRR2=Corrd_end_RR(1)+Zstart_Onramp
ZRR_2=Corrd_end_RR(2)-Xstart_Onramp-LaneWidth