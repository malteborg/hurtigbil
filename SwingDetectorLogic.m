

clc; clear all; close all;
setpointRight = 70;
setpointLeft = 110;
setpointStraight = 90;
ACCerror = 35;

 

for i = 0:20
    y = 100*(1-exp(-0.3i));
    disp (i);
    disp (y);
end




