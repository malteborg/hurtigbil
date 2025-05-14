

clc; clear all; close all;
setpointRight = 70;
setpointLeft = 110;
setpointStraight = 90;
ACCerror = 35;

for i = 0:255
    if i > setpointLeft
        disp('Left'); % Display "Left"
        disp(i);
    elseif i < setpointRight
        disp('Right'); % Display "Right"
        disp(i);
    else
        disp('Straight'); % Optional: Handle values between setpointRight and setpointLeft
    end
end

for i = 0:255
    if (i <(setpointLeft-ACCerror))
        disp('straight left');
        disp(i);
    end
end

for i = 0:255
    if (i >(setpointRight+ACCerror))
        disp('straight right');
        disp(i);
        
    end
end

