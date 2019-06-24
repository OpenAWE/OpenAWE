function [yaw,pitch,roll] = yaw_pitch_roll(R)
yaw = atan2(R(2,1),R(1,1));
pitch = atan2(-R(3,1),sqrt(R(1,1)^2+R(2,1)^2));
roll = atan2(R(3,2),R(3,3));