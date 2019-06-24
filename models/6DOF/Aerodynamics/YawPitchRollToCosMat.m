function R = yawPitchRollToCosMat(yaw, pitch, roll)

R     = [cos(pitch)*cos(yaw), -cos(roll)*sin(yaw) + sin(roll)*sin(pitch)*cos(yaw), sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw);
         cos(pitch)*sin(yaw), cos(roll)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw),  -sin(roll)*cos(yaw) + cos(roll)*sin(pitch)*sin(yaw);
         -sin(pitch),         sin(roll)*cos(pitch),                                cos(roll)*cos(pitch)];