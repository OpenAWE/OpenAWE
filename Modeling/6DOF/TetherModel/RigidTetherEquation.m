function tetherEquation = RigidTetherEquation(positionNav,velocityNav,accelNav,tetherLength,tetherSpeed,tetherAccel)
% return tether equation

tetherEquation = dot(positionNav,accelNav)+dot(velocityNav,velocityNav)-tetherLength*tetherAccel-tetherSpeed^2;