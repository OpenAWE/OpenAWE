function tetherForce = RigidTetherForce(positionNav,alpha,rotBodyToNav,tetherLength,lambda,airspeed,airDensity,tetherDiameter,tetherDragCoefficient,tetherDensity)
% return tether force on body in navigation frame


tetherDrag      = 0.1250 * airDensity * airspeed^2 * ...
                       tetherDragCoefficient * tetherDiameter * tetherLength;

tetherDragBody  = tetherDrag * -[cos(alpha);0;sin(alpha)];
tetherDragNav   = rotBodyToNav * tetherDragBody;

tetherWeight    = [0;0;tetherDensity*tetherLength*9.81];
tensionForce    = -lambda*positionNav;  

tetherForce     = tetherDragNav + tetherWeight + tensionForce;