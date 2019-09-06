function r = rigid_tether_force(p,alpha,R,l,lambda,airspeed,airDensity,tetherDiameter,tetherDragCoefficient,tetherDensity)
% return tether force on body in navigation frame

tetherDrag      = 0.1250 * airDensity * airspeed^2 * ...
                       tetherDragCoefficient * tetherDiameter * l;

tetherDragBody  = tetherDrag * -[cos(alpha);0;sin(alpha)];
tetherDragNav   = R * tetherDragBody;

tetherWeight    = [0;0;tetherDensity*l*9.81];
tensionForce    = -lambda*p;

r = tensionForce + tetherWeight + tetherDragNav;
