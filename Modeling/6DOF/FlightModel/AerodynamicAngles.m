function [airspeed,alpha,beta] = AerodynamicAngles(velocityNav,rotBodyToNav,windNav)

relativeVelocity      = velocityNav - windNav;
vBody                 = rotBodyToNav.' * relativeVelocity; 

airspeed              = sqrt(relativeVelocity(1)^2 + relativeVelocity(2)^2 + relativeVelocity(3)^2);

% alpha                 = atan(vBody(3) ./ vBody(1));
alpha                 = vBody(3) ./ vBody(1);

% beta                  = asin(vBody(2) / airspeed);
beta                  = vBody(2) / airspeed;

