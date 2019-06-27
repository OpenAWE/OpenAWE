function dynamics(eh, x, z, u, conf)

  % Get access to the system parameters
  wind = conf.wind;
  wingArea = conf.wingArea;
  wingSpan = conf.wingSpan;
  chord    = conf.chord;
  mass     = conf.mass;
  airDensity = conf.airDensity;
  coefficients = conf.coefficients;

  p     = x.p;
  v     = x.v;
  R     = x.R;
  omega = x.omega;
  
  omegad = u.omegad;
  lambda = u.lambda;

  wind_at_altitude = awe.model.wind_at_altitude(wind, p);
  [airspeed,alpha,beta] = awe.model.aerodynamic_angles(v, R, wind_at_altitude);

  aero_forces_body = awe.model.aerodynamic_forces(alpha, ...
                                                  beta, ...
                                                  airspeed, ...
                                                  omega, ....
                                                  coefficients, ...
                                                  airDensity, ...
                                                  wingSpan, ...
                                                  wingArea, ...
                                                  chord);

%   tether_force = awe.model.rigid_tether_force(p, alpha,...
%                                               R, conf.tether_length, lambda, ...
%                                               airspeed, conf.airDensity, ...
%                                               conf.cableDiameter, ...
%                                               conf.dragCoefficient, ...
%                                               conf.cableDensity);

  forces      = R * aero_forces_body + lambda * p;
  accel    = forces / mass + conf.gravNav;

  RDot = R * [0,         omega(3),  -omega(2); ...
              -omega(3), 0,         omega(1) ; ...
              omega(2),  -omega(1), 0        ].';

  eh.setODE('p',      v);
  eh.setODE('v',      accel);
  eh.setODE('R',      RDot);
  eh.setODE('omega',  omegad);

%   tether_eq = awe.model.rigid_tether_equation(p, v, accel);
%   eh.setAlgEquation(tether_eq);
