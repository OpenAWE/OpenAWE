function dynamics(eh, x, z, u, conf)

  % Get access to the system parameters
  wingArea = conf.wingArea;
  wingSpan = conf.wingSpan;
  chord    = conf.chord;
  mass     = conf.mass;

  p     = x.p;
  v     = x.v;
  R     = x.R;
  omega = x.omega;
  
  omegad = u.omegad;
  lambda = z.lambda;

  windNav = awe.model.wind_at_altitude(conf.wind, p);
  [airspeed,alpha,beta] = awe.model.aerodynamic_angles(v, R, windNav);

  aeroForcesBody = awe.model.aerodynamic_forces(alpha, ...
                                                beta, ...
                                                airspeed, ...
                                                omega, ....
                                                conf.coefficients, ...
                                                conf.airDensity, ...
                                                wingSpan, ...
                                                wingArea, ...
                                                chord);

  tether_force = awe.model.rigid_tether_force(p, alpha,...
                                              R, conf.tether_length, lambda, ...
                                              airspeed, conf.airDensity, ...
                                              conf.cableDiameter, ...
                                              conf.dragCoefficient, ...
                                              conf.cableDensity);

  forces      = R * aeroForcesBody + tether_force;

  accelNav    = forces / mass + conf.gravNav;

  RDot = R * [0,         omega(3),  -omega(2); ...
              -omega(3), 0,         omega(1) ; ...
              omega(2),  -omega(1), 0        ].';

  tether_eq = awe.model.rigid_tether_equation(p, v, accelNav);


  eh.setODE('p',      v);
  eh.setODE('v',      accelNav);
  eh.setODE('R',      RDot);
  eh.setODE('omega',  omegad);

  eh.setAlgEquation(tether_eq);
