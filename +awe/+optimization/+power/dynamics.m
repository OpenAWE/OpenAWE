function dynamics(eh, x, z, u, parameters, conf)

  % Get access to the system parameters
  wingArea = parameters.get('wingArea');
  wingSpan = parameters.get('wingSpan');
  chord    = parameters.get('chord');
  mass     = parameters.get('mass');

  p     = x.p;
  v     = x.v;
  R     = x.R;
  omega = x.omega;

  omegad = u.get('bodyAngularAccel');

  l               = x.get('l');
  ld              = x.get('ld');
  ldd             = u.get('ldd');
  lambda          = z.get('lambda');

  windNav = wind_at_altitude(conf.wind, p);
  [airspeed,alpha,beta] = AerodynamicAngles(v, R, windNav);

  aeroForcesBody     = AerodynamicForces( alpha, ...
                                          beta, ...
                                          airspeed, ...
                                          omega, ....
                                          conf.coefficients, ...
                                          conf.airDensity, ...
                                          wingSpan, ...
                                          wingArea, ...
                                          chord);

  tether_force = rigid_tether_force(p, alpha,...
                                    R, l, lambda, ...
                                    airspeed, conf.airDensity, ...
                                    conf.cableDiameter, ...
                                    conf.dragCoefficient, ...
                                    conf.cableDensity);

  forces      = R * aeroForcesBody + tether_force + conf.gravNav;

  accelNav    = forces / mass;

  RDot = R * [0,         omega(3),  -omega(2); ...
              -omega(3), 0,         omega(1) ; ...
              omega(2),  -omega(1), 0        ].';

  tether_eq = awe.models.full.rigid_tether_equation(p, v, accelNav, l, ld, ldd);


  eh.setODE('p',      v);
  eh.setODE('v',      accelNav);
  eh.setODE('R',      RDot);
  eh.setODE('omega',  omegad);

  eh.setODE('l',      ld);
  eh.setODE('ld',     ldd);

  eh.setODE('iwork',  l*ld*lambda);
  
  eh.setODE('p0',     0);
  eh.setODE('v0',     0);
  eh.setODE('R0',     0);
  eh.setODE('omega0', 0);

  eh.setAlgEquation(tether_eq);
