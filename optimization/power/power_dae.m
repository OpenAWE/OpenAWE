function power_dae(eh, x, z, u, parameters, conf)

  % Get access to the system parameters
  wingArea = parameters.get('wingArea');
  wingSpan = parameters.get('wingSpan');
  chord    = parameters.get('chord');
  mass     = parameters.get('mass');

  p     = x.get('positionNav');
  v     = x.get('velocityNav');
  R     = x.get('rotBodyToNav');
  omega = x.get('bodyAngularRate');

  omegad = u.get('bodyAngularAccel');

  l               = x.get('l');
  ld              = x.get('ld');
  ldd             = u.get('ldd');
  lambda          = z.get('lambda');

  windNav = GetWindAtAltitude(conf.wind,p);
  [airspeed,alpha,beta] = AerodynamicAngles(v,R,windNav);

  aeroForcesBody     = AerodynamicForces( alpha, ...
                                          beta, ...
                                          airspeed, ...
                                          omega, ....
                                          conf.coefficients, ...
                                          conf.airDensity, ...
                                          wingSpan, ...
                                          wingArea, ...
                                          chord);

  tetherForce = rigid_tether_force( p, alpha,...
                                    R, l, lambda, ...
                                    airspeed, conf.airDensity, ...
                                    conf.cableDiameter, ...
                                    conf.dragCoefficient, ...
                                    conf.cableDensity);

  forces      = R * aeroForcesBody + tetherForce + conf.gravNav;

  accelNav    = forces / mass;

  RDot = R * [0,         omega(3),  -omega(2); ...
              -omega(3), 0,         omega(1) ; ...
              omega(2),  -omega(1), 0        ].';

  tetherEquation = RigidTetherEquation(p, v, accelNav, l, ld, ldd);


  eh.setODE('positionNav',      v);
  eh.setODE('velocityNav',      accelNav);
  eh.setODE('rotBodyToNav',     RDot);
  eh.setODE('bodyAngularRate',  omegad);

  eh.setODE('l',                ld);
  eh.setODE('dl',               ldd);

  eh.setODE('integratedWork',   l*ld*lambda);

  eh.setAlgEquation(tetherEquation);
