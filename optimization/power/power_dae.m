function power_dae(eh, x, z, u, parameters, conf)

  % Get access to the system parameters
  wingArea = parameters.get('wingArea');
  wingSpan = parameters.get('wingSpan');
  chord    = parameters.get('chord');
  mass     = parameters.get('mass');

  positionNav     = x.get('positionNav');
  velocityNav     = x.get('velocityNav');
  rotBodyToNav    = x.get('rotBodyToNav');
  bodyAngularRate = x.get('bodyAngularRate');

  bodyAngularAccel = u.get('bodyAngularAccel');

  l               = x.get('l');
  dl              = x.get('dl');
  ddl             = u.get('ddl');
  lambda          = z.get('lambda');

  windNav = GetWindAtAltitude(conf.wind,positionNav);
  [airspeed,alpha,beta] = AerodynamicAngles(velocityNav,rotBodyToNav,windNav);

  aeroForcesBody     = AerodynamicForces( alpha, ...
                                          beta, ...
                                          airspeed, ...
                                          bodyAngularRate, ....
                                          conf.coefficients, ...
                                          conf.airDensity, ...
                                          wingSpan, ...
                                          wingArea, ...
                                          chord);

  tetherForce = RigidTetherForce( positionNav,alpha,...
                                  rotBodyToNav,l,lambda, ...
                                  airspeed,conf.airDensity, ...
                                  conf.cableDiameter, ...
                                  conf.dragCoefficient, ...
                                  conf.cableDensity);

  forces      = rotBodyToNav * aeroForcesBody + tetherForce + conf.gravNav;

  accelNav    = LinearPointMassDynamics(forces,mass);

  RDot        = RDotFromAngularRates(rotBodyToNav, bodyAngularRate);

  tetherEquation = RigidTetherEquation(positionNav,velocityNav,accelNav,l,dl,ddl);


  eh.setODE('positionNav',      velocityNav);
  eh.setODE('velocityNav',      accelNav);
  eh.setODE('rotBodyToNav',     RDot);
  eh.setODE('bodyAngularRate',  bodyAngularAccel);

  eh.setODE('l',                dl);
  eh.setODE('dl',               ddl);

  eh.setODE('integratedWork',   l*dl*lambda);

  eh.setAlgEquation(tetherEquation);
