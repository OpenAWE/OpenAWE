function path_cost(ch, x, z, u, params, conf)

  endTime = params.endTime;

  bodyAngularAccel = u.bodyAngularAccel;
  ddl = u.ddl;

  self.addPathCost(params.w_bodyAngularAccel * (bodyAngularAccel.'*bodyAngularAccel)/endTime);
  self.addPathCost(params.w_ddl * ddl^2/endTime);


  positionNav = x.positionNav;
  velocityNav = x.velocityNav;
  rotBodyToNav = x.rotBodyToNav;

  windNavAtAltitude = GetWindAtAltitude(conf.wind, positionNav);

  [~,~,beta] = AerodynamicAngles( velocityNav, rotBodyToNav, windNavAtAltitude);
  ch.addPathCost(params.w_beta * beta^2/endTime);

  ch.add( -1e-4 * state.dl / endTime )
end
