function path_cost(ch, x, u, conf)

  omegad = u.omegad;
  ldd = u.ldd;

  self.addPathCost(conf.w_bodyAngularAccel * (omegad.'*omegad));
  self.addPathCost(conf.w_ddl * ldd^2);

  p = x.p;
  v = x.v;
  R = x.R;

  wind_at_altitude = awe.models.full.wind_at_altitude(conf.wind, p);

  [~,~,beta] = AerodynamicAngles( v, R, wind_at_altitude);
  ch.addPathCost(conf.w_beta * beta^2);

  ch.add( -1e-4 * state.dl )
end
