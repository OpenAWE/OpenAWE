function path_cost(ch, x, u, conf)

  omegad = u.omegad;
  ldd = u.ldd;

  ch.add(conf.w_bodyAngularAccel * (omegad.'*omegad));
  ch.add(conf.w_ddl * ldd^2);

  p = x.p;
  v = x.v;
  R = x.R;

  wind_at_altitude = awe.models.full.wind_at_altitude(conf.wind, p);

  [~,~,beta] = awe.models.full.aerodynamic_angles( v, R, wind_at_altitude);
  ch.add(conf.w_beta * beta^2);

  ch.add( -1e-4 * x.ld )
end
