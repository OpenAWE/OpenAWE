function r = wind_at_altitude(wind,p)
h = -p(3);
r = wind.atBaseAltitude * (h / wind.baseAltitude)^wind.exponent;