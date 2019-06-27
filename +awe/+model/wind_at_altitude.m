function r = wind_at_altitude(wind,p)
if wind.is_constant
  r = wind.atBaseAltitude;
else
  h = -p(3);
  r = wind.atBaseAltitude * (h / wind.baseAltitude)^wind.exponent;
end
  