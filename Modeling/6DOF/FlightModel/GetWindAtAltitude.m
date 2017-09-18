function windAtAltitude = GetWindAtAltitude(wind,positionNav)
h = -positionNav(3);
windAtAltitude = wind.atBaseAltitude * (h / wind.baseAltitude)^wind.exponent;