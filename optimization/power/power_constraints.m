function power_constraints(ch, x, z, u, params, conf)
positionNav = x.get('positionNav');
velocityNav = x.get('velocityNav');
rotBodyToNav = x.get('rotBodyToNav');

windNavAtAltitude = GetWindAtAltitude(conf.wind, positionNav);


[airspeed,alpha,beta] = AerodynamicAngles( velocityNav, rotBodyToNav, windNavAtAltitude);
self.addPathConstraint(airspeed,'<=',params.MAX_AIRSPEED);
self.addPathConstraint(airspeed,'>=',params.MIN_AIRSPEED);
self.addPathConstraint(alpha,'<=',params.MAX_ALPHA);
self.addPathConstraint(alpha,'>=',params.MIN_ALPHA);
self.addPathConstraint(beta,'<=',params.MAX_BETA);
self.addPathConstraint(beta,'>=',params.MIN_BETA);

% line angle <= 30 deg  (or) cos(line angle) >= 60
downVec = rotBodyToNav(:,3);
winchVec = -positionNav/sqrt(positionNav(1)^2+positionNav(2)^2+positionNav(3)^2);
self.addPathConstraint(dot(downVec,winchVec),'>=',cos(60*pi/180));

%      % tension <= 5000 N
%      lambda = algState.get('lambda');
%      l = state.get('l');
%      self.addPathConstraint(lambda*l,'<=',params.MAX_TENSION);
%      self.addPathConstraint(lambda*l,'>=',params.MIN_TENSION);
