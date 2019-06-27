function constraints(ch, x, conf)
% positionNav = x.get('positionNav');
% velocityNav = x.get('velocityNav');
% rotBodyToNav = x.get('rotBodyToNav');
% 
% windNavAtAltitude = GetWindAtAltitude(conf.wind, positionNav);
% 

% [airspeed,alpha,beta] = AerodynamicAngles( velocityNav, rotBodyToNav, windNavAtAltitude);
% ch.add(airspeed,'<=',conf.MAX_AIRSPEED);
% ch.add(airspeed,'>=',conf.MIN_AIRSPEED);
% ch.add(alpha,'<=',conf.MAX_ALPHA);
% ch.add(alpha,'>=',conf.MIN_ALPHA);
% ch.add(beta,'<=',conf.MAX_BETA);
% ch.add(beta,'>=',conf.MIN_BETA);

% line angle <= 30 deg  (or) cos(line angle) >= 60
% downVec = rotBodyToNav(:,3);
% winchVec = -positionNav/sqrt(positionNav(1)^2+positionNav(2)^2+positionNav(3)^2);
% ch.add(dot(downVec,winchVec),'>=',cos(60*pi/180));

%      % tension <= 5000 N
%      lambda = algState.get('lambda');
%      l = state.get('l');
%      ch.add(lambda*l,'<=',params.MAX_TENSION);
%      ch.add(lambda*l,'>=',params.MIN_TENSION);
