function path_cost(ch, x, u, conf)

ch.add(1e-6*(x.'*x));
ch.add(1e-4*(u.'*u));

%   omegad = u.omegad;
%   ldd = u.ldd;
% 
%   ch.add(conf.w_bodyAngularAccel * (omegad.'*omegad));
%   ch.add(conf.w_ddl * ldd^2);
% 
%   p = x.p;
%   v = x.v;
%   R = x.R;
% 
%   wind_at_altitude = awe.model.wind_at_altitude(conf.wind, p);
% 
%   [~,~,beta] = awe.model.aerodynamic_angles( v, R, wind_at_altitude);
%   ch.add(conf.w_beta * beta^2);
% 
%   ch.add( -1e-4 * x.ld )
end
