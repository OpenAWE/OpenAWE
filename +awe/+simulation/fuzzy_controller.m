function omega_dot = fuzzy_controller(x,conf)
p     = x(1:3);
v     = x(4:6);
R     = reshape(x(7:15), 3, 3);
omega = x(16:18);
  
height = -p(3);
sinkrate = v(3);

[airspeed,alpha,~] = awe.model.aerodynamic_angles(v,R, conf.wind.atBaseAltitude);

if height < 100 && airspeed > 10 && sinkrate > 0
  alpha_des = 0.4;
else
  alpha_des = -0.1;
end

if alpha < alpha_des
  pitchrate_des = 0.2;
else
  pitchrate_des = -0.2;
end

if omega(2) < pitchrate_des
  omega_dot = [0;0.2;0];
else
  omega_dot = [0;-0.2;0];
end
